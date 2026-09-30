use crate::args::{get_f32, get_i16, get_str, get_u16_hex_or_dec, get_u64};
use motor_core::bus::{open_transport, CanBus, Transport, TransportParams};
use motor_core::CanFrame;
use motor_vendor_hightorque::{HightorqueController, HightorqueFeedbackState};
use std::collections::HashMap;
use std::convert::TryFrom;
use std::sync::Arc;
use std::time::{Duration, Instant};

const TWO_PI: f32 = std::f32::consts::PI * 2.0;

/// 高创(HT)广播发现:vendor crate 无 scan API,广播查询必须裸总线手写
/// 17 01 帧 + 收 27 01 回帧(收敛 2026-09-29 后,这里是 CLI 仅存的 HT
/// 裸总线代码,仅用于发现;控制/状态/参数全部走 vendor)。
#[derive(Debug, Clone, Copy)]
struct HtScanHit {
    motor_id: u16,
    pos_raw: i16,
    vel_raw: i16,
    tqe_raw: i16,
}

fn send_ht_scan_query(bus: &dyn CanBus, motor_id: u16) -> Result<(), Box<dyn std::error::Error>> {
    let payload = [0x17u8, 0x01, 0, 0, 0, 0, 0, 0];
    bus.send(CanFrame {
        arbitration_id: u32::from(0x8000u16 | motor_id),
        data: payload,
        dlc: payload.len() as u8,
        is_extended: true,
        is_rx: false,
    })?;
    Ok(())
}

fn decode_ht_scan_reply(frame: CanFrame) -> Option<HtScanHit> {
    if frame.dlc < 8 || frame.data[0] != 0x27 || frame.data[1] != 0x01 {
        return None;
    }
    // 电机把自身 id 放在回复 ID 高字节(标准/扩展帧皆然,v2.0.0 固件
    // motor.c::motor_process_state_all:`id = Identifier >> 8`),回复 dest=0。
    if (frame.arbitration_id & 0x00FF) != 0 {
        return None;
    }
    let motor_id = ((frame.arbitration_id >> 8) & 0x7F) as u16;
    Some(HtScanHit {
        motor_id,
        pos_raw: i16::from_le_bytes([frame.data[2], frame.data[3]]),
        vel_raw: i16::from_le_bytes([frame.data[4], frame.data[5]]),
        tqe_raw: i16::from_le_bytes([frame.data[6], frame.data[7]]),
    })
}

fn wait_ht_scan_reply(
    bus: &dyn CanBus,
    motor_id: u16,
    timeout: Duration,
) -> Result<Option<HtScanHit>, Box<dyn std::error::Error>> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        let left = deadline.saturating_duration_since(Instant::now());
        if let Some(frame) = bus.recv(left.min(Duration::from_millis(20)))? {
            if let Some(hit) = decode_ht_scan_reply(frame) {
                if hit.motor_id == motor_id {
                    return Ok(Some(hit));
                }
            }
        }
    }
    Ok(None)
}

fn print_scan_hit(prefix: &str, s: HtScanHit) {
    println!(
        "{} id={} pos_raw={} vel_raw={} tqe_raw={} pos_turn={:+.4} vel_rps={:+.4}",
        prefix,
        s.motor_id,
        s.pos_raw,
        s.vel_raw,
        s.tqe_raw,
        s.pos_raw as f32 * 0.0001,
        s.vel_raw as f32 * 0.00025
    );
}

/// 收敛(2026-09-29):read/ping 打印 vendor 解码后的物理量(带力矩系数
/// 补偿与 status/fault/温度),取代旧手写 raw + 自行换算。
fn print_state(prefix: &str, s: &HightorqueFeedbackState) {
    println!(
        "{} id={} arb_id=0x{:X} status={} fault={} pos={:+.4}rad vel={:+.4}rad/s torq={:+.3}Nm t_mos={:.1}C t_rotor={:.1}C",
        prefix,
        s.can_id,
        s.arbitration_id,
        s.status_code,
        s.fault_code,
        s.pos,
        s.vel,
        s.torq,
        s.t_mos,
        s.t_rotor
    );
}

/// raw int16 刻度 → 物理量(pos: 1 raw = 0.0001 圈;vel: 1 raw = 0.00025 圈/s)。
fn pos_rad_from_args(args: &HashMap<String, String>) -> Result<f32, String> {
    if args.contains_key("raw-pos") {
        let raw = get_i16(args, "raw-pos", 0)?;
        return Ok(raw as f32 * 0.0001 * TWO_PI);
    }
    if args.contains_key("pos-deg") {
        let deg = get_f32(args, "pos-deg", 0.0)?;
        return Ok(deg.to_radians());
    }
    get_f32(args, "pos", 0.0)
}

fn vel_rad_s_from_args(args: &HashMap<String, String>) -> Result<f32, String> {
    if args.contains_key("raw-vel") {
        let raw = get_i16(args, "raw-vel", 0)?;
        return Ok(raw as f32 * 0.00025 * TWO_PI);
    }
    if args.contains_key("vel-deg-s") {
        let deg_s = get_f32(args, "vel-deg-s", 0.0)?;
        return Ok(deg_s.to_radians());
    }
    get_f32(args, "vel", 0.0)
}

fn tau_nm_from_args(args: &HashMap<String, String>) -> Result<Option<f32>, String> {
    if args.contains_key("raw-tqe") {
        return Err(
            "raw-tqe is no longer accepted here: torque is physical Nm (use tau); encoding is vendor-side with model torque coeff".to_string(),
        );
    }
    if args.contains_key("tau") {
        return Ok(Some(get_f32(args, "tau", 0.0)?));
    }
    Ok(None)
}

/// Open the CAN bus for the requested transport. Routes universal transports
/// (socketcan / mcu-serial) through `open_transport` so the platform driver
/// construction lives in one place (core). HighTorque uses standard CAN only,
/// so socketcanfd is rejected (it is not a CAN-FD device here); damiao-only
/// transports are rejected.
fn open_hightorque_bus(
    transport: &str,
    channel: &str,
    serial_port: &str,
    serial_baud: u32,
) -> Result<Arc<dyn CanBus>, Box<dyn std::error::Error>> {
    let p = TransportParams {
        channel,
        serial_port,
        serial_baud,
    };
    let bus: Arc<dyn CanBus> = match transport {
        "auto" | "socketcan" => open_transport(Transport::SocketCan, &p)?,
        "mcu-serial" => open_transport(Transport::McuSerial, &p)?,
        "socketcanfd" => {
            return Err(
                "transport socketcanfd unsupported (hightorque uses standard CAN only)".into(),
            )
        }
        "dm-serial" | "dm-device" => {
            return Err(format!(
                "transport {transport} is damiao-only (hightorque supports auto|socketcan|mcu-serial)"
            )
            .into());
        }
        _ => {
            return Err(format!(
                "unknown HighTorque transport: {transport} (expected auto|socketcan|mcu-serial)"
            )
            .into());
        }
    };
    Ok(bus)
}

/// vendor 无对应命令的裸诊断帧(开环力矩 05 13 / 电压 01-08 / 电流 01-09 /
/// 刹车 01-0F / 05 B4 定时上报配置):只发送不接收,直接走总线副本,
/// 与 vendor 收帧线程互不干扰。
fn send_diag_ext(
    bus: &dyn CanBus,
    motor_id: u16,
    payload: &[u8],
) -> Result<(), Box<dyn std::error::Error>> {
    let mut data = [0u8; 8];
    data[..payload.len()].copy_from_slice(payload);
    bus.send(CanFrame {
        arbitration_id: u32::from(0x8000u16 | motor_id),
        data,
        dlc: payload.len() as u8,
        is_extended: true,
        is_rx: false,
    })?;
    Ok(())
}

pub fn run_hightorque(
    args: &HashMap<String, String>,
    channel: &str,
    model: &str,
    motor_id: u16,
    feedback_id: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let mode = get_str(args, "mode", "ping");
    let loop_n = get_u64(args, "loop", 1)?;
    let dt_ms = get_u64(args, "dt-ms", 20)?;
    let transport = get_str(args, "transport", "auto");
    let serial_port = get_str(args, "serial-port", "/dev/ttyACM0");
    let serial_baud_u64 = get_u64(args, "serial-baud", 921600)?;
    let serial_baud = u32::try_from(serial_baud_u64)
        .map_err(|_| format!("invalid --serial-baud (too large): {serial_baud_u64}"))?;
    let bus = open_hightorque_bus(&transport, channel, &serial_port, serial_baud)?;

    if mode == "scan" {
        let start_id = get_u16_hex_or_dec(args, "start-id", 1)?.clamp(1, 127);
        let end_id = get_u16_hex_or_dec(args, "end-id", 32)?.clamp(1, 127);
        if start_id > end_id {
            return Err("invalid scan range after clamp (start-id > end-id)".into());
        }
        println!(
            "[scan] probing hightorque IDs {}..{} on {} by 0x17/0x01",
            start_id, end_id, channel
        );
        let mut hits = 0usize;
        for id in start_id..=end_id {
            send_ht_scan_query(bus.as_ref(), id)?;
            if let Some(s) = wait_ht_scan_reply(bus.as_ref(), id, Duration::from_millis(80))? {
                print_scan_hit("[hit]", s);
                hits += 1;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        println!("[scan] done vendor=hightorque hits={hits}");
        bus.shutdown()?;
        return Ok(());
    }

    // 收敛(2026-09-29):控制/状态/参数命令全部经 vendor crate
    // (HightorqueController 包住 CoreController + 后台收帧线程),
    // 电机经 add_motor 注册,--model 从此真正生效(型号决定力矩补偿系数,
    // 未知型号码会在 add_motor 处被拒)。裸诊断帧走总线副本。
    let diag_bus = Arc::clone(&bus);
    let ctrl = HightorqueController::new(bus);
    let motor = ctrl
        .add_motor(motor_id, feedback_id, model)
        .map_err(|e| format!("add motor failed: {e}"))?;

    let mut send_count = loop_n.max(1);
    if matches!(mode.as_str(), "ping" | "read") {
        send_count = 1;
    }

    for i in 0..send_count {
        match mode.as_str() {
            "ping" | "read" => {
                motor
                    .request_motor_feedback(Duration::from_millis(500))
                    .map_err(|e| e.to_string())?;
                match motor.latest_state() {
                    Some(s) => print_state("[ok]", &s),
                    None => {
                        return Err(format!(
                            "hightorque {} timeout on id={} (request cmd=0x17,0x01)",
                            mode, motor_id
                        )
                        .into());
                    }
                }
            }
            "pos" => {
                // 普通位置模式 07 07:vendor send_cmd_pos_classic,力矩为
                // 物理量 Nm(型号自适应补偿),缺省无限制(0x8000)。
                let pos = pos_rad_from_args(args)?;
                let tqe = tau_nm_from_args(args)?;
                println!(
                    "[tx] mode=pos id={} pos={:+.4}rad tqe_limit={:?}",
                    motor_id, pos, tqe
                );
                motor
                    .send_cmd_pos_classic(pos, tqe)
                    .map_err(|e| e.to_string())?;
            }
            "vel" => {
                let vel = vel_rad_s_from_args(args)?;
                println!("[tx] mode=vel id={} vel={:+.4}rad/s", motor_id, vel);
                motor.send_cmd_vel(vel).map_err(|e| e.to_string())?;
            }
            "tqe" => {
                let tqe = get_i16(args, "raw-tqe", 0)?;
                println!("[tx] mode=tqe id={} raw-tqe={}", motor_id, tqe);
                let mut data = [0x05, 0x13, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00];
                data[2..4].copy_from_slice(&tqe.to_le_bytes());
                // 旧实帧即 4 字节(0x05 0x13 + tqe int16),后 4 字节不发。
                send_diag_ext(diag_bus.as_ref(), motor_id, &data[..4])?;
            }
            "mit" => {
                // v2.0.0 MIT(电机固件 v4.6.0+):vendor send_cmd_mit,位打包
                // pos(16)/vel(12)/tqe(12)/kp(12)/kd(12),量程内饱和。
                let pos = pos_rad_from_args(args)?;
                let vel = vel_rad_s_from_args(args)?;
                let tau = get_f32(args, "tau", 0.0)?;
                let kp = get_f32(args, "kp", 0.0)?;
                let kd = get_f32(args, "kd", 0.0)?;
                println!(
                    "[tx] mode=mit id={} pos={:+.4}rad vel={:+.4}rad/s tau={:+.3}Nm kp={:.3} kd={:.3}",
                    motor_id, pos, vel, tau, kp, kd
                );
                motor
                    .send_cmd_mit(pos, vel, kp, kd, tau)
                    .map_err(|e| e.to_string())?;
            }
            "volt" => {
                let vol = get_i16(args, "raw-vol", 0)?;
                let mut data = [0x01, 0x00, 0x08, 0x05, 0x1B, 0x00, 0x00];
                data[5..7].copy_from_slice(&vol.to_le_bytes());
                send_diag_ext(diag_bus.as_ref(), motor_id, &data)?;
            }
            "cur" => {
                let cur = get_i16(args, "raw-cur", 0)?;
                let mut data = [0x01, 0x00, 0x09, 0x05, 0x1C, 0x00, 0x00];
                data[5..7].copy_from_slice(&cur.to_le_bytes());
                send_diag_ext(diag_bus.as_ref(), motor_id, &data)?;
            }
            "pos-vel-tqe" => {
                // 协同模式 07 35:vendor send_cmd_pos_vel(速度上限换算),
                // 力矩由 vendor 固定为无限制(0x8000),raw-tqe 不再接受。
                let pos = pos_rad_from_args(args)?;
                let vel = vel_rad_s_from_args(args)?;
                println!(
                    "[tx] mode=pos-vel-tqe id={} pos={:+.4}rad vel={:+.4}rad/s (torque unlimited)",
                    motor_id, pos, vel
                );
                motor
                    .send_cmd_pos_vel(pos, vel)
                    .map_err(|e| e.to_string())?;
            }
            "stop" => {
                motor.disable().map_err(|e| e.to_string())?;
            }
            "brake" => {
                send_diag_ext(diag_bus.as_ref(), motor_id, &[0x01, 0x00, 0x0F])?;
            }
            "conf-write" => {
                // 落盘到 flash(05 B3),vendor store_parameters。
                motor.store_parameters().map_err(|e| e.to_string())?;
            }
            "rezero" => {
                // 0x40 置零 + 自动落盘,vendor set_zero_position。
                motor.set_zero_position().map_err(|e| e.to_string())?;
            }
            "timed-read" => {
                let t_ms = get_i16(args, "period-ms", 100)?;
                let mut data = [0x05, 0xB4, 0x02, 0x00, 0x00];
                data[3..5].copy_from_slice(&t_ms.to_le_bytes());
                send_diag_ext(diag_bus.as_ref(), motor_id, &data)?;
            }
            _ => {
                return Err(format!(
                "unknown hightorque mode: {}. expected ping|scan|read|mit|pos|vel|tqe|volt|cur|pos-vel-tqe|stop|brake|rezero|conf-write|timed-read",
                mode
            )
            .into());
            }
        }
        if send_count > 1 {
            println!("[loop] #{i} sent mode={mode}");
        }
        if i + 1 < send_count {
            std::thread::sleep(Duration::from_millis(dt_ms));
        }
    }

    ctrl.close_bus()?;
    Ok(())
}
