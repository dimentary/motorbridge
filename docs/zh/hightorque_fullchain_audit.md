# 高擎电机（HighTorque）四层接口全链路审计

> 审计日期：2026-09-28（同日完成 G3/G4 修复，见 §8）
> 审计范围：上位机 ws_gateway / Rust CLI motor_cli / Python bindings / C ABI motor_abi × vendor crate
> 基准：
> - vendor 层实现：`motor_vendors/hightorque/`（ht_can v2.0.0 迁移，commit f652d2b）
> - 协议参考固件：`can_h730_v2.0.0.zip` 内 `libelybot_can.c`（参考固件命令集）
> - 协议文档：01-CAN协议解析.pdf（§1.2–§1.8）、表1-寄存器功能、表2-运行模式、表3-报错代码
> - 协议整理稿：[hightorque_can_protocol.md](hightorque_can_protocol.md)（仿 RobStride 手册体例的全命令帧格式与 CAN 帧示例）

---

## 1. 总体结论：只有一条链路真正打通

```
上位机 ws_gateway ────手写裸CAN帧────> CAN 总线        ⚠️ MIT 编码已复用 vendor（其余仍手写）
Rust CLI motor_cli ───手写裸CAN帧────> CAN 总线        ⚠️ 同上（MIT 已切 v2.0.0 位打包帧）
Python bindings ──ctypes──> motor_abi(C ABI) ──> motor_vendor_hightorque   ✅ 唯一真实链路
```

- ws_gateway 与 motor_cli 的 Cargo.toml 已引入 `motor_vendor_hightorque`，但仅复用 `encode_mit_frame`（MIT 位打包编码）；帧收发、状态解码仍是各自手写的第三、第四份实现（连同 vendor crate，`decode_read_reply` 共 3 份拷贝，P0 应答 ID 修复需三处各改一遍）。
- motor_abi 是唯一调用 vendor crate 的层；Python 经 ctypes 调 ABI，故 Python → ABI → vendor 是唯一打通的链路，且只覆盖 vendor 能力的子集。
- 四层协议世代：vendor = v2.0.0；ABI/Python = v2.0.0 子集；CLI 与 ws_gateway 的 MIT 已切 v2.0.0 位打包帧（复用 vendor 编码），其余模式仍为 v1.5.5 手写。
- commit f652d2b 提交信息声称给 CLI 增加了 v2.0.0 模式，实际只改 15 行（应答 ID 高字节修复 + 字符串重标注），未新增任何 v2.0.0 模式。

---

## 2. 全链路能力矩阵（含缺失标注）

以参考固件命令集（libelybot_can.c）+ 协议文档为基准。图例：✅ 已实现并路由｜⚠️ 部分实现/语义偏差｜❌ 未实现｜`手写` = 不经过 vendor crate 的独立实现。

| # | 功能 | 线上帧 | vendor | C ABI | Python | CLI | ws_gateway |
|---|---|---|---|---|---|---|---|
| 1 | 停止 / 清错（模式 0） | `01 00 00` | ✅ disable | ✅（stop=disable 别名） | ✅ | ✅手写 | ✅手写 |
| 2 | MIT 位打包（v4.6.0+） | `0x18000\|id` | ✅ | ✅ | ✅ | ✅ [已修复] 复用 vendor encode_mit_frame | ✅ [已修复] kp/kd 打包进帧 |
| 3 | 协同控制（pos+vel+tqe） | `07 35` | ✅ | ✅ | ✅ | ✅手写（pos-vel-tqe 入口） | ❌ [G4 后] pos_vel 被拒，`07 35` 不再发送（mit 已改发真 MIT 帧） |
| 4 | 梯形控制（v4.6.0+） | `07 2d` | ✅ | ❌ | ❌ | ❌ | ❌ |
| 5 | 普通模式-位置+力矩 | `07 07` | ✅ | ❌ | ❌ | ✅手写（扩展帧） | ❌ |
| 6 | 普通模式-速度+力矩 | `07 07` | ✅ | ✅ | ✅ | ✅手写（扩展帧） | ✅手写（力矩字段置 0） |
| 7 | 力矩模式 | `05 13`（写寄存器 0x13） | ❌ | ❌ | ❌ | ✅手写 | ❌ |
| 8 | DQ 电压（模式 8+写 0x1b） | `01 00 08` + `05 1b` | ❌ | ❌ | ❌ | ✅手写 | ❌ |
| 9 | DQ 电流（模式 9+写 0x1c） | `01 00 09` + `05 1c` | ❌ | ❌ | ❌ | ✅手写 | ❌ |
| 10 | 刹车（模式 15） | `01 00 0F` | ❌ | ❌ | ❌ | ✅手写 | ❌ |
| 11 | 运行模式切换（写 0x000） | `01 00 <mode>` | ⚠️ 空校验不发帧 | ⚠️ 透传 | ⚠️ | ❌ | ❌（直接拒绝） |
| 12 | 通用寄存器读（0x17 族） | `17 <cmd/addr>` | ✅ read_registers | ❌（Damiao-only 报错） | ❌ | ❌ 仅固定 `17 01` | ❌ |
| 13 | 读状态（= #12 特例） | `17 01` | ✅ request_motor_feedback | ✅（500ms 硬编码） | ✅ | ✅手写 | ✅手写 |
| 14 | 通用寄存器写 | `05 <reg> <val>` | ❌ 无 encode_write | ❌ | ❌ | ⚠️ 仅硬编码 volt/cur/tqe | ❌ |
| 15 | 周期上报（0x05 B4） | `05 B4 02 <t>` | ❌ | ❌ | ❌ | ✅手写 | ❌ |
| 16 | 零位设置 | `40 01 04 64 20 63 0A` | ✅ | ✅ | ✅ | ✅手写 | ❌（直接拒绝） |
| 17 | 参数保存 conf_write | `05 B3 02 00 00` | ✅ | ✅ | ✅ | ✅手写 | ❌（直接拒绝） |
| 18 | 固件版本查询 | `15 B5 02` | ✅ | ❌（无 C 函数） | ❌ | ❌ | ❌（version 为 myactuator-only） |
| 19 | 故障码解码（0–47，表3） | 状态帧 byte1 | ✅ decode_fault | ❌ fault 字段被丢弃 | ❌ | ❌ | ❌ status_code 恒 0 |
| 20 | 运行模式显示（0–17，表2） | 状态帧 byte0 | ✅ RunMode | ⚠️ 原始字节进 status_code | ❌ | ❌ | ❌ |
| 21 | 力矩型号补偿（P2-15） | — | ⚠️ 未修正 | — | — | ❌ tau×100 平移 | ❌ 固定 ÷100 |
| 22 | ACK 确认机制 | `41 01 04 OK\r\n` | ✅ send_with_ack | ❌ | ❌ | ❌ | ❌ |
| 23 | enable | 协议无此命令 | Err（协议忠实） | 透传 Err | 抛异常 | 拒绝 | ⚠️ 假成功 no-op |
| 24 | clear_error | ≡ 停止帧（模式 0） | ✅ [已修复] =send_stop 同帧 | ✅ 透传 | ✅ | —（stop 模式同帧，无独立入口） | ✅ [已修复] 发 `01 00 00` |

---

## 3. 缺失分三类

### A 类：vendor 层缺失（任何上层不可能有）

按**实现原语**计数（非功能行数）：

| 原语 | 影响 | 说明 |
|---|---|---|
| 寄存器写 encode_write | #7 力矩、#8 DQ电压、#9 DQ电流（`05 <reg> <val>` 一族多用途） | 表1：0x1b=Q电压 R/W、0x1c=Q电流 R/W、0x13=力矩指令 |
| 真实的模式切换帧 | #8/#9/#10/#11 | `01 00 <mode>` 写寄存器 0x000；当前 ensure_control_mode 只校验 0–17 不发帧 |
| 周期上报 | #15 | `05 B4 02 + t_ms`，应答格式与 0x17 读相同 |
| 力矩型号补偿 | #21 | P2-15 标注“当前未修正” |

功能缺口仍为 7 项（力矩/DQ电压/DQ电流/刹车/模式切换/周期上报/补偿），但实现工作量收敛为上表 4 个原语；补 2 个原语（寄存器写+模式切换）即点亮 4 个功能行。

### B 类：vendor 已实现、ABI 未暴露（Python 随之缺失）

1. 梯形模式 `send_cmd_pos_vel_acc`（#4）— ABI 无函数；协议文档自己都推荐用梯形替代普通位置模式（§2.3 注意）
2. `send_cmd_pos_classic`（#5）
3. `read_registers` + PARAMETER_TABLE 0x00–0x45（#12）— `get_register_*` 对 hightorque 报 Damiao-only
4. `request_firmware_version`（#18）— ABI 无对应 C 函数
5. `send_with_ack`（#22）
6. `torque_coeff`
7. fault_code（#19）：`MotorState` 无 fault 字段，`HightorqueFeedbackState.fault_code`（protocol.rs）被静默丢弃；RunMode（#20）不解码
8. 幽灵符号：.so 导出 10 个 `motor_handle_hightorque_{get,write}_param_*`，全是 "not supported yet" stub，头文件未声明、Python 未绑定，但 capabilities JSON 宣称支持 `param_i8…f32`（违反仓库“不支持的能力不得返回成功”规则）

### C 类：上层手写实现与 vendor 分叉（正确性风险）

1. **帧格式分歧（潜在 bug）**：CLI 与 ws_gateway 把控制命令发成扩展帧 `0x8000|id`；vendor（与参考固件一致）控制类用标准 11 位裸 id，仅查询/配置类用 `0x8000|id`（`motor.rs:283-295`）
2. ~~CLI `mit` 模式实际发 `07 35`（等于 vendor 私有 send_cmd_pos_vel_tqe），非 v2.0.0 MIT 帧；`--kp/--kd` 打印但从不打包~~ **[已修复]**：`mit` 改发 `0x18000|id` 位打包帧，编码复用 vendor `encode_mit_frame`，kp/kd 打包进帧；原 `pos-vel-tqe` 入口保留（协同 `07 35` 是 v2.0.0 合法模式，两入口不再同帧）
3. ~~ws_gateway `mit` 发 `07 35` 且用 `..` 解构丢 kp/kd（`control.rs:96-109`）→ 上位机走 MIT 无刚度/阻尼~~ **[已修复]**：新增 `send_hightorque_mit`（control.rs 处理器 + runtime.rs 连续 tick 都走它），kp/kd 打包；旧路径的 `pos_raw_from_rad`/`tqe_raw_from_tau` 死代码已删
4. ws_gateway `enable`/`disable` 假成功 no-op（连 stop 帧都不发）；`capabilities` 声称 pos_vel/force_pos 但 handler 拒绝 → 能力驱动 UI 出现必报错按钮
5. ws_gateway `vel` 力矩字段写 0（vendor 写 0x8000 无限制）
6. `set_zero_position`/`store_parameters` ABI/Python 通，ws_gateway 直接拒绝
7. 文档漂移：README（中/英）称 force_pos 映射 pos+vel+tqe（实际报错）、~~称 MIT 忽略 kp/kd（vendor 实际已位打包）~~ **[已修复]**（README/PROTOCOL/Python README/CLI help 的 kp/kd 与 clear_error 表述已更新）；PROTOCOL.zh-CN.md 是准确的一份
8. CLI `--model`/`--feedback-id` 对 hightorque 是死参数；scan 强制 send_count=1 忽略 `--loop`
9. ~~CLI/mit 与 pos-vel-tqe 字节级相同~~ **[已修复]**：mit 走 `0x18000|id`，pos-vel-tqe 走 `07 35`，两入口分道；robstride_cli.rs 内 5 处 hightorque compat 分支为死代码

---

## 4. 重复能力接口（协议层归并）

用“功能 / 命令 / 原子”三层定性。**真重复只有 2 处；其余是命令原子复用或特例关系。**

| 组 | 矩阵行 | 命令层 | 功能层 | 定性 |
|---|---|---|---|---|
| G3 | 停止 ≡ clear_error | 同一帧 `01 00 00` | 同一行为（表2 模式 0 名称即“停止，清除错误”） | **真重复**：一个命令一个动作，两个 API 名；clear_error 是伪缺口。**[已修复]**：vendor `clear_error()`=`send_stop()`（带单测），ABI/Python 透传自动生效，gateway clear_error 分支发 `01 00 00` |
| G4a | CLI `mit` ≡ CLI `pos-vel-tqe` | 同一帧、字节级相同 | 同一行为（协同控制） | **真重复**：一份代码两个入口名。**[已修复]**：mit 改走 v2.0.0 `0x18000\|id` 位打包帧（kp/kd 生效），两入口分道 |
| G4b | CLI/gateway 的"MIT" vs 真 MIT | ~~`07 35` vs `0x18000\|id`~~ | ~~协同 ≠ MIT（有无 kp/kd）~~ | **名不副实**，非重复。**[已修复]**：两层的 mit 均改发真 MIT 帧 |
| G1 | #5 位置+力矩 / #6 速度+力矩 | 同一 `07 07` 帧 | 不同（靠 0x8000 填充区分） | **原子复用**（协议 §1.2.1“位置和速度不能同时控制”） |
| G5 | #7 力矩 / #8 DQ电压 / #9 DQ电流 / #10 刹车 / #11 模式切换 | 共享“写模式寄存器+写目标寄存器”两原语 | **不同**（各自独立运行模式，向上暴露的功能接口） | **原子复用**，功能不重复 |
| G2 | #13 读状态 / #12 通用寄存器读 | 同一 0x17 命令族 | 特例 vs 泛化 | **特例关系**：`17 01` = `read_registers(&[addr:1,int16,count:3}])` |

### G2 特例的意义与问题

线上无独立意义；价值全在包装层：① 物理量换算+torque_coeff 补偿（decode_read_reply 带 coeff）；② 写 state 缓存（`motor.rs:322-324`）对接 `latest_state`/MotorDevice 被动生态（周期上报、ACK 序号）；③ 同步语义（ABI request_feedback / gateway state_once）。

**问题**：特例未建在通用读之上，而是平行实现——两个几乎相同的等待循环（`motor.rs:253-264` vs `315-334`）、两个解码器（decode_register_reply vs decode_read_reply）、硬编码帧而非调 encode_read。正确结构应为 `request_motor_feedback = read_registers(...) + 换算 + 写缓存`。

### 协议层归并后的真实命令集（~11 条）

`07 07`（普通）｜`07 35`（协同）｜`07 2d`（梯形）｜`0x18000|id`（MIT）｜`17 <cmd/addr>`（寄存器读）｜`01 00 <mode>`（模式写）｜`05 <reg>`（寄存器写）｜`05 B3`（保存）｜`05 B4`（周期上报）｜`40 01 …`（零位）｜`15 B5`（版本）

矩阵 24 行按此归并后：**功能行数不虚（每行独立功能），虚的是把功能缺口数当工作量**。

---

## 5. 其它厂商上位机数据解读模式（对照）

`build_state_snapshot`（`runtime.rs:156-298`）五分支对比：

| | Damiao | RobStride | MyActuator | Hexfellow | **HighTorque** |
|---|---|---|---|---|---|
| 链 vendor crate | ✅ | ✅ | ✅ | ✅ | ❌ |
| Motor 句柄 | vendor 对象 | vendor 对象 | vendor 对象 | vendor 对象 | 裸 `u16 motor_id` |
| 状态数据 | 物理量+status_name | 物理量+12 故障布尔位 | 物理量（deg） | 物理量（rev/permille） | raw 三元组 |
| gateway 剩余换算 | 无 | mode→字符串 | deg→rad | rev→rad、permille→Nm | **全套 scale 自算、力矩 flat ÷100、status 恒 0** |

寄存器/参数模式：帧编解码+verify 在 vendor，地址语义由客户端传参（Damiao rid 直通 `get_register_f32`，RobStride param_id 直通类型化读写）；唯一 gateway 侧表知识是 Damiao scan 硬编码 rid 21/22/23，但型号识别仍调 vendor `match_models_by_limits`。

**HighTorque 是五家中唯一“上位机内置换算逻辑”的，且已做错**（无 fault、无 run_mode、无力矩系数）。

---

## 6. 修复优先级建议

1. **vendor 层补 2 原语**：encode_write（寄存器写）+ 真实模式切换帧 → 解锁力矩/DQ电压/DQ电流/刹车 4 行；~~顺手把 clear_error 接到停止帧（消除 G3 伪缺口）~~ **[已做，见 §8]**
2. **ABI 补暴露**：梯形模式、read_registers（照 Damiao rid 直通模式）、固件版本、fault_code 进 MotorState → Python 自动受益；删除或实现 10 个幽灵 param 符号并对齐 capabilities JSON
3. **ws_gateway 迁移**：照抄其它四家模式——`MotorHandle::Hightorque` 换 `Arc<HightorqueMotor>`，状态走 latest_state，删 hightorque_ws.rs 129 行手写代码；修正 enable/disable 假成功与 capabilities 虚报
4. **CLI 迁移**：改为调用 vendor crate（Cargo.toml 加依赖，**[已加，当前仅复用 MIT 编码]**），消除帧格式分歧；~~`mit` 改真 MIT 帧或改名为协同模式~~ **[已改真 MIT 帧]**
5. **文档对齐**：README 的 force_pos/kp-kd 表述、CLI 的 "v1.5.5-compat" 标签、demo 双写 flash（set_zero_position 内部已调 store）
6. **测试**：CLI/ws_gateway 的 hightorque 路径当前零测试

---

## 7. 证据索引（关键 file:line）

- vendor 能力：`motor_vendors/hightorque/src/motor.rs`（enable:52 / clear_error:65=send_stop / ensure_control_mode:82 / mit:100 / 梯形:120 / pos_vel 协同:136 / pos_classic:149 / vel:166 / force_pos:174 / store:186 / send_with_ack:196 / version:214 / feedback:234 / read_registers:243 / wait_status:316 / send_control vs send_query:287-294）、`protocol.rs`（encode_mit_frame:574，已 pub 供上层复用）
- ABI 路由：`motor_abi/src/motor_control_ffi.rs`（hightorque 分支 33/88/145/201/229/248/269/309）、`motor_register_ffi.rs`（Damiao-only 报错 43/61/80/103）、`vendor_params/hightorque.rs`（全 stub）、`lib.rs:45`（capabilities 虚报）
- CLI 手写：`motor_cli/src/hightorque_cli.rs`（decode_read_reply 72 / send_ext_with_id 52 / mit 位打包帧 325 / 模式分发）、`Cargo.toml`（已加 motor_vendor_hightorque 依赖）
- ws_gateway 手写：`integrations/ws_gateway/src/vendors/hightorque_ws.rs`（scale 17-23 / send_hightorque_mit 65）、`router/handlers/control.rs`（mit 96-108 / pos_vel 拒绝 210 / vel 266）、`router/handlers/register.rs`（clear_error=停止帧 52-56 / 其余拒绝 90/133/233）、`session/runtime.rs`（mit tick 复用 send_hightorque_mit / status_code 恒 0 215）
- Python：`bindings/python/src/motorbridge/abi.py`（未绑定 10 个 param 符号）、`examples/hightorque_all_interfaces_demo.py`（仅测位置模式）
- 协议事实：表1（0x000 模式 R/W、0x1b Q电压、0x1c Q电流）、表2（模式 0=停止，清除错误、8=DQ电压、9=DQ电流、15=刹车）、协议 PDF §1.2.1（07 07 一帧两用）、§1.3.3（0x17 逐位=通用读）、参考固件 libelybot_can.c（DQ 命令=模式切换+寄存器写组合帧）

---

## 8. 修复记录（G3/G4，2026-09-28）

原则：以 ht_can v2.0.0 协议为准，去掉旧协议路径；编码只留 vendor 一份，上层复用而非抄写。

### G3：clear_error ≡ 停止帧（模式 0）

| 层 | 变更 | 文件 |
|---|---|---|
| vendor | `clear_error()` 由返回 `Err` 改为 `send_stop()`（同帧 `01 00 00`），新增单测 `clear_error_sends_stop_frame` | `motor_vendors/hightorque/src/motor.rs:65` |
| ABI / Python | 无代码变更：透传 vendor 结果，Err→Ok 自动生效 | — |
| ws_gateway | `clear_error` 的 Hightorque 分支由“直接拒绝”改为发 `01 00 00`（与 stop op 同帧） | `router/handlers/register.rs:52-56` |

> **G3 协议证据链（`01 00 <mode>` = 写模式寄存器 0x000）**：表1 寄存器 `0x000 模式 R/W`；表2 模式 0 名称即“停止，清除错误”（8=DQ电压、9=DQ电流、15=刹车）。参考库帧族第三字节恰为表2 模式号（停止 `01 00 00`、刹车 `01 00 0F`），DQ 例程一帧拼“模式写+寄存器写”`{01 00 08, 05 1b <volt>}`，证明 `01 00` 即模式写入专用命令（非通用寄存器写族 `05 <reg> <val>`）；状态帧 byte0 持续回读当前模式（持久状态量）。注意：协议 PDF §1.4 示例写 `0x8000|id` 扩展帧，与 v2.0.0 参考库（控制类裸 id 标准帧）矛盾，以参考库为准（vendor `send_control` 一致）。电机端固件源码不在工作区，“固件内部写 0x000”为外部证据推断。

### G4：MIT 改真 v2.0.0 位打包帧（`0x18000|id`，固件 v4.6.0+）

| 层 | 变更 | 文件 |
|---|---|---|
| vendor | `encode_mit_frame` 由 `pub(crate)` 改 `pub` 并从 lib.rs 导出（CLI/gateway 复用，避免第 4 份拷贝） | `motor_vendors/hightorque/src/protocol.rs:574` |
| CLI | `mit` 模式由发 `07 35` 改为发位打包帧（pos 16/vel 12/tqe 12/kp 12/kd 12，超界饱和）；新增 `send_ext_with_id`（任意 29 位 ID）与 `pos-deg`/`vel-deg-s` 参数换算；删除 kp/kd 忽略说明；`pos-vel-tqe` 入口保留（`07 35` 协同为合法模式，不再与 mit 同帧） | `motor_cli/src/hightorque_cli.rs` |
| ws_gateway | 新增 `send_hightorque_mit`（编码调 vendor）；`mit` 处理器与 runtime 连续 tick 均改走它，kp/kd 打包进帧；删除旧路径死代码 `pos_raw_from_rad`/`tqe_raw_from_tau` | `vendors/hightorque_ws.rs:65`、`router/handlers/control.rs:96-108`、`session/runtime.rs` |

### 文档对齐

- ws_gateway：PROTOCOL.zh-CN.md（§9.1 MIT 表、§10.1 clear_error 适用范围）、README.md / README.zh-CN.md（厂商映射表、模式参数差异）
- Python：README.md（send_mit 注释、模式参数表）、`cli/main.py` help（去掉 "v1.5.5 忽略 kp/kd"）
- CLI：args.rs help、README.md（"v1.5.5-compat" 标签清除）
- demo：`hightorque_all_interfaces_demo.py`（clear_error 语义注释：清错即停机）

### 验证

`cargo test -p motor_vendor_hightorque`（74 通过，含新增 clear_error 单测）、`cargo test -p motor_cli`（8 通过）、`cargo test -p ws_gateway`（6 通过）、`cargo check` 无警告。真机 MIT 路径需固件 v4.6.0+。
