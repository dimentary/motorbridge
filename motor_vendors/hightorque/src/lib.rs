pub mod controller;
pub mod motor;
pub mod protocol;
pub mod registers;

pub use controller::HightorqueController;
pub use motor::HightorqueMotor;
pub use protocol::{
    decode_fault, decode_unit_raw, encode_mit_frame, encode_unit_raw, from_turns, to_turns,
    FirmwareVersion, HightorqueFeedbackState, MotorModel, RunMode, TorqueCoeff, SETTING_ACK,
};
pub use protocol::{
    AngleUnit, DataType, QuantityScale, ACC_SCALE, CUR_SCALE, PID_SCALE, POS_SCALE, TQE_SCALE,
    VEL_SCALE, VOL_SCALE,
};
pub use protocol::{
    MIT_ID_PREFIX, MIT_KP_MAX, MIT_KP_MIN, MIT_KD_MAX, MIT_KD_MIN, MIT_POS_MAX, MIT_POS_MIN,
    MIT_TQE_MAX, MIT_TQE_MIN, MIT_VEL_MAX, MIT_VEL_MIN,
};
pub use protocol::{ReadCmd, RegisterType, RegisterValue};
pub use registers::{parameter_info, RegisterInfo, PARAMETER_TABLE};
