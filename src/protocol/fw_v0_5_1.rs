// Copyright The odrive-can-protocol Contributors

//! ODrive `fw-v0.5.1` CANSimple 协议。
//!
//! 编码仅产生 11 位标准经典 CAN 帧。解码会明确报告扩展 ID 与 CAN FD，避免把其他
//! 协议版本误判为此版本的消息。

use core::fmt;

use super::{EncodedFrame, FrameId, FramePayload, FrameRef};

const COMMAND_MASK: u16 = 0x1f;
const MAX_STANDARD_ID: u16 = 0x07ff;
const MAX_NODE_ID: u32 = 63;

const HEARTBEAT: u8 = 0x01;
const ESTOP: u8 = 0x02;
const MOTOR_ERROR: u8 = 0x03;
const ENCODER_ERROR: u8 = 0x04;
const SENSORLESS_ERROR: u8 = 0x05;
const SET_AXIS_NODE_ID: u8 = 0x06;
const SET_AXIS_REQUESTED_STATE: u8 = 0x07;
const SET_AXIS_STARTUP_CONFIG: u8 = 0x08;
const ENCODER_ESTIMATES: u8 = 0x09;
const ENCODER_COUNT: u8 = 0x0a;
const SET_CONTROLLER_MODES: u8 = 0x0b;
const SET_INPUT_POS: u8 = 0x0c;
const SET_INPUT_VEL: u8 = 0x0d;
const SET_INPUT_TORQUE: u8 = 0x0e;
const SET_VELOCITY_LIMIT: u8 = 0x0f;
const START_ANTICOGGING: u8 = 0x10;
const SET_TRAJ_VEL_LIMIT: u8 = 0x11;
const SET_TRAJ_ACCEL_LIMITS: u8 = 0x12;
const SET_TRAJ_INERTIA: u8 = 0x13;
const IQ: u8 = 0x14;
const SENSORLESS_ESTIMATES: u8 = 0x15;
const REBOOT: u8 = 0x16;
const VBUS_VOLTAGE: u8 = 0x17;
const CLEAR_ERRORS: u8 = 0x18;

/// 已验证的 ODrive CAN 节点号。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeId(u8);

impl NodeId {
    /// 从 `0..=63` 的节点号创建值。
    ///
    /// `63` 在此固件版本中是普通节点，不具有新版协议的广播语义。
    pub const fn new(value: u32) -> Result<Self, EncodeError> {
        if value <= MAX_NODE_ID {
            Ok(Self(value as u8))
        } else {
            Err(EncodeError::InvalidNodeId(value))
        }
    }

    /// 返回节点号。
    pub const fn get(self) -> u8 {
        self.0
    }
}

/// ODrive 轴状态的原始 `u32` 值。
///
/// 未知状态仍会被完整保留，以兼容设备返回的未来或诊断状态。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AxisState(pub u32);

impl AxisState {
    /// 未定义状态。
    pub const UNDEFINED: Self = Self(0);
    /// 空闲状态。
    pub const IDLE: Self = Self(1);
    /// 启动序列状态。
    pub const STARTUP_SEQUENCE: Self = Self(2);
    /// 完整校准序列状态。
    pub const FULL_CALIBRATION_SEQUENCE: Self = Self(3);
    /// 电机校准状态。
    pub const MOTOR_CALIBRATION: Self = Self(4);
    /// 无感控制状态。
    pub const SENSORLESS_CONTROL: Self = Self(5);
    /// 编码器索引搜索状态。
    pub const ENCODER_INDEX_SEARCH: Self = Self(6);
    /// 编码器偏移校准状态。
    pub const ENCODER_OFFSET_CALIBRATION: Self = Self(7);
    /// 闭环控制状态。
    pub const CLOSED_LOOP_CONTROL: Self = Self(8);
    /// 锁相旋转状态。
    pub const LOCKIN_SPIN: Self = Self(9);
    /// 编码器方向查找状态。
    pub const ENCODER_DIR_FIND: Self = Self(10);
    /// 回零状态。
    pub const HOMING: Self = Self(11);
}

/// 主机发送给轴的写命令。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Command {
    /// 请求轴进入急停故障状态。
    Estop,
    /// 设置轴的 CAN 节点号。
    SetAxisNodeId {
        /// 新节点号。
        node_id: NodeId,
    },
    /// 设置请求轴状态。
    ///
    /// 线上字段是完整 `u32`；官方 `fw-v0.5.1` 实现读取时只取其低 16 位。
    SetAxisRequestedState {
        /// 请求的状态原始值。
        state: AxisState,
    },
    /// 设置控制器控制模式与输入模式。
    SetControllerModes {
        /// 控制模式原始枚举值。
        control_mode: i32,
        /// 输入模式原始枚举值。
        input_mode: i32,
    },
    /// 设置位置与两个原始千分之一单位前馈值。
    SetInputPos {
        /// 位置设定值，单位为转（turn）。
        position: f32,
        /// 速度前馈的原始 `i16` 值，每个计数为 `0.001 turn/s`。
        velocity_ff: i16,
        /// 转矩前馈的原始 `i16` 值，每个计数为 `0.001 N·m`。
        torque_ff: i16,
    },
    /// 设置速度与转矩前馈。
    SetInputVel {
        /// 速度设定值，单位为 `turn/s`。
        velocity: f32,
        /// 转矩前馈值，单位为 `N·m`。
        torque_ff: f32,
    },
    /// 设置转矩。
    SetInputTorque {
        /// 转矩设定值，单位为 `N·m`。
        torque: f32,
    },
    /// 设置速度限制。
    SetVelocityLimit {
        /// 速度限制值，单位为 `turn/s`。
        velocity: f32,
    },
    /// 启动抗齿槽校准。
    StartAnticogging,
    /// 设置轨迹速度限制。
    SetTrajVelLimit {
        /// 轨迹速度限制值，单位为 `turn/s`。
        velocity: f32,
    },
    /// 设置轨迹加速度与减速度限制。
    SetTrajAccelLimits {
        /// 轨迹加速度限制值，单位为 `turn/s²`。
        acceleration: f32,
        /// 轨迹减速度限制值，单位为 `turn/s²`。
        deceleration: f32,
    },
    /// 设置轨迹惯量。
    SetTrajInertia {
        /// 轨迹惯量配置值，单位为 `N·m/(turn/s²)`。
        inertia: f32,
    },
    /// 让设备复位。
    Reboot,
    /// 清除设备错误。
    ClearErrors,
}

/// 主机用 RTR 发起的读取请求。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Query {
    /// 读取电机错误位图。
    MotorError,
    /// 读取编码器错误位图。
    EncoderError,
    /// 读取无感估算器错误位图。
    SensorlessError,
    /// 读取编码器位置和速度估算。
    EncoderEstimates,
    /// 读取编码器计数。
    EncoderCount,
    /// 读取 q 轴电流。
    Iq,
    /// 读取无感位置和速度估算。
    SensorlessEstimates,
    /// 读取母线电压。
    VbusVoltage,
}

/// 轴发送的遥测或 RTR 响应。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Response {
    /// 周期性心跳，包含轴错误位图和当前状态。
    Heartbeat {
        /// 轴错误位图；未知位原样保留。
        axis_error: u32,
        /// 当前轴状态；未知值原样保留。
        axis_state: AxisState,
    },
    /// 电机错误位图。
    MotorError(u32),
    /// 编码器错误位图。
    EncoderError(u32),
    /// 无感估算器错误位图。
    SensorlessError(u32),
    /// 编码器位置与速度估算。
    EncoderEstimates {
        /// 位置估算，单位为转（turn）。
        position: f32,
        /// 速度估算，单位为 `turn/s`。
        velocity: f32,
    },
    /// 编码器计数。
    EncoderCount {
        /// Shadow count。
        shadow_count: i32,
        /// 当前圈内的编码器计数。
        count_in_cpr: i32,
    },
    /// q 轴电流设定与测量值。
    Iq {
        /// q 轴电流设定值，单位为安培（A）。
        setpoint: f32,
        /// q 轴电流测量值，单位为安培（A）。
        measured: f32,
    },
    /// 无感估算的位置与速度。
    SensorlessEstimates {
        /// PLL 位置，单位为弧度（rad）。
        position: f32,
        /// 估算速度，单位为 `turn/s`。
        velocity: f32,
    },
    /// 母线电压，单位为伏特（V）。
    VbusVoltage(f32),
}

/// 可以出现在线上的任一 CANSimple 消息方向。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Message {
    /// 主机到轴的写命令。
    Command(Command),
    /// 主机到轴的 RTR 读取请求。
    Request(Query),
    /// 轴到主机的心跳或读取响应。
    Response(Response),
}

/// 无法将值表示为此协议帧时的错误。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncodeError {
    /// 节点号超出 `0..=63`。
    InvalidNodeId(u32),
    /// 写命令字段为 NaN 或无穷大。
    NonFinite {
        /// 不合法的字段名。
        field: &'static str,
    },
}

impl fmt::Display for EncodeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidNodeId(value) => write!(formatter, "invalid ODrive CAN node id: {value}"),
            Self::NonFinite { field } => write!(formatter, "non-finite CAN command field: {field}"),
        }
    }
}

impl core::error::Error for EncodeError {}

/// 无法把输入帧解释为此协议消息时的错误。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// 标准 ID 超出 11 位范围。
    InvalidStandardId(u16),
    /// 本库首版不支持扩展 CAN ID。
    UnsupportedExtendedId,
    /// 本库首版不支持 CAN FD。
    UnsupportedCanFd,
    /// 节点相关的命令号在官方固件中未实现。
    UnsupportedCommand {
        /// 未实现的 5 位命令号。
        command_id: u8,
    },
    /// 该命令号不接受当前帧种类。
    InvalidFrameKind {
        /// 5 位命令号。
        command_id: u8,
    },
    /// 数据帧长度不等于该消息的协议定义长度。
    InvalidLength {
        /// 5 位命令号。
        command_id: u8,
        /// 协议要求的字节数。
        expected: u8,
        /// 实际数据字节数。
        actual: usize,
    },
    /// RTR DLC 超出经典 CAN 的 `0..=8` 范围。
    InvalidRemoteDlc {
        /// 实际 RTR DLC。
        dlc: u8,
    },
    /// `SetAxisNodeId` 数据字段超出 `0..=63`。
    InvalidNodeId(u32),
}

impl fmt::Display for DecodeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidStandardId(id) => {
                write!(formatter, "standard CAN id exceeds 11 bits: {id}")
            }
            Self::UnsupportedExtendedId => formatter.write_str("extended CAN id is unsupported"),
            Self::UnsupportedCanFd => formatter.write_str("CAN FD is unsupported"),
            Self::UnsupportedCommand { command_id } => {
                write!(formatter, "unsupported CAN command: {command_id:#04x}")
            }
            Self::InvalidFrameKind { command_id } => write!(
                formatter,
                "invalid frame kind for CAN command: {command_id:#04x}"
            ),
            Self::InvalidLength {
                command_id,
                expected,
                actual,
            } => write!(
                formatter,
                "invalid length for CAN command {command_id:#04x}: expected {expected}, got {actual}"
            ),
            Self::InvalidRemoteDlc { dlc } => {
                write!(formatter, "invalid classic CAN RTR DLC: {dlc}")
            }
            Self::InvalidNodeId(node_id) => {
                write!(formatter, "invalid ODrive CAN node id in frame: {node_id}")
            }
        }
    }
}

impl core::error::Error for DecodeError {}

/// 将一个协议消息编码为经典标准 CAN 帧。
///
/// 14 个主机写命令按定义使用精确的 0、4 或 8 字节数据长度；其中所有 `f32` 字段
/// 必须是有限数。这个检查不裁决位置、速度、转矩或轨迹参数的物理范围。设备回复的
/// 浮点位模式（包括 NaN 与无穷）按原样编码，所有回复均为 8 字节；只定义前 4 字节的
/// 错误与母线电压回复会将后 4 字节清零。
///
/// RTR 查询总是编码为 DLC 8、不含数据的远程帧。轴状态请求完整编码其 `u32` 位模式，但官方
/// `fw-v0.5.1` 接收实现只读取低 16 位。成功仅表示消息已转换为协议帧：本函数不发送
/// CAN 帧，也不证明设备接收、执行或成功完成命令。
pub fn encode(node: NodeId, message: Message) -> Result<EncodedFrame, EncodeError> {
    let (command_id, data, len, rtr) = match message {
        Message::Command(command) => encode_command(command)?,
        Message::Request(query) => (query_id(query), [0; 8], 8, true),
        Message::Response(response) => encode_response(response),
    };
    let id = (u16::from(node.get()) << 5) | u16::from(command_id);
    Ok(EncodedFrame::new(id, data, len, rtr))
}

/// 将借用的 CAN 帧解码为节点相关的协议消息。
///
/// 此函数先校验标准 ID 是否为 11 位，并在节点过滤前拒绝所有扩展 ID。对于合法标准
/// ID，其他节点和本版本未知／保留命令返回 `Ok(None)`，且不检查其帧种类或长度；节点
/// 相关的 `0x08` 则返回 [`DecodeError::UnsupportedCommand`]，因为官方固件没有实现它。
/// 这个顺序不根据长度推测其他协议版本。
///
/// 对节点相关的已知消息，CAN FD 返回 [`DecodeError::UnsupportedCanFd`]。RTR 只允许 8 个
/// 查询命令，接受 DLC `0..=8` 且不含数据；其他 RTR 形态返回错误。数据帧中的写命令
/// 必须恰为定义的 0、4 或 8 字节，所有回复必须恰为 8 字节。错误和母线电压回复只解码
/// 前 4 字节并忽略尾部；`SetAxisNodeId` 的线上值超过 63 返回错误。
///
/// 解码如实保留线上非有限浮点、完整轴状态和未知错误位；设备错误非零仍是有效协议
/// 消息。成功仅表示帧已转换为消息，不发送 CAN 帧，也不证明设备状态或操作成功。
pub fn decode(node: NodeId, frame: FrameRef<'_>) -> Result<Option<Message>, DecodeError> {
    let id = match frame.id {
        FrameId::Standard(id) if id <= MAX_STANDARD_ID => id,
        FrameId::Standard(id) => return Err(DecodeError::InvalidStandardId(id)),
        FrameId::Extended(_) => return Err(DecodeError::UnsupportedExtendedId),
    };
    if u32::from(id >> 5) != u32::from(node.get()) {
        return Ok(None);
    }
    let command_id = (id & COMMAND_MASK) as u8;
    if command_id == SET_AXIS_STARTUP_CONFIG {
        return Err(DecodeError::UnsupportedCommand { command_id });
    }
    if !is_known(command_id) {
        return Ok(None);
    }

    match frame.payload {
        FramePayload::Fd(_) => Err(DecodeError::UnsupportedCanFd),
        FramePayload::Remote { dlc } => decode_remote(command_id, dlc),
        FramePayload::Data(data) => decode_data(command_id, data),
    }
}

fn encode_command(command: Command) -> Result<(u8, [u8; 8], u8, bool), EncodeError> {
    let mut data = [0; 8];
    let (command_id, len) = match command {
        Command::Estop => (ESTOP, 0),
        Command::SetAxisNodeId { node_id } => {
            put_u32(&mut data, 0, u32::from(node_id.get()));
            (SET_AXIS_NODE_ID, 4)
        }
        Command::SetAxisRequestedState { state } => {
            put_u32(&mut data, 0, state.0);
            (SET_AXIS_REQUESTED_STATE, 4)
        }
        Command::SetControllerModes {
            control_mode,
            input_mode,
        } => {
            put_u32(&mut data, 0, control_mode as u32);
            put_u32(&mut data, 4, input_mode as u32);
            (SET_CONTROLLER_MODES, 8)
        }
        Command::SetInputPos {
            position,
            velocity_ff,
            torque_ff,
        } => {
            finite(position, "position")?;
            put_f32(&mut data, 0, position);
            put_i16(&mut data, 4, velocity_ff);
            put_i16(&mut data, 6, torque_ff);
            (SET_INPUT_POS, 8)
        }
        Command::SetInputVel {
            velocity,
            torque_ff,
        } => {
            finite(velocity, "velocity")?;
            finite(torque_ff, "torque_ff")?;
            put_f32(&mut data, 0, velocity);
            put_f32(&mut data, 4, torque_ff);
            (SET_INPUT_VEL, 8)
        }
        Command::SetInputTorque { torque } => {
            finite(torque, "torque")?;
            put_f32(&mut data, 0, torque);
            (SET_INPUT_TORQUE, 4)
        }
        Command::SetVelocityLimit { velocity } => {
            finite(velocity, "velocity")?;
            put_f32(&mut data, 0, velocity);
            (SET_VELOCITY_LIMIT, 4)
        }
        Command::StartAnticogging => (START_ANTICOGGING, 0),
        Command::SetTrajVelLimit { velocity } => {
            finite(velocity, "velocity")?;
            put_f32(&mut data, 0, velocity);
            (SET_TRAJ_VEL_LIMIT, 4)
        }
        Command::SetTrajAccelLimits {
            acceleration,
            deceleration,
        } => {
            finite(acceleration, "acceleration")?;
            finite(deceleration, "deceleration")?;
            put_f32(&mut data, 0, acceleration);
            put_f32(&mut data, 4, deceleration);
            (SET_TRAJ_ACCEL_LIMITS, 8)
        }
        Command::SetTrajInertia { inertia } => {
            finite(inertia, "inertia")?;
            put_f32(&mut data, 0, inertia);
            (SET_TRAJ_INERTIA, 4)
        }
        Command::Reboot => (REBOOT, 0),
        Command::ClearErrors => (CLEAR_ERRORS, 0),
    };
    Ok((command_id, data, len, false))
}

fn encode_response(response: Response) -> (u8, [u8; 8], u8, bool) {
    let mut data = [0; 8];
    let command_id = match response {
        Response::Heartbeat {
            axis_error,
            axis_state,
        } => {
            put_u32(&mut data, 0, axis_error);
            put_u32(&mut data, 4, axis_state.0);
            HEARTBEAT
        }
        Response::MotorError(error) => {
            put_u32(&mut data, 0, error);
            MOTOR_ERROR
        }
        Response::EncoderError(error) => {
            put_u32(&mut data, 0, error);
            ENCODER_ERROR
        }
        Response::SensorlessError(error) => {
            put_u32(&mut data, 0, error);
            SENSORLESS_ERROR
        }
        Response::EncoderEstimates { position, velocity } => {
            put_f32(&mut data, 0, position);
            put_f32(&mut data, 4, velocity);
            ENCODER_ESTIMATES
        }
        Response::EncoderCount {
            shadow_count,
            count_in_cpr,
        } => {
            put_u32(&mut data, 0, shadow_count as u32);
            put_u32(&mut data, 4, count_in_cpr as u32);
            ENCODER_COUNT
        }
        Response::Iq { setpoint, measured } => {
            put_f32(&mut data, 0, setpoint);
            put_f32(&mut data, 4, measured);
            IQ
        }
        Response::SensorlessEstimates { position, velocity } => {
            put_f32(&mut data, 0, position);
            put_f32(&mut data, 4, velocity);
            SENSORLESS_ESTIMATES
        }
        Response::VbusVoltage(voltage) => {
            put_f32(&mut data, 0, voltage);
            VBUS_VOLTAGE
        }
    };
    (command_id, data, 8, false)
}

fn decode_remote(command_id: u8, dlc: u8) -> Result<Option<Message>, DecodeError> {
    if dlc > 8 {
        return Err(DecodeError::InvalidRemoteDlc { dlc });
    }
    match query_from_id(command_id) {
        Some(query) => Ok(Some(Message::Request(query))),
        None => Err(DecodeError::InvalidFrameKind { command_id }),
    }
}

fn decode_data(command_id: u8, data: &[u8]) -> Result<Option<Message>, DecodeError> {
    let message = match command_id {
        HEARTBEAT => {
            require_len(command_id, data, 8)?;
            Message::Response(Response::Heartbeat {
                axis_error: u32_at(data, 0),
                axis_state: AxisState(u32_at(data, 4)),
            })
        }
        ESTOP => {
            require_len(command_id, data, 0)?;
            Message::Command(Command::Estop)
        }
        MOTOR_ERROR => Message::Response(Response::MotorError(decode_error_response(
            command_id, data,
        )?)),
        ENCODER_ERROR => Message::Response(Response::EncoderError(decode_error_response(
            command_id, data,
        )?)),
        SENSORLESS_ERROR => Message::Response(Response::SensorlessError(decode_error_response(
            command_id, data,
        )?)),
        SET_AXIS_NODE_ID => {
            require_len(command_id, data, 4)?;
            let node_id = u32_at(data, 0);
            let node_id = NodeId::new(node_id).map_err(|_| DecodeError::InvalidNodeId(node_id))?;
            Message::Command(Command::SetAxisNodeId { node_id })
        }
        SET_AXIS_REQUESTED_STATE => {
            require_len(command_id, data, 4)?;
            Message::Command(Command::SetAxisRequestedState {
                state: AxisState(u32_at(data, 0)),
            })
        }
        ENCODER_ESTIMATES => {
            require_len(command_id, data, 8)?;
            Message::Response(Response::EncoderEstimates {
                position: f32_at(data, 0),
                velocity: f32_at(data, 4),
            })
        }
        ENCODER_COUNT => {
            require_len(command_id, data, 8)?;
            Message::Response(Response::EncoderCount {
                shadow_count: u32_at(data, 0) as i32,
                count_in_cpr: u32_at(data, 4) as i32,
            })
        }
        SET_CONTROLLER_MODES => {
            require_len(command_id, data, 8)?;
            Message::Command(Command::SetControllerModes {
                control_mode: u32_at(data, 0) as i32,
                input_mode: u32_at(data, 4) as i32,
            })
        }
        SET_INPUT_POS => {
            require_len(command_id, data, 8)?;
            Message::Command(Command::SetInputPos {
                position: f32_at(data, 0),
                velocity_ff: i16_at(data, 4),
                torque_ff: i16_at(data, 6),
            })
        }
        SET_INPUT_VEL => {
            require_len(command_id, data, 8)?;
            Message::Command(Command::SetInputVel {
                velocity: f32_at(data, 0),
                torque_ff: f32_at(data, 4),
            })
        }
        SET_INPUT_TORQUE => {
            require_len(command_id, data, 4)?;
            Message::Command(Command::SetInputTorque {
                torque: f32_at(data, 0),
            })
        }
        SET_VELOCITY_LIMIT => {
            require_len(command_id, data, 4)?;
            Message::Command(Command::SetVelocityLimit {
                velocity: f32_at(data, 0),
            })
        }
        START_ANTICOGGING => {
            require_len(command_id, data, 0)?;
            Message::Command(Command::StartAnticogging)
        }
        SET_TRAJ_VEL_LIMIT => {
            require_len(command_id, data, 4)?;
            Message::Command(Command::SetTrajVelLimit {
                velocity: f32_at(data, 0),
            })
        }
        SET_TRAJ_ACCEL_LIMITS => {
            require_len(command_id, data, 8)?;
            Message::Command(Command::SetTrajAccelLimits {
                acceleration: f32_at(data, 0),
                deceleration: f32_at(data, 4),
            })
        }
        SET_TRAJ_INERTIA => {
            require_len(command_id, data, 4)?;
            Message::Command(Command::SetTrajInertia {
                inertia: f32_at(data, 0),
            })
        }
        IQ => {
            require_len(command_id, data, 8)?;
            Message::Response(Response::Iq {
                setpoint: f32_at(data, 0),
                measured: f32_at(data, 4),
            })
        }
        SENSORLESS_ESTIMATES => {
            require_len(command_id, data, 8)?;
            Message::Response(Response::SensorlessEstimates {
                position: f32_at(data, 0),
                velocity: f32_at(data, 4),
            })
        }
        REBOOT => {
            require_len(command_id, data, 0)?;
            Message::Command(Command::Reboot)
        }
        VBUS_VOLTAGE => Message::Response(Response::VbusVoltage(decode_vbus_voltage(data)?)),
        CLEAR_ERRORS => {
            require_len(command_id, data, 0)?;
            Message::Command(Command::ClearErrors)
        }
        _ => return Ok(None),
    };
    Ok(Some(message))
}

fn decode_error_response(command_id: u8, data: &[u8]) -> Result<u32, DecodeError> {
    require_len(command_id, data, 8)?;
    Ok(u32_at(data, 0))
}

fn decode_vbus_voltage(data: &[u8]) -> Result<f32, DecodeError> {
    require_len(VBUS_VOLTAGE, data, 8)?;
    Ok(f32_at(data, 0))
}

fn is_known(command_id: u8) -> bool {
    matches!(
        command_id,
        HEARTBEAT
            | ESTOP
            | MOTOR_ERROR
            | ENCODER_ERROR
            | SENSORLESS_ERROR
            | SET_AXIS_NODE_ID
            | SET_AXIS_REQUESTED_STATE
            | ENCODER_ESTIMATES
            | ENCODER_COUNT
            | SET_CONTROLLER_MODES
            | SET_INPUT_POS
            | SET_INPUT_VEL
            | SET_INPUT_TORQUE
            | SET_VELOCITY_LIMIT
            | START_ANTICOGGING
            | SET_TRAJ_VEL_LIMIT
            | SET_TRAJ_ACCEL_LIMITS
            | SET_TRAJ_INERTIA
            | IQ
            | SENSORLESS_ESTIMATES
            | REBOOT
            | VBUS_VOLTAGE
            | CLEAR_ERRORS
    )
}

fn query_id(query: Query) -> u8 {
    match query {
        Query::MotorError => MOTOR_ERROR,
        Query::EncoderError => ENCODER_ERROR,
        Query::SensorlessError => SENSORLESS_ERROR,
        Query::EncoderEstimates => ENCODER_ESTIMATES,
        Query::EncoderCount => ENCODER_COUNT,
        Query::Iq => IQ,
        Query::SensorlessEstimates => SENSORLESS_ESTIMATES,
        Query::VbusVoltage => VBUS_VOLTAGE,
    }
}

fn query_from_id(command_id: u8) -> Option<Query> {
    match command_id {
        MOTOR_ERROR => Some(Query::MotorError),
        ENCODER_ERROR => Some(Query::EncoderError),
        SENSORLESS_ERROR => Some(Query::SensorlessError),
        ENCODER_ESTIMATES => Some(Query::EncoderEstimates),
        ENCODER_COUNT => Some(Query::EncoderCount),
        IQ => Some(Query::Iq),
        SENSORLESS_ESTIMATES => Some(Query::SensorlessEstimates),
        VBUS_VOLTAGE => Some(Query::VbusVoltage),
        _ => None,
    }
}

fn finite(value: f32, field: &'static str) -> Result<(), EncodeError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(EncodeError::NonFinite { field })
    }
}

fn require_len(command_id: u8, data: &[u8], expected: u8) -> Result<(), DecodeError> {
    let actual = data.len();
    if actual == usize::from(expected) {
        Ok(())
    } else {
        Err(DecodeError::InvalidLength {
            command_id,
            expected,
            actual,
        })
    }
}

fn put_u32(data: &mut [u8; 8], offset: usize, value: u32) {
    data[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_i16(data: &mut [u8; 8], offset: usize, value: i16) {
    data[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn put_f32(data: &mut [u8; 8], offset: usize, value: f32) {
    put_u32(data, offset, value.to_bits());
}

fn u32_at(data: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        data[offset],
        data[offset + 1],
        data[offset + 2],
        data[offset + 3],
    ])
}

fn i16_at(data: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes([data[offset], data[offset + 1]])
}

fn f32_at(data: &[u8], offset: usize) -> f32 {
    f32::from_bits(u32_at(data, offset))
}
