// Copyright The odrive-can-protocol Contributors

//! 用 C 与 Rust 共用的独立向量验证 fw-v0.5.1 协议。

use odrive_can_driver::protocol::{
    AxisState, Command, DecodeError, EncodeError, FrameId, FramePayload, FrameRef, Message, NodeId,
    Query, Response, decode, encode,
};

// 将独立字节向量继续送入真实后端构造器，覆盖依赖升级后的长度与帧类型合同。
#[cfg(feature = "embedded-can")]
fn assert_native_conversions(encoded: &odrive_can_driver::protocol::EncodedFrame) {
    let native: bxcan::Frame = encoded.to_embedded_can().unwrap();
    assert_eq!(
        FrameRef::from_classic_embedded_can(&native).unwrap(),
        encoded.as_ref()
    );
    #[cfg(feature = "embassy-stm32")]
    {
        let classic: embassy_stm32::can::frame::Frame = encoded.into();
        let container: embassy_stm32::can::frame::FdFrame = encoded.into();
        assert_eq!(FrameRef::try_from(&classic).unwrap(), encoded.as_ref());
        assert_eq!(FrameRef::try_from(&container).unwrap(), encoded.as_ref());
    }
    #[cfg(all(feature = "socketcan", target_os = "linux"))]
    {
        let native: socketcan::CanFrame = encoded.into();
        assert_eq!(FrameRef::try_from(&native).unwrap(), encoded.as_ref());
    }
}

fn unsigned_number(text: &str) -> u32 {
    if let Some(hex) = text.strip_prefix("0x") {
        u32::from_str_radix(hex, 16).unwrap()
    } else {
        text.parse().unwrap()
    }
}

fn field_bits(text: &str) -> u32 {
    if text.starts_with('-') {
        text.parse::<i32>().unwrap() as u32
    } else {
        unsigned_number(text)
    }
}

fn expected_message(tag: &str, fields: &[u32]) -> Message {
    let float = f32::from_bits;

    let command = match (tag, fields) {
        ("estop", []) => Command::Estop,
        ("set_axis_node_id", [id]) => Command::SetAxisNodeId {
            node_id: NodeId::new(*id).unwrap(),
        },
        ("set_axis_requested_state", [state]) => Command::SetAxisRequestedState {
            state: AxisState(*state),
        },
        ("set_controller_modes", [control, input]) => Command::SetControllerModes {
            control_mode: *control as i32,
            input_mode: *input as i32,
        },
        ("set_input_pos", [position, velocity, torque]) => Command::SetInputPos {
            position: float(*position),
            velocity_ff: i16::try_from(*velocity as i32).unwrap(),
            torque_ff: i16::try_from(*torque as i32).unwrap(),
        },
        ("set_input_vel", [velocity, torque]) => Command::SetInputVel {
            velocity: float(*velocity),
            torque_ff: float(*torque),
        },
        ("set_input_torque", [torque]) => Command::SetInputTorque {
            torque: float(*torque),
        },
        ("set_velocity_limit", [velocity]) => Command::SetVelocityLimit {
            velocity: float(*velocity),
        },
        ("start_anticogging", []) => Command::StartAnticogging,
        ("set_traj_vel_limit", [velocity]) => Command::SetTrajVelLimit {
            velocity: float(*velocity),
        },
        ("set_traj_accel_limits", [acceleration, deceleration]) => Command::SetTrajAccelLimits {
            acceleration: float(*acceleration),
            deceleration: float(*deceleration),
        },
        ("set_traj_inertia", [inertia]) => Command::SetTrajInertia {
            inertia: float(*inertia),
        },
        ("reboot", []) => Command::Reboot,
        ("clear_errors", []) => Command::ClearErrors,
        ("query_motor_error", []) => return Message::Request(Query::MotorError),
        ("query_encoder_error", []) => return Message::Request(Query::EncoderError),
        ("query_sensorless_error", []) => return Message::Request(Query::SensorlessError),
        ("query_encoder_estimates", []) => return Message::Request(Query::EncoderEstimates),
        ("query_encoder_count", []) => return Message::Request(Query::EncoderCount),
        ("query_iq", []) => return Message::Request(Query::Iq),
        ("query_sensorless_estimates", []) => return Message::Request(Query::SensorlessEstimates),
        ("query_vbus_voltage", []) => return Message::Request(Query::VbusVoltage),
        ("heartbeat", [error, state]) => {
            return Message::Response(Response::Heartbeat {
                axis_error: *error,
                axis_state: AxisState(*state),
            });
        }
        ("motor_error", [bits]) => return Message::Response(Response::MotorError(*bits)),
        ("encoder_error", [bits]) => return Message::Response(Response::EncoderError(*bits)),
        ("sensorless_error", [bits]) => return Message::Response(Response::SensorlessError(*bits)),
        ("encoder_estimates", [position, velocity]) => {
            return Message::Response(Response::EncoderEstimates {
                position: float(*position),
                velocity: float(*velocity),
            });
        }
        ("encoder_count", [shadow, cpr]) => {
            return Message::Response(Response::EncoderCount {
                shadow_count: *shadow as i32,
                count_in_cpr: *cpr as i32,
            });
        }
        ("iq", [setpoint, measured]) => {
            return Message::Response(Response::Iq {
                setpoint: float(*setpoint),
                measured: float(*measured),
            });
        }
        ("sensorless_estimates", [position, velocity]) => {
            return Message::Response(Response::SensorlessEstimates {
                position: float(*position),
                velocity: float(*velocity),
            });
        }
        ("vbus_voltage", [voltage]) => {
            return Message::Response(Response::VbusVoltage(float(*voltage)));
        }
        _ => panic!("invalid message fixture: {tag} {fields:?}"),
    };
    Message::Command(command)
}

fn decode_error(tag: &str, fields: &[u32]) -> Option<DecodeError> {
    Some(match (tag, fields) {
        ("invalid_standard_id", [id]) => {
            DecodeError::InvalidStandardId(u16::try_from(*id).unwrap())
        }
        ("unsupported_extended_id", []) => DecodeError::UnsupportedExtendedId,
        ("unsupported_can_fd", []) => DecodeError::UnsupportedCanFd,
        ("unsupported_command", [command_id]) => DecodeError::UnsupportedCommand {
            command_id: u8::try_from(*command_id).unwrap(),
        },
        ("invalid_frame_kind", [command_id]) => DecodeError::InvalidFrameKind {
            command_id: u8::try_from(*command_id).unwrap(),
        },
        ("invalid_length", [command_id, expected, actual]) => DecodeError::InvalidLength {
            command_id: u8::try_from(*command_id).unwrap(),
            expected: u8::try_from(*expected).unwrap(),
            actual: usize::try_from(*actual).unwrap(),
        },
        ("invalid_remote_dlc", [dlc]) => DecodeError::InvalidRemoteDlc {
            dlc: u8::try_from(*dlc).unwrap(),
        },
        ("invalid_node_id", [id]) => DecodeError::InvalidNodeId(*id),
        _ => return None,
    })
}

// PartialEq 单独使用会忽略零的符号并拒绝相同位模式的 NaN。
fn float_payload_bits(message: Message) -> Option<[u32; 3]> {
    use Message::{Command as Cmd, Response as Resp};

    let (first, second) = match message {
        Cmd(Command::SetInputPos {
            position,
            velocity_ff,
            torque_ff,
        }) => return Some([position.to_bits(), velocity_ff as u32, torque_ff as u32]),
        Cmd(Command::SetInputVel {
            velocity,
            torque_ff,
        }) => (velocity, torque_ff),
        Cmd(Command::SetInputTorque { torque }) => (torque, 0.0),
        Cmd(Command::SetVelocityLimit { velocity } | Command::SetTrajVelLimit { velocity }) => {
            (velocity, 0.0)
        }
        Cmd(Command::SetTrajAccelLimits {
            acceleration,
            deceleration,
        }) => (acceleration, deceleration),
        Cmd(Command::SetTrajInertia { inertia }) => (inertia, 0.0),
        Resp(
            Response::EncoderEstimates { position, velocity }
            | Response::SensorlessEstimates { position, velocity },
        ) => (position, velocity),
        Resp(Response::Iq { setpoint, measured }) => (setpoint, measured),
        Resp(Response::VbusVoltage(voltage)) => (voltage, 0.0),
        _ => return None,
    };
    Some([first.to_bits(), second.to_bits(), 0])
}

fn assert_message(actual: Message, expected: Message, row: &str) {
    use core::mem::discriminant;

    let Some(bits) = float_payload_bits(expected) else {
        assert_eq!(actual, expected, "{row}");
        return;
    };
    match (actual, expected) {
        (Message::Command(actual), Message::Command(expected)) => {
            assert_eq!(discriminant(&actual), discriminant(&expected), "{row}");
        }
        (Message::Response(actual), Message::Response(expected)) => {
            assert_eq!(discriminant(&actual), discriminant(&expected), "{row}");
        }
        _ => panic!("{row}: unexpected message {actual:?}"),
    }
    assert_eq!(float_payload_bits(actual), Some(bits), "{row}");
}

#[test]
fn shared_protocol_vectors() {
    let mut case_count = 0;
    for row in include_str!("fixtures/fw_v0_5_1.tsv").lines() {
        if row.is_empty() || row.starts_with('#') {
            continue;
        }
        case_count += 1;
        let columns: Vec<_> = row.split('\t').collect();
        let [mode, node, id, frame, dlc, payload, tag, fields] = columns.as_slice() else {
            panic!("invalid vector: {row}");
        };
        let fields: Vec<_> = if *fields == "-" {
            Vec::new()
        } else {
            fields.split(',').map(field_bits).collect()
        };
        let raw_node = unsigned_number(node);
        match *mode {
            "both" | "decode" => {}
            "reject_invalid_node"
            | "reject_position"
            | "reject_velocity"
            | "reject_torque_ff"
            | "reject_torque"
            | "reject_acceleration"
            | "reject_deceleration"
            | "reject_inertia" => {
                assert_eq!([*id, *frame, *dlc, *payload], ["-"; 4], "{row}");
                let message = expected_message(tag, &fields);
                if *mode == "reject_invalid_node" {
                    assert_eq!(
                        NodeId::new(raw_node),
                        Err(EncodeError::InvalidNodeId(raw_node)),
                        "{row}"
                    );
                } else {
                    let field = mode.strip_prefix("reject_").unwrap();
                    assert_eq!(
                        encode(NodeId::new(raw_node).unwrap(), message),
                        Err(EncodeError::NonFinite { field }),
                        "{row}"
                    );
                }
                continue;
            }
            _ => panic!("invalid mode: {row}"),
        }
        let node = NodeId::new(raw_node).unwrap();
        let bytes: Vec<u8> = if *payload == "-" {
            Vec::new()
        } else {
            assert!(
                payload.len() <= 16 && payload.len().is_multiple_of(2),
                "{row}"
            );
            assert!(
                payload.bytes().all(|byte| byte.is_ascii_hexdigit()),
                "{row}"
            );
            (0..payload.len())
                .step_by(2)
                .map(|i| u8::from_str_radix(&payload[i..i + 2], 16).unwrap())
                .collect()
        };
        let id = unsigned_number(id);
        let dlc =
            u8::try_from(unsigned_number(dlc)).unwrap_or_else(|_| panic!("invalid DLC: {row}"));
        let input = FrameRef {
            id: if matches!(*frame, "extended_fd" | "extended_data") {
                FrameId::Extended(id)
            } else {
                FrameId::Standard(u16::try_from(id).unwrap_or_else(|_| panic!("invalid ID: {row}")))
            },
            payload: match *frame {
                "data" | "extended_data" => {
                    assert_eq!(usize::from(dlc), bytes.len(), "{row}");
                    FramePayload::Data(&bytes)
                }
                "rtr" => {
                    assert!(bytes.is_empty(), "{row}");
                    FramePayload::Remote { dlc }
                }
                "fd" | "extended_fd" => FramePayload::Fd(&bytes),
                _ => panic!("invalid frame: {row}"),
            },
        };
        let decoded = decode(node, input);
        match *tag {
            "unrelated" => {
                assert_eq!(*mode, "decode", "{row}");
                assert!(fields.is_empty(), "{row}");
                assert_eq!(decoded, Ok(None), "{row}");
            }
            _ if let Some(error) = decode_error(tag, &fields) => {
                assert_eq!(*mode, "decode", "{row}");
                assert_eq!(decoded, Err(error), "{row}");
            }
            _ => {
                let expected = expected_message(tag, &fields);
                let actual = decoded
                    .unwrap_or_else(|error| panic!("{row}: {error}"))
                    .unwrap_or_else(|| panic!("{row}: expected a decoded message"));
                assert_message(actual, expected, row);
                if *mode == "both" {
                    let encoded = encode(node, expected).unwrap();
                    assert_eq!(encoded.dlc(), dlc, "{row}");
                    assert_eq!(encoded.data(), bytes, "{row}");
                    assert_eq!(encoded.as_ref(), input, "{row}");
                    #[cfg(feature = "embedded-can")]
                    assert_native_conversions(&encoded);
                    if encoded.is_remote() {
                        for dlc in 0..=8 {
                            let input = FrameRef {
                                payload: FramePayload::Remote { dlc },
                                ..input
                            };
                            assert_eq!(decode(node, input), Ok(Some(expected)), "{row}");
                        }
                    }
                }
            }
        }
    }
    assert!(case_count > 0, "protocol fixture contains no vectors");
}
