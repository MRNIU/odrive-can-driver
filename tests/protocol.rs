// Copyright The odrive-can-protocol Contributors

//! 使用独立字节向量验证 fw-v0.5.1 的命令、查询、回复和输入边界。
//!
//! 测试直接给定预期 ID、字段位置与端序，不以编解码自循环代替协议依据。

use odrive_can_driver::protocol::{
    AxisState, Command, DecodeError, EncodeError, FrameId, FramePayload, FrameRef, Message, NodeId,
    Query, Response, decode, encode,
};

type CommandBuilder = fn(f32) -> Command;

fn node() -> NodeId {
    NodeId::new(3).unwrap()
}

fn data(id: u16, bytes: &[u8]) -> FrameRef<'_> {
    FrameRef {
        id: FrameId::Standard(id),
        payload: FramePayload::Data(bytes),
    }
}

#[test]
fn encodes_every_write_command_to_its_documented_bytes() {
    let reassigned = NodeId::new(63).unwrap();
    let cases: &[(Command, u16, &[u8])] = &[
        (Command::Estop, 0x62, &[]),
        (
            Command::SetAxisNodeId {
                node_id: reassigned,
            },
            0x66,
            &[63, 0, 0, 0],
        ),
        (
            Command::SetAxisRequestedState {
                state: AxisState(0x8000_0008),
            },
            0x67,
            &[8, 0, 0, 128],
        ),
        (
            Command::SetControllerModes {
                control_mode: -2,
                input_mode: 5,
            },
            0x6b,
            &[254, 255, 255, 255, 5, 0, 0, 0],
        ),
        (
            Command::SetInputPos {
                position: 1.5,
                velocity_ff: -2,
                torque_ff: 3,
            },
            0x6c,
            &[0, 0, 192, 63, 254, 255, 3, 0],
        ),
        (
            Command::SetInputVel {
                velocity: -0.0,
                torque_ff: 2.5,
            },
            0x6d,
            &[0, 0, 0, 128, 0, 0, 32, 64],
        ),
        (
            Command::SetInputVel {
                velocity: 0.0,
                torque_ff: -2.5,
            },
            0x6d,
            &[0, 0, 0, 0, 0, 0, 32, 192],
        ),
        (
            Command::SetInputVel {
                velocity: 1.5,
                torque_ff: 0.25,
            },
            0x6d,
            &[0, 0, 192, 63, 0, 0, 128, 62],
        ),
        (
            Command::SetInputVel {
                velocity: -2.5,
                torque_ff: 0.25,
            },
            0x6d,
            &[0, 0, 32, 192, 0, 0, 128, 62],
        ),
        (
            Command::SetInputTorque { torque: -1.25 },
            0x6e,
            &[0, 0, 160, 191],
        ),
        (
            Command::SetVelocityLimit { velocity: 3.0 },
            0x6f,
            &[0, 0, 64, 64],
        ),
        (Command::StartAnticogging, 0x70, &[]),
        (
            Command::SetTrajVelLimit { velocity: 4.0 },
            0x71,
            &[0, 0, 128, 64],
        ),
        (
            Command::SetTrajAccelLimits {
                acceleration: 5.0,
                deceleration: 6.0,
            },
            0x72,
            &[0, 0, 160, 64, 0, 0, 192, 64],
        ),
        (
            Command::SetTrajInertia { inertia: 7.0 },
            0x73,
            &[0, 0, 224, 64],
        ),
        (Command::Reboot, 0x76, &[]),
        (Command::ClearErrors, 0x78, &[]),
    ];

    for (command, id, expected) in cases {
        let encoded = encode(node(), Message::Command(*command)).unwrap();
        assert_eq!(encoded.id(), *id);
        assert_eq!(encoded.data(), *expected);
        assert_eq!(encoded.dlc(), expected.len() as u8);
        assert!(!encoded.is_remote());
        assert_eq!(encoded.as_ref(), data(*id, expected));
    }
}

#[test]
fn decodes_every_write_command_from_its_documented_bytes() {
    let cases: &[(u16, &[u8], Command)] = &[
        (0x62, &[], Command::Estop),
        (
            0x66,
            &[63, 0, 0, 0],
            Command::SetAxisNodeId {
                node_id: NodeId::new(63).unwrap(),
            },
        ),
        (
            0x67,
            &[8, 0, 0, 128],
            Command::SetAxisRequestedState {
                state: AxisState(0x8000_0008),
            },
        ),
        (
            0x6b,
            &[254, 255, 255, 255, 5, 0, 0, 0],
            Command::SetControllerModes {
                control_mode: -2,
                input_mode: 5,
            },
        ),
        (
            0x6c,
            &[0, 0, 192, 63, 254, 255, 3, 0],
            Command::SetInputPos {
                position: 1.5,
                velocity_ff: -2,
                torque_ff: 3,
            },
        ),
        (
            0x6d,
            &[0, 0, 0, 128, 0, 0, 32, 64],
            Command::SetInputVel {
                velocity: -0.0,
                torque_ff: 2.5,
            },
        ),
        (
            0x6e,
            &[0, 0, 160, 191],
            Command::SetInputTorque { torque: -1.25 },
        ),
        (
            0x6f,
            &[0, 0, 64, 64],
            Command::SetVelocityLimit { velocity: 3.0 },
        ),
        (0x70, &[], Command::StartAnticogging),
        (
            0x71,
            &[0, 0, 128, 64],
            Command::SetTrajVelLimit { velocity: 4.0 },
        ),
        (
            0x72,
            &[0, 0, 160, 64, 0, 0, 192, 64],
            Command::SetTrajAccelLimits {
                acceleration: 5.0,
                deceleration: 6.0,
            },
        ),
        (
            0x73,
            &[0, 0, 224, 64],
            Command::SetTrajInertia { inertia: 7.0 },
        ),
        (0x76, &[], Command::Reboot),
        (0x78, &[], Command::ClearErrors),
    ];

    for (id, bytes, expected) in cases {
        assert_eq!(
            decode(node(), data(*id, bytes)).unwrap(),
            Some(Message::Command(*expected))
        );
    }
}

#[test]
fn encodes_all_rtr_queries_with_dlc_eight() {
    let cases = [
        (Query::MotorError, 0x63),
        (Query::EncoderError, 0x64),
        (Query::SensorlessError, 0x65),
        (Query::EncoderEstimates, 0x69),
        (Query::EncoderCount, 0x6a),
        (Query::Iq, 0x74),
        (Query::SensorlessEstimates, 0x75),
        (Query::VbusVoltage, 0x77),
    ];

    for (query, id) in cases {
        let encoded = encode(node(), Message::Request(query)).unwrap();
        assert_eq!(encoded.id(), id);
        assert_eq!(encoded.data(), &[]);
        assert_eq!(encoded.dlc(), 8);
        assert!(encoded.is_remote());
        assert_eq!(
            encoded.as_ref(),
            FrameRef {
                id: FrameId::Standard(id),
                payload: FramePayload::Remote { dlc: 8 }
            }
        );
    }
}

#[test]
fn decodes_rtr_queries_with_any_classic_dlc() {
    let cases = [
        (Query::MotorError, 0x63),
        (Query::EncoderError, 0x64),
        (Query::SensorlessError, 0x65),
        (Query::EncoderEstimates, 0x69),
        (Query::EncoderCount, 0x6a),
        (Query::Iq, 0x74),
        (Query::SensorlessEstimates, 0x75),
        (Query::VbusVoltage, 0x77),
    ];

    for (query, id) in cases {
        for dlc in 0..=8 {
            assert_eq!(
                decode(
                    node(),
                    FrameRef {
                        id: FrameId::Standard(id),
                        payload: FramePayload::Remote { dlc }
                    }
                )
                .unwrap(),
                Some(Message::Request(query))
            );
        }
    }
}

#[test]
fn encodes_and_decodes_all_device_responses() {
    let nan = f32::from_bits(0x7fc0_0123);
    let cases: &[(Response, u16, [u8; 8])] = &[
        (
            Response::Heartbeat {
                axis_error: 0x8000_0001,
                axis_state: AxisState(0x8000_0001),
            },
            0x61,
            [1, 0, 0, 128, 1, 0, 0, 128],
        ),
        (
            Response::MotorError(0x0102_0304),
            0x63,
            [4, 3, 2, 1, 0, 0, 0, 0],
        ),
        (
            Response::EncoderError(0x1112_1314),
            0x64,
            [20, 19, 18, 17, 0, 0, 0, 0],
        ),
        (
            Response::SensorlessError(0x2122_2324),
            0x65,
            [36, 35, 34, 33, 0, 0, 0, 0],
        ),
        (
            Response::EncoderEstimates {
                position: 1.5,
                velocity: -2.0,
            },
            0x69,
            [0, 0, 192, 63, 0, 0, 0, 192],
        ),
        (
            Response::EncoderCount {
                shadow_count: -2,
                count_in_cpr: 7,
            },
            0x6a,
            [254, 255, 255, 255, 7, 0, 0, 0],
        ),
        (
            Response::Iq {
                setpoint: -0.0,
                measured: 2.5,
            },
            0x74,
            [0, 0, 0, 128, 0, 0, 32, 64],
        ),
        (
            Response::SensorlessEstimates {
                position: nan,
                velocity: -1.0,
            },
            0x75,
            [35, 1, 192, 127, 0, 0, 128, 191],
        ),
        (
            Response::VbusVoltage(48.0),
            0x77,
            [0, 0, 64, 66, 0, 0, 0, 0],
        ),
    ];

    for (response, id, bytes) in cases {
        let encoded = encode(node(), Message::Response(*response)).unwrap();
        assert_eq!(encoded.id(), *id);
        assert_eq!(encoded.data(), bytes);
        assert_eq!(encoded.dlc(), 8);
        assert!(!encoded.is_remote());
        let decoded = decode(node(), data(*id, bytes)).unwrap().unwrap();
        match (response, decoded) {
            (
                Response::SensorlessEstimates { position, velocity },
                Message::Response(Response::SensorlessEstimates {
                    position: decoded_position,
                    velocity: decoded_velocity,
                }),
            ) => {
                assert_eq!(position.to_bits(), decoded_position.to_bits());
                assert_eq!(velocity.to_bits(), decoded_velocity.to_bits());
            }
            (expected, Message::Response(actual)) => assert_eq!(*expected, actual),
            (_, actual) => panic!("unexpected decoded message: {actual:?}"),
        }
    }
}

#[test]
fn rejects_invalid_or_unsupported_related_frames() {
    assert_eq!(NodeId::new(64), Err(EncodeError::InvalidNodeId(64)));
    assert_eq!(
        encode(
            node(),
            Message::Command(Command::SetInputTorque {
                torque: f32::INFINITY
            })
        ),
        Err(EncodeError::NonFinite { field: "torque" })
    );
    assert_eq!(
        decode(
            node(),
            FrameRef {
                id: FrameId::Extended(0x123),
                payload: FramePayload::Data(&[])
            }
        ),
        Err(DecodeError::UnsupportedExtendedId)
    );
    assert_eq!(
        decode(node(), data(0x800, &[])),
        Err(DecodeError::InvalidStandardId(0x800))
    );
    assert_eq!(
        decode(
            node(),
            FrameRef {
                id: FrameId::Standard(0x61),
                payload: FramePayload::Fd(&[])
            }
        ),
        Err(DecodeError::UnsupportedCanFd)
    );
    assert_eq!(
        decode(
            node(),
            FrameRef {
                id: FrameId::Standard(0x68),
                payload: FramePayload::Data(&[])
            }
        ),
        Err(DecodeError::UnsupportedCommand { command_id: 0x08 })
    );
    assert_eq!(
        decode(
            node(),
            FrameRef {
                id: FrameId::Standard(0x61),
                payload: FramePayload::Remote { dlc: 8 }
            }
        ),
        Err(DecodeError::InvalidFrameKind { command_id: 0x01 })
    );
    assert_eq!(
        decode(node(), data(0x61, &[0; 4])),
        Err(DecodeError::InvalidLength {
            command_id: 0x01,
            expected: 8,
            actual: 4
        })
    );
    assert_eq!(
        decode(node(), data(0x6e, &[])),
        Err(DecodeError::InvalidLength {
            command_id: 0x0e,
            expected: 4,
            actual: 0
        })
    );
    assert_eq!(
        decode(
            node(),
            FrameRef {
                id: FrameId::Standard(0x69),
                payload: FramePayload::Remote { dlc: 9 }
            }
        ),
        Err(DecodeError::InvalidRemoteDlc { dlc: 9 })
    );
    assert_eq!(
        decode(node(), data(0x66, &[64, 0, 0, 0])),
        Err(DecodeError::InvalidNodeId(64))
    );
}

#[test]
fn rejects_nonfinite_values_in_every_host_float_command() {
    let cases: &[(CommandBuilder, &str)] = &[
        (
            |value| Command::SetInputPos {
                position: value,
                velocity_ff: 0,
                torque_ff: 0,
            },
            "position",
        ),
        (
            |value| Command::SetInputVel {
                velocity: value,
                torque_ff: 0.0,
            },
            "velocity",
        ),
        (
            |value| Command::SetInputVel {
                velocity: 0.0,
                torque_ff: value,
            },
            "torque_ff",
        ),
        (|value| Command::SetInputTorque { torque: value }, "torque"),
        (
            |value| Command::SetVelocityLimit { velocity: value },
            "velocity",
        ),
        (
            |value| Command::SetTrajVelLimit { velocity: value },
            "velocity",
        ),
        (
            |value| Command::SetTrajAccelLimits {
                acceleration: value,
                deceleration: 0.0,
            },
            "acceleration",
        ),
        (
            |value| Command::SetTrajAccelLimits {
                acceleration: 0.0,
                deceleration: value,
            },
            "deceleration",
        ),
        (
            |value| Command::SetTrajInertia { inertia: value },
            "inertia",
        ),
    ];

    for value in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        for (make_command, field) in cases {
            assert_eq!(
                encode(node(), Message::Command(make_command(value))),
                Err(EncodeError::NonFinite { field })
            );
        }
    }
}

#[test]
fn ignores_undefined_error_response_tail_bytes() {
    let message = decode(node(), data(0x63, &[4, 3, 2, 1, 99, 98, 97, 96])).unwrap();
    assert_eq!(
        message,
        Some(Message::Response(Response::MotorError(0x0102_0304)))
    );
}

#[test]
fn ignores_unrelated_nodes_and_unknown_commands_before_payload_validation() {
    assert_eq!(decode(node(), data(0x41, &[])).unwrap(), None);
    assert_eq!(decode(node(), data(0x7f, &[])).unwrap(), None);
}
