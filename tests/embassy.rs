// Copyright The odrive-can-driver Contributors
//! 验证 Embassy 查询帧构造，以及接收路径保留共享总线的原生帧。

#![cfg(feature = "embassy-stm32")]

use embassy_stm32::can::frame::{FdEnvelope, FdFrame, Frame, Header};
use odrive_can_driver::{
    Driver, IngestResult, Instant, Session,
    embassy::{EmbassyReceive, ingest_fd_envelope},
    protocol::{NodeId, Query},
};

fn node() -> NodeId {
    NodeId::new(7).unwrap()
}

fn driver(session: &mut Session) -> Driver<'_> {
    Driver::new(session, node())
}

#[test]
fn all_driver_queries_construct_classic_rtr_frames_for_every_node() {
    let queries = [
        (Query::MotorError, 0x03),
        (Query::EncoderError, 0x04),
        (Query::SensorlessError, 0x05),
        (Query::EncoderEstimates, 0x09),
        (Query::EncoderCount, 0x0a),
        (Query::Iq, 0x14),
        (Query::SensorlessEstimates, 0x15),
        (Query::VbusVoltage, 0x17),
    ];
    for node_id in 0..=63 {
        for (query, command_id) in queries {
            let mut session = Session::new();
            let mut driver = Driver::new(&mut session, NodeId::new(node_id).unwrap());
            let permit = driver
                .prepare_query(query, Instant::from_micros(0), Instant::from_micros(10))
                .unwrap();
            let attempt = driver.begin_send(permit, Instant::from_micros(1)).unwrap();
            let encoded = driver
                .authorize_tx(&attempt, Instant::from_micros(2))
                .unwrap();
            assert!(encoded.data().is_empty());
            // 与 EmbassyTx::poll 相同的原生构造调用；不访问控制器寄存器。
            let classic = Frame::from(&encoded);
            let container = FdFrame::from(&encoded);
            for header in [classic.header(), container.header()] {
                assert_eq!(
                    *header.id(),
                    embedded_can::Id::Standard(
                        embedded_can::StandardId::new(((node_id as u16) << 5) | command_id)
                            .unwrap()
                    )
                );
                assert!(header.rtr());
                assert_eq!(header.len(), 8);
                assert!(!header.fdcan());
                assert!(!header.bit_rate_switching());
            }
        }
    }
}

#[test]
fn unrelated_fd_container_is_returned_without_losing_its_classic_header() {
    let mut session = Session::new();
    let mut driver = driver(&mut session);
    let fd_frame = FdFrame::new_standard((8 << 5) | 3, &[1, 2]).unwrap();
    let frame = FdFrame::new(Header::new(*fd_frame.header().id(), 2, false), &[1, 2]).unwrap();

    let outcome = ingest_fd_envelope(
        &mut driver,
        FdEnvelope { ts: 17, frame },
        Instant::from_micros(3),
    );

    let EmbassyReceive::Frame {
        envelope,
        classification,
    } = outcome
    else {
        panic!("a received frame must retain native ownership");
    };
    assert_eq!(classification, IngestResult::Unrelated);
    assert_eq!(envelope.ts, 17);
    assert!(!envelope.frame.header().fdcan());
    assert_eq!(envelope.frame.header().len(), 2);
    assert_eq!(envelope.frame.data(), &[1, 2]);
}

#[test]
fn unrelated_short_fd_container_is_returned_as_fd() {
    let mut session = Session::new();
    let mut driver = driver(&mut session);
    let frame = FdFrame::new_standard((8 << 5) | 3, &[1, 2]).unwrap();

    let outcome = ingest_fd_envelope(
        &mut driver,
        FdEnvelope { ts: 23, frame },
        Instant::from_micros(3),
    );

    let EmbassyReceive::Frame {
        envelope,
        classification,
    } = outcome
    else {
        panic!("a received frame must retain native ownership");
    };
    assert_eq!(classification, IngestResult::Unrelated);
    assert_eq!(envelope.ts, 23);
    assert!(envelope.frame.header().fdcan());
    assert_eq!(envelope.frame.header().len(), 2);
    assert_eq!(envelope.frame.data(), &[1, 2]);
}

#[test]
fn extended_container_is_returned_when_protocol_rejects_it_before_node_filtering() {
    let mut session = Session::new();
    let mut driver = driver(&mut session);
    let frame = FdFrame::new_extended(0x12_345, &[1]).unwrap();

    let outcome = ingest_fd_envelope(
        &mut driver,
        FdEnvelope { ts: 31, frame },
        Instant::from_micros(3),
    );

    let EmbassyReceive::Frame {
        envelope,
        classification,
    } = outcome
    else {
        panic!("an extended FDCAN container must remain available to the bus owner");
    };
    assert_eq!(
        classification,
        IngestResult::DecodeError(odrive_can_driver::protocol::DecodeError::UnsupportedExtendedId)
    );
    assert_eq!(envelope.ts, 31);
    assert_eq!(envelope.frame.header().len(), 1);
    assert_eq!(envelope.frame.data(), &[1]);
}
