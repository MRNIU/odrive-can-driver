// Copyright The odrive-can-driver Contributors
//! 验证 Embassy FDCAN 接收路径把无关原生帧归还给共享总线调用方。

#![cfg(feature = "embassy-stm32")]

use embassy_stm32::can::frame::{FdEnvelope, FdFrame, Header};
use odrive_can_driver::{
    Driver, IngestResult, Instant, Session,
    embassy::{EmbassyReceive, ingest_fd_envelope},
    protocol::NodeId,
};

fn node() -> NodeId {
    NodeId::new(7).unwrap()
}

fn driver(session: &mut Session) -> Driver<'_> {
    Driver::new(session, node())
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
