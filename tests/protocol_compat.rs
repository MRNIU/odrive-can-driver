// Copyright The odrive-can-protocol Contributors
//! 验证可选适配保留线上字节、RTR、FDF 和错误帧边界，不访问设备。

#![cfg(feature = "embedded-can")]

use embedded_can::{ExtendedId, Frame, StandardId};
use odrive_can_driver::protocol::{
    Command, FrameId, FramePayload, FrameRef, Message, NodeId, Query, encode,
};

#[test]
fn embedded_can_converts_bxcan_data_and_remote_frames() {
    let encoded = encode(
        NodeId::new(1).unwrap(),
        Message::Command(Command::SetInputTorque { torque: -2.5 }),
    )
    .unwrap();
    let native: bxcan::Frame = encoded.to_embedded_can().unwrap();
    assert_eq!(Frame::id(&native), StandardId::new(0x02e).unwrap().into());
    assert_eq!(Frame::data(&native), &[0, 0, 0x20, 0xc0]);
    let received = <bxcan::Frame as Frame>::new(ExtendedId::new(0x021).unwrap(), &[0; 8]).unwrap();
    assert_eq!(
        FrameRef::from_classic_embedded_can(&received).unwrap().id,
        FrameId::Extended(0x021)
    );
    let query = encode(
        NodeId::new(1).unwrap(),
        Message::Request(Query::EncoderEstimates),
    )
    .unwrap();
    let native: bxcan::Frame = query.to_embedded_can().unwrap();
    assert_eq!(
        FrameRef::from_classic_embedded_can(&native)
            .unwrap()
            .payload,
        FramePayload::Remote { dlc: 8 }
    );
}

#[cfg(feature = "embassy-stm32")]
mod embassy {
    use super::*;
    use embassy_stm32::can::frame::{FdFrame, Frame, Header};
    use odrive_can_driver::protocol::compat::embassy::FromEmbassyError;

    #[test]
    fn native_conversions_preserve_classic_remote_and_fd() {
        let frame = Frame::new_standard(0x02e, &[0, 0, 0x20, 0xc0]).unwrap();
        assert_eq!(
            FrameRef::try_from(&frame).unwrap().payload,
            FramePayload::Data(&[0, 0, 0x20, 0xc0])
        );
        let query = encode(
            NodeId::new(1).unwrap(),
            Message::Request(Query::EncoderEstimates),
        )
        .unwrap();
        let classic: Frame = (&query).into();
        let container: FdFrame = (&query).into();
        assert_eq!(
            FrameRef::try_from(&classic).unwrap().payload,
            FramePayload::Remote { dlc: 8 }
        );
        assert!(!container.header().fdcan());
        assert_eq!(
            FrameRef::try_from(&container).unwrap().payload,
            FramePayload::Remote { dlc: 8 }
        );
        let id = StandardId::new(0x021).unwrap().into();
        let frame = Frame::new(Header::new_fd(id, 8, true, false), &[0; 8]).unwrap();
        assert!(matches!(
            FrameRef::try_from(&frame).unwrap().payload,
            FramePayload::Fd(_)
        ));
        let frame = FdFrame::new(Header::new_fd(id, 8, false, false), &[0; 8]).unwrap();
        assert_eq!(
            FrameRef::try_from(&frame).unwrap().payload,
            FramePayload::Fd(&[0; 8])
        );
    }

    #[test]
    fn malformed_public_headers_return_error_without_panicking() {
        let id = StandardId::new(0x021).unwrap().into();
        let frame = Frame::new(Header::new(id, 9, false), &[]).unwrap();
        assert_eq!(
            FrameRef::try_from(&frame),
            Err(FromEmbassyError::InvalidPayloadLength)
        );
        let frame = FdFrame::new(Header::new_fd(id, 65, false, false), &[]).unwrap();
        assert_eq!(
            FrameRef::try_from(&frame),
            Err(FromEmbassyError::InvalidPayloadLength)
        );
    }
}

#[cfg(all(feature = "socketcan", target_os = "linux"))]
mod socketcan {
    use super::*;
    use ::socketcan::{CanAnyFrame, CanErrorFrame, CanFdFrame, CanFrame};
    use odrive_can_driver::protocol::compat::socketcan::FromSocketcanError;

    #[test]
    fn native_conversions_preserve_data_remote_and_short_fd() {
        let command = encode(
            NodeId::new(1).unwrap(),
            Message::Command(Command::SetInputTorque { torque: -2.5 }),
        )
        .unwrap();
        let native: CanFrame = (&command).into();
        assert_eq!(native.id(), StandardId::new(0x02e).unwrap().into());
        assert_eq!(
            FrameRef::try_from(&native).unwrap().payload,
            FramePayload::Data(&[0, 0, 0x20, 0xc0])
        );
        let query = encode(
            NodeId::new(1).unwrap(),
            Message::Request(Query::EncoderEstimates),
        )
        .unwrap();
        let native: CanFrame = (&query).into();
        assert_eq!(
            FrameRef::try_from(&native).unwrap().payload,
            FramePayload::Remote { dlc: 8 }
        );
        let frame =
            CanAnyFrame::Fd(CanFdFrame::new(ExtendedId::new(0x021).unwrap(), &[0; 8]).unwrap());
        let view = FrameRef::try_from(&frame).unwrap();
        assert_eq!(view.id, FrameId::Extended(0x021));
        assert_eq!(view.payload, FramePayload::Fd(&[0; 8]));
    }

    #[test]
    fn linux_error_frames_never_become_odrive_messages() {
        let frame = CanErrorFrame::new_error(0x021, &[0; 8]).unwrap();
        assert_eq!(
            FrameRef::try_from(&CanFrame::Error(frame)),
            Err(FromSocketcanError::ErrorFrame)
        );
        assert_eq!(
            FrameRef::try_from(&CanAnyFrame::Error(frame)),
            Err(FromSocketcanError::ErrorFrame)
        );
    }
}
