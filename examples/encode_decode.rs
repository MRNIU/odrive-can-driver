// Copyright The odrive-can-protocol Contributors

//! 演示速度命令、RTR 查询和 Heartbeat 的协议转换，不连接设备。
//!
//! 运行 `cargo run --example encode_decode`，查看显式节点和原始字段如何交接。

use odrive_can_driver::protocol::fw_v0_5_1::{
    self as protocol, AxisState, Command, Message, NodeId, Query, Response,
};
use odrive_can_driver::protocol::{FrameId, FramePayload, FrameRef};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let node = NodeId::new(1)?;
    let tx = protocol::encode(
        node,
        Message::Command(Command::SetInputVel {
            velocity: -2.5,
            torque_ff: 0.25,
        }),
    )?;
    assert_eq!(tx.id(), 0x02d);
    assert_eq!(tx.data(), &[0x00, 0x00, 0x20, 0xc0, 0x00, 0x00, 0x80, 0x3e]);

    let query = protocol::encode(node, Message::Request(Query::EncoderEstimates))?;
    assert!(query.is_remote());
    assert_eq!(query.dlc(), 8);
    assert!(query.data().is_empty());

    let bytes = [0x01, 0x00, 0x00, 0x80, 0x08, 0x00, 0x01, 0x00];
    let rx = FrameRef {
        id: FrameId::Standard(0x021),
        payload: FramePayload::Data(&bytes),
    };
    assert_eq!(
        protocol::decode(node, rx)?,
        Some(Message::Response(Response::Heartbeat {
            axis_error: 0x8000_0001,
            axis_state: AxisState(0x0001_0008),
        }))
    );
    println!("编码：ID={:#05x}, data={:02x?}", tx.id(), tx.data());
    println!("解码：{:?}", protocol::decode(node, rx)?);
    Ok(())
}
