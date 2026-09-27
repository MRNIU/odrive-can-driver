// Copyright The odrive-can-driver Contributors

//! 在共享 CAN 总线上由调用方保留实际收发所有权的最小示例。

use odrive_can_driver::{
    Driver,
    protocol::{Command, NodeId},
};

fn main() {
    let node = NodeId::new(1).expect("constant node id is valid");
    let mut driver = Driver::new(node);
    let operation = driver
        .prepare_command(Command::ClearErrors, 0, 100)
        .expect("prepared command");
    let attempt = driver.begin_send(operation, 1).expect("before deadline");

    let _frame = attempt.frame();
    // The CAN owner sends `_frame` and then reports its known result.
    attempt
        .not_sent(2)
        .expect("this example did not send the frame");
}
