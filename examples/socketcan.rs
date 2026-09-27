// Copyright The odrive-can-driver Contributors
//! Linux SocketCAN 的一次性 ODrive 操作 CLI。
//!
//! ```text
//! socketcan <interface> <node> <operation> [arguments]
//!
//! operation:
//!   heartbeat | state          等待设备自行发送的一帧 Heartbeat；不发送帧
//!   vbus                       RTR 读取母线电压
//!   encoder                    RTR 读取编码器位置和速度
//!   motor-error | encoder-error RTR 读取对应错误位图
//!   clear-errors               发送 ClearErrors
//!   idle                       发送 SetAxisRequestedState(Idle)
//!   velocity <turn/s> [N*m]    发送 SetInputVel；不会自动进入闭环或在退出时归零
//! ```
//!
//! `heartbeat` 和 `state` 只接收本进程启动后观察到的设备 Heartbeat；CANSimple 中没有主机
//! Heartbeat 命令。本示例独占其 SocketCAN 描述符，收到无关、FD、RTR 或错误帧时只输出其
//! 原始分类后继续等待。共享总线应用应保留原始帧并在自己的统一 RX 循环分发，不能并发运行
//! 本 CLI 消费同一接收队列。
//!
//! 写命令输出 `Submitted` 即结束：这只表示 Linux 内核接受帧，并不是 ODrive ACK。RTR 查询
//! 必须观察到同节点、同回复类型的帧才输出 `Observed`。超时、明确本地失败和 `Unknown` 都以
//! 非零退出；`Unknown` 可能已发送，绝不能据此自动重试。

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("socketcan example only supports Linux SocketCAN");
    std::process::exit(2);
}

#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use std::{
        env, fmt, io, thread,
        time::{Duration, Instant},
    };

    use odrive_can_driver::{
        Driver, IngestResult, OperationId, OperationState, ResponseKind,
        protocol::{AxisState, Command, NodeId, Query},
        socketcan::{SocketCan, SocketCanReceive, SocketCanSendError},
    };
    use socketcan::{SocketOptions, id::ERR_MASK_ALL};

    const TIMEOUT: Duration = Duration::from_secs(1);
    const IDLE_WAIT: Duration = Duration::from_millis(1);

    enum Action {
        Heartbeat,
        Query(Query),
        Command(Command),
    }

    fn usage() -> &'static str {
        "usage: socketcan <interface> <node 0..63> \\
         <heartbeat|state|vbus|encoder|motor-error|encoder-error|clear-errors|idle|velocity> \\
         [velocity_turn_per_s [torque_ff_nm]]"
    }

    fn core_error(error: impl fmt::Debug) -> io::Error {
        io::Error::other(format!("driver state error: {error:?}"))
    }

    fn parse_number(value: Option<String>, name: &str) -> Result<f32, Box<dyn std::error::Error>> {
        value
            .ok_or_else(|| format!("missing {name}"))?
            .parse()
            .map_err(|error| format!("invalid {name}: {error}").into())
    }

    fn parse_action(
        operation: String,
        arguments: &mut impl Iterator<Item = String>,
    ) -> Result<Action, Box<dyn std::error::Error>> {
        let action = match operation.as_str() {
            "heartbeat" | "state" => Action::Heartbeat,
            "vbus" => Action::Query(Query::VbusVoltage),
            "encoder" => Action::Query(Query::EncoderEstimates),
            "motor-error" => Action::Query(Query::MotorError),
            "encoder-error" => Action::Query(Query::EncoderError),
            "clear-errors" => Action::Command(Command::ClearErrors),
            "idle" => Action::Command(Command::SetAxisRequestedState {
                state: AxisState::IDLE,
            }),
            "velocity" => {
                let velocity = parse_number(arguments.next(), "velocity_turn_per_s")?;
                let torque_ff = match arguments.next() {
                    Some(value) => value
                        .parse()
                        .map_err(|error| format!("invalid torque_ff_nm: {error}"))?,
                    None => 0.0,
                };
                Action::Command(Command::SetInputVel {
                    velocity,
                    torque_ff,
                })
            }
            _ => return Err(usage().into()),
        };
        if arguments.next().is_some() {
            return Err(usage().into());
        }
        Ok(action)
    }

    fn log_receive(received: SocketCanReceive) {
        match received.classification {
            Ok(IngestResult::Message(message)) => eprintln!("received ODrive message: {message:?}"),
            Ok(IngestResult::Unrelated) => {
                eprintln!(
                    "received unrelated frame on this CLI socket: {:?}",
                    received.frame
                )
            }
            Ok(IngestResult::DecodeError(error)) => eprintln!(
                "received frame outside this protocol version: {error:?}; raw={:?}",
                received.frame
            ),
            Err(error) => eprintln!(
                "SocketCAN error notification: {error}; raw={:?}",
                received.frame
            ),
        }
    }

    fn receive_or_wait(
        bus: &SocketCan,
        driver: &mut Driver,
        now_ms: impl FnOnce() -> u64,
    ) -> io::Result<()> {
        match bus.receive(driver, now_ms) {
            Ok(received) => {
                log_receive(received);
                Ok(())
            }
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                thread::sleep(IDLE_WAIT);
                Ok(())
            }
            Err(error) => Err(error),
        }
    }

    fn wait_for_heartbeat(
        bus: &SocketCan,
        driver: &mut Driver,
        now_ms: impl Fn() -> u64,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let deadline_ms = now_ms().saturating_add(TIMEOUT.as_millis() as u64);
        loop {
            let now = now_ms();
            if now >= deadline_ms {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "timed out waiting for a new ODrive Heartbeat; no frame was sent",
                )
                .into());
            }
            receive_or_wait(bus, driver, &now_ms)?;
            if let Some(entry) = driver
                .cache()
                .get(ResponseKind::Heartbeat)
                .filter(|entry| entry.received_at_ms < deadline_ms)
            {
                println!(
                    "observed Heartbeat at {} ms: {:?}",
                    entry.received_at_ms, entry.response
                );
                return Ok(());
            }
        }
    }

    fn run_operation(
        bus: &SocketCan,
        driver: &mut Driver,
        id: OperationId,
        is_query: bool,
        now_ms: impl Fn() -> u64,
    ) -> Result<(), Box<dyn std::error::Error>> {
        loop {
            driver.tick(now_ms()).map_err(core_error)?;
            match driver.report(id).map_err(core_error)?.state {
                OperationState::Prepared => match bus.send(driver, id, &now_ms) {
                    Ok(()) => continue,
                    Err(SocketCanSendError::Io(error))
                        if error.kind() == io::ErrorKind::WouldBlock =>
                    {
                        thread::sleep(IDLE_WAIT);
                    }
                    Err(error) => {
                        // The report on the next iteration distinguishes Failed from Unknown.
                        eprintln!("local SocketCAN send result: {error}");
                        thread::sleep(IDLE_WAIT);
                    }
                },
                OperationState::Submitted if !is_query => {
                    let report = driver.take_report(id).map_err(core_error)?;
                    println!(
                        "locally Submitted {:?} at {:?} ms; this is not an ODrive ACK",
                        report.kind, report.submitted_at_ms
                    );
                    return Ok(());
                }
                OperationState::Submitted => receive_or_wait(bus, driver, &now_ms)?,
                OperationState::Observed => {
                    let report = driver.take_report(id).map_err(core_error)?;
                    println!(
                        "Observed {:?} at {:?} ms: {:?}",
                        report.kind, report.terminal_at_ms, report.response
                    );
                    return Ok(());
                }
                OperationState::TimedOut => {
                    let report = driver.take_report(id).map_err(core_error)?;
                    return Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        format!(
                            "operation timed out (submitted_at_ms={:?}, deadline_ms={})",
                            report.submitted_at_ms, report.deadline_ms
                        ),
                    )
                    .into());
                }
                OperationState::Unknown => {
                    let report = driver.report(id).map_err(core_error)?;
                    return Err(io::Error::other(format!(
                        "operation result Unknown (dispatching_at_ms={:?}, submitted_at_ms={:?}); do not retry automatically",
                        report.dispatching_at_ms, report.submitted_at_ms,
                    ))
                    .into());
                }
                OperationState::Failed | OperationState::Cancelled => {
                    let report = driver.take_report(id).map_err(core_error)?;
                    return Err(io::Error::other(format!(
                        "operation ended locally as {:?}",
                        report.state
                    ))
                    .into());
                }
                OperationState::Dispatching => {
                    unreachable!("the send guard is not retained by this loop")
                }
            }
        }
    }

    let mut arguments = env::args().skip(1);
    let interface = arguments.next().ok_or_else(usage)?;
    let node_raw = arguments.next().ok_or_else(usage)?;
    let operation = arguments.next().ok_or_else(usage)?;
    let node = NodeId::new(node_raw.parse()?).map_err(core_error)?;
    let action = parse_action(operation, &mut arguments)?;

    let started = Instant::now();
    let now_ms = || u64::try_from(started.elapsed().as_millis()).unwrap_or(u64::MAX);
    let bus = SocketCan::open(&interface)?;
    // 只修改本 CLI 描述符，使内核错误帧能走到上面的异常处理分支。
    bus.socket().set_error_filter(ERR_MASK_ALL)?;
    let mut driver = Driver::new(node);

    match action {
        Action::Heartbeat => wait_for_heartbeat(&bus, &mut driver, now_ms),
        Action::Query(query) => {
            let id = driver
                .prepare_query(
                    query,
                    now_ms(),
                    now_ms().saturating_add(TIMEOUT.as_millis() as u64),
                )
                .map_err(core_error)?;
            run_operation(&bus, &mut driver, id, true, now_ms)
        }
        Action::Command(command) => {
            let id = driver
                .prepare_command(
                    command,
                    now_ms(),
                    now_ms().saturating_add(TIMEOUT.as_millis() as u64),
                )
                .map_err(core_error)?;
            run_operation(&bus, &mut driver, id, false, now_ms)
        }
    }
}
