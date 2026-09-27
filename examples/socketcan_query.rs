// Copyright The odrive-can-driver Contributors
//! 在 Linux SocketCAN 上发送一次 ODrive 母线电压 RTR 查询并报告本地结果。

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("socketcan_query only supports Linux SocketCAN");
    std::process::exit(2);
}

#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use std::{
        env, fmt, io, thread,
        time::{Duration, Instant},
    };

    use odrive_can_driver::{
        Driver, OperationState,
        protocol::{NodeId, Query, Response},
        socketcan::{SocketCan, SocketCanSendError},
    };

    const QUERY_TIMEOUT_MS: u64 = 1_000;
    const IDLE_WAIT: Duration = Duration::from_millis(1);

    fn core_error(error: impl fmt::Debug) -> io::Error {
        io::Error::other(format!("driver state error: {error:?}"))
    }

    let mut arguments = env::args().skip(1);
    let interface = arguments
        .next()
        .ok_or("usage: socketcan_query <interface> <node>")?;
    let node_raw = arguments
        .next()
        .ok_or("usage: socketcan_query <interface> <node>")?;
    if arguments.next().is_some() {
        return Err("usage: socketcan_query <interface> <node>".into());
    }
    let node = NodeId::new(node_raw.parse()?).map_err(core_error)?;

    let started = Instant::now();
    let now_ms = || u64::try_from(started.elapsed().as_millis()).unwrap_or(u64::MAX);
    let mut driver = Driver::new(node);
    let deadline_ms = now_ms().saturating_add(QUERY_TIMEOUT_MS);
    let id = driver
        .prepare_query(Query::VbusVoltage, now_ms(), deadline_ms)
        .map_err(core_error)?;
    let bus = SocketCan::open(&interface)?;

    loop {
        driver.tick(now_ms()).map_err(core_error)?;
        match driver.report(id).map_err(core_error)?.state {
            OperationState::Prepared => match bus.send(&mut driver, id, &now_ms) {
                Ok(()) => continue,
                Err(SocketCanSendError::Io(error)) if error.kind() == io::ErrorKind::WouldBlock => {
                    thread::sleep(IDLE_WAIT);
                    continue;
                }
                Err(error) => {
                    eprintln!("local SocketCAN send result: {error}");
                    continue;
                }
            },
            OperationState::Submitted => match bus.receive(&mut driver, now_ms) {
                Ok(received) => {
                    if let Err(error) = received.classification {
                        eprintln!("SocketCAN error notification: {error}");
                    }
                }
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => thread::sleep(IDLE_WAIT),
                Err(error) => return Err(error.into()),
            },
            OperationState::Observed => {
                let report = driver.take_report(id).map_err(core_error)?;
                match report.response {
                    Some(Response::VbusVoltage(volts)) => {
                        println!("observed VbusVoltage: {volts:.3} V");
                    }
                    response => println!("observed query response: {response:?}"),
                }
                return Ok(());
            }
            OperationState::TimedOut => {
                let report = driver.take_report(id).map_err(core_error)?;
                let message = format!(
                    "timed out waiting for VbusVoltage after {} ms (submitted_at_ms={:?})",
                    report.deadline_ms.saturating_sub(report.prepared_at_ms),
                    report.submitted_at_ms,
                );
                println!("{message}");
                return Err(io::Error::other(message).into());
            }
            OperationState::Unknown => {
                let report = driver.report(id).map_err(core_error)?;
                let message = format!(
                    "query result unknown (dispatching_at_ms={:?}, submitted_at_ms={:?}); do not retry automatically",
                    report.dispatching_at_ms, report.submitted_at_ms,
                );
                println!("{message}");
                return Err(io::Error::other(message).into());
            }
            OperationState::Failed | OperationState::Cancelled => {
                let report = driver.take_report(id).map_err(core_error)?;
                let message = format!("query ended locally as {:?}", report.state);
                println!("{message}");
                return Err(io::Error::other(message).into());
            }
            OperationState::Dispatching => {
                unreachable!("send attempt is not retained by this loop")
            }
        }
    }
}
