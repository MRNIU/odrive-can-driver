// Copyright The odrive-can-driver Contributors
// 本文件是 H723 专用台架固件：编排板级 FDCAN、USB CDC 与实板诊断，
// 不向发布库引入板型、节点或机械策略。
#![no_std]
#![no_main]

//! H723 专用 ODrive CAN 台架，通过 CDC 显式执行查询、状态、故障与运动场景。
//!
//! 输出区分本地提交、真实 CAN 反馈和软件注入；Heartbeat 不作为命令 ACK。

use embassy_executor::Spawner;
use embassy_futures::{
    join::join,
    select::{Either, select},
};
use embassy_stm32::{bind_interrupts, can, peripherals, rcc, time::Hertz, usb};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embassy_usb::{
    Builder,
    class::cdc_acm::{CdcAcmClass, State},
};
use odrive_can_driver::{
    Driver, IngestResult, OperationState, ResponseKind,
    embassy::{self, EmbassyReceive, EmbassyTransmit},
    protocol::{self, AxisState, Command, FrameRef, Message, NodeId, Query, Response},
};
use panic_halt as _;

type Cdc<'d> = CdcAcmClass<'d, usb::Driver<'d, peripherals::USB_OTG_HS>>;

bind_interrupts!(struct Fdcan1Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
});

bind_interrupts!(struct Fdcan2Irqs {
    FDCAN2_IT0 => can::IT0InterruptHandler<peripherals::FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<peripherals::FDCAN2>;
});

bind_interrupts!(struct UsbIrqs {
    OTG_HS => usb::InterruptHandler<peripherals::USB_OTG_HS>;
});

const NODE: NodeId = match NodeId::new(1) {
    Ok(node) => node,
    Err(_) => panic!("fixed ODrive node is valid"),
};
const ABSENT_TEST_NODE: NodeId = match NodeId::new(63) {
    Ok(node) => node,
    Err(_) => panic!("fixed absent test node is valid"),
};
const COMMAND_TIMEOUT_MS: u64 = 300;
const MOTION_SPEED_TURNS_PER_S: f32 = 0.5;
const MOTION_HOLD_MS: u64 = 500;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(board_config());

    // FDCAN1 is a passive host/CANable observation path. It does not inject any
    // traffic into the external bus.
    let mut host_can = can::CanConfigurator::new(p.FDCAN1, p.PD0, p.PD1, Fdcan1Irqs);
    host_can.set_bitrate(1_000_000);
    let _host_tap = host_can.start(can::OperatingMode::BusMonitoringMode);

    let mut odrive_can = can::CanConfigurator::new(p.FDCAN2, p.PB12, p.PB13, Fdcan2Irqs);
    odrive_can.set_config(
        odrive_can
            .config()
            .set_tx_buffer_mode(can::config::TxBufferMode::Fifo),
    );
    odrive_can.set_bitrate(1_000_000);
    #[cfg(feature = "internal-loopback")]
    let mut odrive_can = odrive_can.into_internal_loopback_mode();
    #[cfg(not(feature = "internal-loopback"))]
    let mut odrive_can = odrive_can.into_normal_mode();

    let mut ep_out = [0; 128];
    let mut otg_config = usb::Config::default();
    // The board's PA9 VBUS-sense availability is not a precondition for this
    // bench. Enumeration itself is separately reported by the host.
    otg_config.vbus_detection = false;
    let usb_driver = usb::Driver::new_fs(
        p.USB_OTG_HS,
        p.PA12,
        p.PA11,
        UsbIrqs,
        &mut ep_out,
        otg_config,
    );

    let mut usb_config = embassy_usb::Config::new(0x1209, 0x7230);
    usb_config.manufacturer = Some("MRNIU");
    usb_config.product = Some("H723 ODrive CAN bench");
    usb_config.serial_number = Some("h723-can-bench");
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 64];
    let mut control = [0; 64];
    let mut cdc_state = State::new();
    let mut builder = Builder::new(
        usb_driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control,
    );
    let mut cdc = CdcAcmClass::new(&mut builder, &mut cdc_state, 64);
    let mut device = builder.build();

    let device_fut = device.run();
    let console_fut = async {
        let mut driver = Driver::new(NODE);
        loop {
            cdc.wait_connection().await;
            if write_line(&mut cdc, b"BOOT: ping timer raw loopback status idle fault absent motion +/- recover inject ...\r\n").await.is_err() {
                continue;
            }
            let _ = console_session(&mut cdc, &mut odrive_can, &mut driver).await;
        }
    };
    join(device_fut, console_fut).await;
}

fn board_config() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(rcc::Hse {
        freq: Hertz(25_000_000),
        mode: rcc::HseMode::Oscillator,
    });
    config.rcc.voltage_scale = rcc::VoltageScale::Scale0;
    config.rcc.pll1 = Some(rcc::Pll {
        source: rcc::PllSource::Hse,
        prediv: rcc::PllPreDiv::Div5,
        mul: rcc::PllMul::Mul104,
        divp: Some(rcc::PllDiv::Div1),
        divq: None,
        divr: None,
    });
    config.rcc.sys = rcc::Sysclk::Pll1P;
    config.rcc.d1c_pre = rcc::AHBPrescaler::Div1;
    config.rcc.ahb_pre = rcc::AHBPrescaler::Div2;
    config.rcc.apb1_pre = rcc::APBPrescaler::Div2;
    config.rcc.apb2_pre = rcc::APBPrescaler::Div2;
    config.rcc.apb3_pre = rcc::APBPrescaler::Div2;
    config.rcc.apb4_pre = rcc::APBPrescaler::Div2;
    config.rcc.pll2 = Some(rcc::Pll {
        source: rcc::PllSource::Hse,
        prediv: rcc::PllPreDiv::Div25,
        mul: rcc::PllMul::Mul400,
        divp: Some(rcc::PllDiv::Div8),
        divq: Some(rcc::PllDiv::Div5),
        divr: None,
    });
    config.rcc.mux.fdcansel = rcc::mux::Fdcansel::Pll2Q;
    config.rcc.hsi48 = Some(rcc::Hsi48Config {
        sync_from_usb: true,
    });
    config.rcc.mux.usbsel = rcc::mux::Usbsel::Hsi48;
    config
}

async fn console_session<'d>(
    cdc: &mut Cdc<'d>,
    can: &mut can::Can<'_>,
    driver: &mut Driver,
) -> Result<(), embassy_usb::driver::EndpointError> {
    let mut input = [0; 64];
    let mut background_rx = 0u32;
    loop {
        let received = select(
            embassy::receive_with_timestamp(can, driver, |envelope| envelope.ts.as_millis()),
            cdc.read_packet(&mut input),
        )
        .await;
        let count = match received {
            Either::Second(Ok(count)) => count,
            Either::Second(Err(error)) => return Err(error),
            // Keep FDCAN2 serviced while the host is idle. This exclusive
            // bench has no other FDCAN2 consumer; the driver still owns
            // protocol classification and cache updates.
            Either::First(Ok(_)) => {
                background_rx = background_rx.saturating_add(1);
                continue;
            }
            Either::First(Err(_)) => continue,
        };
        let line = trim(&input[..count]);
        if background_rx != 0 {
            write_u32_line(
                cdc,
                b"RX_SERVICE: background FDCAN frames=",
                background_rx,
                b"\r\n",
            )
            .await?;
            background_rx = 0;
        }
        match line {
            b"ping" => write_line(cdc, b"SOFTWARE: CDC command path alive\r\n").await?,
            b"timer" => {
                Timer::after(Duration::from_millis(100)).await;
                write_line(cdc, b"SOFTWARE: 100 ms timer elapsed\r\n").await?;
            }
            b"raw" => raw_receive(cdc, can, driver).await?,
            b"loopback" => loopback_probe(cdc, can, driver).await?,
            b"status" => status(cdc, can, driver).await?,
            b"idle" => idle(cdc, can, driver).await?,
            b"fault" => physical_fault_recovery(cdc, can, driver).await?,
            b"absent" => absent_node_query(cdc, can).await?,
            b"motion +" => motion(cdc, can, driver, MOTION_SPEED_TURNS_PER_S).await?,
            b"motion -" => motion(cdc, can, driver, -MOTION_SPEED_TURNS_PER_S).await?,
            b"recover" => recover(cdc, can, driver).await?,
            b"inject stale" => injected_stale(cdc, driver).await?,
            b"inject deviceerror" => injected_device_error(cdc, driver).await?,
            b"inject noresponse" => injected_no_response(cdc, driver).await?,
            _ => write_line(cdc, b"ERROR: ping timer raw loopback status idle fault absent motion +/- recover inject ...\r\n").await?,
        }
    }
}

async fn status<'d>(
    cdc: &mut Cdc<'d>,
    can: &mut can::Can<'_>,
    driver: &mut Driver,
) -> Result<(), embassy_usb::driver::EndpointError> {
    write_line(cdc, b"LOCAL: preparing RTR Vbus query\r\n").await?;
    if let Some(id) = submit_query(can, driver, Query::VbusVoltage).await {
        match await_response(can, driver, id, Some(cdc)).await {
            Some(Response::VbusVoltage(value)) => write_vbus(cdc, value).await?,
            _ => {
                write_line(
                    cdc,
                    b"NO_RESPONSE: Vbus query locally submitted; no matching CAN response\r\n",
                )
                .await?
            }
        }
    } else {
        write_line(
            cdc,
            b"LOCAL_FAILURE: query not retained by FDCAN TX path\r\n",
        )
        .await?;
    }
    Ok(())
}

async fn idle<'d>(
    cdc: &mut Cdc<'d>,
    can: &mut can::Can<'_>,
    driver: &mut Driver,
) -> Result<(), embassy_usb::driver::EndpointError> {
    if submit_command(
        can,
        driver,
        Command::SetAxisRequestedState {
            state: AxisState::IDLE,
        },
    )
    .await
    {
        if wait_for_state(can, driver, AxisState::IDLE).await {
            write_u32_line(
                cdc,
                b"IDLE_STATE: axis_state=0x",
                AxisState::IDLE.0,
                b" error=0\r\n",
            )
            .await?;
            write_line(
                cdc,
                b"PHYSICAL: Idle command submitted and a new Idle heartbeat observed\r\n",
            )
            .await?;
        } else {
            write_line(
                cdc,
                b"STOP_UNKNOWN: Idle locally submitted but no new Idle heartbeat\r\n",
            )
            .await?;
        }
    } else {
        write_line(
            cdc,
            b"STOP_UNKNOWN: Idle was not confirmed by FDCAN TX path\r\n",
        )
        .await?;
    }
    Ok(())
}

async fn motion<'d>(
    cdc: &mut Cdc<'d>,
    can: &mut can::Can<'_>,
    driver: &mut Driver,
    velocity: f32,
) -> Result<(), embassy_usb::driver::EndpointError> {
    write_line(
        cdc,
        b"BENCH: explicit 0.5 turn/s, 500 ms, zero torque feed-forward\r\n",
    )
    .await?;
    // Clear a retained velocity demand while the axis is still Idle before
    // requesting ClosedLoop. A failure remains Unknown and blocks motion.
    if !submit_command(
        can,
        driver,
        Command::SetInputVel {
            velocity: 0.0,
            torque_ff: 0.0,
        },
    )
    .await
    {
        write_line(
            cdc,
            b"STOP_UNKNOWN: initial zero velocity not confirmed; host Fibre fallback required\r\n",
        )
        .await?;
        return Ok(());
    }
    let Some(start_position) = encoder_position(can, driver).await else {
        write_line(
            cdc,
            b"STOP: initial encoder estimate not observed; no motion command sent\r\n",
        )
        .await?;
        return Ok(());
    };
    let closed = submit_command(
        can,
        driver,
        Command::SetAxisRequestedState {
            state: AxisState::CLOSED_LOOP_CONTROL,
        },
    )
    .await
        && wait_for_state(can, driver, AxisState::CLOSED_LOOP_CONTROL).await;
    if !closed {
        write_line(cdc, b"STOP_UNKNOWN: closed-loop state not physically observed; host USB fallback required\r\n").await?;
        return Ok(());
    }
    if !submit_command(
        can,
        driver,
        Command::SetInputVel {
            velocity,
            torque_ff: 0.0,
        },
    )
    .await
    {
        write_line(
            cdc,
            b"STOP_UNKNOWN: velocity submission unconfirmed; host USB fallback required\r\n",
        )
        .await?;
        return Ok(());
    }
    // No USB CDC write is permitted between nonzero velocity submission and
    // the bounded zero/Idle sequence: an unplugged or stalled host must not
    // stretch the requested 500 ms electrical hold.
    Timer::after(Duration::from_millis(MOTION_HOLD_MS)).await;
    let zero_ok = submit_command(
        can,
        driver,
        Command::SetInputVel {
            velocity: 0.0,
            torque_ff: 0.0,
        },
    )
    .await;
    let idle_ok = submit_command(
        can,
        driver,
        Command::SetAxisRequestedState {
            state: AxisState::IDLE,
        },
    )
    .await;
    if zero_ok && idle_ok && wait_for_state(can, driver, AxisState::IDLE).await {
        if let Some(end_position) = encoder_position(can, driver).await {
            write_u32_line(
                cdc,
                b"MOTION_START_BITS: 0x",
                start_position.to_bits(),
                b"\r\n",
            )
            .await?;
            write_u32_line(cdc, b"MOTION_END_BITS: 0x", end_position.to_bits(), b"\r\n").await?;
            write_u32_line(
                cdc,
                b"MOTION_DELTA_BITS: 0x",
                (end_position - start_position).to_bits(),
                b"\r\n",
            )
            .await?;
            if (end_position - start_position).abs() >= 0.005
                && (end_position - start_position) * velocity > 0.0
            {
                write_line(
                    cdc,
                    b"PHYSICAL: Idle heartbeat and encoder position delta observed\r\n",
                )
                .await?;
            } else {
                write_line(cdc, b"NO_MOTION_EVIDENCE: Idle observed but signed encoder delta did not meet 0.005 turn\r\n").await?;
            }
        } else {
            write_line(
                cdc,
                b"STOP_UNKNOWN: Idle observed but final encoder estimate missing\r\n",
            )
            .await?;
        }
    } else {
        write_line(
            cdc,
            b"STOP_UNKNOWN: final Idle not physically observed; host USB fallback required\r\n",
        )
        .await?;
    }
    Ok(())
}

async fn physical_fault_recovery<'d>(
    cdc: &mut Cdc<'d>,
    can: &mut can::Can<'_>,
    driver: &mut Driver,
) -> Result<(), embassy_usb::driver::EndpointError> {
    write_line(
        cdc,
        b"PHYSICAL_TEST: requesting CAN Estop, then observing error heartbeat\r\n",
    )
    .await?;
    if !submit_command(can, driver, Command::Estop).await {
        write_line(
            cdc,
            b"STOP_UNKNOWN: Estop was not confirmed by FDCAN TX path\r\n",
        )
        .await?;
        return Ok(());
    }
    let Some(axis_error) = wait_for_estop(can, driver).await else {
        write_line(
            cdc,
            b"STOP_UNKNOWN: Estop error heartbeat missing; host USB fallback required\r\n",
        )
        .await?;
        return Ok(());
    };
    write_error(cdc, b"PHYSICAL: Estop heartbeat axis_error=0x", axis_error).await?;
    let clear = submit_command(can, driver, Command::ClearErrors).await;
    let idle = submit_command(
        can,
        driver,
        Command::SetAxisRequestedState {
            state: AxisState::IDLE,
        },
    )
    .await;
    if clear && idle && wait_for_state(can, driver, AxisState::IDLE).await {
        write_line(
            cdc,
            b"PHYSICAL: Estop error heartbeat then ClearErrors and Idle heartbeat observed\r\n",
        )
        .await?;
    } else {
        write_line(
            cdc,
            b"STOP_UNKNOWN: recovery heartbeat missing; host USB fallback required\r\n",
        )
        .await?;
    }
    Ok(())
}

async fn absent_node_query<'d>(
    cdc: &mut Cdc<'d>,
    can: &mut can::Can<'_>,
) -> Result<(), embassy_usb::driver::EndpointError> {
    let mut absent = Driver::new(ABSENT_TEST_NODE);
    write_line(
        cdc,
        b"LOCAL: RTR Vbus query sent to node 63, expected absent\r\n",
    )
    .await?;
    if let Some(id) = submit_query(can, &mut absent, Query::VbusVoltage).await {
        if await_response(can, &mut absent, id, None).await.is_some() {
            write_line(
                cdc,
                b"UNEXPECTED: node 63 replied; it is not an absent-node test result\r\n",
            )
            .await?;
        } else {
            write_line(
                cdc,
                b"ABSENT_TARGET: node 63 query locally submitted and timed out\r\n",
            )
            .await?;
        }
    } else {
        write_line(
            cdc,
            b"LOCAL_FAILURE: node 63 query not retained by FDCAN TX path\r\n",
        )
        .await?;
    }
    Ok(())
}

async fn recover<'d>(
    cdc: &mut Cdc<'d>,
    can: &mut can::Can<'_>,
    driver: &mut Driver,
) -> Result<(), embassy_usb::driver::EndpointError> {
    let clear = submit_command(can, driver, Command::ClearErrors).await;
    let stop = submit_command(
        can,
        driver,
        Command::SetAxisRequestedState {
            state: AxisState::IDLE,
        },
    )
    .await;
    if clear && stop && wait_for_state(can, driver, AxisState::IDLE).await {
        write_line(
            cdc,
            b"RECOVERY: ClearErrors and Idle submitted; Idle heartbeat observed\r\n",
        )
        .await?;
    } else {
        write_line(
            cdc,
            b"RECOVERY_UNKNOWN: completion not observed; host USB fallback required\r\n",
        )
        .await?;
    }
    Ok(())
}

async fn submit_command(can: &mut can::Can<'_>, driver: &mut Driver, command: Command) -> bool {
    let now = now_ms();
    let deadline = now + COMMAND_TIMEOUT_MS;
    let Ok(id) = driver.prepare_command(command, now, deadline) else {
        return false;
    };
    let remaining = deadline.saturating_sub(now_ms());
    if remaining == 0 {
        let _ = driver.cancel(id, now_ms());
        let _ = driver.take_report(id);
        return false;
    }
    match with_timeout(
        Duration::from_millis(remaining),
        embassy::transmit(can, driver, id, || now_ms()),
    )
    .await
    {
        Ok(Ok(EmbassyTransmit::Submitted { displaced: None })) => {
            matches!(driver.take_report(id), Ok(report) if report.state == OperationState::Submitted)
        }
        // Fifo mode and this single owner make displacement unreachable in a
        // valid bench run. The raw frame is deliberately surfaced by the
        // backend, and any occurrence aborts the scene instead of discarding
        // a command and continuing with an invented queue state.
        Ok(Ok(EmbassyTransmit::Submitted { displaced: Some(_) })) => {
            let _ = driver.take_report(id);
            false
        }
        // Either condition leaves the active operation Unknown. It remains
        // deliberately occupied; caller reports STOP_UNKNOWN and the host
        // takes the separately labelled USB fallback path.
        Ok(Ok(EmbassyTransmit::Uncertain { .. })) | Err(_) => false,
        Ok(Err(_)) => {
            let _ = driver.take_report(id);
            false
        }
    }
}

async fn submit_query(
    can: &mut can::Can<'_>,
    driver: &mut Driver,
    query: Query,
) -> Option<odrive_can_driver::OperationId> {
    // 应用独占此测试总线。发送快速 RTR 查询前，逐帧服务可能在上一场景等待期间
    // 到达的三槽 FIFO；帧照常进入共享核心，不复位、清空或重建 CAN 外设。
    for _ in 0..3 {
        if with_timeout(
            Duration::from_millis(1),
            embassy::receive_with_timestamp(can, driver, |envelope| envelope.ts.as_millis()),
        )
        .await
        .is_err()
        {
            break;
        }
    }
    let now = now_ms();
    let deadline = now + COMMAND_TIMEOUT_MS;
    let Ok(id) = driver.prepare_query(query, now, deadline) else {
        return None;
    };
    let remaining = deadline.saturating_sub(now_ms());
    if remaining == 0 {
        let _ = driver.cancel(id, now_ms());
        let _ = driver.take_report(id);
        return None;
    }
    match with_timeout(
        Duration::from_millis(remaining),
        embassy::transmit(can, driver, id, || now_ms()),
    )
    .await
    {
        Ok(Ok(EmbassyTransmit::Submitted { displaced: None })) => Some(id),
        Ok(Ok(EmbassyTransmit::Submitted { displaced: Some(_) })) => {
            let _ = driver.take_report(id);
            None
        }
        Ok(Ok(EmbassyTransmit::Uncertain { .. })) | Err(_) => None,
        Ok(Err(_)) => {
            let _ = driver.take_report(id);
            None
        }
    }
}

async fn await_response(
    can: &mut can::Can<'_>,
    driver: &mut Driver,
    id: odrive_can_driver::OperationId,
    mut trace: Option<&mut Cdc<'_>>,
) -> Option<Response> {
    let deadline = driver.report(id).ok()?.deadline_ms;
    while now_ms() < deadline {
        let remaining = deadline.saturating_sub(now_ms());
        let mut received_at = 0;
        if let Ok(Ok(EmbassyReceive::Frame {
            frame,
            classification,
        })) = with_timeout(
            Duration::from_millis(remaining),
            embassy::receive_with_timestamp(can, driver, |envelope| {
                received_at = envelope.ts.as_millis();
                received_at
            }),
        )
        .await
        {
            if let Some(cdc) = trace.as_deref_mut() {
                let _ = write_raw_frame(cdc, &frame, classification).await;
                let _ =
                    write_u32_line(cdc, b"RX_TIMESTAMP_MS: 0x", received_at as u32, b"\r\n").await;
            }
            if matches!(driver.report(id), Ok(report) if report.state == OperationState::Observed) {
                return driver
                    .take_report(id)
                    .ok()
                    .and_then(|report| report.response);
            }
        }
        let _ = driver.tick(now_ms());
    }
    let _ = driver.tick(now_ms());
    let _ = driver.take_report(id);
    None
}

async fn encoder_position(can: &mut can::Can<'_>, driver: &mut Driver) -> Option<f32> {
    let id = submit_query(can, driver, Query::EncoderEstimates).await?;
    match await_response(can, driver, id, None).await {
        Some(Response::EncoderEstimates { position, .. }) => Some(position),
        _ => None,
    }
}

async fn wait_for_state(can: &mut can::Can<'_>, driver: &mut Driver, state: AxisState) -> bool {
    let boundary = now_ms();
    let deadline = now_ms() + COMMAND_TIMEOUT_MS;
    while now_ms() < deadline {
        let remaining = deadline.saturating_sub(now_ms());
        if let Ok(Ok(EmbassyReceive::Frame { .. })) = with_timeout(
            Duration::from_millis(remaining),
            embassy::receive_with_timestamp(can, driver, |envelope| envelope.ts.as_millis()),
        )
        .await
        {
            if let Some(cached) = driver.cache().get(ResponseKind::Heartbeat) {
                if cached.received_at_ms >= boundary {
                    if let Response::Heartbeat {
                        axis_error: 0,
                        axis_state,
                    } = cached.response
                    {
                        if axis_state == state {
                            return true;
                        }
                    }
                }
            }
        }
    }
    false
}

async fn wait_for_estop(can: &mut can::Can<'_>, driver: &mut Driver) -> Option<u32> {
    let boundary = now_ms();
    let deadline = now_ms() + COMMAND_TIMEOUT_MS;
    while now_ms() < deadline {
        let remaining = deadline.saturating_sub(now_ms());
        if let Ok(Ok(EmbassyReceive::Frame { .. })) = with_timeout(
            Duration::from_millis(remaining),
            embassy::receive_with_timestamp(can, driver, |envelope| envelope.ts.as_millis()),
        )
        .await
        {
            if let Some(cached) = driver.cache().get(ResponseKind::Heartbeat) {
                if cached.received_at_ms >= boundary
                    && matches!(cached.response, Response::Heartbeat { axis_error, .. } if axis_error & 0x4000 != 0)
                {
                    if let Response::Heartbeat { axis_error, .. } = cached.response {
                        return Some(axis_error);
                    }
                }
            }
        }
    }
    None
}

async fn raw_receive<'d>(
    cdc: &mut Cdc<'d>,
    can: &mut can::Can<'_>,
    driver: &mut Driver,
) -> Result<(), embassy_usb::driver::EndpointError> {
    write_line(cdc, b"LOCAL: draining up to three raw FDCAN2 RX frames\r\n").await?;
    for _ in 0..3 {
        match with_timeout(
            Duration::from_millis(COMMAND_TIMEOUT_MS),
            embassy::receive_with_timestamp(can, driver, |envelope| envelope.ts.as_millis()),
        )
        .await
        {
            Ok(Ok(EmbassyReceive::Frame {
                frame,
                classification,
            })) => write_raw_frame(cdc, &frame, classification).await?,
            Ok(Ok(EmbassyReceive::Invalid { .. })) => {
                write_line(
                    cdc,
                    b"RAW: invalid protocol view; native FDCAN frame retained\r\n",
                )
                .await?
            }
            Ok(Err(_)) => write_line(cdc, b"RAW_ERROR: FDCAN receive error\r\n").await?,
            Err(_) => {
                write_line(cdc, b"RAW_TIMEOUT: no further FDCAN frame\r\n").await?;
                break;
            }
        }
    }
    write_line(cdc, b"RAW_DONE: bounded raw receive finished\r\n").await?;
    Ok(())
}

async fn loopback_probe<'d>(
    cdc: &mut Cdc<'d>,
    can: &mut can::Can<'_>,
    driver: &mut Driver,
) -> Result<(), embassy_usb::driver::EndpointError> {
    #[cfg(feature = "internal-loopback")]
    {
        if submit_command(
            can,
            driver,
            Command::SetInputVel {
                velocity: 0.0,
                torque_ff: 0.0,
            },
        )
        .await
        {
            match with_timeout(
                Duration::from_millis(COMMAND_TIMEOUT_MS),
                embassy::receive_with_timestamp(can, driver, |envelope| envelope.ts.as_millis()),
            )
            .await
            {
                Ok(Ok(EmbassyReceive::Frame {
                    frame,
                    classification,
                })) => {
                    write_raw_frame(cdc, &frame, classification).await?;
                    write_line(cdc, b"LOOPBACK: controller/library TX and RX path observed; no external device evidence\r\n").await?;
                }
                _ => write_line(cdc, b"LOOPBACK_FAILURE: no received local frame\r\n").await?,
            }
        } else {
            write_line(
                cdc,
                b"LOOPBACK_FAILURE: local transmit was not submitted\r\n",
            )
            .await?;
        }
    }
    #[cfg(not(feature = "internal-loopback"))]
    {
        let _ = (can, driver);
        write_line(
            cdc,
            b"CONFIG_ERROR: loopback command requires internal-loopback image\r\n",
        )
        .await?;
    }
    Ok(())
}

async fn write_vbus<'d>(
    cdc: &mut Cdc<'d>,
    value: f32,
) -> Result<(), embassy_usb::driver::EndpointError> {
    // The raw IEEE-754 bits keep the board log allocation-free and preserve
    // the exact device value for host-side decoding.
    write_u32_line(
        cdc,
        b"PHYSICAL: Vbus response bits=0x",
        value.to_bits(),
        b"\r\n",
    )
    .await
}

async fn write_error<'d>(
    cdc: &mut Cdc<'d>,
    prefix: &[u8],
    error: u32,
) -> Result<(), embassy_usb::driver::EndpointError> {
    write_u32_line(cdc, prefix, error, b"\r\n").await
}

async fn write_raw_frame<'d>(
    cdc: &mut Cdc<'d>,
    frame: &can::frame::FdFrame,
    classification: IngestResult,
) -> Result<(), embassy_usb::driver::EndpointError> {
    use embedded_can::Id;
    let raw_id = match frame.id() {
        Id::Standard(id) => id.as_raw() as u32,
        Id::Extended(id) => id.as_raw(),
    };
    let class: &[u8] = match classification {
        IngestResult::Message(_) => b"MESSAGE",
        IngestResult::Unrelated => b"UNRELATED",
        IngestResult::DecodeError(_) => b"DECODE_ERROR",
    };
    let mut line = [0u8; 192];
    let mut used = copy_bytes(&mut line, 0, b"RAW: id=0x");
    used = push_hex(&mut line, used, raw_id, 8);
    used = copy_bytes(&mut line, used, b" dlc=");
    used = push_decimal(&mut line, used, frame.data().len() as u32);
    used = copy_bytes(
        &mut line,
        used,
        if frame.header().fdcan() {
            b" fd=1"
        } else {
            b" fd=0"
        },
    );
    used = copy_bytes(
        &mut line,
        used,
        if frame.header().rtr() {
            b" rtr=1"
        } else {
            b" rtr=0"
        },
    );
    used = copy_bytes(&mut line, used, b" class=");
    used = copy_bytes(&mut line, used, class);
    used = copy_bytes(&mut line, used, b" data=");
    for byte in frame.data() {
        used = push_hex(&mut line, used, *byte as u32, 2);
    }
    used = copy_bytes(&mut line, used, b"\r\n");
    write_line(cdc, &line[..used]).await
}

async fn write_u32_line<'d>(
    cdc: &mut Cdc<'d>,
    prefix: &[u8],
    value: u32,
    suffix: &[u8],
) -> Result<(), embassy_usb::driver::EndpointError> {
    let mut line = [0u8; 80];
    let mut used = copy_bytes(&mut line, 0, prefix);
    used = push_hex(&mut line, used, value, 8);
    used = copy_bytes(&mut line, used, suffix);
    write_line(cdc, &line[..used]).await
}

fn copy_bytes(out: &mut [u8], at: usize, input: &[u8]) -> usize {
    out[at..at + input.len()].copy_from_slice(input);
    at + input.len()
}
fn push_hex(out: &mut [u8], mut at: usize, value: u32, width: usize) -> usize {
    for shift in (0..width).rev() {
        out[at] = b"0123456789abcdef"[((value >> (shift * 4)) & 0xf) as usize];
        at += 1;
    }
    at
}
fn push_decimal(out: &mut [u8], mut at: usize, mut value: u32) -> usize {
    let start = at;
    loop {
        out[at] = b'0' + (value % 10) as u8;
        at += 1;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    out[start..at].reverse();
    at
}

async fn injected_stale<'d>(
    cdc: &mut Cdc<'d>,
    _driver: &mut Driver,
) -> Result<(), embassy_usb::driver::EndpointError> {
    let mut simulated = Driver::new(NODE);
    let id = simulated
        .prepare_query(Query::VbusVoltage, 10, 20)
        .expect("test operation prepares");
    let attempt = simulated.begin_send(id, 11).expect("test operation begins");
    attempt.submitted(11).expect("test operation submits");
    let encoded = protocol::encode(NODE, Message::Response(Response::VbusVoltage(48.0)))
        .expect("constant test response encodes");
    let frame: can::frame::Frame = (&encoded).into();
    if let Ok(frame_ref) = FrameRef::try_from(&frame) {
        let _ = simulated.ingest(frame_ref, 10);
    }
    simulated.tick(20).expect("test clock advances");
    let timed_out =
        matches!(simulated.take_report(id), Ok(report) if report.state == OperationState::TimedOut);
    write_line(
        cdc,
        if timed_out {
            b"SOFTWARE_INJECTION: stale response rejected, query timed out; no physical CAN\r\n"
        } else {
            b"SOFTWARE_INJECTION_FAILURE: stale scenario did not reach timeout\r\n"
        },
    )
    .await
}

async fn injected_device_error<'d>(
    cdc: &mut Cdc<'d>,
    _driver: &mut Driver,
) -> Result<(), embassy_usb::driver::EndpointError> {
    let mut simulated = Driver::new(NODE);
    let encoded = protocol::encode(
        NODE,
        Message::Response(Response::Heartbeat {
            axis_error: 1,
            axis_state: AxisState::IDLE,
        }),
    )
    .expect("constant test response encodes");
    let frame: can::frame::Frame = (&encoded).into();
    if let Ok(frame_ref) = FrameRef::try_from(&frame) {
        let _ = simulated.ingest(frame_ref, 10);
    }
    let cached_error = matches!(simulated.cache().get(ResponseKind::Heartbeat), Some(entry) if matches!(entry.response, Response::Heartbeat { axis_error: 1, .. }));
    write_line(
        cdc,
        if cached_error {
            b"SOFTWARE_INJECTION: device-error heartbeat cached; no physical CAN\r\n"
        } else {
            b"SOFTWARE_INJECTION_FAILURE: device-error heartbeat not cached\r\n"
        },
    )
    .await
}

async fn injected_no_response<'d>(
    cdc: &mut Cdc<'d>,
    _driver: &mut Driver,
) -> Result<(), embassy_usb::driver::EndpointError> {
    let mut simulated = Driver::new(NODE);
    let id = simulated
        .prepare_query(Query::VbusVoltage, 10, 20)
        .expect("test operation prepares");
    let attempt = simulated.begin_send(id, 11).expect("test operation begins");
    attempt.submitted(11).expect("test operation submits");
    simulated.tick(20).expect("test clock advances");
    let timed_out =
        matches!(simulated.take_report(id), Ok(report) if report.state == OperationState::TimedOut);
    write_line(
        cdc,
        if timed_out {
            b"SOFTWARE_INJECTION: no-response timeout simulated; no physical CAN\r\n"
        } else {
            b"SOFTWARE_INJECTION_FAILURE: no-response did not time out\r\n"
        },
    )
    .await
}

async fn write_line<'d>(
    cdc: &mut Cdc<'d>,
    mut data: &[u8],
) -> Result<(), embassy_usb::driver::EndpointError> {
    // CDC ACM full-speed bulk packets are at most 64 B. Logging must not make
    // a completed hardware result disappear merely because its text is long.
    while !data.is_empty() {
        let count = data.len().min(64);
        cdc.write_packet(&data[..count]).await?;
        data = &data[count..];
    }
    Ok(())
}

fn trim(mut data: &[u8]) -> &[u8] {
    while matches!(data.last(), Some(b'\r' | b'\n' | b' ')) {
        data = &data[..data.len() - 1];
    }
    data
}

fn now_ms() -> u64 {
    Instant::now().as_millis()
}
