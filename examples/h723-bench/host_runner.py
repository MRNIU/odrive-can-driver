#!/usr/bin/env python3
# Copyright The odrive-can-driver Contributors
# 本文件是 H723 台架的一次性主机运行器：读取 bench CDC，并只在异常或
# motion/fault 收束时以 Fibre 作一次 ODrive Idle 边缘读回。
"""One-shot USB CDC runner for the H723 CAN bench.

The runner reads only the bench CDC port. It does not poll ODrive during a
motion command. Every scene
ends by sending `idle`, regardless of whether the preceding USB text was
complete; that is a second, host-to-H723 request and not proof that ODrive
received or executed the H723 CAN Idle command.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import sys
import threading
import time

import serial
import serial.tools.list_ports


def discover_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    matches = [port.device for port in serial.tools.list_ports.comports() if "H723 ODrive CAN bench" in (port.description or "")]
    if len(matches) != 1:
        raise RuntimeError(f"expected exactly one H723 bench CDC port, found {matches!r}; pass --port")
    return matches[0]


def wait_boot(port: serial.Serial, wait_s: float = 2.0) -> None:
    # macOS does not reliably enable this CDC IN endpoint until it sees one
    # outbound packet. An empty line is harmless: the bench reports its boot
    # banner before parsing the console command.
    port.write(b"\n")
    port.flush()
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", errors="replace").rstrip()
        if line:
            print(f"boot: {line}")
            if line.startswith(("BOOT:", "ERROR:")):
                return
    raise RuntimeError("H723 CDC boot banner not received")


def transact(port: serial.Serial, command: str, terminal: tuple[str, ...], wait_s: float) -> tuple[list[str], bool]:
    port.write((command + "\n").encode("ascii"))
    port.flush()
    deadline = time.monotonic() + wait_s
    lines: list[str] = []
    while time.monotonic() < deadline:
        line = port.readline()
        if line:
            text = line.decode("ascii", errors="replace").rstrip()
            lines.append(text)
            print(f"{command}: {text}")
            if text.startswith(terminal):
                return lines, True
            if text.startswith((
                "STOP_UNKNOWN:",
                "RECOVERY_UNKNOWN:",
                "LOCAL_FAILURE:",
                "UNEXPECTED:",
                "NO_MOTION_EVIDENCE:",
                "NO_RESPONSE:",
                "LOOPBACK_FAILURE:",
                "CONFIG_ERROR:",
                "RAW_ERROR:",
                "SOFTWARE_INJECTION_FAILURE:",
                "ERROR:",
            )):
                return lines, False
    return lines, False


@contextmanager
def odrive_session(serial_number: str):
    """End the legacy Fibre workers before Python tears down libusb."""
    try:
        import odrive
        from fibre.utils import Event
    except ImportError as error:
        raise RuntimeError("Fibre checks require odrive==0.5.1.post0") from error
    termination = Event()
    existing_threads = set(threading.enumerate())
    try:
        device = odrive.find_any(
            serial_number=serial_number,
            timeout=5,
            search_cancellation_token=termination,
            channel_termination_token=termination,
        )
        if device is None:
            raise RuntimeError("ODrive Fibre discovery timed out")
        yield device
    finally:
        termination.set()
        # This single-threaded runner creates no other workers in the scope.
        # Fibre 0.5.1 exposes cancellation but no join handle; its discovery and
        # receiver threads otherwise survive into native USB finalization.
        workers = set(threading.enumerate()) - existing_threads
        deadline = time.monotonic() + 3
        for worker in workers:
            worker.join(max(0, deadline - time.monotonic()))
        if any(worker.is_alive() for worker in workers):
            raise RuntimeError("Fibre workers did not terminate; USB cleanup incomplete")


def odrive_idle_and_edge_read(serial_number: str) -> bool:
    """One Fibre Idle write plus one state/error read; never loop or poll."""
    with odrive_session(serial_number) as device:
        device.axis0.requested_state = 1
        state = device.axis0.current_state
        error = device.axis0.error
        idle_error_free = state == 1 and error == 0
        print(
            "FIBRE_FALLBACK: axis0 Idle requested; "
            f"edge current_state={state}; axis_error=0x{error:08x}; "
            f"idle_error_free={idle_error_free}"
        )
        return idle_error_free


def odrive_motion_preflight(serial_number: str) -> bool:
    """Read the explicit motion envelope once; never write configuration."""
    with odrive_session(serial_number) as device:
        axis = device.axis0
        current_lim = axis.motor.config.current_lim
        vel_limit = axis.controller.config.vel_limit
        watchdog_enabled = axis.config.enable_watchdog
        watchdog_timeout = axis.config.watchdog_timeout
        control_mode = axis.controller.config.control_mode
        input_mode = axis.controller.config.input_mode
        state = axis.current_state
        axis_error = axis.error
        motor_calibrated = axis.motor.is_calibrated
        encoder_ready = axis.encoder.is_ready

        accepted = (
            0 < current_lim <= 4
            and 0.5 <= vel_limit <= 1
            and watchdog_enabled
            and 1 <= watchdog_timeout <= 3
            and control_mode == 2
            and input_mode == 1
            and state == 1
            and axis_error == 0
            and motor_calibrated
            and encoder_ready
        )
        print(
            "FIBRE_PREFLIGHT: "
            f"current_lim={current_lim}; vel_limit={vel_limit}; "
            f"watchdog_enabled={watchdog_enabled}; watchdog_timeout={watchdog_timeout}; "
            f"control_mode={control_mode}; input_mode={input_mode}; "
            f"current_state={state}; axis_error=0x{axis_error:08x}; "
            f"motor_calibrated={motor_calibrated}; encoder_ready={encoder_ready}; "
            f"accepted={accepted}"
        )
        return accepted


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "scenes",
        nargs="+",
        help="一个或多个场景，按给定顺序在同一 CDC 会话中执行",
        choices=[
            "ping", "timer", "raw", "loopback", "fault", "absent", "status",
            "motion+", "motion-", "recover",
            "inject-stale", "inject-deviceerror", "inject-noresponse",
        ],
    )
    parser.add_argument("--port", help="CDC device path; autodetection matches the bench USB product")
    parser.add_argument("--odrive-serial", help="ODrive Fibre USB serial; required for fault and motion")
    parser.add_argument("--baud", type=int, default=115200, help="CDC terminal setting; USB CDC ignores line rate")
    args = parser.parse_args()
    if any(scene == "fault" or scene.startswith("motion") for scene in args.scenes) and not args.odrive_serial:
        parser.error("fault and motion require an explicit --odrive-serial")

    commands = {
        "motion+": "motion +",
        "motion-": "motion -",
        "inject-stale": "inject stale",
        "inject-deviceerror": "inject deviceerror",
        "inject-noresponse": "inject noresponse",
    }
    port_name = discover_port(args.port)
    terminals = {
        "ping": ("SOFTWARE: CDC command path alive",),
        "timer": ("SOFTWARE: 100 ms timer elapsed",),
        "raw": ("RAW_DONE: bounded raw receive finished",),
        "loopback": ("LOOPBACK: controller/library TX and RX path observed",),
        "fault": ("PHYSICAL: Estop error heartbeat then ClearErrors and Idle heartbeat observed",),
        "absent": ("ABSENT_TARGET: node 63 query locally submitted and timed out",),
        "status": ("PHYSICAL: Vbus response bits=0x",),
        "motion+": ("PHYSICAL: Idle heartbeat and encoder position delta observed",),
        "motion-": ("PHYSICAL: Idle heartbeat and encoder position delta observed",),
        "recover": ("RECOVERY: ClearErrors and Idle submitted; Idle heartbeat observed",),
        "inject-stale": ("SOFTWARE_INJECTION: stale response rejected, query timed out; no physical CAN",),
        "inject-deviceerror": ("SOFTWARE_INJECTION: device-error heartbeat cached; no physical CAN",),
        "inject-noresponse": ("SOFTWARE_INJECTION: no-response timeout simulated; no physical CAN",),
    }
    control_only = {
        "ping", "timer", "raw", "loopback",
        "inject-stale", "inject-deviceerror", "inject-noresponse",
    }
    lines: list[str] = []
    idle_lines: list[str] = []
    scenes_ok = True
    idle_ok = False
    with serial.Serial(port_name, args.baud, timeout=0.1, write_timeout=1) as port:
        # The bench only accepts commands after it reports the CDC session is
        # live. This prevents a buffered command being mistaken for a result
        # from a preceding connection.
        wait_boot(port)
        try:
            for scene in args.scenes:
                if scene.startswith("motion"):
                    try:
                        if not odrive_motion_preflight(args.odrive_serial):
                            print("FIBRE_PREFLIGHT_FAILURE: motion envelope rejected", file=sys.stderr)
                            scenes_ok = False
                            break
                    except Exception as error:
                        print(f"FIBRE_PREFLIGHT_FAILURE: {error}", file=sys.stderr)
                        scenes_ok = False
                        break
                command = commands.get(scene, scene)
                scene_lines, scene_ok = transact(port, command, terminals[scene], 2.0)
                lines.extend(scene_lines)
                if not scene_ok:
                    scenes_ok = False
                    break
        finally:
            # Controller-only and injected scenes never address the ODrive.
            # Any sequence containing an ODrive scene receives one final,
            # independently observed H723 CAN Idle request.
            if all(scene in control_only for scene in args.scenes):
                idle_ok = True
            else:
                idle_lines, idle_ok = transact(
                    port,
                    "idle",
                    ("PHYSICAL: Idle command submitted and a new Idle heartbeat observed",),
                    1.0,
                )

    abnormal = not scenes_ok or not idle_ok
    # Estop can be locally accepted before a later CAN failure; run the USB
    # fallback for a fault scene as well as for every motion scene. A normal
    # motion still obtains only one edge read after its CAN Idle-HB proof.
    if any(scene.startswith("motion") or scene == "fault" for scene in args.scenes):
        try:
            # This is a final edge read on normal completion and an immediate
            # physical USB fallback on any missing/abnormal CAN result. It is
            # explicitly not evidence that CAN Idle was delivered.
            usb_idle_ok = odrive_idle_and_edge_read(args.odrive_serial)
            if not usb_idle_ok:
                abnormal = True
        except Exception as error:
            print(f"USB_FALLBACK_FAILURE: {error}", file=sys.stderr)
            abnormal = True
    if abnormal:
        return 2
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, serial.SerialException) as error:
        print(f"runner error: {error}", file=sys.stderr)
        raise SystemExit(2)
