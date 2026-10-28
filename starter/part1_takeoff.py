"""Part 1: connect, wait until ready, switch to GUIDED, arm, take off to 15 m.

Every step confirms its outcome from COMMAND_ACK and/or telemetry, with a
timeout. Any failure prints a clear error and exits non-zero.
"""

import sys
import time

from pymavlink import mavutil

CONNECTION_STRING = "tcp:127.0.0.1:5760"
TARGET_ALT_M = 15.0
ALT_TOLERANCE_M = 0.5

M = mavutil.mavlink

# EKF must report a good attitude, velocity and absolute position before a
# guided takeoff is possible.
EKF_REQUIRED = (
    M.EKF_ATTITUDE
    | M.EKF_VELOCITY_HORIZ
    | M.EKF_VELOCITY_VERT
    | M.EKF_POS_HORIZ_ABS
    | M.EKF_POS_VERT_ABS
)


class StepError(Exception):
    """A step was rejected or timed out."""


class Link:
    """Thin wrapper that pumps messages and tracks the latest vehicle state."""

    def __init__(self, conn: mavutil.mavfile) -> None:
        self.conn = conn
        self.custom_mode: int | None = None
        self.armed = False
        self.rel_alt_m: float | None = None

    def recv(self, timeout: float):
        """Return the next message (or None), updating tracked state."""
        msg = self.conn.recv_match(blocking=True, timeout=timeout)
        if msg is None:
            return None
        mtype = msg.get_type()
        if mtype == "HEARTBEAT":
            # Only trust heartbeats from the autopilot itself (not GCSes etc.).
            if (
                msg.get_srcSystem() == self.conn.target_system
                and msg.autopilot != M.MAV_AUTOPILOT_INVALID
            ):
                self.custom_mode = msg.custom_mode
                self.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
        elif mtype == "GLOBAL_POSITION_INT":
            self.rel_alt_m = msg.relative_alt / 1000.0
        elif mtype == "STATUSTEXT":
            print(f"  [autopilot] {msg.text}")
        return msg

    def wait_for(self, predicate, timeout: float, what: str):
        """Pump messages until predicate(msg) is truthy; raise on timeout."""
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            msg = self.recv(min(remaining, 1.0))
            if msg is not None and predicate(msg):
                return msg
        raise StepError(f"timed out after {timeout:.0f}s waiting for {what}")

    def command(self, cmd: int, *params: float, timeout: float = 5.0, retries: int = 3):
        """Send COMMAND_LONG and return once it is ACCEPTED; raise otherwise.

        Resends if no ACK arrives (messages can be lost), using the
        confirmation field so the autopilot can tell it's a retransmission.
        """
        p = list(params) + [0.0] * (7 - len(params))
        name = M.enums["MAV_CMD"][cmd].name
        for confirmation in range(retries):
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                cmd, confirmation, *p,
            )
            try:
                ack = self.wait_for(
                    lambda m: m.get_type() == "COMMAND_ACK"
                    and m.command == cmd
                    and m.result != M.MAV_RESULT_IN_PROGRESS,
                    timeout, f"COMMAND_ACK for {name}",
                )
            except StepError:
                print(f"  no ACK for {name}, retrying ({confirmation + 1}/{retries})")
                continue
            if ack.result != M.MAV_RESULT_ACCEPTED:
                result = M.enums["MAV_RESULT"][ack.result].name
                raise StepError(f"{name} rejected: {result}")
            return ack
        raise StepError(f"no COMMAND_ACK for {name} after {retries} attempts")


def wait_until_ready(link: Link, timeout: float = 120.0) -> None:
    """Wait for a 3D GPS fix, a converged EKF and passing pre-arm checks."""
    print("Waiting for GPS fix, EKF convergence and pre-arm checks ...")
    gps_ok = ekf_ok = prearm_ok = False

    def ready(msg) -> bool:
        nonlocal gps_ok, ekf_ok, prearm_ok
        mtype = msg.get_type()
        if mtype == "GPS_RAW_INT":
            gps_ok = msg.fix_type >= M.GPS_FIX_TYPE_3D_FIX
        elif mtype == "EKF_STATUS_REPORT":
            ekf_ok = (msg.flags & EKF_REQUIRED) == EKF_REQUIRED and not (
                msg.flags & M.EKF_CONST_POS_MODE
            )
        elif mtype == "SYS_STATUS":
            bit = M.MAV_SYS_STATUS_PREARM_CHECK
            prearm_ok = bool(msg.onboard_control_sensors_health & bit)
        return gps_ok and ekf_ok and prearm_ok

    link.wait_for(ready, timeout, "vehicle ready (GPS fix + EKF + pre-arm)")
    print("Vehicle ready.")


def set_mode(link: Link, mode_name: str, timeout: float = 10.0) -> None:
    mapping = link.conn.mode_mapping()
    if mode_name not in mapping:
        raise StepError(f"unknown mode {mode_name!r}")
    mode_id = mapping[mode_name]
    print(f"Switching to {mode_name} ...")
    link.command(
        M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
    )
    # ACK only means "accepted" — confirm the heartbeat reports the new mode.
    link.wait_for(
        lambda m: m.get_type() == "HEARTBEAT" and link.custom_mode == mode_id,
        timeout, f"heartbeat to report {mode_name}",
    )
    print(f"Mode confirmed: {mode_name}")


def arm(link: Link, timeout: float = 10.0) -> None:
    print("Arming ...")
    link.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    print("  arm command ACCEPTED")
    link.wait_for(
        lambda m: m.get_type() == "HEARTBEAT" and link.armed,
        timeout, "heartbeat to report ARMED",
    )
    print("Armed confirmed by telemetry.")


def takeoff(link: Link, alt_m: float, guided_mode: int, timeout: float = 60.0) -> None:
    print(f"Taking off to {alt_m:.1f} m ...")
    link.command(M.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt_m)
    print("  takeoff command ACCEPTED")

    deadline = time.monotonic() + timeout
    last_print = 0.0
    while (remaining := deadline - time.monotonic()) > 0:
        msg = link.recv(min(remaining, 1.0))
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == "HEARTBEAT" and (not link.armed or link.custom_mode != guided_mode):
            raise StepError("vehicle disarmed or left GUIDED during climb")
        if mtype != "GLOBAL_POSITION_INT":
            continue
        alt = link.rel_alt_m
        if abs(alt - alt_m) <= ALT_TOLERANCE_M:
            print(f"  altitude {alt:6.2f} m")
            print("REACHED")
            return
        now = time.monotonic()
        if now - last_print >= 0.5:
            last_print = now
            print(f"  altitude {alt:6.2f} m")
    raise StepError(
        f"timed out after {timeout:.0f}s climbing to {alt_m} m "
        f"(last altitude {link.rel_alt_m})"
    )


def main() -> int:
    print(f"Connecting to {CONNECTION_STRING} ...")
    try:
        conn = mavutil.mavlink_connection(CONNECTION_STRING)
    except OSError as e:
        print(f"ERROR: could not connect: {e}")
        return 1

    try:
        hb = conn.wait_heartbeat(timeout=30)
        if hb is None:
            print("ERROR: no heartbeat. Is the simulator running? (docker compose up -d)")
            return 1
        print(f"Heartbeat from system {conn.target_system} component {conn.target_component}")

        # Ask the autopilot to stream telemetry at 4 Hz.
        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component,
            M.MAV_DATA_STREAM_ALL, 4, 1,
        )

        link = Link(conn)
        wait_until_ready(link)
        set_mode(link, "GUIDED")
        arm(link)
        takeoff(link, TARGET_ALT_M, conn.mode_mapping()["GUIDED"])
        return 0
    except StepError as e:
        print(f"ERROR: {e}")
        return 1
    except KeyboardInterrupt:
        print("Interrupted.")
        return 130
    finally:
        conn.close()


if __name__ == "__main__":
    sys.exit(main())
