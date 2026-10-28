"""Part 3: fly the 80 m square with a battery failsafe monitor.

While the square is flown, SYS_STATUS battery voltage is checked on every
message. If it drops below 11.0 V the mission is aborted immediately: goto
commands stop, RTL is commanded (and confirmed), and the abort reason is
logged with a timestamp. The script then waits for landing + disarm.

To exercise the failsafe, ~20 s into the square the script itself sets the
simulator parameter SIM_BATT_VOLTAGE to 10.5 via PARAM_SET and confirms it
from the PARAM_VALUE echo.

Everything runs in a single loop: no threads, no asyncio. Messages are
pumped with short timeouts and every pending action (goto resend, parameter
set/retry, battery check) is serviced from that loop.
"""

import math
import sys
import time
from datetime import datetime

from pymavlink import mavutil

CONNECTION_STRING = "tcp:127.0.0.1:5760"
TARGET_ALT_M = 15.0
ALT_TOLERANCE_M = 0.5

SIDE_M = 80.0
CORNER_TOLERANCE_M = 2.0
SQUARE_TIMEOUT_S = 240.0
RTL_TIMEOUT_S = 180.0
EARTH_RADIUS_M = 6378137.0

BATT_ABORT_V = 11.0
FAULT_PARAM = "SIM_BATT_VOLTAGE"
FAULT_VALUE_V = 10.5
FAULT_AT_S = 20.0  # seconds after the square starts
PARAM_RETRY_S = 1.0
PARAM_MAX_ATTEMPTS = 5

M = mavutil.mavlink

# Position-only setpoint: ignore velocity, acceleration, yaw and yaw rate.
POSITION_ONLY_MASK = (
    M.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | M.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | M.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | M.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | M.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | M.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | M.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | M.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

# EKF must report a good attitude, velocity and absolute position before a
# guided takeoff is possible.
EKF_REQUIRED = (
    M.EKF_ATTITUDE
    | M.EKF_VELOCITY_HORIZ
    | M.EKF_VELOCITY_VERT
    | M.EKF_POS_HORIZ_ABS
    | M.EKF_POS_VERT_ABS
)


def log(text: str) -> None:
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{ts}] {text}", flush=True)


class StepError(Exception):
    """A step was rejected or timed out."""


class Link:
    """Thin wrapper that pumps messages and tracks the latest vehicle state."""

    def __init__(self, conn: mavutil.mavfile) -> None:
        self.conn = conn
        self.custom_mode: int | None = None
        self.armed = False
        self.rel_alt_m: float | None = None
        self.voltage_v: float | None = None

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
        elif mtype == "SYS_STATUS":
            # voltage_battery is mV; UINT16_MAX means "unknown".
            if 0 < msg.voltage_battery < 0xFFFF:
                self.voltage_v = msg.voltage_battery / 1000.0
        elif mtype == "STATUSTEXT":
            log(f"  [autopilot] {msg.text}")
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
                log(f"  no ACK for {name}, retrying ({confirmation + 1}/{retries})")
                continue
            if ack.result != M.MAV_RESULT_ACCEPTED:
                result = M.enums["MAV_RESULT"][ack.result].name
                raise StepError(f"{name} rejected: {result}")
            return ack
        raise StepError(f"no COMMAND_ACK for {name} after {retries} attempts")


class ParamSetter:
    """Non-blocking PARAM_SET with retry, confirmed by the PARAM_VALUE echo.

    Call tick() from the main loop to (re)send, and feed every PARAM_VALUE
    to handle(). This keeps the mission loop (and battery monitor) running
    while we wait for the autopilot to confirm.
    """

    def __init__(self, link: Link, name: str, value: float) -> None:
        self.link = link
        self.name = name
        self.value = value
        self.attempts = 0
        self.last_sent: float | None = None
        self.confirmed = False

    def tick(self, now: float) -> None:
        if self.confirmed:
            return
        if self.last_sent is not None and now - self.last_sent < PARAM_RETRY_S:
            return
        if self.attempts >= PARAM_MAX_ATTEMPTS:
            raise StepError(
                f"{self.name} not confirmed after {PARAM_MAX_ATTEMPTS} PARAM_SET attempts"
            )
        self.attempts += 1
        self.last_sent = now
        log(f"FAULT INJECTION: PARAM_SET {self.name}={self.value} "
            f"(attempt {self.attempts}/{PARAM_MAX_ATTEMPTS})")
        self.link.conn.mav.param_set_send(
            self.link.conn.target_system,
            self.link.conn.target_component,
            self.name.encode(),
            self.value,
            M.MAV_PARAM_TYPE_REAL32,
        )

    def handle(self, msg) -> None:
        if self.confirmed or self.last_sent is None or msg.param_id != self.name:
            return
        if abs(msg.param_value - self.value) < 1e-3:
            self.confirmed = True
            log(f"FAULT INJECTION: confirmed {self.name}={msg.param_value:.2f} "
                f"by PARAM_VALUE")
        else:
            # Echo shows the old value; let tick() resend.
            log(f"  PARAM_VALUE {self.name}={msg.param_value:.2f} (not yet applied)")


def wait_until_ready(link: Link, timeout: float = 120.0) -> None:
    """Wait for a 3D GPS fix, a converged EKF and passing pre-arm checks."""
    log("Waiting for GPS fix, EKF convergence and pre-arm checks ...")
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
        return gps_ok and ekf_ok and prearm_ok and link.voltage_v is not None

    link.wait_for(ready, timeout, "vehicle ready (GPS fix + EKF + pre-arm)")
    log(f"Vehicle ready. Battery {link.voltage_v:.2f} V.")
    if link.voltage_v < BATT_ABORT_V:
        raise StepError(
            f"battery already {link.voltage_v:.2f} V (< {BATT_ABORT_V} V) before takeoff; "
            "reset the simulator (docker compose restart)"
        )


def set_mode(link: Link, mode_name: str, timeout: float = 10.0) -> None:
    mapping = link.conn.mode_mapping()
    if mode_name not in mapping:
        raise StepError(f"unknown mode {mode_name!r}")
    mode_id = mapping[mode_name]
    log(f"Switching to {mode_name} ...")
    link.command(
        M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
    )
    # ACK only means "accepted" — confirm the heartbeat reports the new mode.
    link.wait_for(
        lambda m: m.get_type() == "HEARTBEAT" and link.custom_mode == mode_id,
        timeout, f"heartbeat to report {mode_name}",
    )
    log(f"Mode confirmed: {mode_name}")


def arm(link: Link, timeout: float = 10.0) -> None:
    log("Arming ...")
    link.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    log("  arm command ACCEPTED")
    link.wait_for(
        lambda m: m.get_type() == "HEARTBEAT" and link.armed,
        timeout, "heartbeat to report ARMED",
    )
    log("Armed confirmed by telemetry.")


def takeoff(link: Link, alt_m: float, guided_mode: int, timeout: float = 60.0) -> None:
    log(f"Taking off to {alt_m:.1f} m ...")
    link.command(M.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt_m)
    log("  takeoff command ACCEPTED")

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
            log(f"  altitude {alt:6.2f} m")
            log("REACHED")
            return
        now = time.monotonic()
        if now - last_print >= 1.0:
            last_print = now
            log(f"  altitude {alt:6.2f} m")
    raise StepError(
        f"timed out after {timeout:.0f}s climbing to {alt_m} m "
        f"(last altitude {link.rel_alt_m})"
    )


def offset(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Shift a lat/lon (degrees) by metres north/east (flat-earth, fine at 80 m)."""
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Horizontal distance between two lat/lon points (haversine)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def send_position_target(link: Link, lat: float, lon: float, alt_m: float) -> None:
    link.conn.mav.set_position_target_global_int_send(
        0,  # time_boot_ms (ignored)
        link.conn.target_system,
        link.conn.target_component,
        M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        POSITION_ONLY_MASK,
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
        0, 0, 0,  # velocity
        0, 0, 0,  # acceleration
        0, 0,  # yaw, yaw rate
    )


def fly_square_monitored(link: Link, alt_m: float, guided_mode: int) -> str | None:
    """Fly the square while monitoring battery voltage.

    Returns None if the square completed, or the abort reason if the battery
    failsafe tripped. Goto commands are only sent from inside this loop, so
    returning stops them.
    """
    msg = link.wait_for(
        lambda m: m.get_type() == "GLOBAL_POSITION_INT", 5.0, "GLOBAL_POSITION_INT"
    )
    start_lat, start_lon = msg.lat / 1e7, msg.lon / 1e7
    corners = [
        ("corner 1 (N)", *offset(start_lat, start_lon, SIDE_M, 0.0)),
        ("corner 2 (NE)", *offset(start_lat, start_lon, SIDE_M, SIDE_M)),
        ("corner 3 (E)", *offset(start_lat, start_lon, 0.0, SIDE_M)),
        ("start corner", start_lat, start_lon),
    ]
    fault = ParamSetter(link, FAULT_PARAM, FAULT_VALUE_V)

    t0 = time.monotonic()
    log(f"MISSION START: 80 m square from ({start_lat:.7f}, {start_lon:.7f}); "
        f"battery abort threshold {BATT_ABORT_V} V; fault injection at T+{FAULT_AT_S:.0f}s")
    idx = 0
    name, lat, lon = corners[idx]
    log(f"T+{0.0:5.1f}s  heading to {name}")
    last_send = last_print = 0.0
    dist = None

    while True:
        now = time.monotonic()
        t = now - t0
        if t > SQUARE_TIMEOUT_S:
            raise StepError(f"square not finished after {SQUARE_TIMEOUT_S:.0f}s")

        if t >= FAULT_AT_S:
            fault.tick(now)

        # SET_POSITION_TARGET has no ACK; resend so a dropped packet can't stall us.
        if now - last_send >= 1.0:
            send_position_target(link, lat, lon, alt_m)
            last_send = now

        msg = link.recv(0.2)
        if msg is None:
            continue
        mtype = msg.get_type()

        if mtype == "SYS_STATUS" and link.voltage_v is not None:
            if link.voltage_v < BATT_ABORT_V:
                where = "unknown" if dist is None else f"{dist:.1f} m"
                reason = (
                    f"battery voltage {link.voltage_v:.2f} V below {BATT_ABORT_V:.1f} V "
                    f"threshold at T+{t:.1f}s, en route to {name} (distance {where}), "
                    f"alt {link.rel_alt_m:.1f} m"
                )
                log(f"ABORT: {reason}")
                log("ABORT: goto commands stopped")
                return reason
        elif mtype == "PARAM_VALUE":
            fault.handle(msg)
        elif mtype == "HEARTBEAT":
            if not link.armed or link.custom_mode != guided_mode:
                raise StepError(f"vehicle disarmed or left GUIDED while flying to {name}")
        elif mtype == "GLOBAL_POSITION_INT":
            dist = distance_m(msg.lat / 1e7, msg.lon / 1e7, lat, lon)
            if dist <= CORNER_TOLERANCE_M:
                log(f"T+{t:5.1f}s  {name} reached ({dist:.1f} m)")
                idx += 1
                if idx == len(corners):
                    log(f"T+{t:5.1f}s  square complete, no battery abort")
                    return None
                name, lat, lon = corners[idx]
                log(f"T+{t:5.1f}s  heading to {name}")
                last_send = 0.0  # send the new target right away
                dist = None
            elif now - last_print >= 1.0:
                last_print = now
                log(f"T+{t:5.1f}s  {dist:6.1f} m to {name}  "
                    f"alt {link.rel_alt_m:5.1f} m  batt {link.voltage_v:5.2f} V")


def rtl_and_wait_for_disarm(link: Link) -> None:
    rtl_mode = link.conn.mode_mapping()["RTL"]
    set_mode(link, "RTL")
    log("Returning to launch; waiting for landing and disarm ...")
    deadline = time.monotonic() + RTL_TIMEOUT_S
    last_print = 0.0
    while (remaining := deadline - time.monotonic()) > 0:
        msg = link.recv(min(remaining, 1.0))
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == "HEARTBEAT":
            if not link.armed:
                log(f"Landed and disarmed (alt {link.rel_alt_m:.2f} m).")
                return
            if link.custom_mode != rtl_mode:
                raise StepError("vehicle left RTL before landing")
        elif mtype == "GLOBAL_POSITION_INT" and time.monotonic() - last_print >= 1.0:
            last_print = time.monotonic()
            log(f"  RTL  alt {link.rel_alt_m:5.1f} m  batt {link.voltage_v:5.2f} V")
    raise StepError(f"timed out after {RTL_TIMEOUT_S:.0f}s waiting for landing + disarm")


def main() -> int:
    log(f"Connecting to {CONNECTION_STRING} ...")
    try:
        conn = mavutil.mavlink_connection(CONNECTION_STRING)
    except OSError as e:
        log(f"ERROR: could not connect: {e}")
        return 1

    try:
        hb = conn.wait_heartbeat(timeout=30)
        if hb is None:
            log("ERROR: no heartbeat. Is the simulator running? (docker compose up -d)")
            return 1
        log(f"Heartbeat from system {conn.target_system} component {conn.target_component}")

        # Ask the autopilot to stream telemetry at 4 Hz.
        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component,
            M.MAV_DATA_STREAM_ALL, 4, 1,
        )

        link = Link(conn)
        guided = conn.mode_mapping()["GUIDED"]
        wait_until_ready(link)
        set_mode(link, "GUIDED")
        arm(link)
        takeoff(link, TARGET_ALT_M, guided)
        abort_reason = fly_square_monitored(link, TARGET_ALT_M, guided)
        rtl_and_wait_for_disarm(link)
        if abort_reason:
            log(f"SUMMARY: mission ABORTED by battery failsafe — {abort_reason}. "
                "Vehicle returned to launch and landed safely.")
        else:
            log("SUMMARY: mission completed; battery failsafe did not trigger.")
        return 0
    except StepError as e:
        log(f"ERROR: {e}")
        return 1
    except KeyboardInterrupt:
        log("Interrupted.")
        return 130
    finally:
        conn.close()


if __name__ == "__main__":
    sys.exit(main())
