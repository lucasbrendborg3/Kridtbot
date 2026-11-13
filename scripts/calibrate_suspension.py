#!/usr/bin/env python3
"""Home the suspension ball screws of kridtbot's four ODrives over CAN (bottom end stop only).

All selected legs at the same time (or one after another with --one-by-one):
  1. Drive slowly DOWN until the bottom end stop is felt (the torque rises clearly beyond what it
     takes to move the leg with the rover standing on it). Each leg stops at its own end stop while
     the others carry on. That position becomes 0: the ODrive's position estimate is reset so that
     the bottom end stop = 0 turns.
  2. Move all homed legs together to the same height (--park-mm, default 20 mm above the bottom stop;
     a negative value takes them back to where they started) and keep holding it (--idle-after
     switches them off instead). With --one-by-one each leg first returns to where it started.
If anything goes wrong on one leg, all legs stop; that leg goes idle, the others keep holding.

The top end stop is deliberately never searched for: driving into it could unscrew the ball screw.
The only upper limit is the position "max" in ros2_control_odrive.xacro.

Results are written to ~/.ros/kridtbot/suspension_limits.yaml (the bottom end stop and the margin;
the ball screw moves 5 mm per turn, matching the ros_odrive hardware interface). The hardware
interface uses bottom + --margin-mm as each leg's lower limit, and warns if a leg has none.

The ODrives count position from wherever the motor is at power-up, so the "bottom = 0" frame
only lasts until the ODrives lose power: run this script after every power-up.

Nothing else may use can_odrive while this runs (stop the kridtbot launch first).
Ctrl+C at any time switches every ODrive to idle.

Examples:
  ros2 run kridtbot calibrate_suspension                  # all four legs together
  ros2 run kridtbot calibrate_suspension --one-by-one     # the same, one leg after another
  ros2 run kridtbot calibrate_suspension --nodes 1        # only the left rear leg
"""

import argparse
import datetime
import os
import socket
import struct
import sys
import threading
import time

import yaml

MM_PER_TURN = 5.0
NODES = {0: "left_front_linear", 1: "left_rear_linear", 2: "right_rear_linear", 3: "right_front_linear"}
# Read by the ODrive hardware interface (limits_file in ros2_control_odrive.xacro)
DEFAULT_OUTPUT = os.path.expanduser("~/.ros/kridtbot/suspension_limits.yaml")

# ODrive CANSimple commands (arbitration ID = node_id << 5 | command)
HEARTBEAT, GET_ERROR, SET_AXIS_STATE, ENCODER_ESTIMATES = 0x01, 0x03, 0x07, 0x09
SET_CONTROLLER_MODE, SET_INPUT_POS, SET_INPUT_VEL = 0x0B, 0x0C, 0x0D
CLEAR_ERRORS, SET_ABSOLUTE_POSITION, GET_TORQUES = 0x18, 0x19, 0x1C
IDLE, CLOSED_LOOP = 1, 8
VELOCITY_CONTROL, POSITION_CONTROL, PASSTHROUGH = 2, 3, 1

DOWN = -1.0     # negative turns = down (the direction the rover's weight pushes)


class CalibrationError(Exception):
    def __init__(self, msg, node=None):
        super().__init__(msg)
        self.node = node        # the leg the problem is on, if known


class ODriveBus:
    """Sends ODrive commands and keeps the latest broadcast values of every node."""

    def __init__(self, iface):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.settimeout(0.1)
        self.latest = {}
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while self.running:
            try:
                cid, dlc, data = struct.unpack("=IB3x8s", self.sock.recv(16))
            except socket.timeout:
                continue
            cid &= 0x7FF
            with self.lock:
                self.latest[(cid >> 5, cid & 0x1F)] = (time.time(), data[:dlc])

    def send(self, node, cmd, payload=b""):
        self.sock.send(struct.pack("=IB3x8s", (node << 5) | cmd, len(payload), payload.ljust(8, b"\0")))

    def _get(self, node, cmd, max_age=0.5):
        with self.lock:
            entry = self.latest.get((node, cmd))
        if entry is None or time.time() - entry[0] > max_age:
            return None
        return entry[1]

    # --- readings -------------------------------------------------------------------------
    def position(self, node):
        d = self._get(node, ENCODER_ESTIMATES)
        return struct.unpack("<f", d[0:4])[0] if d else None

    def velocity(self, node):
        d = self._get(node, ENCODER_ESTIMATES)
        return struct.unpack("<f", d[4:8])[0] if d else None

    def torque(self, node):
        d = self._get(node, GET_TORQUES)
        return struct.unpack("<f", d[4:8])[0] if d and len(d) >= 8 else None

    def state(self, node):
        d = self._get(node, HEARTBEAT)
        return d[4] if d else None

    def errors(self, node):
        hb, err = self._get(node, HEARTBEAT), self._get(node, GET_ERROR, max_age=3.0)
        axis_error = struct.unpack("<I", hb[0:4])[0] if hb else None
        active, disarm = (struct.unpack("<II", err[0:8]) if err else (None, None))
        return axis_error, active, disarm

    # --- commands -------------------------------------------------------------------------
    def set_state(self, node, state):
        self.send(node, SET_AXIS_STATE, struct.pack("<I", state))

    def set_mode(self, node, control_mode):
        self.send(node, SET_CONTROLLER_MODE, struct.pack("<II", control_mode, PASSTHROUGH))

    def input_vel(self, node, vel):
        self.send(node, SET_INPUT_VEL, struct.pack("<ff", vel, 0.0))

    def input_pos(self, node, pos):
        self.send(node, SET_INPUT_POS, struct.pack("<fhh", pos, 0, 0))

    def clear_errors(self, node):
        self.send(node, CLEAR_ERRORS)

    def set_absolute_position(self, node, pos):
        self.send(node, SET_ABSOLUTE_POSITION, struct.pack("<f", pos))

    def idle_all(self, nodes):
        for node in nodes:
            self.set_state(node, IDLE)

    def close(self):
        self.running = False


def log(msg):
    print(f"[{datetime.datetime.now():%H:%M:%S}] {msg}", flush=True)


def wait_for(cond, timeout, what):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return
        time.sleep(0.01)
    raise CalibrationError(f"timed out waiting for {what}")


def check_armed(bus, node):
    if bus.state(node) != CLOSED_LOOP:
        axis_error, active, disarm = bus.errors(node)
        raise CalibrationError(f"ODrive disarmed (state {bus.state(node)}, axis_error 0x{axis_error or 0:08X}, "
                               f"active_errors 0x{active or 0:08X}, disarm_reason 0x{disarm or 0:08X})")


def enter_closed_loop(bus, node, control_mode):
    """Arm the ODrive holding still: position mode holds the current position, velocity mode holds 0."""
    pos = bus.position(node)
    bus.set_mode(node, control_mode)
    if control_mode == POSITION_CONTROL:
        bus.input_pos(node, pos)
    else:
        bus.input_vel(node, 0.0)
    bus.set_state(node, CLOSED_LOOP)
    wait_for(lambda: bus.state(node) == CLOSED_LOOP, 2.0, "closed loop control")


def hold_all(bus, nodes):
    """Hold every leg where it is right now (position control at its current position)."""
    for node in nodes:
        pos = bus.position(node)
        if pos is not None and bus.state(node) == CLOSED_LOOP:
            bus.set_mode(node, POSITION_CONTROL)
            bus.input_pos(node, pos)


def seek_bottom_stops(bus, nodes, args):
    """Drive the legs `nodes` down together at a slow speed; each leg stops (and backs off a little)
    at its own bottom end stop while the others carry on. Returns {node: (stop position, info)}.
    If anything goes wrong on any leg, every leg stops and holds where it is, and CalibrationError
    is raised.

    The end stop is detected from the torque, not the speed: with the rover's weight on the leg the
    ball screw moves in a stick-slip way (measured on the rover: stuck for up to 1 s at a time while
    the torque builds to ~-0.13 Nm lowering, then it jumps ahead), so a "stopped moving" rule would
    fire during normal movement. At the end stop the leg can't move at all, so the ODrive's speed
    controller keeps raising the torque well past that value."""
    direction = DOWN
    speed = args.speed
    log(f"seeking the bottom end stop of {'legs ' + str(list(nodes)) if len(nodes) > 1 else 'node ' + str(nodes[0])} "
        f"at {speed * MM_PER_TURN:.1f} mm/s (stop at {direction * args.stop_torque:+.2f} Nm) ...")
    for node in nodes:
        enter_closed_loop(bus, node, VELOCITY_CONTROL)

    t0 = time.time()
    legs = {n: {"start": bus.position(n), "torques": [], "peak": 0.0, "beyond_since": None,
                "progress_pos": bus.position(n), "progress_time": t0, "result": None} for n in nodes}
    try:
        while not all(leg["result"] for leg in legs.values()):
            elapsed = time.time() - t0
            for node, leg in legs.items():
                try:
                    check_armed(bus, node)
                    if leg["result"]:
                        continue
                    bus.input_vel(node, direction * speed * min(1.0, elapsed / 0.5))

                    pos, torque = bus.position(node), bus.torque(node)
                    if pos is None:
                        raise CalibrationError("lost encoder messages from the ODrive")
                    start = leg["start"]
                    if abs(pos - start) > args.max_travel_mm / MM_PER_TURN:
                        raise CalibrationError(f"no bottom end stop found within {args.max_travel_mm:.0f} mm - stopping for safety")
                    if (pos - start) * direction < -args.wrong_way_mm / MM_PER_TURN:
                        raise CalibrationError(f"leg moved {abs(pos - start) * MM_PER_TURN:.1f} mm the wrong way - check the motor wiring/calibration")

                    reason = None
                    if torque is not None:
                        along = torque * direction        # positive = pushing downwards
                        leg["peak"] = max(leg["peak"], along)
                        if along > args.max_torque:
                            reason = f"torque {torque:+.2f} Nm hit the {args.max_torque:.2f} Nm safety cap"
                        elif along > args.stop_torque:
                            leg["beyond_since"] = leg["beyond_since"] or time.time()
                            if time.time() - leg["beyond_since"] >= args.stop_confirm:
                                reason = f"torque {torque:+.2f} Nm stayed beyond {direction * args.stop_torque:+.2f} Nm"
                        else:
                            leg["beyond_since"] = None
                            if elapsed > 0.5:
                                leg["torques"].append(along)

                    # Last safety net: no real progress for a long time (normal sticking lasts up to ~1 s)
                    if (pos - leg["progress_pos"]) * direction > args.no_progress_mm / MM_PER_TURN:
                        leg["progress_pos"], leg["progress_time"] = pos, time.time()
                    elif time.time() - leg["progress_time"] > args.no_progress_time:
                        reason = reason or f"less than {args.no_progress_mm} mm of progress in {args.no_progress_time:.0f} s"

                    if reason:
                        # This leg is done: stop it, and hold it backed off a little so the motor isn't
                        # left pushing into the stop. The other legs carry on.
                        bus.input_vel(node, 0.0)
                        moving = sum(leg["torques"]) / len(leg["torques"]) if leg["torques"] else 0.0
                        bus.set_mode(node, POSITION_CONTROL)
                        bus.input_pos(node, pos - direction * args.backoff_mm / MM_PER_TURN)
                        leg["result"] = (pos, {"stop_reason": reason, "moving_torque_nm": round(moving, 3),
                                               "peak_torque_nm": round(leg["peak"], 3)})
                        log(f"  node {node} ({NODES[node]}): bottom end stop after {abs(pos - start) * MM_PER_TURN:.1f} mm: "
                            f"{reason} (moving needed {moving:.2f} Nm on average)")
                except CalibrationError as e:
                    raise CalibrationError(f"node {node} ({NODES[node]}): {e}", node=node) from e
            time.sleep(0.01)
    except BaseException:
        hold_all(bus, nodes)
        raise
    time.sleep(0.5)
    return {n: leg["result"] for n, leg in legs.items()}


def set_bottom_as_zero(bus, node, bottom):
    """Reset the ODrive's position estimate so that the bottom end stop reads 0 turns."""
    bus.set_state(node, IDLE)
    wait_for(lambda: bus.state(node) == IDLE, 2.0, "idle")
    new_pos = bus.position(node) - bottom
    bus.set_absolute_position(node, new_pos)
    wait_for(lambda: bus.position(node) is not None and abs(bus.position(node) - new_pos) < 0.05, 2.0,
             "the ODrive to accept the new zero position")
    return bus.position(node)


def leg_result(node, down_info, args, now):
    """What is saved for one homed leg. Replaces anything saved for it before (e.g. old top limits)."""
    return {"node_id": node, "hard_min_turns": 0.0, "hard_min_m": 0.0, "margin_mm": args.margin_mm,
            "down": down_info, "homed": now}


def move_all_to(bus, nodes, target, speed, max_torque):
    """Move several legs together, to the same height (target in turns above their bottom stop) or
    each to its own (target = {node: turns}). Each leg gets a linear ramp of the same duration, so
    they all arrive at the same moment."""
    targets = target if isinstance(target, dict) else {node: target for node in nodes}
    for node in nodes:
        enter_closed_loop(bus, node, POSITION_CONTROL)
    starts = {node: bus.position(node) for node in nodes}
    duration = max(abs(targets[n] - p) for n, p in starts.items()) / speed
    t0 = time.time()
    while True:
        frac = min(1.0, (time.time() - t0) / duration) if duration > 0 else 1.0
        for node in nodes:
            bus.input_pos(node, starts[node] + (targets[node] - starts[node]) * frac)
            check_armed(bus, node)
            torque = bus.torque(node)
            if torque is not None and abs(torque) > max_torque:
                for n in nodes:
                    bus.input_pos(n, bus.position(n))      # hold where they are
                raise CalibrationError(f"node {node} torque {torque:+.2f} Nm hit the {max_torque:.2f} Nm safety cap")
        if frac >= 1.0:
            break
        time.sleep(0.01)
    time.sleep(0.5)
    return {node: bus.position(node) for node in nodes}


def home_leg(bus, node, args):
    """Home one leg on its own, then take it back to where it started. Returns its result."""
    log(f"node {node} ({NODES[node]})")
    bus.clear_errors(node)
    time.sleep(0.2)
    start = bus.position(node)
    if start is None:
        raise CalibrationError("no encoder messages from this ODrive - is it powered and on the bus?")

    bottom, down_info = seek_bottom_stops(bus, [node], args)[node]
    start_from_bottom = start - bottom
    set_bottom_as_zero(bus, node, bottom)
    log("  position reset: bottom end stop = 0 turns")
    result = leg_result(node, down_info, args, datetime.datetime.now().isoformat(timespec="seconds"))

    # Back to where the leg started (at least the margin above the bottom), so the rover stays as
    # level as possible while the other legs are homed
    park = max(start_from_bottom, args.margin_mm / MM_PER_TURN)
    log(f"  back to {park * MM_PER_TURN:.1f} mm (where it started)")
    move_all_to(bus, [node], park, args.speed, args.max_torque)
    bus.set_state(node, IDLE)
    log("  done, ODrive idle")
    return result


def home_legs_together(bus, nodes, args):
    """Home several legs at the same time. Returns ({node: result}, {node: start height in turns})."""
    for node in nodes:
        bus.clear_errors(node)
    time.sleep(0.2)
    starts = {node: bus.position(node) for node in nodes}

    bottoms = seek_bottom_stops(bus, nodes, args)
    now = datetime.datetime.now().isoformat(timespec="seconds")
    for node in nodes:
        set_bottom_as_zero(bus, node, bottoms[node][0])
    log(f"positions reset: bottom end stop = 0 turns on legs {list(nodes)}")

    results = {node: leg_result(node, bottoms[node][1], args, now) for node in nodes}
    start_heights = {node: starts[node] - bottoms[node][0] for node in nodes}
    return results, start_heights


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--iface", default="can_odrive")
    parser.add_argument("--nodes", type=int, nargs="+", default=sorted(NODES), choices=sorted(NODES))
    parser.add_argument("--speed", type=float, default=0.5, help="seek speed in turns/s (default 0.5 = 2.5 mm/s)")
    parser.add_argument("--margin-mm", type=float, default=3.0, help="lower limit = this far above the bottom stop (default 3 mm)")
    # Torque defaults measured on the rover standing on the ground (node 2, 2.5 mm/s): lowering needs down
    # to -0.13 Nm; the ODrive's own current-limit trip was seen at ~1.9 Nm.
    parser.add_argument("--stop-torque", type=float, default=0.30, help="bottom end stop when the lowering torque stays below minus this (Nm, default 0.30)")
    parser.add_argument("--stop-confirm", type=float, default=0.15, help="the torque must stay beyond the stop torque this long (s, default 0.15)")
    parser.add_argument("--max-torque", type=float, default=0.8, help="stop immediately beyond this torque in either direction (Nm, default 0.8)")
    parser.add_argument("--no-progress-mm", type=float, default=0.5, help="safety net: stop if the leg moves less than this ... (mm, default 0.5)")
    parser.add_argument("--no-progress-time", type=float, default=3.0, help="... in this many seconds (default 3)")
    parser.add_argument("--wrong-way-mm", type=float, default=1.0, help="abort if the leg moves this far the wrong way (mm, default 1)")
    parser.add_argument("--backoff-mm", type=float, default=1.0, help="back off this far after touching the stop (default 1 mm)")
    parser.add_argument("--max-travel-mm", type=float, default=150.0, help="give up if no stop is found within this distance (default 150 mm)")
    parser.add_argument("--park-mm", type=float, default=20.0, help="at the end, move all homed legs together to this height above their bottom stop (mm, default 20; negative = leave them where they started)")
    parser.add_argument("--idle-after", action="store_true", help="switch the ODrives to idle after parking (default: keep holding the park height, since an idle leg can slowly sink under the rover's weight)")
    parser.add_argument("--output", default=os.path.normpath(DEFAULT_OUTPUT))
    parser.add_argument("--yes", action="store_true", help="don't ask for confirmation")
    parser.add_argument("--one-by-one", action="store_true", help="home the legs one after another (default: all at the same time)")
    # Ignore the ROS arguments that ros2 run / ros2 launch may append
    argv = sys.argv[1:]
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    args = parser.parse_args(argv)

    if os.popen("ps -eo args").read().count("lib/controller_manager/ros2_control_node") > 0:
        sys.exit("The kridtbot launch (ros2_control_node) is running and uses the ODrives - stop it first.")

    saved = {}
    if os.path.exists(args.output):
        with open(args.output) as f:
            saved = (yaml.safe_load(f) or {}).get("legs", {})

    bus = ODriveBus(args.iface)
    time.sleep(0.5)
    missing = [n for n in args.nodes if bus.position(n) is None]
    if missing:
        bus.close()
        sys.exit(f"No messages from ODrive node(s) {missing} on {args.iface}.")

    together = not args.one_by_one and len(args.nodes) > 1
    print(f"Homing suspension legs {args.nodes} on {args.iface}, "
          f"{'all at the same time' if together else 'one after another'}. Each leg moves slowly down to "
          f"its bottom stop. Ctrl+C stops everything.")
    results, failed = {}, {}
    holding = []
    start_heights = {}
    try:
        if together:
            if not args.yes and input(f"\nHome legs {args.nodes} together? [y/N] ").strip().lower() != "y":
                log("cancelled")
            else:
                try:
                    leg_results, start_heights = home_legs_together(bus, args.nodes, args)
                    results = {NODES[n]: r for n, r in leg_results.items()}
                except CalibrationError as e:
                    failed["together"] = str(e)
                    # The other legs keep holding where they stopped (an idle leg can drop under the
                    # rover's weight); the leg with the problem goes idle (it may be faulty).
                    holding = [n for n in args.nodes if n != e.node and bus.state(n) == CLOSED_LOOP]
                    log(f"FAILED: {e}")
                    log(f"  {'node ' + str(e.node) + ' set to idle; ' if e.node is not None else ''}"
                        f"legs {holding} holding where they stopped")
        else:
            for node in args.nodes:
                if not args.yes and input(f"\nHome node {node} ({NODES[node]})? [y/N] ").strip().lower() != "y":
                    log(f"node {node} skipped")
                    continue
                try:
                    results[NODES[node]] = home_leg(bus, node, args)
                except CalibrationError as e:
                    bus.set_state(node, IDLE)
                    failed[NODES[node]] = str(e)
                    log(f"  FAILED: {e} - ODrive set to idle")

        homed = [n for n in args.nodes if NODES[n] in results]
        lowest = args.margin_mm / MM_PER_TURN      # never park below the lower limit
        if homed and args.park_mm < 0 and start_heights:
            # Legs homed together: take them back to where each one started (at least the margin up)
            back = {n: max(start_heights[n], lowest) for n in homed}
            log(f"moving legs {homed} back to where they started ...")
            move_all_to(bus, homed, back, args.speed, args.max_torque)
            holding = [] if args.idle_after else homed
        elif homed and args.park_mm >= 0:
            target = args.park_mm / MM_PER_TURN
            if target < lowest:
                raise CalibrationError(f"park height {args.park_mm} mm is below the lower limit ({args.margin_mm} mm)")
            log(f"moving legs {homed} together to {args.park_mm:.1f} mm above their bottom stops ...")
            reached = move_all_to(bus, homed, target, args.speed, args.max_torque)
            for n, p in reached.items():
                log(f"  node {n} ({NODES[n]}) at {p * MM_PER_TURN:.1f} mm")
            holding = [] if args.idle_after else homed
    except CalibrationError as e:
        log(f"PARKING STOPPED: {e} - all ODrives set to idle")
        failed["park"] = str(e)
    except KeyboardInterrupt:
        log("Interrupted")
        holding = []
    finally:
        bus.idle_all([n for n in args.nodes if n not in holding])
        time.sleep(0.2)
        bus.close()
    if holding:
        log(f"legs {holding} are holding their position (ODrives stay in position control; "
            f"they go idle at power-off, or when the kridtbot launch takes over)")

    if results:
        saved.update(results)
        os.makedirs(os.path.dirname(args.output), exist_ok=True)
        with open(args.output, "w") as f:
            f.write("# Suspension bottom homing per leg, by: ros2 run kridtbot calibrate_suspension\n"
                    "# Positions are measured from the bottom end stop (0) upward; 5 mm per motor turn.\n"
                    "# Lower limit = hard_min_m + margin_mm. The top stop is never measured.\n"
                    "# Only valid while the legs have been homed since the ODrives last powered up\n"
                    "# (run it after every power-up). Read by the ODrive hardware interface.\n")
            yaml.safe_dump({"legs": saved}, f, sort_keys=True)
        log(f"saved {len(results)} leg(s) to {args.output}")
    for joint, err in failed.items():
        log(f"{joint}: {err}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
