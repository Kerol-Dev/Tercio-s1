import time
import math
from TercioStepperLib import Bridge, Stepper

# ==========================
# Config
# ==========================
BASE_ID = 1
AXIS1_ID = 2
AXIS2_ID = 3

MAX_RPS = 20.0
MAX_RPS2 = 100.0
CURRENT_MA = 2000


# ==========================
# Helpers
# ==========================

def wait_for_state(bridge: Bridge, stepper_ids, timeout=3.0):
    """Wait for telemetry for all IDs."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        states = [bridge.get_state(i) for i in stepper_ids]
        if all(s is not None for s in states):
            return states
        time.sleep(0.05)
    return [bridge.get_state(i) for i in stepper_ids]


def calc_min_time(dist, vmax, amax):
    """Time for trapezoidal or triangular profile."""
    if dist <= 0:
        return 0
    t_acc = vmax / amax
    d_acc = 0.5 * amax * t_acc * t_acc

    if dist < 2 * d_acc:
        return 2.0 * math.sqrt(dist / amax)
    else:
        return 2 * t_acc + (dist - 2 * d_acc) / vmax


def move_sync(bridge, steppers, target_angles, vmax, amax, min_dur=0.4):
    """Move all steppers synchronously to target angles."""
    ids = [s.id for s in steppers]
    states = wait_for_state(bridge, ids, timeout=1.0)
    if not all(states):
        print("⚠ No telemetry for one or more joints, skipping move")
        return

    start = [st.currentAngle for st in states]
    dists = [abs(t - s) for t, s in zip(target_angles, start)]

    durations = [calc_min_time(d, vmax, amax) for d in dists]
    move_time = max(max(durations), min_dur)

    steps = max(30, int(move_time / 0.02))
    dt = move_time / steps

    for i in range(1, steps+1):
        a = i / steps
        for idx, stp in enumerate(steppers):
            ang = start[idx] + (target_angles[idx] - start[idx]) * a
            stp.set_target_angle(ang)
        time.sleep(dt)


# ==========================
# Main Program
# ==========================

def main():
    bridge = Bridge()
    bridge.open()

    base = Stepper(bridge, BASE_ID)
    axis1 = Stepper(bridge, AXIS1_ID)
    axis2 = Stepper(bridge, AXIS2_ID)

    steppers = [base, axis1, axis2]

    # use radians
    for s in steppers:
        s.set_units_degrees(False)

    waypoints = []

    try:
        while True:
            print("\n==== 3-DOF WAYPOINT TOOL ====")
            print("1) Record waypoints (manual snapshots)")
            print("2) Playback waypoints")
            print("3) List waypoints")
            print("q) Quit")
            mode = input("> ").strip().lower()

            # ---------------------------
            # RECORD MODE (manual snapshot)
            # ---------------------------
            if mode == "1":
                print("\n--- RECORD MODE ---")
                print("Motors OFF. Move arm by hand.")
                print("Press ENTER to record a waypoint.")
                print("Type 'done' to finish.\n")

                for s in steppers:
                    s.enable_motor(False)

                while True:
                    cmd = input("Record waypoint? [Enter] or type 'done': ").strip().lower()
                    if cmd == "done":
                        break

                    states = wait_for_state(bridge, [s.id for s in steppers], timeout=2)
                    if not all(states):
                        print("⚠ Telemetry not ready, try again")
                        continue

                    wp = [st.currentAngle for st in states]
                    waypoints.append(wp)

                    print(f"✓ Recorded: base={wp[0]:+.3f}, axis1={wp[1]:+.3f}, axis2={wp[2]:+.3f}")

                print(f"Finished recording. Total: {len(waypoints)}")

            # ---------------------------
            # PLAYBACK MODE
            # ---------------------------
            elif mode == "2":
                if not waypoints:
                    print("No waypoints.")
                    continue

                print("\n--- PLAYBACK ---")
                print(f"Waypoints: {len(waypoints)}")
                print(f"Torque={CURRENT_MA} mA, speed={MAX_RPS} rps, accel={MAX_RPS2} rps²\n")

                # Enable and configure motors
                for s in steppers:
                    s.enable_motor(True)
                    s.set_current_ma(CURRENT_MA)
                    s.set_speed_limit_rps(MAX_RPS)
                    s.set_accel_limit_rps2(MAX_RPS2)

                input("Press ENTER to start...")

                for i, wp in enumerate(waypoints):
                    print(f"Moving to waypoint {i}: {wp}")
                    move_sync(bridge, steppers, wp,
                              vmax=MAX_RPS, amax=MAX_RPS2, min_dur=0.4)
                    time.sleep(0.1)

                print("Playback finished.")

            # ---------------------------
            # LIST WAYPOINTS
            # ---------------------------
            elif mode == "3":
                if not waypoints:
                    print("No waypoints.")
                else:
                    print("\n--- WAYPOINTS ---")
                    for i, wp in enumerate(waypoints):
                        print(f"[{i}] base={wp[0]:+.3f}, axis1={wp[1]:+.3f}, axis2={wp[2]:+.3f}")

            elif mode == "q":
                break

            else:
                print("Invalid option.")

    finally:
        bridge.close()
        print("Closed.")


if __name__ == "__main__":
    main()