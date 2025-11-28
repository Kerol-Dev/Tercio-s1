import time
from TercioStepperLib import Bridge, Stepper

# -----------------------------------------------------------
# Gear ratios
# motorAngle = jointAngle * gearRatio
# -----------------------------------------------------------
GEAR_12 = 8.0    # ID 12
GEAR_4  = 19.0   # ID 4
GEAR_3  = 20.0   # ID 3

# -----------------------------------------------------------
# User-defined joint waypoints (degrees)
# Each entry: [joint12, joint4, joint3]
# -----------------------------------------------------------
waypoints = [
    [0, 0, 0],
    [45, -45, 130],
    [90, 45, -75],
    [60, -60, 100],
    [0, 0, 0],
]

# -----------------------------------------------------------
# Setup
# -----------------------------------------------------------
bridge = Bridge()
bridge.open()

step12 = Stepper(bridge, 12)
step4  = Stepper(bridge, 4)
step3  = Stepper(bridge, 3)
steppers = [step12, step4, step3]
ratios   = [GEAR_12, GEAR_4, GEAR_3]

for s in steppers:
    s.set_units_degrees(True)
    s.set_microsteps(32)
    s.set_accel_limit_rps2(60)
    s.set_pid(3, 0, 0)
    s.enable_motor(False)

time.sleep(0.4)

# -----------------------------------------------------------
# Movement helpers
# -----------------------------------------------------------
def wait_all(targets, tol=0.7):
    while True:
        done = True
        for s, t in zip(steppers, targets):
            st = s.get_axis_state()
            if st is None or abs(st.currentAngle - t) > tol:
                done = False
                break
        if done:
            return
        time.sleep(0.05)


def move_all(joint_angles, travel_time=0.75):
    # Convert to motor angles using gear ratios
    motor_targets = [j * r for j, r in zip(joint_angles, ratios)]

    # Compute biggest travel distance
    current = []
    for s in steppers:
        st = s.get_axis_state()
        current.append(st.currentAngle if st else 0)

    dists = [abs(c - t) for c, t in zip(current, motor_targets)]
    max_dist = max(dists)

    if travel_time > 0:
        rps = max(0.5, (max_dist / 360.0) / travel_time)
        for s in steppers:
            s.set_speed_limit_rps(15)

    for s, t in zip(steppers, motor_targets):
        s.set_target_angle(t)
    wait_all(motor_targets)

# -----------------------------------------------------------
# Replay
# -----------------------------------------------------------
print("Enabling motors and replaying backwards")
for s in steppers:
    s.enable_motor(True)
time.sleep(0.3)

for wp in reversed(waypoints):
    print(" → Joint angles:", wp)
    move_all(wp)

print("Done. Disabling motors.")
for s in steppers:
    s.enable_motor(False)
bridge.close()