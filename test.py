from TercioLab import Bridge, Stepper
import time

bridge = Bridge()
bridge.open()

stepperA = Stepper(bridge, 3)
stepperB = Stepper(bridge, 4)
stepperC = Stepper(bridge, 5)

TOL = 3.0
POLL = 0.1

def is_reached(stepper: Stepper, target: float, tol: float = TOL) -> bool:
    """
    If axis_state is None -> treat as 'not available' and skip (return True so it doesn't block).
    Otherwise check angle error.
    """
    st = stepper.get_axis_state()
    if st is None:
        return True
    return abs(st.currentAngle - target) <= tol

def wait_all_or_skip_none(targetA: float, targetB: float, targetC: float, tol: float = TOL):
    # Wait until all available axes are within tolerance.
    while True:
        a_ok = is_reached(stepperA, targetA, tol)
        b_ok = is_reached(stepperB, targetB, tol)
        c_ok = is_reached(stepperC, targetC, tol)

        if a_ok and b_ok and c_ok:
            return

        time.sleep(POLL)

a = 0
while a < 3:
    # Move +
    stepperA.set_target_angle(1200)
    stepperB.set_target_angle(1800)
    stepperC.set_target_angle(-1500)
    wait_all_or_skip_none(1200, 1800, -1500)
    time.sleep(0.25)

    # Move -
    stepperA.set_target_angle(-1200)
    stepperB.set_target_angle(-1800)
    stepperC.set_target_angle(1500)
    wait_all_or_skip_none(-1200, -1800, 1500)
    time.sleep(0.25)
    a += 1

stepperA.set_target_angle(0)
stepperB.set_target_angle(0)
stepperC.set_target_angle(0)
