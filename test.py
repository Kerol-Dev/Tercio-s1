import time
from TercioStepperLib import Bridge, Stepper

# ==========================
# Config
bridge = Bridge()
bridge.open()
stepper = Stepper(bridge,1)
stepper.set_current_ma(2000)
stepper.set_speed_limit_rps(40.0)
stepper.set_accel_limit_rps2(100)
stepper.enable_motor(True)
stepper.set_pid(2,0,0)
stepper.set_units_degrees(True)
stepper.set_target_angle(1000)
