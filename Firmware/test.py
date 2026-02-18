from Libraries.Python.TercioBridge import Bridge, Stepper
import time
bridge = Bridge()
bridge.open()

stepper = Stepper(bridge, 1)
stepper2 = Stepper(bridge, 2)

stepper.set_microsteps(128)
stepper2.set_microsteps(128)

stepper.set_speed_limit_rps(5)
stepper2.set_speed_limit_rps(5)
stepper2.set_target_angle(-10000)
stepper.set_target_angle(-10000)
