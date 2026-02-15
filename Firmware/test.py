from Libraries.Python.TercioBridge import Bridge, Stepper
import time
bridge = Bridge()
bridge.open()

stepper = Stepper(bridge, 2)


while True:
    stepper.set_target_angle(300)
    time.sleep(1)
    stepper.set_target_angle(-300)
    time.sleep(1)