from Firmware.Libraries.Python.TercioBridge import Bridge, Stepper


bridge = Bridge()
bridge.open()

stepper = Stepper(bridge, 1)

while True:
    if(stepper.get_axis_state() is not None):
        print(stepper.get_axis_state().currentAngle)