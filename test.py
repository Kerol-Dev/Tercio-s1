"""Prints the live position of node 1 (protocol v2)."""
import time

from Firmware.Libraries.Python.TercioBridge import Bus, Stepper

with Bus() as bus:
    stepper = Stepper(bus, 1)
    while True:
        if stepper.position is not None:
            print(f"{stepper.position:.2f}°  {stepper.telemetry.state.name}")
        time.sleep(0.05)
