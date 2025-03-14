from XRPLib.defaults import *
from VL53L0X import *
from machine import I2C,Pin
import time

i2c0 = I2C(0, scl=Pin(21), sda=Pin(20), freq=50000)
i2c1 = I2C(1, scl=Pin(19), sda=Pin(18), freq=50000)

tof0 = VL53L0X(i2c0)
tof1 = VL53L0X(i2c1)

# Pre: 12 to 18 (initialized to 14 by default)
# Final: 8 to 14 (initialized to 10 by default)

# the measuting_timing_budget is a value in ms, the longer the budget, the more accurate the reading. 
tof0.set_measurement_timing_budget(40000)
tof1.set_measurement_timing_budget(40000)

# Sets the VCSEL (vertical cavity surface emitting laser) pulse period for the 
# given period type (VL53L0X::VcselPeriodPreRange or VL53L0X::VcselPeriodFinalRange) 
# to the given value (in PCLKs). Longer periods increase the potential range of the sensor. 
# Valid values are (even numbers only):

# tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 18)
tof0.set_Vcsel_pulse_period(tof0.vcsel_period_type[0], 12)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[0], 12)

# tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 14)
tof0.set_Vcsel_pulse_period(tof0.vcsel_period_type[1], 8)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[1], 8)

while True:
# Start ranging
    print(tof0.ping(), "mm", tof1.ping())