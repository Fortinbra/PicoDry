import machine
from machine import Pin, I2C

import utime
from breakout_bme280 import BreakoutBME280
from pimoroni_i2c import PimoroniI2C
from pimoroni import PICO_EXPLORER_I2C_PINS

# PINS_BREAKOUT_GARDEN = {"sda": 4, "scl": 5}
# PINS_PICO_EXPLORER = {"sda": 20, "scl": 21}

i2c = PimoroniI2C(**PICO_EXPLORER_I2C_PINS)
bme = BreakoutBME280(i2c=i2c, address=0x77)

# Define a GPIO pin
pin = machine.Pin(0, machine.Pin.OUT)

# Define PID parameters
Kp = 1.0
Ki = 0.1
Kd = 0.05
set_point = 50.0

# Initialize integral and previous error
integral = 0.0
prev_error = 0.0
# Define another GPIO pin for the onboard LED
led = machine.Pin(25, machine.Pin.OUT)

# In a loop, read humidity value and use the PID controller to control the GPIO pin
while True:
    temperature, pressure, humidity = bme.read()

    humidity = humidity / 1024  # Convert to percentage

    # Calculate error
    error = set_point - humidity

    # Calculate integral
    integral += error

    # Calculate derivative
    derivative = error - prev_error

    # Calculate output
    output = Kp*error + Ki*integral + Kd*derivative

    # Control the GPIO pin based on the output
    if output > 0:
        pin.high()
        led.on()
    else:
        pin.low()
        led.off()
    # Update previous error
    prev_error = error

    utime.sleep(1)