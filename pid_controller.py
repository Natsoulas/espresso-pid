import RPi.GPIO as GPIO
import time

# PID gains found by Lily's analysis
Kp = 0.431  # Proportional
Ki = 0.020509  # Integral
Kd = 0.04 # Derivative

# Controller output minimum voltage and maximum voltage.
min_voltage = 0
max_voltage = 3.3

# Initialize variables
setpoint = 95.0  # Setpoint is 95 degrees celsius
previous_error = 0
integral = 0

# Main loop for PID
while True:
    # Thermocouple readings
    temperature = 27.0 
    # Replace with actual thermocouple readings from pin using GPIO library
    
    # Calculate the error
    error = setpoint - temperature

    # Calculate the integral term
    integral = integral + error

    # Calculate the derivative term
    derivative = error - previous_error

    # Calculate the control output
    output = Kp * error + Ki * integral + Kd * derivative

   # Scale the output to match the voltage range (0-3.3V)
   # This ensures that the control output doesn't saturate.
    scaled_output = min(max(min_voltage, output), max_voltage)

    # Send the control output to the SSR which sends it to the heating element.
    # Our SSR works for voltages above 3V, so the controller's output will only
    # activate the SSR when it is above 3V.
    

    # Store the current error for the next iteration
    previous_error = error

    # Use time library to set a sampling time for the controller
    time.sleep(0.01)
