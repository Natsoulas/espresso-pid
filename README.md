# Espresso Machine PID Temperature Control Code

This README provides an overview of code that demonstrates a Proportional-Integral-Derivative (PID) temperature control system. The code is designed to control the temperature of a Rancilio Silvia espresso machine such that it reaches and maintains a setpoint temperature using PID control.

## Code Overview

The pid_controller.cpp file contains Arduino C++ code for a basic implementation of a PID controller for brew temperature regulation. This code runs on an Arduino Uno microcontroller. The software consists of the following components:

### PID Controller Parameters

- `Kp`: The Proportional gain. Adjusts the immediate response to the error between the setpoint and the current temperature.
- `Ki`: The Integral gain. Adjusts the response to the accumulation of past errors over time.
- `Kd`: The Derivative gain. Adjusts the response based on the rate of change of the error.

### Control Variables

- `setpoint`: The desired temperature (in degrees Celsius) that the system should reach and maintain.
- `previous_error`: The previous error value, used for calculating the derivative term.
- `integral`: The integral term, which accumulates past errors.

### Main PID Loop

The main loop runs continuously and performs the following actions:

1. Read the current temperature from a thermocouple (simulated by a static value in the code).
2. Calculate the error as the difference between the setpoint temperature and the current temperature.
3. Calculate the integral term by accumulating the error over time.
4. Calculate the derivative term as the difference between the current error and the previous error.
5. Compute the control output using the PID equation: `output = Kp * error + Ki * integral + Kd * derivative`.
6. Limit the control output to a certain range to avoid saturation. In this code, it's limited between 0 and 3.3 volts.
7. Update the previous error for the next iteration.
8. Add a time delay of 0.01 seconds using the `delay` function to set the sampling time for the controller.

### Control Output

The code caps the control output by determining whether the output is above 3.3V or not. The microcontroller in use can only output 3.3V, so the cap is set there to avoid saturation. Another thing to note is that the activation voltage of the Solid State Relay in use is 3V.

## Usage

This code serves as a basic example of a PID controller for temperature control. To adapt it for your specific application, you will need to replace the constant input thermocouple reading with actual temperature data and ensure that the control output interfaces with your heating element or system as required.

## Disclaimer

This code is for a specific system and should not be used for espresso machine temperature control without proper testing, calibration, and safety considerations. Temperature control systems can be critical, and any implementation should be thoroughly validated and tested to ensure safe and reliable operation.

# Simulation README

This simulation demonstrates a basic implementation of a Proportional-Integral-Derivative (PID) temperature control system. The simulation runs for approximately 5 seconds, mimicking the duration of a shot of espresso. It calculates and records temperature, time, and control output voltage data, which is saved to a CSV file.

## Files

- `simulation.cpp`: C++ code for the simulation.
- `Makefile`: Makefile for compiling and running the simulation.
- `plot_data.py`: Python script for plotting simulation data.

## Running the Simulation

1. **Compile the Simulation:**
   - Open a terminal in the simulation directory.
   - Run `make` to compile the simulation.

2. **Run the Simulation:**
   - Execute `./simulation` to run the simulation.
   - The simulation data will be saved in `simulation_data.csv`.

3. **Plot Simulation Data:**
   - Ensure you have Python installed.
   - Install required libraries by running `pip install pandas matplotlib`.
   - Run `python plot_data.py` to visualize temperature and control output over time.

## Important Notes

- Adjust simulation parameters in `simulation.cpp` as needed.
- The provided Python script assumes the CSV file structure and requires `pandas` and `matplotlib`.

Feel free to customize and extend this simulation for your specific requirements.
