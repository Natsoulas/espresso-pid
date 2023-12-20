# Espresso Machine PID Temperature Control Code

The software in the file named "pid_controller.cpp" is a Proportional-Integral-Derivative (PID) control algorithm that can be run on a microcontroler to control the temperature of a Rancilio Silvia espresso machine. This algorithm is designed around the hardware of this PID control system. The main hardware components include the espresso machine, an Arduino Uno microcontroller, a Solid State Relay (SSR), and two thermocouples. For more details please refer to the report pdf (Espress_PID_Report.pdf) in this repository.


## Comparison to the Default Thermostat Control

This PID algorithm outperforms the thermostat control by brewing with an order of magnitude smaller variance in water temperature and reaches within one degree Celsius of its setpoint temperature. For a full performance comparison, refer to the report.

## Code Overview

The `pid_controller.cpp` file contains Arduino C++ code for a basic implementation of a PID-based control algorithm designed for brew temperature regulation. The software comprises the following components:

### PID Controller Parameters

- `Kp_brew`, `Ki_brew`, `Kd_brew`: The Proportional, Integral, and Derivative gains for the brew temperature control loop. These parameters fine-tune the controller's response to errors in maintaining the brew temperature controller.
- `Kp_boiler`, `Ki_boiler`, `Kd_boiler`: Similar gains specific to the boiler temperature controller.

### Control Variables

- `setpoint_brew`, `setpoint_boiler`: The desired temperatures (in degrees Celsius) that the system should reach and maintain for brewing and the boiler, respectively.
- `previous_error_brew`, `previous_error_boiler`: The previous error values, used for calculating the derivative term.
- `integral_brew`, `integral_boiler`: The integral terms, accumulating past errors over time for the respective temperature controller.

### Main PID Loop

The main loop runs continuously and performs the following actions:

1. Read the current temperatures from thermocouples (simulated by analogRead in the code) for both brewing and the boiler.
2. Calculate the errors as the differences between the setpoint temperatures and the current temperatures.
3. Calculate the integral terms by accumulating the errors over time.
4. Calculate the derivative terms as the differences between the current errors and the previous errors.
5. Compute the control outputs using the PID equation: `output = Kp * error + Ki * integral + Kd * derivative`.
6. Limit the control outputs to a certain voltage range to prevent saturation. In this code, it's between `min_voltage` and `max_voltage`.
7. Update the previous errors for the next iteration.
8. Add a time delay (`Ts` milliseconds) using the `delay` function to set the sampling time for the controller.

### Control Output

The code caps the control outputs based on the specified voltage range. The microcontroller can only output voltages within this range, ensuring compatibility with the activation voltage (3V) of the Solid State Relay (SSR) used in the system.


## Usage

Following a similar design to the report and performing similar assembly and testing for this control system on a Rancilio Silvia should yield similar results. Note that the machine used in this study was the "Miss Silvia" model.

## Files

- `pid_controller.cpp`: C++ control algortihm code for Arduino Uno.
- `data_processing.py`: python software for indexing and plotting CSV from Arduino.
- `Espresso_PID_Report.pdf`: Technical report that encompassses software and hardware implementation.
- `test_data`: directory with saved CSV files from HITL testing.
- `espresso_sim`: C++ Simulation for espresso machine PID control.
- `espresso_simulink`: Simulink-based Simulation for espresso machine PID control.
- `data_analysis_MATLAB`: MATLAB scripting for test data analysis.

## Disclaimer

This code is for a specific system and should not be used for espresso machine temperature control without proper testing, calibration, tuning, and safety considerations. Temperature control systems are critical to Espresso Machine operational safety, and any implementation should be thoroughly validated and tested to ensure reliability.

# Simulation README

This simulation demonstrates a basic implementation of a Proportional-Integral-Derivative (PID) temperature control system. The simulation runs for approximately 5 seconds, mimicking the duration of a shot of espresso. It calculates and records temperature, time, and control output voltage data, which is saved to a CSV file.

## Files

- `espresso_sim/simulation.cpp`: C++ code for the simulation.
- `espresso_sim/Makefile`: Makefile for compiling and running the simulation.
- `espresso_sim/plot_data.py`: Python script for plotting simulation data.

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
