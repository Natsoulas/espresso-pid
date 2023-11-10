#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <cmath>

// PID gains found by analysis
const double Kp = 0.431;    // Proportional
const double Ki = 0.020509; // Integral
const double Kd = 0.04;     // Derivative

// Controller output minimum voltage and maximum voltage.
const double min_voltage = 0.0;
const double max_voltage = 3.3;

// Initialize variables
const double setpoint = 95.0; // Setpoint is 95 degrees Celsius
double previous_error = 0.0;
double integral = 0.0;

// Simulation time in seconds (5 seconds for a shot of espresso)
const double simulation_time = 5.0;

// Sampling time for each iteration in seconds (0.001 seconds)
const double sampling_time = 0.001;

// Save interval in seconds (0.01 seconds)
const double save_interval = 0.01;

// Function to simulate reading temperature from thermocouple
double simulateTemperature(double startTemperature, double endTemperature, double totalTime, double currentTime) {
    // Linear interpolation to simulate gradual temperature increase
    return startTemperature + (endTemperature - startTemperature) * (currentTime / totalTime);
}

// Function to simulate controlling SSR with scaled output
void simulateControl(double scaled_output) {
    // Simulate controlling SSR (print to console for demonstration purposes)
    std::cout << "Simulating SSR Control with Output: " << scaled_output << std::endl;
}

int main() {
    // Open CSV file for writing
    std::ofstream csvFile("sim_results/simulation_data.csv");
    csvFile << "Time,Temperature,Control_Output\n";

    // Number of iterations
    int num_iterations = static_cast<int>(simulation_time / sampling_time);

    // Save interval in terms of iterations
    int save_interval_iterations = static_cast<int>(save_interval / sampling_time);

    // Simulation loop
    for (int iteration = 0; iteration <= num_iterations; ++iteration) {
        
        // Calculate current time in simulation.
        double currentTime = iteration * sampling_time;

        // Simulate reading temperature with gradual increase
        double temperature = simulateTemperature(25.0, 95.0, simulation_time, currentTime);

        // Calculate the error
        double error = setpoint - temperature;

        // Calculate the integral term
        integral += error;

        // Calculate the derivative term
        double derivative = error - previous_error;

        // Calculate the control output
        double output = Kp * error + Ki * integral + Kd * derivative;

        // Scale the output to match the voltage range (0-3.3V)
        double scaled_output = std::min(std::max(min_voltage, output), max_voltage);

        // Print the current time, temperature, and control output to the CSV file at the specified interval
        if (iteration % save_interval_iterations == 0) {
            csvFile << iteration * sampling_time << "," << temperature << "," << scaled_output << "\n";
        }

        // Simulate controlling SSR
        simulateControl(scaled_output);

        // Store the current error for the next iteration
        previous_error = error;

        // Sleep for sampling time to simulate the desired iteration time
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sampling_time * 1000)));
    }

    // Close CSV file
    csvFile.close();

    return 0;
}
