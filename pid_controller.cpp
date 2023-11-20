#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"

// Initialize variable to store the start time
unsigned long startTime = 0;

// Set sampling time.
// Adjust based on control system design.
const int Ts = 10; // milliseconds

// Initialize constants for printing in "testing" mode;
int print_interval = 50;
int iteration = 1;

// Use csv printing mode?
// If false, the print mode is "testing".
bool print_csv = true;

// Assign control output pin.
// Make sure the arduino pin assigned has PWM support (labelled with a sine wave).
const int controlPin = 9;

// PID gains for t_brew
const double Kp_brew = 1.2; // Proportional
const double Ki_brew = 0.078; // Integral
const double Kd_brew = 0.08; // Derivative

// PID gains for t_boiler
const double Kp_boiler = 1.0; // Proportional
const double Ki_boiler = 0.0; // Integral
const double Kd_boiler = 0.3; // Derivative

// Controller outputs either a minimum or maximum voltage.
// Arduino scales 0-255 to 0-5V.
// Min = SSR->off (still completes circuit).
// Max = SSR->on.
// When circuit is incomplete, operation of brew switch is off-nominal.
const double min_voltage = 5;
const double max_voltage = 255;

// Initialize variables for brew.
const double setpoint_brew = 95.0; // degrees Celsius
double previous_error_brew = 0.0;
double integral_brew = 0.0;
const double brew_threshold = 90.0;
int brew_counter = 0;
double temperature_brew_old = 22.0;

// Initialize variables for boiler.
const double setpoint_boiler = 98.5; // degrees Celsius
double previous_error_boiler = 0.0;
double integral_boiler = 0.0;
double boiler_error_margin = 2.0; // Percentage error tolerance for boiler setpoint.
double boiler_threshold = 7.0; // If PID output signal is above 7, SSR is on, otherwise it is off.
bool boilerSetpointReached = false; // If within tolerance of setpoint, this switches to false.
double temperature_boiler_old = 22.0;

// Thermocouples setup.
// Brew thermocouple pins:
#define MAXDO_BREW   4
#define MAXCS_BREW   7
#define MAXCLK_BREW  8

// Boiler thermocouple pins:
#define MAXDO_BOILER   12
#define MAXCS_BOILER   11
#define MAXCLK_BOILER  2

// Assign breakoutboard pins to adafruit thermocouple reading software.
Adafruit_MAX31855 thermocouple_brew(MAXCLK_BREW, MAXCS_BREW, MAXDO_BREW); // Thermocouple for t_brew
Adafruit_MAX31855 thermocouple_boiler(MAXCLK_BOILER, MAXCS_BOILER, MAXDO_BOILER); // Thermocouple for t_boiler

void setup() {
  Serial.begin(9600);

  // Record the start time.
  startTime = millis();

  // Thermocouple setup.
  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(1000);
  Serial.print("Initializing sensors...");
  
  if (!thermocouple_brew.begin() || !thermocouple_boiler.begin()) {
    Serial.println("ERROR. Check your connections.");
    while (1) delay(10);
  }
  
  Serial.println("DONE.");

  // Control pin setup
  pinMode(controlPin, OUTPUT);
}

void loop() {
  // Record the current time
  unsigned long currentTime = millis();

  // Calculate the elapsed time since the control loop started
  unsigned long elapsedTime = currentTime - startTime;

  // Read thermocouples
  double temperature_brew = thermocouple_brew.readCelsius();
  double temperature_boiler = thermocouple_boiler.readCelsius();

  // Filter out NaN readings.
  if (isnan(temperature_brew)) {
    temperature_brew = temperature_brew_old;
  }
  if (isnan(temperature_boiler)) {
    temperature_boiler = temperature_boiler_old;
  }

  // Save off current temperature for filtering.
  temperature_brew_old = temperature_brew;
  temperature_boiler_old = temperature_boiler;

  // Calculate errors.
  double error_brew = setpoint_brew - temperature_brew;
  double error_boiler = setpoint_boiler - temperature_boiler;

  // Calculate integral terms.
  integral_brew += error_brew;
  integral_boiler += error_boiler;

  // Calculate derivative terms
  double derivative_brew = error_brew - previous_error_brew;
  double derivative_boiler = error_boiler - previous_error_boiler;

  // Calculate control outputs
  double output_brew = Kp_brew * error_brew + Ki_brew * integral_brew + Kd_brew * derivative_brew;
  double output_boiler = Kp_boiler * error_boiler + Ki_boiler * integral_boiler + Kd_boiler * derivative_boiler;
 
  // Scale the outputs to match the voltage range (0-5V)
  // Arduino writes to output
  if (output_boiler > boiler_threshold) {
    scaled_output_boiler = max_voltage;
  } else if (output_boiler <= boiler_threshold) {
    scaled_output_boiler = min_voltage;
  } 
  if (output_brew > boiler_threshold) {
    scaled_output_brew = max_voltage;
  } else if (output_brew <= boiler_threshold) {
    scaled_output_brew = min_voltage;
  } 

  // Check if the boiler setpoint is reached
  if (abs(error_boiler) / setpoint_boiler * 100 < boiler_error_margin) {
    boilerSetpointReached = true;
  } else {
    boilerSetpointReached = false;
  }
  
  bool brewstarted = false;
  // Checks if brew tc is really hot (this would indicate brew has started/ongoing.)
  if (temperature_brew > brew_threshold){
    brewstarted = true;
    brew_counter++;
    if (brew_counter == 1) {
      // If brew just started (1st iteration during brew), reset integral error term.
      integral_brew = 0;
    }
  } else {
    brewstarted = false;
  }

  // Send the appropriate control output to the SSR
  if (brewstarted) {
    analogWrite(controlPin, scaled_output_brew);
  } else {
    analogWrite(controlPin, scaled_output_boiler);
  }

  // Store the current errors for the next iteration
  previous_error_brew = error_brew;
  previous_error_boiler = error_boiler;
  iteration++;

  double percentage_boiler = abs(error_boiler) / setpoint_boiler * 100;

  // print data in "csv mode" or "testing mode":
  if (print_csv) {
    Serial.print(elapsedTime);
    Serial.print(",");
    Serial.print(temperature_brew);
    Serial.print(",");
    Serial.print(scaled_output_brew);
    Serial.print(",");
    Serial.print(brewstarted);
    Serial.print(",");
    Serial.print(brew_counter);
    Serial.print(",");
    Serial.print(error_brew);
    Serial.print(",");
    Serial.print(integral_brew);
    Serial.print(",");
    Serial.print(derivative_brew);
    Serial.print(",");
    Serial.print(temperature_boiler);
    Serial.print(",");
    Serial.print(scaled_output_boiler);
    Serial.print(",");
    Serial.print(percentage_boiler);
    Serial.print(",");
    Serial.print(error_boiler);
    Serial.print(",");
    Serial.print(integral_boiler);
    Serial.print(",");
    Serial.print(derivative_boiler);
    Serial.print(",");
    Serial.println(boilerSetpointReached);
  } else if (iteration % print_interval == 0 && !print_csv) {
      // Print essential variables every print interval
      // with labels if not in print to csv mode:
      Serial.print("Temperature Brew:");
      Serial.println(temperature_brew); 
      Serial.print("Brew Control Out: ");
      Serial.println(scaled_output_brew);
      Serial.print("Temperature Boiler: ");
      Serial.println(temperature_boiler);
      Serial.print("Boiler Control Out: ");
      Serial.println(scaled_output_boiler);
      Serial.print("Boiler Setpoint Reached? :");
      Serial.println(boilerSetpointReached ? "1" : "0"); 
      Serial.print("Percentage boiler error: ");
      Serial.println(percentage_boiler);
  }

  // Use delay to set a sampling time for the controller.
  delay(Ts);
}