#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
// This is for an Arduino Uno controller
// and a type K thermocouple with an Adafruit MAX1855 breakout board. 

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

// Thermocouple setup
#define MAXDO   4
#define MAXCS   7
#define MAXCLK  8
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// Control pin setup
// Make sure the control pin has PWM support.
const int controlPin = 9; // Pin for controlling the SSR

void setup() {
  Serial.begin(9600);

  // Thermocouple setup
  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR. Check your connections.");
    while (1) delay(10);
  }
  Serial.println("DONE.");

  // Control pin setup
  pinMode(controlPin, OUTPUT);
}

void loop() {
  // Read thermocouple
  double temperature = thermocouple.readCelsius();
  Serial.print("Temperature = ");
  Serial.println(temperature);

  // Calculate the error
  double error = setpoint - temperature;

  // Calculate the integral term
  integral += error;

  // Calculate the derivative term
  double derivative = error - previous_error;

  // Calculate the control output
  double output = Kp * error + Ki * integral + Kd * derivative;

  // Scale the output to match the voltage range (0-3.3V)
  // This ensures that the control output doesn't saturate.
  double scaled_output = constrain(output, min_voltage, max_voltage);

  // Print the control output (for testing purposes)
  Serial.print("Control Output = ");
  Serial.println(scaled_output);

  // Send the control output to the SSR
  analogWrite(controlPin, map(scaled_output, 0, 3.3, 0, 255));

  // Store the current error for the next iteration
  previous_error = error;

  // Use delay to set a sampling time for the controller (10 ms).
  delay(1000); // You can adjust the delay based on your desired sampling time for the controller
}

