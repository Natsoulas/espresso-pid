#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"

// Constants for sampling time and printing modes
const int Ts = 10; // milliseconds
const int print_interval = 50;
const bool print_csv = true;
int iteration = 1;

// Pin assignments
const int controlPin = 9;
const int MAXDO_BREW = 4, MAXCS_BREW = 7, MAXCLK_BREW = 8;
const int MAXDO_BOILER = 12, MAXCS_BOILER = 11, MAXCLK_BOILER = 2;

// PID gains
const double Kp_brew = 1.2, Ki_brew = 0.078, Kd_brew = 0.08;
const double Kp_boiler = 1.0, Ki_boiler = 0.0, Kd_boiler = 0.3;

// Voltage range
const double min_voltage = 5, max_voltage = 255;

// Setpoints and thresholds
const double setpoint_brew = 95.0, setpoint_boiler = 98.5;
const double brew_threshold = 90.0, boiler_error_margin = 2.0, boiler_threshold = 7.0;

// Adafruit MAX31855 instances
Adafruit_MAX31855 thermocouple_brew(MAXCLK_BREW, MAXCS_BREW, MAXDO_BREW);
Adafruit_MAX31855 thermocouple_boiler(MAXCLK_BOILER, MAXCS_BOILER, MAXDO_BOILER);

// Variables for PID control
unsigned long startTime = 0;
double integral_brew = 0.0, integral_boiler = 0.0;
double previous_error_brew = 0.0, previous_error_boiler = 0.0;

// Variables for brew
int brew_counter = 0;
double temperature_brew_old = 22.0;

// Variables for boiler
bool boilerSetpointReached = false;
double temperature_boiler_old = 22.0;
int percentage_boiler;


void setup() {
  Serial.begin(9600);
  startTime = millis();
  Serial.println("MAX31855 test");
  delay(1000);
  Serial.print("Initializing sensors...");

  if (!thermocouple_brew.begin() || !thermocouple_boiler.begin()) {
    Serial.println("ERROR. Check your connections.");
    while (1) delay(10);
  }

  Serial.println("DONE.");
  pinMode(controlPin, OUTPUT);
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;

  double temperature_brew = readTemperature(thermocouple_brew, temperature_brew_old);
  double temperature_boiler = readTemperature(thermocouple_boiler, temperature_boiler_old);

  double error_brew = setpoint_brew - temperature_brew;
  double error_boiler = setpoint_boiler - temperature_boiler;

  integral_brew += error_brew;
  integral_boiler += error_boiler;

  double derivative_brew = error_brew - previous_error_brew;
  double derivative_boiler = error_boiler - previous_error_boiler;

  double output_brew = calculateOutput(Kp_brew, Ki_brew, Kd_brew, error_brew, integral_brew, derivative_brew);
  double output_boiler = calculateOutput(Kp_boiler, Ki_boiler, Kd_boiler, error_boiler, integral_boiler, derivative_boiler);

  double scaled_output_brew = scaleOutput(output_brew, boiler_threshold);
  double scaled_output_boiler = scaleOutput(output_boiler, boiler_threshold);

  updateSetpointReached(error_boiler);

  bool brewstarted = checkBrewStatus(temperature_brew);

  updateControlOutput(brewstarted, scaled_output_brew, scaled_output_boiler);

  updatePreviousErrors(error_brew, error_boiler);

  printData(elapsedTime, temperature_brew, scaled_output_brew, brewstarted, brew_counter, error_brew,
            integral_brew, derivative_brew, temperature_boiler, scaled_output_boiler, percentage_boiler, error_boiler,
            integral_boiler, derivative_boiler, boilerSetpointReached);
  iteration++;
  delay(Ts);
}

double readTemperature(Adafruit_MAX31855 &thermocouple, double &oldTemperature) {
  double temperature = thermocouple.readCelsius();
  if (isnan(temperature)) {
    temperature = oldTemperature;
  }
  oldTemperature = temperature;
  return temperature;
}

double calculateOutput(double Kp, double Ki, double Kd, double error, double &integral, double derivative) {
  return Kp * error + Ki * integral + Kd * derivative;
}

double scaleOutput(double output, double threshold) {
  if (output > threshold) {
    return max_voltage;
  } else {
    return min_voltage;
  }
}

void updateSetpointReached(double &error_boiler) {
  percentage_boiler = abs(error_boiler) / setpoint_boiler * 100;
  boilerSetpointReached = (percentage_boiler < boiler_error_margin);
}

bool checkBrewStatus(double temperature_brew) {
  bool brewstarted = (temperature_brew > brew_threshold);
  if (brewstarted) {
    brew_counter++;
    if (brew_counter == 1) {
      integral_brew = 0;
    }
  }
  return brewstarted;
}

void updateControlOutput(bool brewstarted, double scaled_output_brew, double scaled_output_boiler) {
  if (brewstarted) {
    analogWrite(controlPin, scaled_output_brew);
  } else {
    analogWrite(controlPin, scaled_output_boiler);
  }
}

void updatePreviousErrors(double error_brew, double error_boiler) {
  previous_error_brew = error_brew;
  previous_error_boiler = error_boiler;
}

void printData(unsigned long elapsedTime, double temperature_brew, double scaled_output_brew, bool brewstarted,
               int brew_counter, double error_brew, double integral_brew, double derivative_brew,
               double temperature_boiler, double scaled_output_boiler, double percentage_boiler,
               double error_boiler, double integral_boiler, double derivative_boiler, bool boilerSetpointReached) {
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
}