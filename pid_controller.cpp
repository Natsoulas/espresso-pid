#include <Arduino.h>
#include <SPI.h>

// Constants for sampling time and printing modes
const int Ts = 1; // milliseconds
const int print_interval = 50;
const bool print_csv = true;
int iteration = 1;

// Pin assignments
const int controlPin = 9;

// PID gains
const double Kp_brew = 1, Ki_brew = 0.001, Kd_brew = 10;
const double Kp_boiler = 10, Ki_boiler = 0.00001, Kd_boiler = 15;

// Voltage range
const double min_voltage = 1, max_voltage = 255;

// Setpoints and thresholds
const double setpoint_brew = 95.0, setpoint_boiler = 106.0;
const double brew_threshold = 90.0, boiler_error_margin = 1.0, boiler_threshold = 20.0;


// Adafruit AD8495 instances (Thermocouple breakout board)
int     analogPin_brew = A0;     // Pin for brew thermocouple
int     val_brew = 0;      // variable to store the ADC value from A0
float   temperature_brew;  // Temperature value in celsius degree
float   setup_gain_brew = 0.005;
float   setup_ref_brew  = 1.26313;

int     analogPin_boiler = A1;     // Pin for brew thermocouple
int     val_boiler = 0;      // variable to store the ADC value from A1
float   temperature_boiler;  // Temperature value in celsius degree
float   setup_gain_boiler = 0.005;
float   setup_ref_boiler  = 1.26313;

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
  Serial.println("DONE.");
  pinMode(controlPin, OUTPUT);
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;

  // double temperature_brew = readTemperature(thermocouple_brew, temperature_brew_old);
  // double temperature_boiler = readTemperature(thermocouple_boiler, temperature_boiler_old);
  val_brew = analogRead(analogPin_brew);                   // read the input pin
  double temperature_brew = (float(val_brew) * setup_gain_brew - setup_ref_brew)/0.005 ; 

  val_boiler = analogRead(analogPin_boiler);                   // read the input pin
  double temperature_boiler = (float(val_boiler) * setup_gain_boiler - setup_ref_boiler)/0.005 ; 

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
