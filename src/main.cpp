#include <Arduino.h>
#include <Wire.h> // used for i2c
#include "DFRobot_B_LUX_V30B.h"
#define MOISTURE_SENSOR 16
#define PWM_CHANNEL 0 // channel to generate the PWM signal
#define PWM_GPIO 26 // GPIO pin to output PWM signal
#define PWM_RESOLUTION 8 // number of bits (8 bits -> 0-255)
#define PWM_FREQ 5000 // Hz
#define IR_SENSOR 4
#define LIGHT_SENSOR 18

DFRobot_B_LUX_V30B myLux(LIGHT_SENSOR);

void setupMoistureSensor() {
  pinMode(MOISTURE_SENSOR, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);
}
void runMoistureSensor() {
  int reading = analogRead(MOISTURE_SENSOR);
  if (reading < 2000) {
    digitalWrite(BUILTIN_LED, HIGH);
  } else {
    digitalWrite(BUILTIN_LED, LOW);
  }
  Serial.println(reading);
}

void setupPWM(uint8_t channel, int frequency, uint8_t resolution, uint8_t gpioPin) {
  pinMode(gpioPin, OUTPUT);
  ledcSetup(channel, frequency, resolution);
  ledcAttachPin(gpioPin, channel);
  ledcWrite(channel, 0);
}
void setPWMDutyCycle(uint8_t duty_cycle) {
  // duty_cycle -> percent ON as a whole number 0-100
  uint8_t normalized_duty_cycle = (duty_cycle * ((1 << PWM_RESOLUTION) - 1)) / 100;
  ledcWrite(PWM_CHANNEL, normalized_duty_cycle);
}

void setupIRSensor() {
  pinMode(IR_SENSOR, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);
}
void runIRSensor() {
  int reading = digitalRead(IR_SENSOR);
  if (reading == HIGH) {
    digitalWrite(BUILTIN_LED, HIGH);
  } else {
    digitalWrite(BUILTIN_LED, LOW);
  }
  Serial.println(reading);
}

void setupLightSensor() {
  myLux.begin();
}
void runLightSensor() {
  Serial.print("value: ");
  Serial.print(myLux.lightStrengthLux());
  Serial.println(" (lux).");
  delay(1000);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setupMoistureSensor();
  // setupPWM(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION, PWM_GPIO);
  // setupIRSensor();
  // setupLightSensor();
}

void loop() {
  // put your main code here, to run repeatedly:
  runMoistureSensor();
  // for(uint8_t i = 0; i < 100; i++) {
  //   setPWMDutyCycle(i);
  //   delay(50);
  // }
  // for(uint8_t i = 100; i > 0; i--) {
  //   setPWMDutyCycle(i);
  //   delay(50);
  // }
  // runIRSensor();
  // runLightSensor();
}