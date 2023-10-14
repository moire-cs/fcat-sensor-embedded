#include <SHT1x.h>
#include <Arduino.h>
#include "driver/gpio.h"

// NO SCREEN

const int moisture = GPIO_NUM_25;
const int light = GPIO_NUM_26;
const int clockPin = GPIO_NUM_12;
const int dataPin = GPIO_NUM_13;

SHT1x sht1x(dataPin, clockPin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(moisture, INPUT);
  pinMode(light, INPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);


}

void loop() {
  int moisture_val = 100 - (int)((analogRead(moisture) - 196) / 3.24);
  int light_val = (int)analogRead(light) / 10;
  int tempF = sht1x.readTemperatureF();
  int humidity = sht1x.readHumidity();
  Serial.println(String(moisture_val) + "%M " + String(tempF) + "F " + humidity + "%H " + light_val + "%L");
  delay(250);
  // Serial.println("Hello World");
}

