#include <DallasTemperature.h>
#include <OneWire.h>
#include <LiquidCrystal.h>
#include <SHT1x.h>



const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2, moisture = A2, light = A0;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
const int clockPin = 6;
const int dataPin = 7;

// #define ONE_WIRE_BUS 7

SHT1x sht1x(dataPin, clockPin);

// OneWire oneWire(ONE_WIRE_BUS);

// DallasTemperature sensors(&oneWire);

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16, 2);
  // sensors.begin();
  lcd.print("Measurement Node");
  Serial.begin(9600);

}

void loop() {
  // sensors.requestTemperatures();
  lcd.setCursor(0, 1);
  int moisture_val = 100 - (int)((analogRead(moisture) - 196) / 3.24);
  int light_val = (int)analogRead(light) / 10;
  int tempF = sht1x.readTemperatureF();
  int humidity = sht1x.readHumidity();
  lcd.print(String(moisture_val) + "%M " + String(tempF) + "F " + humidity + "%H " + light_val + "%L");
  delay(250);
  Serial.println(analogRead(light));
  // put your main code here, to run repeatedly:

}
