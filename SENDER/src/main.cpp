#include <SHT1x-ESP.h>
#include <Arduino.h>
#include "driver/gpio.h"
// #include <RH_RF95.h>
// #include <RHReliableDatagram.h>

// #define SERVER_ADDRESS 1

#include <SPI.h>
#include <LoRa.h>

// NO SCREEN
// 5,17,16,4, 27, 14,15
const int moisture = GPIO_NUM_26; // could change
const int light = GPIO_NUM_25;    // could change
const int clockPin = GPIO_NUM_18;
const int dataPin = GPIO_NUM_22; // could change

const int misoPin = GPIO_NUM_23;
const int mosiPin = GPIO_NUM_19;
const int nssPin = GPIO_NUM_5;
const int resetPin = GPIO_NUM_14;
const int dio0Pin = GPIO_NUM_2;  // 13
const int dio1Pin = GPIO_NUM_12; // 12

// RH_RF95 driver;

// RHReliableDatagram manager(driver, SERVER_ADDRESS);

SHT1x sht1x(dataPin, clockPin);
int counter = 0;

int readings[4] = { 0, 0, 0, 0 };

// uint8_t data[] = "Pong";
// uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  // SPI.begin(SCK, MISO, MOSI, SS);

  // pinMode(moisture, INPUT);
  // pinMode(light, INPUT);
  pinMode(clockPin, OUTPUT);
  // pinMode(dataPin, OUTPUT);
  pinMode(misoPin, INPUT);

  pinMode(mosiPin, OUTPUT);
  // pinMode(SS, OUTPUT);
  // pin

  // pinMode(resetPin, OUTPUT);
  // pinMode(dio0Pin, INPUT);

  while (!Serial)
    Serial.println("LoRa Sender");

  // setup LoRa transceiver module
  //  LoRa.setPins(ss, rst, dio0);
  LoRa.setPins(SS, resetPin, dio0Pin);

  // replace the LoRa.begin(---E-) argument with your location's frequency
  // 433E6 for Asia
  // 866E6 for Europe
  // 915E6 for North America
  // LoRa.setSPIFrequency(1E6);
  // SPI.begin(SCK, MISO, MOSI, nssPin);
  while (!LoRa.begin(915E6)) {
    Serial.println(".");
    delay(500);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  LoRa.setTxPower(20);
  Serial.println("LoRa Initializing OK! (sender)");
}
void getReadings()
{
  int moisture_val = 100 - (int)((analogRead(moisture) - 900) / 20);
  // int light_val = (int)(100 * analogRead(light) / 4095);
  int tempF = sht1x.readTemperatureF();
  int humidity = sht1x.readHumidity();

  // // int new_readings[4] = { moisture_val, /*tempF,*/ /*humidity*/ light_val };
  readings[0] = moisture_val;
  // readings[1] = light_val;
  readings[2] = tempF;
  readings[3] = (int)sht1x.readHumidity();
  // return readings;
  // return[moisture_val, light_val];
};
void loop()
{
  // int* readings = getReadings();
  // getReadings();
  delay(500);
  Serial.println("Sending packet: " + String(counter) + ": " + String(readings[0]) + "%M " + String(readings[2]) + "F " + readings[3] + "%H " + readings[1] + "%L");
  // Serial.println(sht1x.readRawData(ShtCommand::MeasureRelativeHumidity, dataPin, clockPin));
  // Serial.println("Sending packet " + String(counter) + ": " + String(moisture_val) + "%M " + String(tempF) + "F " + humidity + "%H " + light_val + "%L");

  // sends hello over radio
  LoRa.beginPacket();
  LoRa.print("hello");
  // LoRa.print(String(counter) + ": " + String(readings[0]));
  // LoRa.print(String(counter) + ": " + String(readings[0]) + "%M " + String(readings[2]) + "F " + readings[3] + "%H " + readings[1] + "%L");

  LoRa.endPacket();

  counter++;
  delay(250);
}
