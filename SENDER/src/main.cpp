#include <SHT1x.h>
#include <Arduino.h>
#include "driver/gpio.h"
// #include <RH_RF95.h>
// #include <RHReliableDatagram.h>

// #define SERVER_ADDRESS 1

#include <SPI.h>
#include <LoRa.h>

// NO SCREEN
// 5,17,16,4, 27, 14,15
const int moisture = GPIO_NUM_32;
const int light = GPIO_NUM_33;
const int clockPin = GPIO_NUM_18;
const int dataPin = GPIO_NUM_22;

const int misoPin = GPIO_NUM_19;
const int mosiPin = GPIO_NUM_23;
const int nssPin = GPIO_NUM_5;
const int resetPin = GPIO_NUM_14;
const int dio0Pin = GPIO_NUM_15;
// const int dio1Pin = GPIO_NUM_2;

// RH_RF95 driver;

// RHReliableDatagram manager(driver, SERVER_ADDRESS);

SHT1x sht1x(clockPin, dataPin);
int counter = 0;



// uint8_t data[] = "Pong";
// uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(moisture, INPUT);
  pinMode(light, INPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(misoPin, INPUT);
  pinMode(mosiPin, OUTPUT);
  // pinMode(nssPin, OUTPUT);
  pinMode(resetPin, OUTPUT);
  pinMode(dio0Pin, INPUT);
  // digitalWrite(nssPin, HIGH);

  // pinMode(dio1Pin, INPUT);

  // driver.setFrequency(915.0);
  // driver.setTxPower(23, false);
  // driver.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);

  while (!Serial)
    Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  // LoRa.setPins(ss, rst, dio0);
  LoRa.setPins(nssPin, resetPin, dio0Pin);

  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  LoRa.setSPIFrequency(1E6);
  SPI.begin(SCK, MISO, MOSI, SS);
  while (!LoRa.begin(915E6)) {
    Serial.println(".");
    delay(500);
  }
  // Change sync word (0xF3) to match the receiver
 // The sync word assures you don't get LoRa messages from other LoRa transceivers
 // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}


void loop() {
  int moisture_val = 100 - (int)((analogRead(moisture) - 900) / 20);
  // int moisture_val = (int)analogRead(moisture);
  int light_val = (int)(100 * analogRead(light) / 4095);
  int tempF = sht1x.readTemperatureF();
  int humidity = sht1x.readHumidity();

  delay(250);

  Serial.println("Sending packet " + String(counter) + ": " + String(moisture_val) + "%M " + String(tempF) + "F " + humidity + "%H " + light_val + "%L");

  // sends hello over radio
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.endPacket(true);

  counter++;
}
