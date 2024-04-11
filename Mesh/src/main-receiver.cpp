#include <SPI.h>
#include <LoRa.h>

// define the pins used by the transceiver module
const int ss = GPIO_NUM_5;
const int rst = GPIO_NUM_14;
const int dio0 = GPIO_NUM_2;

void setup()
{

    pinMode(GPIO_NUM_18, OUTPUT);
    pinMode(MISO, INPUT);

    pinMode(MOSI, OUTPUT);
    // initialize Serial Monitor
    Serial.begin(115200);
    while (!Serial)
        Serial.println("LoRa Receiver");

    // setup LoRa transceiver module
    LoRa.setPins(ss, rst, dio0);
    // pinMode(ss, OUTPUT);

    // replace the LoRa.begin(---E-) argument with your location's frequency
    // 433E6 for Asia
    // 866E6 for Europe
    // 915E6 for North America
    while (!LoRa.begin(915E6))
    {
        Serial.println(".");
        delay(500);
    }
    // Change sync word (0xF3) to match the receiver
    // The sync word assures you don't get LoRa messages from other LoRa transceivers
    // ranges from 0-0xFF
    LoRa.setSyncWord(0xF3);
    Serial.println("LoRa Initializing OK! (receiver)");
}

void loop()
{
    // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {
        // received a packet
        Serial.print("Received packet '");

        // read packet
        while (LoRa.available())
        {
            String LoRaData = LoRa.readString();
            Serial.print(LoRaData);
        }

        // print RSSI of packet
        Serial.print("' with RSSI ");
        Serial.println(LoRa.packetRssi());
    }
}