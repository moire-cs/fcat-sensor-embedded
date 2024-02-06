// #define RH_TEST_NETWORK 1 // activate Forced Topology

#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <cstring>
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "ClosedCube_HDC1080.h"
#include "esp_wifi.h"
#include <driver/adc.h>

/*
    Plan:
    - Create sync function to sync each node to a particular time
    - Measure data 4x per day, and send to gateway
    - For pulse: measure time between pulses
    - Determine power to cover distance
        - Rerun power analysis
        - Do we want dynamic power to radio?

*/

#define RF95_FREQ 915.0 // USA and Ecuador
#define WDT_TIMEOUT 15

#if defined(RFM95_CS) && defined(RFM95_RST) && defined(RFM95_INT)
#else
// Board pinout
#define RFM95_CS 5
#define RFM95_RST 14
#define RFM95_INT 13
#endif

#define SENDING_MODE 0
#define RECEIVING_MODE 1

// TODO: Define sensor pinouts
ClosedCube_HDC1080 hdc1080;

// sensor pins
#define battery 25 // FIXME: Do we need this either if we're using ADC
#define moisture 34
#define light 35

#define clockPin 18
#define dataPin 21

// pins for radio
#define misoPin 23
#define mosiPin 19
#define nssPin 5
#define resetPin 14
#define dio0Pin 13
#define dio1Pin 12

// For Pulse Counter
#define WAIT_TIME 500
#define MIN -21300 * WAIT_TIME / 1000 // will change based on soil moisture measurements
#define MAX -740 * WAIT_TIME / 1000   // will change based on soil moisture measurements
int16_t moisture_count = 0x10;        // Do we need 16? or is 0x00 ok
int counter = 0;

double readings[5] = {0, 0, 0, 0, 0};

/*

sda: 21
scl: 22
moisture: 34
light: 35
battery: 25
ar: 26

*/

#if defined(SELF_ADDRESS) && defined(TARGET_ADDRESS)
const uint8_t selfAddress_ = SELF_ADDRESS;
const uint8_t targetAddress_ = TARGET_ADDRESS;
#else
// Topology
// define the node address
#define NODE_ADDRESS 3
#define ENDNODE_ADDRESS 255 // purposefully using the last number
// TODO: according to this, we might have a max of 256 nodes in one mesh

// selfAddress is node
// targetAddress will be our gateway
const uint8_t selfAddress_ = ENDNODE_ADDRESS; // CHANGE THIS!!!
const uint8_t targetAddress_ = NODE_ADDRESS;  // integer value
#endif

// radio driver & message mesh delivery/receipt manager
RH_RF95 RFM95Modem_(RFM95_CS, RFM95_INT);
RHMesh RHMeshManager_(RFM95Modem_, selfAddress_);
uint8_t mode_ = RECEIVING_MODE;

// these are expected to be global/externally exposed variables, if you plan to
// make a class to wrap this
std::string msgSend =
    String("Hello from node " + String(selfAddress_) + "!").c_str();
std::string msgRcv;

void rhSetup();

void setup()
{
    Serial.begin(115200);
    esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);               // add current thread to WDT watch

    rhSetup();
    Serial.println(" ---------------- LORA NODE " + String(selfAddress_) +
                   " INIT ---------------- ");
}

long _lastSend = 0, sendInterval_ = 3000; // send every 10 seconds
uint8_t _msgRcvBuf[RH_MESH_MAX_MESSAGE_LEN];

// Function for getting measurements
void getReadings()
{
    // Enable the Power Rail
    digitalWrite(GPIO_NUM_26, HIGH);

    // TODO: Do we need this delay (probably not)
    delay(100);

    // Moisture Reading
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    // TODO: Take time here
    // FIXME: We need to delay but we also need to be taking the time for this
    delay(WAIT_TIME);
    pcnt_get_counter_value(PCNT_UNIT_0, &moisture_count);
    pcnt_counter_pause(PCNT_UNIT_0);
    // TODO: Take time here
    // TODO: calculate difference
    readings[0] = (double)moisture_count;

    // Light Reading
    // int light_val = (int)(100 * analogRead(light) / 4095);
    double light_val = (double)analogRead(light);
    readings[1] = light_val;

    // Temperature/Humidity Reading
    // TODO: Do we need to begin this every time
    hdc1080.begin(0x40);
    readings[2] = hdc1080.readTemperature() * 1.8 + 32;
    readings[3] = hdc1080.readHumidity();

    // Battery Reading
    int battery_level = 0;
    esp_err_t r = adc2_get_raw(ADC2_CHANNEL_8, ADC_WIDTH_12Bit, &battery_level);
    readings[4] = (battery_level / 4095.0) * 3.3;

    // Disable the Power Rail
    digitalWrite(GPIO_NUM_26, LOW);
};

void loop()
{
    uint8_t _msgFrom;
    uint8_t _msgRcvBufLen = sizeof(_msgRcvBuf);

    if ((millis() - _lastSend > sendInterval_) &&
        selfAddress_ != ENDNODE_ADDRESS) // was ENDNODE_ADDRESS, could be targetAddress_
    {
        mode_ = SENDING_MODE;
    }

    if (mode_ == SENDING_MODE)
    {
        // Send a message to another rhmesh node
        // TODO: Take data here
        // TODO: Send our data here
        String packetInfo = "Sending packet: " + String(counter) + ": " + String(readings[0]) + "%M " + String(readings[2]) + "F " + readings[3] + "%H " + readings[1] + "%L " + readings[4] + " mV";
        Serial.printf("Sending \"%s\" to %d...", packetInfo, targetAddress_);
        uint8_t _err =
            RHMeshManager_.sendtoWait(reinterpret_cast<uint8_t *>(&packetInfo[0]),
                                      packetInfo.length(), targetAddress_);
        if (_err == RH_ROUTER_ERROR_NONE)
        {
            // message successfully be sent to the target node, or next neighboring
            // expecting to recieve a simple reply from the target node
            esp_task_wdt_reset();
            Serial.printf(" successfull! Awaiting for Reply\n");

            if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, &_msgRcvBufLen, 3000,
                                                  &_msgFrom))
            {
                char buf_[RH_MESH_MAX_MESSAGE_LEN];

                std::sprintf(buf_, "%s", reinterpret_cast<char *>(_msgRcvBuf));
                msgRcv = std::string(buf_);
                Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", _msgFrom,
                              msgRcv.c_str(), RFM95Modem_.lastRssi());
            }
            else
            {
                Serial.println("No reply, is the target node running?");
            }

            esp_task_wdt_reset();
        }
        else
        {
            Serial.println(
                "sendtoWait failed. No response from intermediary node, are they "
                "running?");

            // TODO: If nobody received, up power? Maybe a function to do default power applied (starts at low value and increases, then stays at whatever value works)
        }
        _lastSend = millis();
        mode_ = RECEIVING_MODE;
    }

    if (mode_ == RECEIVING_MODE)
    {
        // while at it, wait for a message from other nodes
        Serial.println("Receiving mode active");

        if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, &_msgRcvBufLen, 3000,
                                              &_msgFrom))
        {
            char buf_[RH_MESH_MAX_MESSAGE_LEN];

            esp_task_wdt_reset();
            Serial.println("Received a message");
            std::sprintf(buf_, "%s", reinterpret_cast<char *>(_msgRcvBuf));
            msgRcv = std::string(buf_);

            // do something with message, for example pass it through a callback
            Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", _msgFrom,
                          msgRcv.c_str(), RFM95Modem_.lastRssi());

            msgRcv = "";

            std::string _msgRply =
                String("Hi node " + String(_msgFrom) + ", got the message!").c_str();
            uint8_t _err = RHMeshManager_.sendtoWait(
                reinterpret_cast<uint8_t *>(&_msgRply[0]), _msgRply.size(), _msgFrom);
            if (_err == RH_ROUTER_ERROR_NONE)
            {
                // message successfully received by either final target node, or next
                // neighboring node. do nothing...
            }
            else
            {
                Serial.println("Fail to send reply...");
            }

            esp_task_wdt_reset();
        }
    }

    esp_task_wdt_reset();
}

void rhSetup()
{
    if (!RHMeshManager_.init())
        Serial.println("init failed");
    RFM95Modem_.setTxPower(17, false);
    RFM95Modem_.setFrequency(RF95_FREQ);
    RFM95Modem_.setCADTimeout(500);
}