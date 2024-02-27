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
#include "radio_pinouts_and_constants.h"
#include "meshing.h"
#include "measurement.h"
#include <EEPROM.h>

struct timeval tv_now;
struct timeval start;

void rhSetup();
void runReceiver(uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void runSending(String packetInfo, uint8_t targetAddress_, uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);

void setup()
{
    gettimeofday(&start, NULL);
    Serial.begin(115200);
    esp_wifi_set_mode(WIFI_MODE_NULL);

    pinMode(moisture, INPUT);
    pinMode(light, INPUT);
    pinMode(battery, INPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(misoPin, INPUT);
    pinMode(mosiPin, OUTPUT);
    pinMode(GPIO_NUM_26, OUTPUT);

    esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);

    adc2_config_channel_atten(ADC2_CHANNEL_8, ADC_ATTEN_0db);
    // add current thread to WDT watch
    EEPROM.begin(EEPROM_SIZE);
    measureSetup();
    rhSetup();
    Serial.println(" ---------------- LORA NODE " + String(selfAddress_) +
                   " INIT ---------------- ");
}

long _lastSend = 0, sendInterval_ = 3000; // send every 10 seconds
uint8_t _msgRcvBuf[RH_MESH_MAX_MESSAGE_LEN];

void loop()
{
    getReadings();
    String packetInfo = "Sending packet: " + String(readings[0]) + "%M " + String(readings[2]) + "F " + readings[3] + "%H " + readings[1] + "%L " + readings[4] + " mV";
    Serial.println(packetInfo);
    gettimeofday(&tv_now, NULL);
    boolean isFull = saveReadings(&tv_now);

    if (!isFull)
    {
        Serial.println("Start: " + String(start.tv_sec) + "." + String(start.tv_usec));
        uint64_t sleepTime = ((timer + start.tv_sec - tv_now.tv_sec)) * microseconds + start.tv_usec - tv_now.tv_usec;
        Serial.println("Sleeping at: " + String(tv_now.tv_sec) + "." + String(tv_now.tv_usec) + " seconds and for: " + String((double)sleepTime / microseconds) + " seconds");
        esp_err_t sleep_error = esp_sleep_enable_timer_wakeup(sleepTime); // takes into account time between start and sleep
        esp_deep_sleep_start();
    }
    printReadings();

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
        runSending(packetInfo, targetAddress_, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
        _lastSend = millis();
        mode_ = RECEIVING_MODE;
    }

    if (mode_ == RECEIVING_MODE)
    {
        runReceiver(_msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
    }
    esp_task_wdt_reset();

    // Prepare for new readings (would be next day)
    clearReadings();
    // tv_now = {0, 0};
    Serial.println("Sleeping: " + String(tv_now.tv_sec) + "." + String(tv_now.tv_usec) + " seconds");
    esp_err_t sleep_error = esp_sleep_enable_timer_wakeup(timer * microseconds);
    esp_deep_sleep_start();
}
