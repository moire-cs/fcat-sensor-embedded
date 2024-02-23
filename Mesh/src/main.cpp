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
#include "pinouts_and_constants.h"
#include "mesh_functionality.h"

ClosedCube_HDC1080 hdc1080;

void rhSetup();
void runReceiver(uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void runSending(String packetInfo, uint8_t targetAddress_, uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);

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
    delay(SOIL_PULSE_COUNT_DELAY);
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
        runSending(packetInfo, targetAddress_, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
        _lastSend = millis();
        mode_ = RECEIVING_MODE;
    }

    if (mode_ == RECEIVING_MODE)
    {
        runReceiver(_msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
    }
    esp_task_wdt_reset();
}
