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

struct timeval tv_now;
struct timeval start;

void rhSetup();
void runSender(String packetInfo, uint8_t targetAddress_, uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void runReceiver(uint16_t time, uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);

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

    measureSetup();
    rhSetup();
    Serial.println(" ---------------- LORA NODE " + String(selfAddress_) +
                   " INIT ---------------- ");
}

uint8_t _msgRcvBuf[RH_MESH_MAX_MESSAGE_LEN];

void loop()
{
    // It wakes and starts here (always takes a measurement upon waking up)
    Measurement m = getReadings();
    m.printMeasurement();            // Prints measurement (this will not be needed later)
    boolean isFull = saveReading(m); // save the reading to flash (also gets a boolean if the readings are full)

    // change to sending mode if readings are full AND this is not the endnode
    mode_ = (isFull && selfAddress_ != ENDNODE_ADDRESS) ? SENDING_MODE : SENSING_MODE;

    if (mode_ == SENSING_MODE)
    {
        Serial.println("Start: " + String(start.tv_sec) + "." + String(start.tv_usec));
        gettimeofday(&tv_now, NULL); // get time of day
        // Calculates time it takes between startup and now
        uint64_t sleepTime = ((timer + start.tv_sec - tv_now.tv_sec)) * microseconds + start.tv_usec - tv_now.tv_usec;
        Serial.println("Sleeping at: " + String(tv_now.tv_sec) + "." + String(tv_now.tv_usec) + " seconds and for: " + String((double)sleepTime / microseconds) + " seconds");
        esp_err_t sleep_error = esp_sleep_enable_timer_wakeup(sleepTime); // takes into account time between start and sleep
        esp_deep_sleep_start();
    }

    printReadings(); // unnecessary later

    uint8_t _msgFrom;
    uint8_t _msgRcvBufLen = sizeof(_msgRcvBuf);

    if (mode_ == SENDING_MODE)
    {
        // Send a message to another rhmesh node
        // TODO: Send our data here
        String packetInfo = "Hello"; // temp message
        Serial.printf("Sending data to %d...", targetAddress_);
        runSender(&packetInfo, targetAddress_, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);

        // Prepare for new readings (would be next day)
        isFull = false;
        clearReadings();

        mode_ = RECEIVING_MODE;
    }

    if (mode_ == RECEIVING_MODE)
    {
        Serial.println("Receiving mode active");
        // We need to be receiving for a random time
        uint16_t wait_time = random(1000, 5000);
        runReceiver(wait_time, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
    }
    esp_task_wdt_reset();
    gettimeofday(&tv_now, NULL);
    Serial.println("Sleeping: " + String(tv_now.tv_sec) + "." + String(tv_now.tv_usec) + " seconds");
    esp_err_t sleep_error = esp_sleep_enable_timer_wakeup(timer * microseconds);
    esp_deep_sleep_start();
}
