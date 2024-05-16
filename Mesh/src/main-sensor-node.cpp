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
#include "node-info.h"
#include "node-numbers.h"
#include "radio_pinouts_and_constants.h"
#include "meshing.h"

#include "measurement.h"
#include "soc/rtc_cntl_reg.h"


timeval start, tv_now;
bool doSleep = true;
float battery_factor = 0.95;
int battery_max = 3760; // Battery at 4 V --> (4 * 4095/(3.3 * 1.32)) --> 1.32 is undoing the voltage division
uint16_t wait_time = 0;

void rhSetup();
bool runTimeSyncReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void runSender(uint8_t targetAddress_, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void runReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void wait();
void send();
void receive();
void sleep();
void sense();

void setup() {
    gettimeofday(&start, NULL);
    Serial.begin(115200);
    esp_wifi_set_mode(WIFI_MODE_NULL);

    esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);

    adc2_config_channel_atten(ADC2_CHANNEL_8, ADC_ATTEN_0db);
    // add current thread to WDT watch

    measureSetup();
    rhSetup();
    Serial.println(" ---------------- LORA NODE " + String(selfAddress_) +
        " INIT ---------------- ");
    ENABLE_ACC_RAIL();
    delay(100);
    int battery_level;
    esp_err_t r = adc2_get_raw(ADC2_CHANNEL_8, ADC_WIDTH_12Bit, &battery_level); // this will be out of 4095
    doSleep = !(battery_level >= (battery_max * battery_factor));
    
    Serial.printf("Battery Level: %d\n", battery_level);
    Serial.println("doSleep: " + String(doSleep));
    DISABLE_ACC_RAIL();
    if (battery_level <= 3560)
        sleep();
}

void loop() {
    switch (state) {
    case WAITING:
        wait();
        break;
    case SENSING:
        sense();
        break;
    case SENDING:
        send();
        break;
    case RECEIVING:
        receive();
        break;
    default:
        Serial.println("Reached default");
        break;
    }
}

void wait() {
    uint8_t _msgFrom;
    uint8_t _timeSyncRcvBufLen = sizeof(_timeSyncRcvBuf);
    Serial.println("waiting...");

    esp_task_wdt_reset();
    if (runTimeSyncReceiver(1000, _timeSyncRcvBuf, &_timeSyncRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_)) {

        gettimeofday(&start, NULL);
        
        Serial.println("Received data from: " + String(_msgFrom));

        int data_count = 4;
        float* tokens = (float*)malloc(sizeof(float) * data_count);
        splitn(tokens, timeSyncRcv.c_str(), ", ", data_count);

        duration = (tokens[0]); // 0.02
        num_measurements = (tokens[1]); // 2
        time_sync_tolerance = (tokens[2]); // 0.007
        sync_duration = (int) (tokens[3]); // 20
        Serial.printf("%0.4f\n", duration); // 0.0200

        srand(selfAddress_ + curr_time);

        timer = ((int) (duration * hours_to_seconds)) / num_measurements; // (equally spaces out measurements) converted to microseconds in code

        state = RECEIVING;
    }
}

void send() {
    esp_task_wdt_reset();
    uint8_t _msgFrom;
    uint8_t _msgRcvBufLen = sizeof(_msgRcvBuf);

    Serial.printf("Sending data to %d...", targetAddress_);
    runSender(targetAddress_, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);

    Serial.println("Receiving mode active");
    runReceiver((sync_duration - wait_time) + 1000, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);

    // Prepare for new readings
    isFull = false;
    clearReadings();
    state = SENSING;
    sleep();

}

void receive() {
    esp_task_wdt_reset();
    uint8_t _msgFrom;
    uint8_t _msgRcvBufLen = sizeof(_msgRcvBuf);

    Serial.println("Receiving mode active");

    // We need to be receiving for a random time
    wait_time = (rand() % (sync_duration + 1000 + 1)) + 1000;
    runReceiver(wait_time, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
    esp_task_wdt_reset();
    state = SENDING;
}

void sense() {
    Measurement m = getReadings();
    // printMeasurement(m);  // Prints measurement (this will not be needed later)
    doSleep = !(m.battery_level >= (battery_max * battery_factor));
    isFull = saveReading(m); // save the reading to flash (also gets a boolean if the readings are full)

    state = (isFull && selfAddress_ != ENDNODE_ADDRESS) ? WAITING : SENSING;
    if (state == SENSING) {
        sleep();
    }
}

void sleep() {
    gettimeofday(&tv_now, NULL); // get time of day

    // Calculates time it takes between startup and now
    uint64_t sleepTime = (((timer + start.tv_sec - tv_now.tv_sec)) * microseconds + start.tv_usec - tv_now.tv_usec) * (1-5*time_sync_tolerance);
    Serial.println("Sleeping at: " + String(tv_now.tv_sec) + "." + String(tv_now.tv_usec) + " seconds and for: " + String((double)sleepTime / microseconds) + " seconds");
    
    esp_task_wdt_reset();
    DISABLE_ACC_RAIL();
    RFM95Modem_.sleep(); 
    // If battery isn't too charged, sleep
    if (!doSleep) {
        unsigned long start_time_count = millis();
        while ((millis() - start_time_count) < sleepTime/1000) {
            delay(1000);
            esp_task_wdt_reset();
            Serial.print(".");
        }
        gettimeofday(&start, NULL); 
    } else {
        esp_err_t sleep_error = esp_sleep_enable_timer_wakeup(sleepTime); // takes into account time between start and sleep
        esp_task_wdt_reset();
        esp_deep_sleep_start();
    }
    
}