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
#include "soc/rtc_cntl_reg.h"

timeval start, tv_now;

void rhSetup();
bool runTimeSyncReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void runSender(String packetInfo, uint8_t targetAddress_, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void runReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void wait();
void send();
void receive();
void sleep();
void sense();

void setup() {
    // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector  
    // CLEAR_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_RST_ENA);
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
}

uint8_t _msgRcvBuf[RH_MESH_MAX_MESSAGE_LEN];
uint8_t _timeSyncRcvBuf[RH_MESH_MAX_MESSAGE_LEN];

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
        // Serial.println("waiting 2");
        Serial.println("Received data from: " + String(_msgFrom));
        // "{ "numMeasurements" : "4", "duration" : "24", "syncTimeTolerance" : "5", "meshTimeTolerance" : "5" }"
        // "numMeasurements, duration, syncTimeTolerance, meshTimeTolerance"
        int data_count = 4;
        float* tokens = splitn(timeSyncRcv, ", ", data_count);

        // duration = (tokens[0]);
        // num_measurements = (tokens[1]);
        // time_sync_tolerance = (tokens[2]);
        // mesh_sync_tolerance = (tokens[3]);
        duration = 0.02;
        num_measurements = 2;
        time_sync_tolerance = 0.005;
        mesh_sync_tolerance = 0.005;

        timer = duration * hours_to_seconds / (num_measurements); // (equally spaces out measurements) converted to microseconds in code

        state = RECEIVING;
    }

}

void send() {
    esp_task_wdt_reset();
    uint8_t _msgFrom;
    uint8_t _msgRcvBufLen = sizeof(_msgRcvBuf);

    // Send a message to another rhmesh node
    // TODO: Send our data here
    // String packetInfo = "Hello"; // temp message
    Serial.printf("Sending data to %d...", targetAddress_);
    String packetInfo = "Hello World!";
    runSender(packetInfo, targetAddress_, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);

    // Prepare for new readings (would be next day)
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
    uint16_t wait_time = random(1000, 5000);
    runReceiver(wait_time, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
    esp_task_wdt_reset();
    state = SENDING;
}

void sense() {
    Measurement m = getReadings();
    printMeasurement(m);             // Prints measurement (this will not be needed later)
    isFull = saveReading(m); // save the reading to flash (also gets a boolean if the readings are full)

    state = (isFull && selfAddress_ != ENDNODE_ADDRESS) ? WAITING : SENSING;
    if (state == SENSING) {
        sleep();
    }
}

void sleep() {
    Serial.println("Start: " + String(start.tv_sec) + "." + String(start.tv_usec));
    Serial.println("Timer: " + String(timer));
    gettimeofday(&tv_now, NULL); // get time of day

    // Calculates time it takes between startup and now
    uint64_t sleepTime = ((timer + start.tv_sec - tv_now.tv_sec)) * microseconds + start.tv_usec - tv_now.tv_usec;
    Serial.println("Sleeping at: " + String(tv_now.tv_sec) + "." + String(tv_now.tv_usec) + " seconds and for: " + String((double)sleepTime / microseconds) + " seconds");
    esp_err_t sleep_error = esp_sleep_enable_timer_wakeup(sleepTime); // takes into account time between start and sleep
    esp_task_wdt_reset();
    esp_deep_sleep_start();
}