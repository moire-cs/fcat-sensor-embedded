#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <cstring>
#include "driver/gpio.h"

#include "node-numbers.h"
#include "radio_pinouts_and_constants.h"
#include "gateway-info.h"
#include "gateway-meshing.h"
#include ".env.h"
// #include <WiFi.h>
// #include <WiFiClientSecure.h>
#include "time.h"
#include "soc/rtc_cntl_reg.h"
#include <driver/adc.h>
#include "esp_wifi.h"

// #include "gateway-backend.h"

// Variable to save current epoch time
unsigned long epochTime;


void rhSetup();
// void setupWiFi();
// void cycle();
void runTimeSync();
void receive();
void sleep();
std::string getSerialMessage();
String buildTimeSyncMessage();
void runGatewayReceiver(int wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void runGatewaySender(unsigned int* settings, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);

struct timeval start;
struct timeval end;
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector  
    CLEAR_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_RST_ENA);
    Serial.begin(115200);
    gettimeofday(&start, NULL);

    esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);

    pinMode(GPIO_NUM_26, OUTPUT);
    digitalWrite(GPIO_NUM_26, HIGH);
    delay(100);
    adc2_config_channel_atten(ADC2_CHANNEL_8, ADC_ATTEN_0db);
    esp_wifi_set_mode(WIFI_MODE_NULL);
    
    int battery_level;
    esp_err_t r = adc2_get_raw(ADC2_CHANNEL_8, ADC_WIDTH_12Bit, &battery_level); // this will be out of 4095
    Serial.println(1.32*3.3*battery_level/(4095));
    digitalWrite(GPIO_NUM_26, LOW);

    rhSetup();
    Serial.println(" ---------------- GATEWAY " + String(selfAddress_) +
        " INIT ---------------- ");
}

void loop() {

    runTimeSync();
    receive();
    sleep();
    // cycle();
}

void runTimeSync() {
    unsigned int last_dur = dur;
    
    esp_task_wdt_reset();
    Serial.printf("CYCLE\n");
    std::string time_sync_message = getSerialMessage();
    int data_count = 4;
    if (time_sync_message.length() != 0) {
        float* tokens = (float*)malloc(sizeof(float) * data_count);
        splitn(tokens, time_sync_message, ", ", data_count);
        dur = tokens[0];
        num_meas = tokens[1];
        time_sync_tol = tokens[2];
        sync_dur = (int) (tokens[3] * 1000);
    }

    cur_times = getTimes(last_dur);
    
    settings = buildTimeSyncMessage();
    
    uint8_t _msgFrom;
    uint8_t _msgRcvBufLen = sizeof(_msgRcvBuf);
    Serial.printf("Sending data to %d...", RH_BROADCAST_ADDRESS);
    runGatewaySender(settings, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
}

void receive() {
    esp_task_wdt_reset();
    uint8_t _msgFrom;
    uint8_t _msgRcvBufLen = sizeof(_msgRcvBuf);
    runGatewayReceiver((sync_dur) + 1000, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
}

void sleep() {
    esp_task_wdt_reset();
    gettimeofday(&end, NULL);
    uint64_t time_taken = (end.tv_sec - start.tv_sec) * microseconds + end.tv_usec - start.tv_usec;
    uint64_t sleepTime = dur * hours_to_seconds * microseconds - time_taken;
    Serial.println("Sleeping for: " + String((double)sleepTime / microseconds) + " seconds");
    Serial.printf("Time Factor: %0.4f\n", time_factor);
    esp_err_t sleep_error = esp_sleep_enable_timer_wakeup(sleepTime*time_factor); // takes into account time between start and sleep
    esp_task_wdt_reset();
    esp_deep_sleep_start();
}

std::string getSerialMessage() {
    Serial.println("Reading Serial");

    bool end = false;
    std::string content = "";
    char character;
    unsigned long time = millis();

    try {
        
     while(!end && ((millis() - time) < 1000)) {
          while(Serial.available()) {
               character = Serial.read();
               content+= character;
               if(character == '\n') {
                    end = true;
               }
          }
     }
    } catch (...) {
        Serial.println("ERROR READING SERIAL!!");
    }
    Serial.println("Not reading Serial");
    return content;
}
