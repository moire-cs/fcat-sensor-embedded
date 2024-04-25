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
#include <WiFi.h>
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
void runGatewayReceiver(int wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void runGatewaySender(unsigned int* settings, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);

struct timeval start;
struct timeval end;
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector  
    CLEAR_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_RST_ENA);
    Serial.begin(115200);
    gettimeofday(&start, NULL);

    // setupWiFi();

    esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);

    pinMode(GPIO_NUM_26, OUTPUT);
    digitalWrite(GPIO_NUM_26, HIGH);
    delay(100);
    adc2_config_channel_atten(ADC2_CHANNEL_8, ADC_ATTEN_0db);
    esp_wifi_set_mode(WIFI_MODE_NULL);
    
    int battery_level;
    esp_err_t r = adc2_get_raw(ADC2_CHANNEL_8, ADC_WIDTH_12Bit, &battery_level); // this will be out of 4095
    Serial.printf("%0d\n", battery_level);
    digitalWrite(GPIO_NUM_26, LOW);

    rhSetup();
    Serial.println(" ---------------- GATEWAY " + String(selfAddress_) +
        " INIT ---------------- ");
}

// uint8_t _msgRcvBuf[RH_MESH_MAX_MESSAGE_LEN];

// Get the current time
unsigned long getTime() {
    time_t now;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        // Serial.println("Failed to obtain time");
        return (0);
    }
    time(&now);
    return now;
}

// Initialize WiFi (Gateway will use it to send data to backend)
// void initWiFi() {
//     WiFi.mode(WIFI_STA);
//     WiFi.begin(ssid, password);
//     Serial.print("Connecting to WiFi ..");
//     while (WiFi.status() != WL_CONNECTED) {
//         Serial.print('.');
//         delay(1000);
//     }
//     Serial.println(WiFi.localIP());
// }

void loop() {
    // gettimeofday(&start, NULL);
    // epochTime = getTime();

    // GET route to receive information from backend (cycle period, num measurements, etc.)
    // unsigned long gatewaySleep = 24; // hours
    runTimeSync();
    receive();
    sleep();
    // cycle();
}

void runTimeSync() {
    cur_times = getTimes();
    esp_task_wdt_reset();

    uint8_t _msgFrom;
    uint8_t _msgRcvBufLen = sizeof(_msgRcvBuf);
    Serial.printf("Sending data to %d...", RH_BROADCAST_ADDRESS);
    runGatewaySender(settings, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
}

void receive() {
    esp_task_wdt_reset();
    uint8_t _msgFrom;
    uint8_t _msgRcvBufLen = sizeof(_msgRcvBuf);
    runGatewayReceiver(15 * 1000, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
}

void sleep() {
    esp_task_wdt_reset();
    gettimeofday(&end, NULL);
    uint64_t time_taken = (end.tv_sec - start.tv_sec) * microseconds + end.tv_usec - start.tv_usec;
    uint64_t sleepTime = dur * hours_to_seconds * microseconds * (1 + 6 * time_sync_tolerance) - time_taken;
    Serial.println("Sleeping for: " + String((double)sleepTime / microseconds) + " seconds");
    esp_err_t sleep_error = esp_sleep_enable_timer_wakeup(sleepTime); // takes into account time between start and sleep
    esp_task_wdt_reset();
    esp_deep_sleep_start();
}
