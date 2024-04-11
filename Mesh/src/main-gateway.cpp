#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <cstring>
#include "driver/gpio.h"
// #include "radio_pinouts_and_constants.h"
#include "meshing.h"
// #include ".env.h"
// #include <WiFi.h>
// #include <WiFiClientSecure.h>
#include "time.h"
#include "gateway-backend.h"

// Variable to save current epoch time
unsigned long epochTime;

void rhSetup();
void setupWiFi();
void cycle();
void runGatewayReceiver(int wait_time, uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void runGatewaySender(unsigned int *settings, uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);

struct timeval start;
struct timeval end;
void setup()
{
    Serial.begin(115200);

    setupWiFi();

    esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);

    rhSetup();
    Serial.println(" ---------------- GATEWAY " + String(selfAddress_) +
                   " INIT ---------------- ");
}

uint8_t _msgRcvBuf[RH_MESH_MAX_MESSAGE_LEN];

// Get the current time
unsigned long getTime()
{
    time_t now;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        // Serial.println("Failed to obtain time");
        return (0);
    }
    time(&now);
    return now;
}

// Initialize WiFi (Gateway will use it to send data to backend)
void initWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print('.');
        delay(1000);
    }
    Serial.println(WiFi.localIP());
}

void loop()
{
    gettimeofday(&start, NULL);
    epochTime = getTime();

    // GET route to receive information from backend (cycle period, num measurements, etc.)
    unsigned long gatewaySleep = 24; // hours

    cycle();

    gettimeofday(&end, NULL);
    uint64_t time_taken = (end.tv_sec - start.tv_sec) * microseconds + end.tv_usec - start.tv_usec;
    esp_err_t sleep_error = esp_sleep_enable_timer_wakeup(gatewaySleep * hours_to_seconds * microseconds - time_taken); // takes into account time between start and sleep
    esp_deep_sleep_start();
}

void cycle()
{

    uint8_t _msgFrom;
    uint8_t _msgRcvBufLen = sizeof(_msgRcvBuf);

    if (mode_ == SENDING_MODE)
    {
        unsigned int settings[4] = {time_period, num_measurements, timer, microseconds};
        Serial.printf("Sending data to %d...", targetAddress_);
        runGatewaySender(settings, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);

        mode_ = RECEIVING_MODE;
    }
    if (mode_ == RECEIVING_MODE)
    {
        runGatewayReceiver(120 * 1000, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
    }
}