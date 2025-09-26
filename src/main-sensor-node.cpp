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
#include "esp_attr.h" 


timeval start, tv_now;
timeval end;
const uint64_t microsecond = 1000000ULL; // 1 s = 1e6 us

float duration;
void rhSetup();
bool runTimeSyncReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void runSender(uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void runReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void wait();
void send();
void receive();
void sleep();
void sense();
static const char* TAG = "ArduinoTask";

// ---------------------------------------------------------
// Setups：
// ---------------------------------------------------------
#define ARDUINO_TASK_STACK_SIZE 16348    // stack for arduino task; due to change; try 8192
#define ARDUINO_TASK_PRIORITY    5       // Arduino task priority

// Global variable used to record the start time of each cycle (in microseconds), all based on esp_timer_get_time()

uint64_t start_time;

// Define the bit in the event group used to indicate Arduino task completion
#define ARDUINO_FINISHED_BIT     (1 << 0)

// ---------------------------------------------------------
// Static memory allocation: for creating the Arduino task
// ---------------------------------------------------------
static StaticTask_t arduinoTaskTCB;
static StackType_t arduinoTaskStack[ARDUINO_TASK_STACK_SIZE];

// Global event group handle, used by the Arduino task to notify the main task
EventGroupHandle_t arduino_event_group = NULL;

// ---------------------------------------------------------
// Arduino task function
//Everything but timing
// ---------------------------------------------------------
void arduinoTask(void *pvParameters) {
    // Execute one full cycle of the state machine:
    // According to the original order: WAITING -> SENSING -> RECEIVING -> SENDING

    while (1) {
        Serial.printf("[SensorNode] Current state: %d\n", state);
        switch (state) {
            case WAITING:
                wait();   
                break;
            case SENSING:
                sense(); 
                break;
            case RECEIVING:
                receive(); 
                break;
            case SENDING:
                send();  
                goto cycle_end;  // exit task after sending for sleep
            default:
                Serial.println("Reached default state");
                break;
        }
    }
cycle_end:
    // Notify main task: current cycle completed
    xEventGroupSetBits(arduino_event_group, ARDUINO_FINISHED_BIT);
    vTaskDelete(NULL); // kill itself
}


void wait() {
    Serial.println("\n====== wait()  ======");      
    uint8_t _msgFrom;
    uint8_t _timeSyncRcvBufLen = sizeof(_timeSyncRcvBuf);
    Serial.println("[wait] waiting for gateway…");   // [LOG]
    esp_task_wdt_reset();

    if (runTimeSyncReceiver(5000, _timeSyncRcvBuf, &_timeSyncRcvBufLen,
                            &_msgFrom, RFM95Modem_, RHMeshManager_)) {

        gettimeofday(&start, NULL);
        Serial.printf("[wait] Received data from node %d\n", _msgFrom); 
        Serial.printf("[wait] Raw payload: \"%s\"\n", timeSyncRcv.c_str());

        int data_count = 4;
        float* tokens = (float*)malloc(sizeof(float) * data_count);
        splitn(tokens, timeSyncRcv.c_str(), ", ", data_count);

        duration             =  tokens[0];
        num_measurements     =  tokens[1];
        time_sync_tolerance  =  tokens[2];
        mesh_sync_tolerance  =  tokens[3];

        timer = duration * hours_to_seconds / num_measurements;

        Serial.printf("[wait] Parsed: duration=%.3f h, num=%d, "
                      "syncTol=%.3f, meshTol=%.3f, timer(us)=%llu\n",
                      duration, (int)num_measurements,
                      time_sync_tolerance, mesh_sync_tolerance, timer);

        state = SENSING;
        Serial.println("[wait] ➜ state = RECEIVING");       // [LOG]
    } 
    else {
        state = SENSING;
        Serial.println("[wait] Time‑sync failed, will retry next cycle");  // [LOG]
    }
}

void send() {
    // [LOG]
    Serial.println("\n====== send() ======");    

    esp_task_wdt_reset();
    uint8_t _msgFrom;
    uint8_t _msgRcvBufLen = sizeof(_msgRcvBuf);

    Serial.printf("[send] Target address = %d\n", GATEWAY_ADDR);    // [LOG]

    // The original call
    runSender(_msgRcvBuf, &_msgRcvBufLen,
              &_msgFrom, RFM95Modem_, RHMeshManager_);

    // Prepare for next readings
    isFull = false;
    clearReadings();

    // [LOG]
    Serial.println("[send] Readings cleared, isFull reset → SENSING");
}


void receive() {
    Serial.println("\n====== receive() ======");
    esp_task_wdt_reset();
    uint8_t _msgFrom;
    uint8_t _msgRcvBufLen = sizeof(_msgRcvBuf);

    Serial.println("Receiving mode active");
    // We need to be receiving for a random time
    uint16_t wait_time = random(1000, 5000);
    runReceiver(wait_time, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
     
    esp_task_wdt_reset();
    Serial.println("[receive] (placeholder) ➜ state = SENDING");
    state = SENDING;
}

void sense() {
    Serial.println("\n====== sense() ======");

    Measurement m = getReadings();
    Serial.print("[sense] Measurement captured: ");
    printMeasurement(m);

    bool isFullLocal = saveReading(m);
    Serial.printf("[sense] saveReading → isFull=%d\n", isFullLocal);

    //state = (isFullLocal && selfAddress_ != ENDNODE_ADDRESS) ? WAITING : SENSING;
    state = RECEIVING;
    Serial.printf("[sense] Next state = %d\n", state);
    
}




// ---------------------------------------------------------
// ESP-IDF main entry function
// 1. Call initArduino() to initialize the Arduino environment;
// 2. Create an event group for communication between the Arduino task and the main task;
// 3. In a loop, statically create the Arduino task, wait for a completion signal from Arduino,
//    then delete the task, wait for a delay, and recreate the task to form a periodic cycle.
// ---------------------------------------------------------
extern "C" void app_main(void) {
    // Initialize Arduino environment and Serial
    initArduino();
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "ESP-IDF sensor_node app_main started");

    // Set WiFi to NULL mode (if WiFi is not needed)
    esp_wifi_set_mode(WIFI_MODE_NULL);

    // Initialize watchdog, ADC, sensors, radio module, etc.
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
    adc2_config_channel_atten(ADC2_CHANNEL_8, ADC_ATTEN_0db);
    measureSetup();
    rhSetup();
    state = WAITING;
    Serial.println(" ---------------- LORA NODE " + String(selfAddress_) +
                   " INIT ---------------- ");

    // Create event group for signaling task completion
    arduino_event_group = xEventGroupCreate();

    // Main loop: repeatedly create sensorTask, wait for it to complete, then delay to next cycle
    while (true) {
        // Create sensorNode task (static memory allocation)
        gettimeofday(&start, NULL);
        TaskHandle_t sensorTaskHandle = xTaskCreateStatic(
            arduinoTask,
            "arduinoTask",
            ARDUINO_TASK_STACK_SIZE,
            NULL,
            ARDUINO_TASK_PRIORITY,
            arduinoTaskStack,
            &arduinoTaskTCB
        );
        ESP_LOGI(TAG, "Sensor task created successfully");

        // Wait for sensorTask to complete one full cycle (wait for event group)
        EventBits_t bits = xEventGroupWaitBits(
            arduino_event_group,
            ARDUINO_FINISHED_BIT,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY
        );
        if (bits & ARDUINO_FINISHED_BIT) {
            ESP_LOGI(TAG, "Sensor task cycle completed");
        }
        // Task delete
        ESP_LOGI(TAG, "Sensor task deleted");

        gettimeofday(&end, NULL);                       // NEW: record cycle end time
        uint64_t time_taken_us = (end.tv_sec  - start.tv_sec)  * microsecond
                            + (end.tv_usec - start.tv_usec);

        const uint64_t TARGET_US = 50ULL * microsecond;        // Match 50 s with gateway

        uint64_t sleep_us = (time_taken_us < TARGET_US)
                        ? (TARGET_US - time_taken_us)
                        : 100000ULL;                           // If overtime, sleep at least 0.1 s

        Serial.printf("Cycle exec: %.2f s, deep-sleep: %.2f s\n",
                    time_taken_us / 1e6, sleep_us / 1e6);

        esp_sleep_enable_timer_wakeup(sleep_us);
        Serial.println("Entering deep sleep …");
        esp_deep_sleep_start();
    }
}





