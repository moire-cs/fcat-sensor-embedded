#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <cstring>
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_clk.h"
#include "ClosedCube_HDC1080.h"
#include "esp_wifi.h"
#include <driver/adc.h>
#include "radio_pinouts_and_constants.h"
//#include "meshing.h"
#include "measurement.h"
#include <EEPROM.h>
#include "esp_timer.h"

// Log tag (ESP-IDF style)
static const char* TAG = "APP";

RTC_DATA_ATTR int64_t start_time;

// Configuration parameters
#define ARDUINO_TASK_STACK_SIZE 4096    // Stack size for Arduino task (bytes); due to change
#define ARDUINO_TASK_PRIORITY    5       // Arduino task priority

// Bit in event group indicating Arduino task completion
#define ARDUINO_FINISHED_BIT     (1 << 0)

// Static memory allocation for Arduino task creation
static StaticTask_t arduinoTaskTCB;
static StackType_t arduinoTaskStack[ARDUINO_TASK_STACK_SIZE];

// Global event group handle for Arduino task notification to main task
EventGroupHandle_t arduino_event_group = NULL;

// Arduino task function
// Currently only contains sensor tasks
void arduinoTask(void *pvParameters) {
    Serial.println("Arduino task running");

    // Initialize sensor
    measureSetup();

    // Perform measurements
    Measurement currentReading = getReadings();
    boolean isFull = saveReading(currentReading);

    // If storage limit is not reached, print the readings
    if (!isFull) {
        Serial.println("Printing");
    }
    printReadings();
    clearReadings();

    // Notify main task that the work is completed
    xEventGroupSetBits(arduino_event_group, ARDUINO_FINISHED_BIT);

    // Enter infinite loop waiting for main task to delete
    while (true) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/*
// Adjust deep sleep time using RTC calibration data from external 32.768kHz crystal
// Parameters:
//   sleep_time_us - Desired deep sleep duration in microseconds
//   cal_ref       - Ideal reference calibration value, typically 32768

void start_adjusted_deep_sleep(uint64_t sleep_time_us, uint32_t cal_ref) {

    uint32_t cal_device = esp_clk_slowclk_cal_get();
    ESP_LOGI(TAG, "RTC calibration: %u", cal_device);

    uint64_t adjusted_sleep_time_us = sleep_time_us * cal_ref / cal_device;
    ESP_LOGI(TAG, "RTC calibration: device = %u, reference = %u, adjusted sleep time = %llu us",
             cal_device, cal_ref, adjusted_sleep_time_us);

    esp_sleep_enable_timer_wakeup(adjusted_sleep_time_us);
    esp_deep_sleep_start();
}
*/

// Main entry point for ESP-IDF application
// 1. Initialize Arduino environment using initArduino()
// 2. Create event group for communication between Arduino task and main task
// 3. In a loop, statically create the Arduino task, wait for completion signal from Arduino task,
//    delete the task, and recreate it after a delay, forming a periodic loop.
extern "C" void app_main(void) {
    // Initialize Arduino environment
    initArduino();

    // Initialize Serial (if not initialized by initArduino())
    Serial.begin(115200);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "ESP-IDF app_main started");

    // Create event group to receive completion signal from Arduino task
    arduino_event_group = xEventGroupCreate();

    while (true) {
        // Create Arduino task (static memory allocation)
        TaskHandle_t arduinoTaskHandle = xTaskCreateStatic(
            arduinoTask,                 // Task entry function
            "arduinoTask",               // Task name
            ARDUINO_TASK_STACK_SIZE,     // Stack size (bytes)
            NULL,                        // Task parameters (unused here)
            ARDUINO_TASK_PRIORITY,       // Task priority
            arduinoTaskStack,            // Static stack memory
            &arduinoTaskTCB              // Static task control block
        );
        ESP_LOGI(TAG, "Arduino task created successfully");

        // Wait for completion signal from Arduino task (blocking wait)
        EventBits_t bits = xEventGroupWaitBits(
            arduino_event_group,        // Event group handle
            ARDUINO_FINISHED_BIT,       // Bit to wait for
            pdTRUE,                     // Auto-clear the bit upon receiving
            pdFALSE,                    // Wait for any single bit
            portMAX_DELAY               // Wait indefinitely
        );
        if (bits & ARDUINO_FINISHED_BIT) {
            ESP_LOGI(TAG, "Received Arduino task completion signal");
        }

        // Delete Arduino task
        vTaskDelete(arduinoTaskHandle);
        ESP_LOGI(TAG, "Arduino task deleted");

        // Clear event group bits (if not auto-cleared)
        xEventGroupClearBits(arduino_event_group, ARDUINO_FINISHED_BIT);
        ESP_LOGI("Deep Sleep", "Entering deep sleep for 5 seconds...");

        // Desired deep sleep duration set to 5 seconds (5000000 microseconds)
        #define DESIRED_SLEEP_TIME_US 5000000
        // Adjust and enter deep sleep using external 32.768kHz crystal calibration
        // Reference calibration value set to 32768 (ideal value)
        //start_adjusted_deep_sleep(DESIRED_SLEEP_TIME_US, 32768);
    }
}