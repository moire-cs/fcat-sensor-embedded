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

float duration;
void rhSetup();
bool runTimeSyncReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
void runSender(uint8_t targetAddress_, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_);
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
#define ARDUINO_TASK_STACK_SIZE 4096    // 分配给 Arduino 任务的栈大小（字节）; due to change
#define ARDUINO_TASK_PRIORITY    5       // Arduino 任务优先级

// 全局变量，用于记录每个周期的起始时间（单位：微秒），全部基于 esp_timer_get_time()
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
// 执行状态机的一个完整周期：
    // 按照原代码的状态顺序： WAITING -> SENSING -> RECEIVING -> SENDING
    // 各状态的功能函数（wait(), sense(), receive(), send(), sleep()）均假定在 include 文件中实现

    while (1) {
        switch (state) {
            case WAITING:
                wait();   // 等待同步（解析时间参数，设置 timer 等）
                break;
            case SENSING:
                sense();  // 采集数据
                break;
            case RECEIVING:
                receive(); // 接收数据
                break;
            case SENDING:
                send();   // 发送数据，并调用 sleep() 进行延时等待下一周期
                // 这里认为 send() 内部调用了 sleep()，完成本周期
                goto cycle_end;  // 退出状态机循环，结束本次任务
            default:
                Serial.println("Reached default state");
                break;
        }
    }
cycle_end:
    // 通知主任务：本周期结束
    xEventGroupSetBits(arduino_event_group, ARDUINO_FINISHED_BIT);
    vTaskDelete(NULL);
}


// ----------------------------
// 改写 sleep() 函数：
// 计算本周期已经过的时间（基于 esp_timer_get_time()），
// 并利用 vTaskDelay() 延时剩余时间（微秒 -> 毫秒转换后使用 pdMS_TO_TICKS()）。
// ----------------------------
void sleep() {
    uint64_t now_time = esp_timer_get_time();
    uint64_t elapsed = now_time - start_time;
    // 若 timer 未设置，则默认周期为5秒
    if (timer == 0) {
        timer = 5000000ULL;
    }
    uint64_t sleepTime = (elapsed < timer) ? (timer - elapsed) : 0;
    Serial.println("Elapsed: " + String((double)elapsed / microseconds, 3) +
                   " s, sleeping for: " + String((double)sleepTime / microseconds, 3) + " s");
    vTaskDelay(pdMS_TO_TICKS(sleepTime / 1000ULL));
    start_time = esp_timer_get_time();
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
        float* tokens = (float*)malloc(sizeof(float) * data_count);
        splitn(tokens, timeSyncRcv.c_str(), ", ", data_count);
        // for (int k = 0; k < data_count; k++) {
        //     Serial.printf("%0.3f, ", tokens[k]);
        // }
        duration = (tokens[0]);
        num_measurements = (tokens[1]);
        time_sync_tolerance = (tokens[2]);
        mesh_sync_tolerance = (tokens[3]);
        // duration = 0.02;
        // num_measurements = 2;
        // time_sync_tolerance = 0.005;
        // mesh_sync_tolerance = 0.005;

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
    runSender(targetAddress_, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);

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
    boolean isFull = saveReading(m); // save the reading to flash (also gets a boolean if the readings are full)

    state = (isFull && selfAddress_ != ENDNODE_ADDRESS) ? WAITING : SENSING;
    if (state == SENSING) {
        sleep();
    }
}




// ---------------------------------------------------------
// ESP-IDF main entry function
// 1. Call initArduino() to initialize the Arduino environment;
// 2. Create an event group for communication between the Arduino task and the main task;
// 3. In a loop, statically create the Arduino task, wait for a completion signal from Arduino,
//    then delete the task, wait for a delay, and recreate the task to form a periodic cycle.
// ---------------------------------------------------------
extern "C" void app_main(void) {
    // 初始化 Arduino 环境和 Serial
    initArduino();
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "ESP-IDF sensor_node app_main 启动");

    // 设置 WiFi 为 NULL 模式（若不需要WiFi）
    esp_wifi_set_mode(WIFI_MODE_NULL);

    // 初始化看门狗、ADC、传感器、无线模块等
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
    adc2_config_channel_atten(ADC2_CHANNEL_8, ADC_ATTEN_0db);
    measureSetup();
    rhSetup();
    Serial.println(" ---------------- LORA NODE " + String(selfAddress_) +
                   " INIT ---------------- ");

    // 创建事件组，用于传递任务完成信号
    arduino_event_group = xEventGroupCreate();

    // 用 esp_timer_get_time() 记录本周期起始时间（单位：微秒）
    start_time = esp_timer_get_time();

    // 主循环：反复创建 sensorTask 任务，等待其完成后再延时到下一个周期
    while (true) {
        // 将状态重置为初始状态
        state = WAITING;
        // 创建 sensorNode 任务（静态内存分配）
        TaskHandle_t sensorTaskHandle = xTaskCreateStatic(
            arduinoTask,
            "arduinoTask",
            ARDUINO_TASK_STACK_SIZE,
            NULL,
            ARDUINO_TASK_PRIORITY,
            arduinoTaskStack,
            &arduinoTaskTCB
        );
        ESP_LOGI(TAG, "Sensor task 创建成功");

        // 等待 sensorTask 完成一个完整周期（事件组等待）
        EventBits_t bits = xEventGroupWaitBits(
            arduino_event_group,
            ARDUINO_FINISHED_BIT,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY
        );
        if (bits & ARDUINO_FINISHED_BIT) {
            ESP_LOGI(TAG, "Sensor task 周期完成");
        }

        // 删除任务（通常 sensorTask 已自行删除，但确保删除）
        vTaskDelete(sensorTaskHandle);

        // 计算本周期内已消耗的时间（单位：微秒）
        uint64_t now_time = esp_timer_get_time();
        uint64_t elapsed = now_time - start_time;
        // 计算延时：如果本周期不足 timer，则延时剩余时间
        uint64_t delay_time_us = (elapsed < timer) ? (timer - elapsed) : 0;
        ESP_LOGI(TAG, "本周期耗时: %llu us, 延时: %llu us", elapsed, delay_time_us);

        // 使用 vTaskDelay() 延时（将微秒转换为毫秒，再转换为 FreeRTOS tick）
        vTaskDelay(pdMS_TO_TICKS(delay_time_us / 1000ULL));

        // 更新周期起始时间
        start_time = esp_timer_get_time();
    }
}




