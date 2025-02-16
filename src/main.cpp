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
//#include "meshing.h"
#include "measurement.h"
#include <EEPROM.h>


// 日志标签（ESP-IDF 风格）
static const char* TAG = "APP";

// ---------------------------------------------------------
// 全局时间变量，等到wifi和radio再搞进去：
// ---------------------------------------------------------
//struct timeval tv_now;
//struct timeval start;



// ---------------------------------------------------------
// 配置参数：
// ---------------------------------------------------------
#define ARDUINO_TASK_STACK_SIZE 4096    // 分配给 Arduino 任务的栈大小（字节）; due to change
#define ARDUINO_TASK_PRIORITY    5       // Arduino 任务优先级

// 定义事件组中用于标识 Arduino 任务完成工作的位
#define ARDUINO_FINISHED_BIT     (1 << 0)

// ---------------------------------------------------------
// 静态分配内存：用于创建 Arduino 任务
// ---------------------------------------------------------
static StaticTask_t arduinoTaskTCB;
static StackType_t arduinoTaskStack[ARDUINO_TASK_STACK_SIZE];

// 全局事件组句柄，用于 Arduino 任务通知主任务
EventGroupHandle_t arduino_event_group = NULL;

// ---------------------------------------------------------
// Arduino 任务函数
//
// 目前只放sensor任务进来
// ---------------------------------------------------------
void arduinoTask(void *pvParameters) {
    Serial.println("现在是 arduino");

    //初始化sensor
    measureSetup();

    //测量数据
    Measurement currentReading = getReadings();
    boolean isFull = saveReading(currentReading);

    // 如果未达到存储上限，则进入睡眠模式
    if (!isFull) {
        Serial.println("Printing");
    }
    printReadings();
    clearReadings();
    // 任务完成主要工作后，发送完成信号给主任务
    xEventGroupSetBits(arduino_event_group, ARDUINO_FINISHED_BIT);

    // 进入无限循环等待主任务删除
    while (true) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ---------------------------------------------------------
// ESP-IDF 主入口函数
// 1. 调用 initArduino() 初始化 Arduino 环境；
// 2. 创建事件组供 Arduino 任务与主任务通信；
// 3. 在循环中，静态创建 Arduino 任务，等待 Arduino 发出完成信号后删除任务，
//    然后延时后重新创建任务，形成周期性循环。
// ---------------------------------------------------------
extern "C" void app_main(void) {
    // 初始化 Arduino 环境
    initArduino();

    // 初始化 Serial（如未在 initArduino() 内部自动初始化）
    Serial.begin(115200);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "ESP-IDF app_main 启动");

    // 创建事件组，用于接收 Arduino 任务完成信号
    arduino_event_group = xEventGroupCreate();

    while (true) {
        // 创建 Arduino 任务（静态内存分配）
        TaskHandle_t arduinoTaskHandle = xTaskCreateStatic(
            arduinoTask,                 // 任务入口函数
            "arduinoTask",               // 任务名称
            ARDUINO_TASK_STACK_SIZE,     // 栈大小（字节）
            NULL,                        // 任务参数（此处未使用）
            ARDUINO_TASK_PRIORITY,       // 任务优先级
            arduinoTaskStack,            // 静态分配的栈内存
            &arduinoTaskTCB              // 静态分配的任务控制块
        );
        ESP_LOGI(TAG, "Arduino 任务创建成功");

        // 等待 Arduino 任务发送完成信号（阻塞等待）
        EventBits_t bits = xEventGroupWaitBits(
            arduino_event_group,        // 事件组句柄
            ARDUINO_FINISHED_BIT,       // 需要等待的位
            pdTRUE,                     // 自动清除接收到的位
            pdFALSE,                    // 不要求同时等待多个位
            portMAX_DELAY               // 无限等待
        );
        if (bits & ARDUINO_FINISHED_BIT) {
            ESP_LOGI(TAG, "收到 Arduino 任务完成信号");
        }

        // 删除 Arduino 任务
        vTaskDelete(arduinoTaskHandle);
        ESP_LOGI(TAG, "Arduino 任务已删除");

        // 清除事件组中的位（如果还未自动清除）
        xEventGroupClearBits(arduino_event_group, ARDUINO_FINISHED_BIT);

        // 等待 5 秒后重新创建任务，形成周期性循环
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
