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


// 日志标签（ESP-IDF 风格）
static const char* TAG = "APP";

// ---------------------------------------------------------
// 全局时间变量，等到wifi和radio再搞进去：
// ---------------------------------------------------------
//struct timeval tv_now;
//struct timeval start;

RTC_DATA_ATTR int64_t start_time;

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
// 利用外部 32.768kHz 晶振的 RTC 校准数据调整深睡眠时间
//
// 参数：
//   sleep_time_us - 期望深睡眠时间（微秒）
//   cal_ref       - 理想参考校准值，通常设为 32768
// ---------------------------------------------------------

void start_adjusted_deep_sleep(uint64_t sleep_time_us, uint32_t cal_ref) {
    // 获取当前设备的 RTC 校准值（单位：ticks/秒），
    // 理想情况下外部 32.768kHz 晶振应接近 32768 ticks/s
    uint32_t cal_device = esp_clk_slowclk_cal_get();

    // 根据校准因子调整深睡眠时间，确保睡眠时长一致
    uint64_t adjusted_sleep_time_us = sleep_time_us * cal_ref / cal_device;
    ESP_LOGI(TAG, "RTC 校准: 设备 = %u, 参考 = %u, 调整后睡眠时间 = %llu us",
             cal_device, cal_ref, adjusted_sleep_time_us);

    // 设置深睡眠定时器（单位微秒）
    esp_sleep_enable_timer_wakeup(adjusted_sleep_time_us);
    esp_deep_sleep_start();
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
        ESP_LOGI("Deep Sleep", "Entering deep sleep for 5 seconds...");
        // 期望的深睡眠时间设为 5 秒（5000000 微秒）
        #define DESIRED_SLEEP_TIME_US 5000000
        // 使用外部 32.768kHz 晶振校准调整后进入深睡眠，
        // 参考校准值设为 32768（理想值）
        start_adjusted_deep_sleep(DESIRED_SLEEP_TIME_US, 32768);
        //start_adjusted_deep_sleep(timer, cal_ref);

    }
}
