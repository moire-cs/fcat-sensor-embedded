#include <Arduino.h>
#include "driver/gpio.h"
// #include "driver/pulse_cnt.h"
// // // // #include <RH_RF95.h>
// // // // #include <RHReliableDatagram.h>

// // // // #define SERVER_ADDRESS 1

#include <SPI.h>
// // #include <LoRa.h>
#include "ClosedCube_HDC1080.h"
// // // #include <esp32_pcnt.h>

// // // // NO SCREEN
// // // // 5,17,16,4, 27, 14,15
// // // /*

// // // sda: 21
// // // scl: 22
// // // moisture: 34
// // // light: 35
// // // battery: 25
// // // ar: 26

// // // */
// // // ClosedCube_HDC1080 hdc1080;
// // // // PulseCounter pc0;
// // // const int moisture = GPIO_NUM_34; // could change
// // // const int light = GPIO_NUM_35;    // could change
// // // const int clockPin = GPIO_NUM_18;
// // // const int dataPin = GPIO_NUM_21; // could change

// // // const int misoPin = GPIO_NUM_23;
// // // const int mosiPin = GPIO_NUM_19;
// // // const int nssPin = GPIO_NUM_5;
// // // const int resetPin = GPIO_NUM_14;
// // // const int dio0Pin = GPIO_NUM_13;  // 13 or 2
// // // const int dio1Pin = GPIO_NUM_12; // 12

// // // // volatile bool pc0_int_flag = false; // counter 0 interrupt flag
// // // // volatile bool clock_int_flag = false; // clock signal interrupt flag

// // // // RH_RF95 driver;

// // // // RHReliableDatagram manager(driver, SERVER_ADDRESS);

// // // // SHT1x sht1x(dataPin, clockPin);
// // // int counter = 0;

// // // double readings[4] = { 0, 0, 0, 0 };

// // // // uint8_t data[] = "Pong";
// // // // uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

// // // // portMUX_TYPE pcntMux0 = portMUX_INITIALIZER_UNLOCKED;
// // // // IRAM_ATTR void pc0_isr(void*) {
// // // //     // Prevent context switching during the interrupt service routine with an ISR spinlock
// // // //     portENTER_CRITICAL_ISR(&pcntMux0);
// // // //     // set interrupt flag
// // // //     pc0_int_flag = true;
// // // //     // get events
// // // //     portEXIT_CRITICAL_ISR(&pcntMux0);
// // // // }

// // // void setup()
// // // {
// // //     // put your setup code here, to run once:
// // //     Serial.begin(115200);
// // //     // SPI.begin(SCK, MISO, MOSI, SS);

// // //     // pinMode(moisture, INPUT);
// // //     // pinMode(light, INPUT);
// // //     pinMode(clockPin, OUTPUT);
// // //     // pinMode(dataPin, OUTPUT);
// // //     pinMode(misoPin, INPUT);

// // //     pinMode(mosiPin, OUTPUT);
// // //     pinMode(GPIO_NUM_26, OUTPUT);
// // //     digitalWrite(GPIO_NUM_26, HIGH);
// // //     // pinMode(SS, OUTPUT);
// // //     hdc1080.begin(0x40);
// // //     // pcnt_isr_service_install(0);
// // //     // pc0.initialise(moisture, PCNT_PIN_NOT_USED);
// // //     // pc0.set_mode(PCNT_COUNT_DIS, PCNT_COUNT_INC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);
// // //     // pc0.set_filter_value(1000);

// // //     // pc0.clear();
// // //     // pc0.resume();
// // //     // pin

// // //     // pinMode(resetPin, OUTPUT);
// // //     // pinMode(dio0Pin, INPUT);

// // //     while (!Serial)
// // //         Serial.println("LoRa Sender");

// // //     // setup LoRa transceiver moduleu
// // //     //  LoRa.setPins(ss, rst, dio0);
// // //     LoRa.setPins(SS, resetPin, dio0Pin);

// // //     // replace the LoRa.begin(---E-) argument with your location's frequency
// // //     // 433E6 for Asia
// // //     // 866E6 for Europe
// // //     // 915E6 for North America
// // //     // LoRa.setSPIFrequency(1E6);
// // //     // SPI.begin(SCK, MISO, MOSI, nssPin);
// // //     while (!LoRa.begin(915E6)) {
// // //         Serial.println(".");
// // //         delay(500);
// // //     }
// // //     // Change sync word (0xF3) to match the receiver
// // //     // The sync word assures you don't get LoRa messages from other LoRa transceivers
// // //     // ranges from 0-0xFF
// // //     LoRa.setSyncWord(0xF3);
// // //     // LoRa.setTxPower(20);
// // //     Serial.println("LoRa Initializing OK! (sender)");
// // // }
// // // // void printTandRH(HDC1080_MeasurementResolution humidity, HDC1080_MeasurementResolution temperature) {
// // // //   hdc1080.setResolution(humidity, temperature);

// // // //   HDC1080_Registers reg = hdc1080.readRegister();


// // // //   Serial.print("T=");
// // // //   Serial.print(hdc1080.readTemperature());
// // // //   Serial.print("C, RH=");
// // // //   Serial.print(hdc1080.readHumidity());
// // // //   Serial.println("%");
// // // // }
// // // void getReadings()
// // // {
// // //     digitalWrite(GPIO_NUM_26, HIGH);
// // //     // int moisture_val = 100 - (int)((analogRead(moisture) - 900) / 20);
// // //     // int light_val = (int)(100 * analogRead(light) / 4095);
// // //     // int tempF = sht1x.readTemperatureF();
// // //     // int humidity = sht1x.readHumidity();
// // //     // if (pc0_int_flag) {
// // //     //     pc0_int_flag = false; //reset flag
// // //     //     Serial.print("pc0 : ");
// // //     //     switch (pc0.event_status()) // test event type
// // //     //     {
// // //     //     case evt_thres0:
// // //     //         Serial.println("Threshold 0");
// // //     //         break;
// // //     //     case evt_thres1:
// // //     //         Serial.println("Threshold 1");;
// // //     //         break;
// // //     //     case evt_high_lim:
// // //     //         Serial.println("High Limit");;
// // //     //         break;
// // //     //     }
// // //     // }

// // //     // // int new_readings[4] = { moisture_val, /*tempF,*/ /*humidity*/ light_val };
// // //     // readings[0] = moisture_val;
// // //     // readings[1] = light_val;
// // //     readings[2] = hdc1080.readTemperature() * 1.8 + 32;
// // //     readings[3] = hdc1080.readHumidity();
// // //     // return readings;
// // //     // return[moisture_val, light_val];
// // //     digitalWrite(GPIO_NUM_26, LOW);
// // // };

// // // void loop()
// // // {
// // //     // int* readings = getReadings();
// // //     getReadings();
// // //     delay(500);
// // //     Serial.println("Sending packet: " + String(counter) + ": " + String(readings[0]) + "%M " + String(readings[2]) + "F " + readings[3] + "%H " + readings[1] + "%L");
// // //     // Serial.println(sht1x.readRawData(ShtCommand::MeasureRelativeHumidity, dataPin, clockPin));
// // //     // Serial.println("Sending packet " + String(counter) + ": " + String(moisture_val) + "%M " + String(tempF) + "F " + humidity + "%H " + light_val + "%L");

// // //     // sends hello over radio
// // //     LoRa.beginPacket();
// // //     LoRa.print("hello");
// // //     // LoRa.print(String(counter) + ": " + String(readings[0]));
// // //     // LoRa.print(String(counter) + ": " + String(readings[0]) + "%M " + String(readings[2]) + "F " + readings[3] + "%H " + readings[1] + "%L");

// // //     LoRa.endPacket();

// // //     counter++;
// // //     delay(500);
// // // }

// // /*
// //  * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
// //  *
// //  * SPDX-License-Identifier: CC0-1.0
// //  */

// //  // #include "sdkconfig.h"
// //  // #include "freertos/FreeRTOS.h"
// //  // #include "freertos/task.h"
// //  // #include "freertos/queue.h"
// //  // #include "esp_log.h"
// #include "driver/pcnt.h"
// #include "driver/gpio.h"
// // // #include "esp_sleep.h"

// // static const char* TAG = "example";

// // #define EXAMPLE_PCNT_HIGH_LIMIT 100
// // #define EXAMPLE_PCNT_LOW_LIMIT  -100

// // #define EXAMPLE_EC11_GPIO_A 34
// // #define EXAMPLE_EC11_GPIO_B 0

// // static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx)
// // {
// //     BaseType_t high_task_wakeup;
// //     QueueHandle_t queue = (QueueHandle_t)user_ctx;
// //     // send event data to queue, from this interrupt callback
// //     xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
// //     return (high_task_wakeup == pdTRUE);
// // }

// // void app_main(void)
// // {
// //     ESP_LOGI(TAG, "install pcnt unit");
// //     pcnt_unit_config_t unit_config = {
// //         .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
// //         .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
// //     };
// //     pcnt_unit_handle_t pcnt_unit = NULL;
// //     ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

// //     ESP_LOGI(TAG, "set glitch filter");
// //     pcnt_glitch_filter_config_t filter_config = {
// //         .max_glitch_ns = 1000,
// //     };
// //     ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

// //     ESP_LOGI(TAG, "install pcnt channels");
// //     pcnt_chan_config_t chan_a_config = {
// //         .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
// //         .level_gpio_num = EXAMPLE_EC11_GPIO_B,
// //     };
// //     pcnt_channel_handle_t pcnt_chan_a = NULL;
// //     ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
// //     pcnt_chan_config_t chan_b_config = {
// //         .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
// //         .level_gpio_num = EXAMPLE_EC11_GPIO_A,
// //     };
// //     pcnt_channel_handle_t pcnt_chan_b = NULL;
// //     ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

// //     ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
// //     ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
// //     ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
// //     ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
// //     ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

// //     ESP_LOGI(TAG, "add watch points and register callbacks");
// //     int watch_points[] = { EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT };
// //     for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
// //         ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
// //     }
// //     pcnt_event_callbacks_t cbs = {
// //         .on_reach = example_pcnt_on_reach,
// //     };
// //     QueueHandle_t queue = xQueueCreate(10, sizeof(int));
// //     ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

// //     ESP_LOGI(TAG, "enable pcnt unit");
// //     ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
// //     ESP_LOGI(TAG, "clear pcnt unit");
// //     ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
// //     ESP_LOGI(TAG, "start pcnt unit");
// //     ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

// // #if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
// //     // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
// //     ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
// //     ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
// //     ESP_ERROR_CHECK(esp_light_sleep_start());
// // #endif

// //     // Report counter value
// //     int pulse_count = 0;
// //     int event_count = 0;
// //     while (1) {
// //         if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) {
// //             ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
// //         }
// //         else {
// //             ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
// //             ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
// //         }
// //     }
// // }

// /*  PulseSensor Starter Project and Signal Tester
//  *  The Best Way to Get Started  With, or See the Raw Signal of, your PulseSensor.com™ & Arduino.
//  *
//  *  Here is a link to the tutorial
//  *  https://pulsesensor.com/pages/code-and-guide
//  *
//  *  WATCH ME (Tutorial Video):
//  *  https://www.youtube.com/watch?v=RbB8NSRa5X4
//  *
//  *
// -------------------------------------------------------------
// 1) This shows a live human Heartbeat Pulse.
// 2) Live visualization in Arduino's Cool "Serial Plotter".
// 3) Blink an LED on each Heartbeat.
// 4) This is the direct Pulse Sensor's Signal.
// 5) A great first-step in troubleshooting your circuit and connections.
// 6) "Human-readable" code that is newbie friendly."

// */


// //  Variables
// int PulseSensorPurplePin = GPIO_NUM_34;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
// // int LED13 = 13;   //  The on-board Arduion LED


// int Signal;                // holds the incoming raw data. Signal value can range from 0-1024
// int Threshold = 550;            // Determine which Signal to "count as a beat", and which to ingore.


// // The SetUp Function:
// void setup() {
//     // pinMode(LED13, OUTPUT);         // pin that will blink to your heartbeat!
//     Serial.begin(115200);         // Set's up Serial Communication at certain speed.

// }

// // The Main Loop Function
// void loop() {

//     Signal = analogRead(PulseSensorPurplePin);  // Read the PulseSensor's value.
//     // Assign this value to the "Signal" variable.

//     Serial.println(Signal);                    // Send the Signal value to Serial Plotter.


//     if (Signal > Threshold) {                          // If the signal is above "550", then "turn-on" Arduino's on-Board LED.
//         Serial.println("above");
//     }
//     else {
//         Serial.println("below");              //  Else, the sigal must be below "550", so "turn-off" this LED.
//     }


//     delay(10);


// }
/*

   Pulse counter module - Flowmeter

   Based on example found on esp-idf repo: https://github.com/espressif/esp-idf/tree/6c49f1924/examples/peripherals/pcnt

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
   */
   /* Pulse counter module - Example

      For other examples please check:
      https://github.com/espressif/esp-idf/tree/master/examples

      This example code is in the Public Domain (or CC0 licensed, at your option.)

      Unless required by applicable law or agreed to in writing, this
      software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
      CONDITIONS OF ANY KIND, either express or implied.
   */
   // #include "freertos/FreeRTOS.h"
   // #include "freertos/task.h"
   // #include "freertos/queue.h"
   // #include "driver/ledc.h"
   // #include "driver/pcnt.h"
   // #include "esp_attr.h"
   // #include "esp_log.h"

   // static const char* TAG = "example";

   // /**
   //  * TEST CODE BRIEF
   //  *
   //  * Use PCNT module to count rising edges generated by LEDC module.
   //  *
   //  * Functionality of GPIOs used in this example:
   //  *   - GPIO18 - output pin of a sample 1 Hz pulse generator,
   //  *   - GPIO4 - pulse input pin,
   //  *   - GPIO5 - control input pin.
   //  *
   //  * Load example, open a serial port to view the message printed on your screen.
   //  *
   //  * To do this test, you should connect GPIO18 with GPIO4.
   //  * GPIO5 is the control signal, you can leave it floating with internal pull up,
   //  * or connect it to ground. If left floating, the count value will be increasing.
   //  * If you connect GPIO5 to GND, the count value will be decreasing.
   //  *
   //  * An interrupt will be triggered when the counter value:
   //  *   - reaches 'thresh1' or 'thresh0' value,
   //  *   - reaches 'l_lim' value or 'h_lim' value,
   //  *   - will be reset to zero.
   //  */
   // #define PCNT_H_LIM_VAL      10
   // #define PCNT_L_LIM_VAL     -10
   // #define PCNT_THRESH1_VAL    5
   // #define PCNT_THRESH0_VAL   -5
   // #define PCNT_INPUT_SIG_IO   4  // Pulse Input GPIO
   // #define PCNT_INPUT_CTRL_IO  5  // Control GPIO HIGH=count up, LOW=count down
   // #define LEDC_OUTPUT_IO      18 // Output GPIO of a sample 1 Hz pulse generator

   // xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events

   // /* A sample structure to pass events from the PCNT
   //  * interrupt handler to the main program.
   //  */
   // typedef struct {
   //     int unit;  // the PCNT unit that originated an interrupt
   //     uint32_t status; // information on the event type that caused the interrupt
   // } pcnt_evt_t;

   // /* Decode what PCNT's unit originated an interrupt
   //  * and pass this information together with the event type
   //  * the main program using a queue.
   //  */
   // static void IRAM_ATTR pcnt_example_intr_handler(void* arg)
   // {
   //     pcnt_unit_t pcnt_unit = ((int)arg) (pcnt_unit_t);
   //     pcnt_evt_t evt;
   //     evt.unit = pcnt_unit;
   //     /* Save the PCNT event type that caused an interrupt
   //        to pass it to the main program */
   //     pcnt_get_event_status(pcnt_unit, &evt.status);
   //     xQueueSendFromISR(pcnt_evt_queue, &evt, NULL);
   // }

   // /* Configure LED PWM Controller
   //  * to output sample pulses at 1 Hz with duty of about 10%
   //  */
   // static void ledc_init(void)
   // {
   //     // Prepare and then apply the LEDC PWM timer configuration
   //     ledc_timer_config_t ledc_timer;
   //     ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
   //     ledc_timer.timer_num = LEDC_TIMER_1;
   //     ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
   //     ledc_timer.freq_hz = 1;  // set output frequency at 1 Hz
   //     ledc_timer.clk_cfg = LEDC_AUTO_CLK;
   //     ledc_timer_config(&ledc_timer);

   //     // Prepare and then apply the LEDC PWM channel configuration
   //     ledc_channel_config_t ledc_channel;
   //     ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
   //     ledc_channel.channel = LEDC_CHANNEL_1;
   //     ledc_channel.timer_sel = LEDC_TIMER_1;
   //     ledc_channel.intr_type = LEDC_INTR_DISABLE;
   //     ledc_channel.gpio_num = LEDC_OUTPUT_IO;
   //     ledc_channel.duty = 100; // set duty at about 10%
   //     ledc_channel.hpoint = 0;
   //     ledc_channel_config(&ledc_channel);
   // }

   // /* Initialize PCNT functions:
   //  *  - configure and initialize PCNT
   //  *  - set up the input filter
   //  *  - set up the counter events to watch
   //  */
   // static void pcnt_example_init(pcnt_unit_t unit)
   // {
   //     /* Prepare configuration for the PCNT unit */
   //     pcnt_config_t pcnt_config = {
   //         // Set PCNT input signal and control GPIOs
   //         .pulse_gpio_num = PCNT_INPUT_SIG_IO,
   //         .ctrl_gpio_num = PCNT_PIN_NOT_USED, // dont use

   //         // What to do when control input is low or high?
   //         // .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
   //         // .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high

   //          // What to do on the positive / negative edge of pulse input?
   //         .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
   //         .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge

   //         // Set the maximum and minimum limit values to watch
   //         .counter_h_lim = PCNT_H_LIM_VAL,
   //         .counter_l_lim = PCNT_L_LIM_VAL,
   //         .unit = unit,
   //         .channel = PCNT_CHANNEL_0,


   //     };
   //     /* Initialize PCNT unit */
   //     pcnt_unit_config(&pcnt_config);

   //     /* Configure and enable the input filter */
   //     pcnt_set_filter_value(unit, 100);
   //     pcnt_filter_enable(unit);

   //     /* Set threshold 0 and 1 values and enable events to watch */
   //     pcnt_set_event_value(unit, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
   //     pcnt_event_enable(unit, PCNT_EVT_THRES_1);
   //     pcnt_set_event_value(unit, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
   //     pcnt_event_enable(unit, PCNT_EVT_THRES_0);
   //     /* Enable events on zero, maximum and minimum limit values */
   //     pcnt_event_enable(unit, PCNT_EVT_ZERO);
   //     pcnt_event_enable(unit, PCNT_EVT_H_LIM);
   //     pcnt_event_enable(unit, PCNT_EVT_L_LIM);

   //     /* Initialize PCNT's counter */
   //     pcnt_counter_pause(unit);
   //     pcnt_counter_clear(unit);

   //     /* Install interrupt service and add isr callback handler */
   //     pcnt_isr_service_install(0);
   //     pcnt_isr_handler_add(unit, pcnt_example_intr_handler, (void*)unit);

   //     /* Everything is set up, now go to counting */
   //     pcnt_counter_resume(unit);
   // }

   // void app_main(void)
   // {
   //     pcnt_unit_t pcnt_unit = PCNT_UNIT_0;
   //     /* Initialize LEDC to generate sample pulse signal */
   //     ledc_init();

   //     /* Initialize PCNT event queue and PCNT functions */
   //     pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
   //     pcnt_example_init(pcnt_unit);

   //     int16_t count = 0;
   //     pcnt_evt_t evt;
   //     portBASE_TYPE res;
   //     while (1) {
   //         /* Wait for the event information passed from PCNT's interrupt handler.
   //          * Once received, decode the event type and print it on the serial monitor.
   //          */
   //         res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
   //         if (res == pdTRUE) {
   //             pcnt_get_counter_value(pcnt_unit, &count);
   //             ESP_LOGI(TAG, "Event PCNT unit[%d]; cnt: %d", evt.unit, count);
   //             if (evt.status & PCNT_EVT_THRES_1) {
   //                 ESP_LOGI(TAG, "THRES1 EVT");
   //             }
   //             if (evt.status & PCNT_EVT_THRES_0) {
   //                 ESP_LOGI(TAG, "THRES0 EVT");
   //             }
   //             if (evt.status & PCNT_EVT_L_LIM) {
   //                 ESP_LOGI(TAG, "L_LIM EVT");
   //             }
   //             if (evt.status & PCNT_EVT_H_LIM) {
   //                 ESP_LOGI(TAG, "H_LIM EVT");
   //             }
   //             if (evt.status & PCNT_EVT_ZERO) {
   //                 ESP_LOGI(TAG, "ZERO EVT");
   //             }
   //         }
   //         else {
   //             pcnt_get_counter_value(pcnt_unit, &count);
   //             ESP_LOGI(TAG, "Current counter value :%d", count);
   //         }
   //     }
   // }
   /* Pulse counter module - Example

      For other examples please check:
      https://github.com/espressif/esp-idf/tree/master/examples

      This example code is in the Public Domain (or CC0 licensed, at your option.)

      Unless required by applicable law or agreed to in writing, this
      software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
      CONDITIONS OF ANY KIND, either express or implied.
   */
   // #include <Arduino.h>
   // #include <stdio.h>
   // #include "freertos/FreeRTOS.h"
   // #include "freertos/portmacro.h"
   // #include "freertos/task.h"
   // #include "freertos/queue.h"
   // #include "driver/periph_ctrl.h"
   // #include "driver/ledc.h"
   // #include "driver/gpio.h"
   // #include "driver/pcnt.h"
   // #include "esp_attr.h"
   // #include "esp_log.h"

   //    // the number of the LED pin
   // const int pwmFanPin1 = 21;  // 21 corresponds to GPIO21?

   // // setting PWM properties
   // const int freq = 1;
   // const int ledChannel = 0;
   // const int resolution = 8;
   // /**
   //  * TEST CODE BRIEF
   //  *
   //  * Use PCNT module to count rising edges generated by LEDC module.
   //  *
   //  * Functionality of GPIOs used in this example:
   //  *   - GPIO18 - output pin of a sample 1 Hz pulse generator,
   //  *   - GPIO4 - pulse input pin,
   //  *   - GPIO5 - control input pin.
   //  *
   //  * Load example, open a serial port to view the message printed on your screen.
   //  *
   //  * To do this test, you should connect GPIO18 with GPIO4.
   //  * GPIO5 is the control signal, you can leave it floating with internal pull up,
   //  * or connect it to ground. If left floating, the count value will be increasing.
   //  * If you connect GPIO5 to GND, the count value will be decreasing.
   //  *
   //  * An interrupt will be triggered when the counter value:
   //  *   - reaches 'thresh1' or 'thresh0' value,
   //  *   - reaches 'l_lim' value or 'h_lim' value,
   //  *   - will be reset to zero.
   //  */
   //  // #define PCNT_UNIT      PCNT_UNIT_0
   //  // #define PCNT_H_LIM_VAL      20
   //  // #define PCNT_L_LIM_VAL     -20
   //  // #define PCNT_THRESH1_VAL    50
   //  // #define PCNT_THRESH0_VAL   -50
   //  // #define PCNT_INPUT_SIG_IO   4  // Pulse Input GPIO
   //  // #define PCNT_INPUT_CTRL_IO  5  // Control GPIO HIGH=count up, LOW=count down
   //  // #define LEDC_OUTPUT_IO      18 // Output GPIO of a sample 1 Hz pulse generator

   // xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
   // pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

   // /* A sample structure to pass events from the PCNT
   //  * interrupt handler to the main program.
   //  */
   // typedef struct {
   //     int unit;  // the PCNT unit that originated an interrupt
   //     uint32_t status; // information on the event type that caused the interrupt
   //     unsigned long timeStamp; // The time the event occured
   // } pcnt_evt_t;

   // /* Decode what PCNT's unit originated an interrupt
   //  * and pass this information together with the event type
   //  * and timestamp to the main program using a queue.
   //  */
   // static void IRAM_ATTR pcnt_intr_handler(void* arg)
   // {
   //     unsigned long currentMillis = millis(); //Time at instant ISR was called
   //     uint32_t intr_status = pcnt.int_st.val;
   //     int i = 0;
   //     pcnt_evt_t evt;
   //     portBASE_TYPE HPTaskAwoken = pdFALSE;


   //     for (i = 0; i < PCNT_UNIT_MAX; i++) {
   //         if (intr_status & (BIT(i))) {
   //             evt.unit = i;
   //             /* Save the PCNT event type that caused an interrupt
   //                to pass it to the main program */
   //             evt.status = PCNT.status_unit[i].val;
   //             evt.timeStamp = currentMillis;
   //             PCNT.int_clr.val = BIT(i);
   //             xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
   //             if (HPTaskAwoken == pdTRUE) {
   //                 portYIELD_FROM_ISR();
   //             }
   //         }
   //     }
   // }


   // /* Initialize PCNT functions for one channel:
   //  *  - configure and initialize PCNT with pos-edge counting
   //  *  - set up the input filter
   //  *  - set up the counter events to watch
   //  * Variables:
   //  * UNIT - Pulse Counter #, INPUT_SIG - Signal Input Pin, INPUT_CTRL - Control Input Pin,
   //  * Channel - Unit input channel, H_LIM - High Limit, L_LIM - Low Limit,
   //  * THRESH1 - configurable limit 1, THRESH0 - configurable limit 2,
   //  */
   // void pcnt_init_channel(pcnt_unit_t PCNT_UNIT, int PCNT_INPUT_SIG_IO, int PCNT_INPUT_CTRL_IO = PCNT_PIN_NOT_USED, pcnt_channel_t PCNT_CHANNEL = PCNT_CHANNEL_0, int PCNT_H_LIM_VAL = 19, int PCNT_L_LIM_VAL = -20, int PCNT_THRESH1_VAL = 50, int PCNT_THRESH0_VAL = -50) {
   //     /* Prepare configuration for the PCNT unit */
   //     pcnt_config_t pcnt_config;
   //     // Set PCNT input signal and control GPIOs
   //     pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;
   //     pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;
   //     pcnt_config.channel = PCNT_CHANNEL;
   //     pcnt_config.unit = PCNT_UNIT;
   //     // What to do on the positive / negative edge of pulse input?
   //     pcnt_config.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
   //     pcnt_config.neg_mode = PCNT_COUNT_DIS;   // Keep the counter value on the negative edge
   //     // What to do when control input is low or high?
   //     pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting direction if low
   //     pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
   //     // Set the maximum and minimum limit values to watch
   //     pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
   //     pcnt_config.counter_l_lim = PCNT_L_LIM_VAL;

   //     /* Initialize PCNT unit */
   //     pcnt_unit_config(&pcnt_config);
   //     /* Configure and enable the input filter */
   //     pcnt_set_filter_value(PCNT_UNIT, 100);
   //     pcnt_filter_enable(PCNT_UNIT);

   //     /* Set threshold 0 and 1 values and enable events to watch */
   //     // pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
   //     // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_1);
   //     // pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
   //     // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_0);
   //     /* Enable events on zero, maximum and minimum limit values */
   //     pcnt_event_enable(PCNT_UNIT, PCNT_EVT_ZERO);
   //     pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);
   //     // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_L_LIM);

   //     /* Initialize PCNT's counter */
   //     pcnt_counter_pause(PCNT_UNIT);
   //     pcnt_counter_clear(PCNT_UNIT);
   //     /* Register ISR handler and enable interrupts for PCNT unit */
   //     pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
   //     pcnt_intr_enable(PCNT_UNIT);

   //     /* Everything is set up, now go to counting */
   //     pcnt_counter_resume(PCNT_UNIT);
   //     pcnt_counter_resume(PCNT_UNIT_1);
   // }

   // /* Count RPM Function - takes first timestamp and last timestamp,
   // number of pulses, and pulses per revolution */
   // int countRPM(int firstTime, int lastTime, int pulseTotal, int pulsePerRev) {
   //     int timeDelta = (lastTime - firstTime); //lastTime - firstTime
   //     if (timeDelta <= 0) { // This means we've gotten something wrong
   //         return -1;
   //     }
   //     return ((60000 * (pulseTotal / pulsePerRev)) / timeDelta);
   // }
   // void setup() {
   //     // put your setup code here, to run once:
   //     ledcSetup(ledChannel, freq, resolution);

   //     // attach the channel to the GPIO to be controlled
   //     ledcAttachPin(21, ledChannel);
   //     ledcWrite(ledChannel, 10); // 1Hz PWM with duty cycle of 10/255
   // }

   // void loop()
   // {
   //     /* Initialize PCNT event queue and PCNT functions */
   //     pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
   //     pcnt_init_channel(PCNT_UNIT_0, 4); // Initialize Unit 0 to pin 4
   //     pcnt_init_channel(PCNT_UNIT_1, 21); // Initialize Unit 1 to pin 21
   //     pcnt_init_channel(PCNT_UNIT_2, 5); // Unit 2 to pin 5
   //     int RPM0 = -1; // Fan 0 RPM
   //     int RPM1 = -1; // Fan 1 RPM
   //     int RPM2 = -1; // Fan 2 RPM
   //     int lastStamp0 = 0; //for previous time stamp for fan 0
   //     int lastStamp1 = 0; // for fan 1
   //     int lastStamp2 = 0; // for fan 2

   //     pcnt_evt_t evt;
   //     portBASE_TYPE res;
   //     for (;;) {
   //         /* Wait for the event information passed from PCNT's interrupt handler.
   //         * Once received, decode the event type and print it on the serial monitor.
   //         */
   //         res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
   //         if (res == pdTRUE) {
   //             printf("Event PCNT unit[%d]; Status: %u\n", evt.unit, evt.status);
   //             if (evt.unit == 0) { // Fan 0 - TURN THIS BLOCK INTO A FUNCTION THAT TAKES THE FAN OBJECT
   //                 if (lastStamp0 == 0) {
   //                     lastStamp0 = evt.timeStamp;
   //                 }
   //                 RPM0 = countRPM(lastStamp0, evt.timeStamp, 20, 2);
   //                 if (RPM0 == -1) {
   //                     printf("RPM Calc error detected!\n");
   //                     continue;
   //                 }
   //                 lastStamp0 = evt.timeStamp;
   //             }
   //             if (evt.unit == 1) { // Fan 1
   //                 if (lastStamp1 == 0) {
   //                     lastStamp1 = evt.timeStamp;
   //                 }
   //                 RPM1 = countRPM(lastStamp1, evt.timeStamp, 20, 2);
   //                 if (RPM1 == -1) {
   //                     printf("RPM Calc error detected!\n");
   //                     continue;
   //                 }
   //                 lastStamp1 = evt.timeStamp; // Now the time is old
   //             }
   //             if (evt.unit == 2) { // Fan 2
   //                 if (lastStamp2 == 0) {
   //                     lastStamp2 = evt.timeStamp;
   //                 }
   //                 RPM2 = countRPM(lastStamp2, evt.timeStamp, 20, 2);
   //                 if (RPM2 == -1) {
   //                     printf("RPM Calc error detected!\n");
   //                     continue;
   //                 }
   //                 lastStamp2 = evt.timeStamp; // Now the time is old
   //             }
   //             printf("Fan 0 RPM: %d , Fan 1 RPM: %d, Fan 2 RPM: %d", RPM0, RPM1, RPM2);
   //         }
   //     }
   //     if (user_isr_handle) {
   //         //Free the ISR service handle.
   //         esp_intr_free(user_isr_handle);
   //         user_isr_handle = NULL;
   //     }
   // }


// BLOG Eletrogate
// ESP32 Frequency Meter
// ESP32 DevKit 38 pins + LCD
// https://blog.eletrogate.com/esp32-frequencimetro-de-precisao
// Rui Viana and Gustavo Murta august/2020

// #include "stdio.h"                                                        // Library STDIO
// #include "driver/ledc.h"                                                  // Library ESP32 LEDC
// #include "driver/pcnt.h"                                                  // Library ESP32 PCNT
// #include "soc/pcnt_struct.h"

// #define LCD_OFF                                                            // LCD_ON, if want use LCD 4 bits interface 
// #define LCD_I2C_OFF                                                       // LCD_I2C_ON, if want use I2C LCD (PCF8574) 

// #ifdef LCD_I2C_ON                                                         // If I2C LCD enabled 
// #define I2C_SDA 21                                                        // LCD I2C SDA - GPIO_21
// #define I2C_SCL 22                                                        // LCD I2C SCL - GPIO_22
// #include <Wire.h>                                                         // I2C Libray 
// #include <LiquidCrystal_PCF8574.h>                                        // Library LCD with PCF8574
// LiquidCrystal_PCF8574 lcd(0x3F);                                          // Instance LCD I2C - adress x3F
// #endif                                                                    // LCD I2C

// #ifdef LCD_ON                                                             // LCD with 4 bits interface enabled 
// #include <LiquidCrystal.h>                                                // Library LCD
// LiquidCrystal lcd(4, 16, 17, 5, 18, 19);                                  // Instance - ports RS,ENA,D4,D5,D6,D7
// #endif                                                                    // LCD

// #define PCNT_COUNT_UNIT       PCNT_UNIT_0                                 // Set Pulse Counter Unit - 0 
// #define PCNT_COUNT_CHANNEL    PCNT_CHANNEL_0                              // Set Pulse Counter channel - 0 

// #define PCNT_INPUT_SIG_IO     GPIO_NUM_34                                 // Set Pulse Counter input - Freq Meter Input GPIO 34
// #define LEDC_HS_CH0_GPIO      GPIO_NUM_33                                 // Saida do LEDC - gerador de pulsos - GPIO_33
// #define PCNT_INPUT_CTRL_IO    PCNT_PIN_NOT_USED                                 // Set Pulse Counter Control GPIO pin - HIGH = count up, LOW = count down  
// #define OUTPUT_CONTROL_GPIO   PCNT_PIN_NOT_USED                                 // Timer output control port - GPIO_32
// #define PCNT_H_LIM_VAL        overflow                                    // Overflow of Pulse Counter 

// #define IN_BOARD_LED          GPIO_NUM_2                                  // ESP32 native LED - GPIO 2

// bool            flag = true;                                     // Flag to enable print frequency reading
// uint32_t        overflow = 20000;                                    // Max Pulse Counter value
// int16_t         pulses = 0;                                        // Pulse Counter value
// uint32_t        multPulses = 0;                                        // Quantidade de overflows do contador PCNT
// uint32_t        sample_time = 1000000;                                  // sample time of 1 second to count pulses
// uint32_t        osc_freq = 12543;                                    // Oscillator frequency - initial 12543 Hz (may be 1 Hz to 40 MHz)
// uint32_t        mDuty = 0;                                        // Duty value
// uint32_t        resolution = 0;                                        // Resolution value
// float           frequency = 0;                                        // frequency value
// char            buf[32];                                                  // Buffer

// esp_timer_create_args_t create_args;                                      // Create an esp_timer instance
// esp_timer_handle_t timer_handle;                                          // Create an single timer

// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;                     // portMUX_TYPE to do synchronism
// void init_frequencyMeter();
// //----------------------------------------------------------------------------------------
// void setup()
// {
//     Serial.begin(115200);                                                   // Serial Console Arduino 115200 Bps
//     Serial.println(" Input the Frequency - 1 to 40 MHz");                   // Console print

// #ifdef LCD_I2C_ON                                                         // If usinf I2C LCD
//     Wire.begin(I2C_SDA, I2C_SCL);                                           // Begin I2C Interface
//     lcd.setBacklight(255);                                                  // Set I2C LCD Backlight ON
// #endif

// #if defined LCD_ON || defined LCD_I2C_ON                                  // If using LCD or I2C LCD     
//     lcd.begin(16, 2);                                                       // LCD init 16 x2
//     lcd.print("  Frequency:");                                              // LCD print
// #endif

//     init_frequencyMeter();                                                 // Initialize Frequency Meter
// }

// //----------------------------------------------------------------------------
// void init_osc_freq()                                                     // Initialize Oscillator to test Freq Meter
// {
//     resolution = (log(80000000 / osc_freq) / log(2)) / 2;                // Calc of resolution of Oscillator
//     if (resolution < 1) resolution = 1;                                     // set min resolution 
//     // Serial.println(resolution);                                          // Print
//     mDuty = (pow(2, resolution)) / 2;                                       // Calc of Duty Cycle 50% of the pulse
//     // Serial.println(mDuty);                                               // Print

//     ledc_timer_config_t ledc_timer = {};                                    // LEDC timer config instance

//     ledc_timer.duty_resolution = ledc_timer_bit_t(resolution);             // Set resolution
//     ledc_timer.freq_hz = osc_freq;                                       // Set Oscillator frequency
//     ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;                           // Set high speed mode
//     ledc_timer.timer_num = LEDC_TIMER_0;                                    // Set LEDC timer index - 0
//     ledc_timer_config(&ledc_timer);                                         // Set LEDC Timer config

//     ledc_channel_config_t ledc_channel = {};                                // LEDC Channel config instance

//     ledc_channel.channel = LEDC_CHANNEL_0;                               // Set HS Channel - 0
//     ledc_channel.duty = mDuty;                                        // Set Duty Cycle 50%
//     ledc_channel.gpio_num = LEDC_HS_CH0_GPIO;                             // LEDC Oscillator output GPIO 33
//     ledc_channel.intr_type = LEDC_INTR_DISABLE;                            // LEDC Fade interrupt disable
//     ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;                         // Set LEDC high speed mode
//     ledc_channel.timer_sel = LEDC_TIMER_0;                                 // Set timer source of channel - 0
//     ledc_channel_config(&ledc_channel);                                     // Config LEDC channel
// }

// //----------------------------------------------------------------------------------
// static void IRAM_ATTR pcnt_intr_handler(void* arg)                        // Counting overflow pulses
// {
//     portENTER_CRITICAL_ISR(&timerMux);                                      // disabling the interrupts
//     multPulses++;                                                           // increment Overflow counter
//     PCNT.int_clr.val = BIT(PCNT_COUNT_UNIT);                                // Clear Pulse Counter interrupt bit
//     portEXIT_CRITICAL_ISR(&timerMux);                                       // enabling the interrupts
// }

// //----------------------------------------------------------------------------------
// void init_PCNT(void)                                                      // Initialize and run PCNT unit
// {
//     pcnt_config_t pcnt_config = { };                                        // PCNT unit instance

//     pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;                         // Pulse input GPIO 34 - Freq Meter Input
//     pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;                         // Control signal input GPIO 35
//     pcnt_config.unit = PCNT_COUNT_UNIT;                                     // Unidade de contagem PCNT - 0
//     pcnt_config.channel = PCNT_COUNT_CHANNEL;                               // PCNT unit number - 0
//     pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;                             // Maximum counter value - 20000
//     pcnt_config.pos_mode = PCNT_COUNT_INC;                                  // PCNT positive edge count mode - inc
//     pcnt_config.neg_mode = PCNT_COUNT_INC;                                  // PCNT negative edge count mode - inc
//     pcnt_config.lctrl_mode = PCNT_MODE_DISABLE;                             // PCNT low control mode - disable
//     pcnt_config.hctrl_mode = PCNT_MODE_KEEP;                                // PCNT high control mode - won't change counter mode
//     pcnt_unit_config(&pcnt_config);                                         // Initialize PCNT unit

//     pcnt_counter_pause(PCNT_COUNT_UNIT);                                    // Pause PCNT unit
//     pcnt_counter_clear(PCNT_COUNT_UNIT);                                    // Clear PCNT unit

//     pcnt_event_enable(PCNT_COUNT_UNIT, PCNT_EVT_H_LIM);                     // Enable event to watch - max count
//     pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);                    // Setup Register ISR handler
//     pcnt_intr_enable(PCNT_COUNT_UNIT);                                      // Enable interrupts for PCNT unit

//     pcnt_counter_resume(PCNT_COUNT_UNIT);                                   // Resume PCNT unit - starts count
// }

// //----------------------------------------------------------------------------------
// void read_PCNT(void* p)                                                   // Read Pulse Counter
// {
//     // gpio_set_level(OUTPUT_CONTROL_GPIO, 0);                                 // Stop counter - output control LOW
//     pcnt_get_counter_value(PCNT_COUNT_UNIT, &pulses);                       // Read Pulse Counter value
//     flag = true;                                                            // Change flag to enable print
// }

// //---------------------------------------------------------------------------------
// void init_frequencyMeter()
// {
//     // init_osc_freq();                                                        // Initialize Oscillator
//     init_PCNT();                                                            // Initialize and run PCNT unit

//     // gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO);                              // Set GPIO pad
//     // gpio_set_direction(OUTPUT_CONTROL_GPIO, GPIO_MODE_OUTPUT);              // Set GPIO 32 as output

//     create_args.callback = read_PCNT;                                       // Set esp-timer argument
//     esp_timer_create(&create_args, &timer_handle);                          // Create esp-timer instance

//     // gpio_set_direction(IN_BOARD_LED, GPIO_MODE_OUTPUT);                     // Set LED inboard as output

//     gpio_matrix_in(PCNT_INPUT_SIG_IO, SIG_IN_FUNC226_IDX, false);           // Set GPIO matrin IN - Freq Meter input
//     gpio_matrix_out(IN_BOARD_LED, SIG_IN_FUNC226_IDX, false, false);        // Set GPIO matrix OUT - to inboard LED
// }

// //----------------------------------------------------------------------------------------
// char* ultos_recursive(unsigned long val, char* s, unsigned radix, int pos) // Format an unsigned long (32 bits) into a string
// {
//     int c;
//     if (val >= radix)
//         s = ultos_recursive(val / radix, s, radix, pos + 1);
//     c = val % radix;
//     c += (c < 10 ? '0' : 'a' - 10);
//     *s++ = c;
//     if (pos % 3 == 0) *s++ = ',';
//     return s;
// }
// //----------------------------------------------------------------------------------------
// char* ltos(long val, char* s, int radix)                                  // Format an long (32 bits) into a string
// {
//     if (radix < 2 || radix > 36) {
//         s[0] = 0;
//     }
//     else {
//         char* p = s;
//         if (radix == 10 && val < 0) {
//             val = -val;
//             *p++ = '-';
//         }
//         p = ultos_recursive(val, p, radix, 0) - 1;
//         *p = 0;
//     }
//     return s;
// }

// //---------------------------------------------------------------------------------
// void loop()
// {
//     if (flag == true)                                                     // If count has ended
//     {
//         flag = false;                                                       // change flag to disable print
//         frequency = (pulses + (multPulses * overflow)) / 2;               // Calculation of frequency
//         printf("Frequency : %s", (ltos(frequency, buf, 10)));               // Print frequency with commas
//         printf(" Hz \n");                                                   // Print unity Hz

// #if defined LCD_ON || defined LCD_I2C_ON                                // If using LCD or I2C LCD  
//         lcd.setCursor(2, 1);                                                // Set cursor position - column and row
//         lcd.print((ltos(frequency, buf, 10)));                              // LCD print frequency
//         lcd.print(" Hz              ");                                     // LCD print unity Hz
// #endif

//         multPulses = 0;                                                     // Clear overflow counter
//         // Put your function here, if you want
//         delay(100);                                                        // Delay 100 ms
//         // Put your function here, if you want

//         pcnt_counter_clear(PCNT_COUNT_UNIT);                                // Clear Pulse Counter
//         esp_timer_start_once(timer_handle, sample_time);                    // Initialize High resolution timer (1 sec)
//         // gpio_set_level(OUTPUT_CONTROL_GPIO, 1);                             // Set enable PCNT count
//     }

//     String inputString = "";                                               // clear temporary string
//     // osc_freq = 0;                                                          // Clear oscillator frequency
//     while (Serial.available()) {
//         char inChar = (char)Serial.read();                                   // Reads a byte on the console
//         inputString += inChar;                                               // Add char to string
//         if (inChar == '\n')                                                  // If new line (enter)
//         {
//             osc_freq = inputString.toInt();                                    // Converts String into integer value
//             inputString = "";                                                  // Clear string
//         }
//     }
//     if (osc_freq != 0)                                                     // If some value inputted to oscillator frequency
//     {
//         init_osc_freq();                                                    // reconfigure ledc function - oscillator 
//     }
// }

#include "driver/pcnt.h"  // ESP32 library for pulse count
#include "soc/pcnt_struct.h"
//   // e.g. stored in following path C:\Users\User\Documents\Arduino\hardware\arduino-esp32-master\tools\sdk\include\driver\driver\pcnt.h
//   // when in the Arduino IDE properties the sketchbook storage location is set to C:\Users\User\Documents\Arduino

// #define PCNT_FREQ_UNIT      PCNT_UNIT_0                      // select ESP32 pulse counter unit 0 (out of 0 to 7 indipendent counting units)
//                                                              // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html

// int SPEED_IR_INPUT_PIN = GPIO_NUM_34;                                 // Input D15 = signal from IR-diode for pulse counter
// bool _flag = 0;  // only for testing ####################################

// int16_t PulseCounter = 0;                                // pulse counter, max. value is 65536
// int OverflowCounter = 0;                                // pulse counter overflow counter
// int PCNT_H_LIM_VAL = 10000;                            // upper limit of counting  max. 32767, write +1 to overflow counter, when reached 
// uint16_t PCNT_FILTER_VAL = 1000;                             // filter (damping, inertia) value for avoiding glitches in the count, max. 1023

// // not in use, copy from example code ########################################
// //float frequencia = 0;                                      // Frequencia medida
// //String unidade;                                            // Unidade de medida da escala
// //unsigned long tempo;                                       // base de tempo da medida dos pulsos
// //int prescaler;                                             // frequency devider of timer
// //bool conterOK = false;

// pcnt_isr_handle_t user_isr_handle = NULL;                    // interrupt handler - not used
// hw_timer_t* timer = NULL;                                   // Instancia do timer

// void IRAM_ATTR CounterOverflow(void* arg) {                  // Interrupt for overflow of pulse counter
//     OverflowCounter = OverflowCounter + 1;                     // increase overflow counter
//     PCNT.int_clr.val = BIT(PCNT_FREQ_UNIT);                    // clean overflow flag
//     pcnt_counter_clear(PCNT_FREQ_UNIT);                        // zero and reset of pulse counter unit
// }

// void initPulseCounter() {                                    // initialise pulse counter
//     pcnt_config_t pcntFreqConfig = { };                        // Instance of pulse counter
//     pcntFreqConfig.pulse_gpio_num = SPEED_IR_INPUT_PIN;        // pin assignment for pulse counter = GPIO 15
//     pcntFreqConfig.pos_mode = PCNT_COUNT_INC;                  // count rising edges (=change from low to high logical level) as pulses
//     pcntFreqConfig.counter_h_lim = PCNT_H_LIM_VAL;             // set upper limit of counting 
//     pcntFreqConfig.unit = PCNT_FREQ_UNIT;                      // select ESP32 pulse counter unit 0
//     pcntFreqConfig.channel = PCNT_CHANNEL_0;                   // select channel 0 of pulse counter unit 0
//     pcnt_unit_config(&pcntFreqConfig);                         // configur rigisters of the pulse counter

//     pcnt_counter_pause(PCNT_FREQ_UNIT);                        // pause puls counter unit
//     pcnt_counter_clear(PCNT_FREQ_UNIT);                        // zero and reset of pulse counter unit

//     pcnt_event_enable(PCNT_FREQ_UNIT, PCNT_EVT_H_LIM);         // enable event for interrupt on reaching upper limit of counting
//     pcnt_isr_register(CounterOverflow, NULL, 0, &user_isr_handle);  // configure register overflow interrupt handler
//     pcnt_intr_enable(PCNT_FREQ_UNIT);                          // enable overflow interrupt

//     pcnt_set_filter_value(PCNT_FREQ_UNIT, PCNT_FILTER_VAL);    // set damping, inertia 
//     pcnt_filter_enable(PCNT_FREQ_UNIT);                        // enable counter glitch filter (damping)

//     pcnt_counter_resume(PCNT_FREQ_UNIT);                       // resume counting on pulse counter unit
// }

// void Read_Reset_PCNT() {                                     // function for reading pulse counter (for timer)
//     pcnt_get_counter_value(PCNT_FREQ_UNIT, &PulseCounter);     // get pulse counter value - maximum value is 16 bits

//     // resetting counter as if example, delet for application in PiedPiperS
//     OverflowCounter = 0;                                       // set overflow counter to zero
//     pcnt_counter_clear(PCNT_FREQ_UNIT);                        // zero and reset of pulse counter unit
//     //conterOK = true;                                         // not in use, copy from example code ########################################
// }

// void Read_PCNT() {                                           // function for reading pulse counter (for timer)
//     pcnt_get_counter_value(PCNT_FREQ_UNIT, &PulseCounter);     // get pulse counter value - maximum value is 16 bits
// }

