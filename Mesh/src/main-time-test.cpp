// #include <RHMesh.h>
// #include <RH_RF95.h>
#include <Arduino.h>
#include <SPI.h>
// #include <RHMesh.h>
// #include <RH_RF95.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <driver/adc.h>
// #include "radio_pinouts_and_constants.h"
// #include "meshing.h"
#include "node-numbers.h"

#define microseconds 1000000 
#define RF95_FREQ 915.0 // USA and Ecuador
#define WDT_TIMEOUT 15
// #if defined(RFM95_CS) && defined(RFM95_RST) && defined(RFM95_INT)
// #else

// Board pinout (from mesh)
// #define RFM95_CS 5
// #define RFM95_RST 14
// #define RFM95_INT 13
// #endif
// uint8_t _msgRcvBuf[RH_MESH_MAX_MESSAGE_LEN];
// RH_RF95 RFM95Modem_(RFM95_CS, RFM95_INT);
// RHMesh RHMeshManager_(RFM95Modem_, selfAddress_);

// void rhSetup() {
//     if (!RHMeshManager_.init())
//         Serial.println("init failed");
//     RFM95Modem_.setTxPower(17, false);
//     RFM95Modem_.setFrequency(RF95_FREQ);
//     RFM95Modem_.setCADTimeout(500);
// }

// void runReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
//     // while at it, wait for a message from other nodes
//     // TODO: I don't believe this node passes on the message

//     if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, wait_time, _msgFrom)) {

//         // char buf_[RH_MESH_MAX_MESSAGE_LEN];
//         char buf_[RH_MESH_MAX_MESSAGE_LEN];

//         esp_task_wdt_reset();
//         Serial.println("Received a message");
//         std::sprintf(buf_, "%s", reinterpret_cast<char*>(_msgRcvBuf));


//         // msgRcv = (struct Measurement)buf_; // should be able to set it to this
//         // Measurement* received = reinterpret_cast<Measurement*>(&buf_);

//         // do something with message, for example pass it through a callback
//         // Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", _msgFrom,
//         //     received, RFM95Modem_.lastRssi());

//         // clears msgRcv
//         // memset(msgRcv, 0, sizeof(msgRcv));

//         std::string _msgRply = String("Hi node " + String(*_msgFrom) + ", got the message!").c_str();
//         uint8_t _err = RHMeshManager_.sendtoWait(
//             reinterpret_cast<uint8_t*>(&_msgRply[0]), _msgRply.size(), *_msgFrom);
//         if (_err != RH_ROUTER_ERROR_NONE) {
//             Serial.println("Fail to send reply...");
//         }
//         esp_task_wdt_reset();
//     }
//     esp_task_wdt_reset();
// }
timeval start;
timeval end;
double diff;

void setup() {
    Serial.begin(115200);
    // rhSetup();
    // uint8_t _msgFrom;
    // uint8_t _msgRcvBufLen = sizeof(_msgRcvBuf);
    gettimeofday(&start, NULL);
    
    Serial.println("Wakeup: " + String(start.tv_sec) + "." + String(start.tv_usec));
    // runReceiver(5000, _msgRcvBuf, &_msgRcvBufLen, &_msgFrom, RFM95Modem_, RHMeshManager_);
    gettimeofday(&end, NULL);
    // esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
    // esp_task_wdt_add(NULL);
    Serial.println("End Wakeup: " + String(end.tv_sec) + "." + String(end.tv_usec));
    // diff = (end.tv_sec - start.tv_sec) * microseconds + (end.tv_usec - start.tv_usec);
    Serial.printf("Difference: %0.6f\n",((double)diff)/microseconds);
    // Serial.println("Difference micro: " + String(diff));
    pinMode(GPIO_NUM_26, OUTPUT);
    digitalWrite(GPIO_NUM_26, HIGH);
    delay(2000);
    int battery_level;
    esp_err_t r = adc2_get_raw(ADC2_CHANNEL_8, ADC_WIDTH_12Bit, &battery_level); // this will be out of 4095
    Serial.println(1.27*battery_level*1090/(4095*270));
    // Serial.println(1.27*3.3*battery_level/(double)4095);
    // 0.949
    // Serial.println((battery_level/(double)4095 * 3.3/((double)270/1090))*1.27);

    digitalWrite(GPIO_NUM_26, LOW);

}

void sleep() {
    // gettimeofday(&tv_now, NULL); // get time of day
    // Calculates time it takes between startup and now
    uint64_t sleepTime = 1 * 60 * microseconds; //((timer + start.tv_sec - tv_now.tv_sec)) * microseconds + start.tv_usec - tv_now.tv_usec;

    // Serial.println("Sleeping at: " + String(tv_now.tv_sec) + "." + String(tv_now.tv_usec) + " seconds and for: " + String((double)sleepTime / microseconds) + " seconds");
    Serial.printf("Sleeping for %0.1f minutes", (double)sleepTime /microseconds);
    esp_err_t sleep_error = esp_sleep_enable_timer_wakeup(sleepTime); // takes into account time between start and sleep
    // esp_task_wdt_reset();
    esp_deep_sleep_start();
}

void loop() {
    sleep();
}