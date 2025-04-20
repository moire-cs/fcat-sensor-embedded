// Let's not do sendToWait for now, since it doesn't work
#ifndef GATEWAY_ADDR
#define GATEWAY_ADDR 1
#endif

void postData(struct Packet p) {
    if (p.node_number == 0 && p.data == nullptr) {
        Serial.println("[ERROR] postData received invalid Packet");
        return;}
    String json = "MESSAGE:{";

    // Node ID
    json += "\"nodeId\": \"" + String(p.node_number) + "\",";

    // Sensors
    json += "\"sensors\": [\"moisture\",\"temperature\",\"humidity\",\"light\",\"battery\"],";

    // Times
    json += "\"times\": [";
    time_t now;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        // Serial.println("Failed to obtain time");
        now = 0;
    }
    time(&now);
    json += String(now) ; // current time
    json += "],";

    // Messages
    json += "\"messages\": [";
    bool firstMsg = true;
    for (int i = 0; i < MAX_MEASUREMENTS; i++) {
        if (p.data[i].moisture_percent == 0 && p.data[i].temperature == 0) continue;
        if (!firstMsg) json += ",";
        firstMsg = false;

        json += "[";
        json += String(p.data[i].moisture_percent) + ",";
        json += String(p.data[i].temperature, 2) + ",";
        json += String(p.data[i].humidity, 2) + ",";
        json += String(p.data[i].light_level) + ",";
        json += String(p.data[i].battery_level);
        json += "]";
    }
    json += "]";

    json += "}";

    Serial.println("---- JSON Packet to Send ----");
    Serial.println(json);
    Serial.println("-----------------------------");
}


// ② Add randomized delay (0–max_ms ms) to reduce collisions
inline void txJitter(uint16_t max_ms = 800) {
    uint32_t r = esp_random() % max_ms;
    // Serial.printf("Random delay: %d ms\n", r);
    vTaskDelay(pdMS_TO_TICKS(r));
}



bool runTimeSyncReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf,
    uint8_t* _msgRcvBufLen, uint8_t* _msgFrom,
    RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    Serial.printf("[TimeSync] Waiting up to %d ms...\n", wait_time);

    if (!RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, wait_time, _msgFrom)) {
        Serial.println("[TimeSync] ❌ Timeout – did not receive settings.");
        return false;
    }

    char buf_[RH_MESH_MAX_MESSAGE_LEN];
    std::sprintf(buf_, "%s", reinterpret_cast<char*>(_msgRcvBuf));
    timeSyncRcv = String(buf_).c_str();
    Serial.printf("[TimeSync] ✅ Received time sync from node %u: \"%s\"\n", *_msgFrom, buf_);

    return true;
}

void runReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    // while at it, wait for a message from other nodes
    // TODO: I don't believe this node passes on the message
    Serial.println("--------------------------------------------------------------");
    Serial.printf("Enter runReceiver function, wait time: %d ms\n", wait_time);
    Serial.printf("Current node address: %u\n", selfAddress_);
    Serial.printf("Target node Location: %u\n", *_msgFrom);
    if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, wait_time, _msgFrom)) {
        Serial.println("Received a message！");
        // char buf_[RH_MESH_MAX_MESSAGE_LEN];
        char buf_[RH_MESH_MAX_MESSAGE_LEN];

        esp_task_wdt_reset();
        Serial.println("Received a message");
        std::sprintf(buf_, "%s", reinterpret_cast<char*>(_msgRcvBuf));
        Serial.printf("Original Message received (as string)：\"%s\"\n", buf_);


        // msgRcv = (struct Measurement)buf_; // should be able to set it to this
        Measurement* received = reinterpret_cast<Measurement*>(&buf_);
        if (received == nullptr) {
            Serial.println("[ERROR] Received null measurement pointer");
            return;
        }
        // do something with message, for example pass it through a callback
        Serial.printf("[%d] Received measurement. RSSI: %d\n", *_msgFrom, RFM95Modem_.lastRssi());
        printMeasurement(*received); 
        
        // clears msgRcv
        // memset(msgRcv, 0, sizeof(msgRcv));

        std::string _msgRply = String("Hi node " + String(*_msgFrom) + ", got the message!").c_str();
        uint8_t _err = RHMeshManager_.sendtoWait(
            reinterpret_cast<uint8_t*>(&_msgRply[0]), _msgRply.size(), *_msgFrom);
        if (_err != RH_ROUTER_ERROR_NONE) {
            Serial.println("Fail to send reply...");
        }
        esp_task_wdt_reset();

    }
    Serial.println("Exiting runReceiver function");
    Serial.println("--------------------------------------------------------------");
}

void runSender(uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    // Need to look into sending structs over this
    // print logs
    Serial.println("----------------------------------------------------------------");
    Serial.println("进入 runSender 函数");
    Serial.printf("当前节点地址: %u\n", selfAddress_);
   // Serial.printf("目标节点地址（原始传入参数）: %u\n", targetAddress_);
    Serial.println("开始构造数据包...");
    Serial.printf("[Send] Node %u → Gateway %u\n",
        selfAddress_, GATEWAY_ADDR);

    Packet p;
    p.node_number = selfAddress_;
    p.sensors[0] = 0;
    p.sensors[1] = 1;
    p.sensors[2] = 2;
    p.sensors[3] = 3;
    for (int i = 0; i < MAX_MEASUREMENTS; i++) {
        memcpy(&p.data[i], &measurements[i], sizeof(Measurement));
    }
    Serial.printf("Packet formed,Packet size: %d bits\n", sizeof(p));
    // 2. Add jitter to avoid synchronized sends across nodes
    txJitter();  // Default: random 0–800ms


    //Sample code for broadcasting to all for the sake of testing
    Serial.println("Sending Packet...");
    //uint8_t _err = RHMeshManager_.sendtoWait(reinterpret_cast<uint8_t*>(&p), sizeof(p), RH_BROADCAST_ADDRESS); 

    //original code
    //uint8_t _err = RHMeshManager_.sendtoWait(reinterpret_cast<uint8_t*>(&p), sizeof(p), targetAddress_); 
    
    uint8_t _err = RHMeshManager_.sendtoWait(
        reinterpret_cast<uint8_t*>(&p), sizeof(p), GATEWAY_ADDR);

    if (_err == RH_ROUTER_ERROR_NONE) {
        // message successfully be sent to the target node, or next neighboring
        // expecting to recieve a simple reply from the target node
        esp_task_wdt_reset();
        Serial.printf("Packet sent successfull!");
        
        //We'll blcok this part for now, since we don't need to send a reply to the sender

        Serial.printf("Awaiting for Reply\n");

        if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, 3000, _msgFrom)) {
            char buf_[RH_MESH_MAX_MESSAGE_LEN];

            std::sprintf(buf_, "%s", reinterpret_cast<char*>(_msgRcvBuf));
            Measurement* received = reinterpret_cast<Measurement*>(&buf_);

            Serial.printf("[%d] Received measurement. RSSI: %d\n", *_msgFrom, RFM95Modem_.lastRssi());
            printMeasurement(*received); 
        }
        else {
            Serial.println("No reply, is the target node running?");
        }

        // esp_task_wdt_reset();
    }
    else {
        Serial.println(
            "sendtoWait failed. No response from intermediary node, are they "
            "running?");

        // TODO: If nobody received, up power? Maybe a function to do default power applied (starts at low value and increases, then stays at whatever value works)
    }
    esp_task_wdt_reset();
    Serial.println("退出 runSender 函数");
    Serial.println("----------------------------------------------------------------");
}

void runGatewaySender(String settings, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    Serial.println("==============================================================");
    Serial.println("进入 runGatewaySender 函数");
    Serial.printf("待发送 gateway 设置内容: \"%s\"\n", settings.c_str());
    Serial.printf("设置内容长度: %d 字节\n", settings.length());

    Serial.println("调用 sendto() 广播消息...");
    uint8_t _err = RHMeshManager_.sendto(
        reinterpret_cast<uint8_t*>(&settings[0]),
        settings.length(),
        RH_BROADCAST_ADDRESS);

    if (_err == RH_ROUTER_ERROR_NONE) {
        Serial.println("[GatewaySender] ✅ Settings broadcasted successfully.");
    } else {
        Serial.printf("[GatewaySender] ❌ sendto failed. Error code: %d\n", _err);
    }
    esp_task_wdt_reset();
    Serial.println("退出 runGatewaySender 函数");
    Serial.println("==============================================================");
}

/*
void runGatewayReceiver(int wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {

    Serial.println("Receiving mode active");
    uint64_t start = millis();

    // For wait_time, the gateway will wait for messages (multiple), and then post them to the backend
    while (millis() - start < wait_time) {
        esp_task_wdt_reset();
        if (RHMeshManager_.recvfromAck(_msgRcvBuf, _msgRcvBufLen, _msgFrom)) {
            char buf_[RH_MESH_MAX_MESSAGE_LEN];

            esp_task_wdt_reset();
            Serial.println("Received a message");

            Packet* received = reinterpret_cast<Packet*>(_msgRcvBuf);

            printPacket(*received);
            postData(*received);

            for (int i = 0; i < MAX_MEASUREMENTS; i++) {
                 printMeasurements(received[i]);

            }

            // Resets msgRcv
            memset(msgRcv, 0, sizeof(msgRcv));
            esp_task_wdt_reset();

            *we'll block this part for now, since we don't need to send a reply to the sender


            std::string _msgRply = String("Hi node " + String(*_msgFrom) + ", got the message!").c_str();
            uint8_t _err = RHMeshManager_.sendtoWait(
                reinterpret_cast<uint8_t*>(&_msgRply[0]), _msgRply.size(), *_msgFrom);
            if (_err != RH_ROUTER_ERROR_NONE) {
                Serial.println("Fail to send reply...");
            }
            esp_task_wdt_reset();
        }
        esp_task_wdt_reset();
        //delay(50);
    }
}
*/
// struct Measurement* packetInfo, , uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_
void runGatewaySender(String settings, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    Serial.println("==============================================================");
    Serial.println("进入 runGatewaySender 函数");
     // 打印发送的设置字符串及其长度
     Serial.printf("待发送 gateway 设置内容: \"%s\"\n", settings.c_str());
     Serial.printf("设置内容长度: %d 字节\n", settings.length());
     
     // 使用 sendtoWait 发送数据，当前通过广播地址发送
     Serial.println("调用 sendtoWait() 发送消息...");
     uint8_t _err = RHMeshManager_.sendtoWait(
        reinterpret_cast<uint8_t*>(&settings[0]),
        settings.length(),
        RH_BROADCAST_ADDRESS);


    if (_err == RH_ROUTER_ERROR_NONE) {
        // message successfully be sent to the target node, or next neighboring
        // expecting to recieve a simple reply from the target node
        esp_task_wdt_reset();
        Serial.printf(" successfull! Awaiting for Reply\n");
        Serial.println("sendtoWait 结果: 发送成功！");
        Serial.println("开始等待对端回复，超时时间: 1000 毫秒...");
        // This eventually will need to be replaced
        if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, 1000, _msgFrom)) {
            char buf_[RH_MESH_MAX_MESSAGE_LEN];
            std::sprintf(buf_, "%s", reinterpret_cast<char*>(_msgRcvBuf));
            //     // Measurement *received = reinterpret_cast<Measurement *>(buf_);


            //String received = String(buf_).c_str();
            // Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", *_msgFrom,
            //     received.c_str(), RFM95Modem_.lastRssi());
            // return received data as a string to print logs
            String received = String(buf_);
            Serial.printf("收到来自节点 [%d] 的回复：\"%s\"\n", *_msgFrom, received.c_str());
            Serial.printf("该回复的 RSSI 值为: %d\n", RFM95Modem_.lastRssi());
            Serial.println(received);
        }
        else {
            Serial.println("No reply, is the target node running?");
        }

        esp_task_wdt_reset();
    }
    else {
        Serial.println(
            "sendtoWait failed. No response from intermediary node, are they "
            "running?");
        Serial.printf("返回错误码: %d\n", _err);

        // TODO: If nobody received, up power? Maybe a function to do default power applied (starts at low value and increases, then stays at whatever value works)
    }
    esp_task_wdt_reset();
    Serial.println("退出 runGatewaySender 函数");
    Serial.println("==============================================================");
}