//#include "radio_pinouts_and_constants.h"

void postData(struct Packet p) {
    String json = "{";

    // Node ID
    json += "\"nodeId\": \"" + String(p.node_number) + "\",";

    // Sensors
    json += "\"sensors\": [\"moisture\",\"temperature\",\"humidity\",\"light\",\"battery\"],";

    // Times
    json += "\"times\": [";
    bool firstTime = true;
    for (int i = 0; i < MAX_MEASUREMENTS; i++) {
        if (p.data[i].moisture_percent == 0 && p.data[i].temperature == 0) continue;
        if (!firstTime) json += ",";
        firstTime = false;
        json += String(p.data[i].timestamp);
    }
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


bool runTimeSyncReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    // while at it, wait for a message from other nodes
    Serial.println("--------------------------------------------------------------");
    Serial.printf("进入 runReceiver 函数，等待时间: %d 毫秒\n", wait_time);
    Serial.printf("当前节点地址: %u\n", selfAddress_);
    if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, wait_time, _msgFrom)) {

        char buf_[RH_MESH_MAX_MESSAGE_LEN];

        esp_task_wdt_reset();
        Serial.println("Received a message for time sync from ");
        Serial.println(*_msgFrom);
        std::sprintf(buf_, "%s", reinterpret_cast<char*>(_msgRcvBuf));
        timeSyncRcv = String(buf_).c_str();

        // do something with message, for example pass it through a callback
        Serial.printf("Rebroadcasting to available nodes...\n");

        uint8_t _err = RHMeshManager_.sendtoWait(
            reinterpret_cast<uint8_t*>(&timeSyncRcv[0]), timeSyncRcv.length(), RH_BROADCAST_ADDRESS);
        if (_err != RH_ROUTER_ERROR_NONE) {
            Serial.println("Fail to send broadcast...");
        }
        esp_task_wdt_reset();
        Serial.println("已同步，退出 runTimeSync 函数");
        Serial.println("--------------------------------------------------------------");
        return true;
        
    }
    esp_task_wdt_reset();
    Serial.println("未同步，退出 runTimeSync 函数");
    Serial.println("--------------------------------------------------------------");
    return false;
}

void runReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    // while at it, wait for a message from other nodes
    // TODO: I don't believe this node passes on the message
    Serial.println("--------------------------------------------------------------");
    Serial.printf("进入 runReceiver 函数，等待时间: %d 毫秒\n", wait_time);
    Serial.printf("当前节点地址: %u\n", selfAddress_);
    Serial.printf("目标节点地址（原始传入参数）: %u\n", *_msgFrom);
    if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, wait_time, _msgFrom)) {
        Serial.println("检测到收到消息的信号！");
        // char buf_[RH_MESH_MAX_MESSAGE_LEN];
        char buf_[RH_MESH_MAX_MESSAGE_LEN];

        esp_task_wdt_reset();
        Serial.println("Received a message");
        std::sprintf(buf_, "%s", reinterpret_cast<char*>(_msgRcvBuf));
        // 将接收到的缓冲区转换为字符串
        Serial.printf("接收到的原始消息内容（字符串化）：\"%s\"\n", buf_);


        // msgRcv = (struct Measurement)buf_; // should be able to set it to this
        Measurement* received = reinterpret_cast<Measurement*>(&buf_);
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
    Serial.println("退出 runReceiver 函数");
    Serial.println("--------------------------------------------------------------");
}

void runSender(uint8_t targetAddress_, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    // Need to look into sending structs over this
    // print logs
    Serial.println("----------------------------------------------------------------");
    Serial.println("进入 runSender 函数");
    Serial.printf("当前节点地址: %u\n", selfAddress_);
    Serial.printf("目标节点地址（原始传入参数）: %u\n", targetAddress_);
    Serial.println("开始构造数据包...");

    Packet p;
    p.node_number = selfAddress_;
    p.sensors[0] = 0;
    p.sensors[1] = 1;
    p.sensors[2] = 2;
    p.sensors[3] = 3;
    for (int i = 0; i < MAX_MEASUREMENTS; i++) {
        memcpy(&p.data[i], &measurements[i], sizeof(Measurement));
    }
    Serial.printf("数据包构造完成，数据包大小：%d 字节\n", sizeof(p));

    //For now we broadcast to all for the sake of testing
    Serial.println("正在发送数据包至广播地址...");
    uint8_t _err = RHMeshManager_.sendtoWait(reinterpret_cast<uint8_t*>(&p), sizeof(p), RH_BROADCAST_ADDRESS); 

    //original code
    //uint8_t _err = RHMeshManager_.sendtoWait(reinterpret_cast<uint8_t*>(&p), sizeof(p), targetAddress_); 


    if (_err == RH_ROUTER_ERROR_NONE) {
        // message successfully be sent to the target node, or next neighboring
        // expecting to recieve a simple reply from the target node
        esp_task_wdt_reset();
        Serial.printf("Packet sent successfull!");
        
        //We'll blcok this part for now, since we don't need to send a reply to the sender
        /*
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
        }*/

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
            /*
            for (int i = 0; i < MAX_MEASUREMENTS; i++) {
                 printMeasurements(received[i]);

            }
            */
            // Resets msgRcv
            memset(msgRcv, 0, sizeof(msgRcv));
            esp_task_wdt_reset();
            /*
            *we'll block this part for now, since we don't need to send a reply to the sender
            */
           /*
            std::string _msgRply = String("Hi node " + String(*_msgFrom) + ", got the message!").c_str();
            uint8_t _err = RHMeshManager_.sendtoWait(
                reinterpret_cast<uint8_t*>(&_msgRply[0]), _msgRply.size(), *_msgFrom);
            if (_err != RH_ROUTER_ERROR_NONE) {
                Serial.println("Fail to send reply...");
            }*/
            esp_task_wdt_reset();
        }
        esp_task_wdt_reset();
        //delay(50);
    }
}
// struct Measurement* packetInfo, , uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_
void runGatewaySender(String settings, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    Serial.println("==============================================================");
    Serial.println("进入 runGatewaySender 函数");
     // 打印发送的设置字符串及其长度
     Serial.printf("待发送 gateway 设置内容: \"%s\"\n", settings.c_str());
     Serial.printf("设置内容长度: %d 字节\n", settings.length());
     
     // 使用 sendtoWait 发送数据，当前通过广播地址发送
     Serial.println("调用 sendtoWait() 发送消息...");
    uint8_t _err = RHMeshManager_.sendtoWait(reinterpret_cast<uint8_t*>(&settings[0]), settings.length(), RH_BROADCAST_ADDRESS);


    if (_err == RH_ROUTER_ERROR_NONE) {
        // message successfully be sent to the target node, or next neighboring
        // expecting to recieve a simple reply from the target node
        esp_task_wdt_reset();
        Serial.printf(" successfull! Awaiting for Reply\n");
        Serial.println("sendtoWait 结果: 发送成功！");
        Serial.println("开始等待对端回复，超时时间: 250 毫秒...");
        // This eventually will need to be replaced
        if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, 250, _msgFrom)) {
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