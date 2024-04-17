#include <vector> 
#define gateway_wait 1000
void postData(struct Measurement m);

bool runTimeSyncReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    // while at it, wait for a message from other nodes
    if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, wait_time, _msgFrom)) {

        // char buf_[RH_MESH_MAX_MESSAGE_LEN];
        char buf_[RH_MESH_MAX_MESSAGE_LEN];

        esp_task_wdt_reset();
        Serial.println("Received a message");
        std::sprintf(buf_, "%s", reinterpret_cast<char*>(_msgRcvBuf));
        timeSyncRcv = String(buf_).c_str();

        // msgRcv = (struct Measurement)buf_; // should be able to set it to this
        // Measurement* received = reinterpret_cast<Measurement*>(&buf_);

        // do something with message, for example pass it through a callback
        Serial.printf("Rebroadcasting to available nodes...\n");

        // clears msgRcv
        // memset(msgRcv, 0, sizeof(msgRcv));

        // std::string _msgRply = String("Hi node " + String(*_msgFrom) + ", got the message!").c_str();
        uint8_t _err = RHMeshManager_.sendtoWait(
            reinterpret_cast<uint8_t*>(&timeSyncRcv), timeSyncRcv.size(), RH_BROADCAST_ADDRESS);
        if (_err != RH_ROUTER_ERROR_NONE) {
            Serial.println("Fail to send broadcast...");
        }
        esp_task_wdt_reset();
        return true;
    }
    esp_task_wdt_reset();
    return false;
}

void runReceiver(uint16_t wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    // while at it, wait for a message from other nodes
    // TODO: I don't believe this node passes on the message

    if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, wait_time, _msgFrom)) {

        // char buf_[RH_MESH_MAX_MESSAGE_LEN];
        char buf_[RH_MESH_MAX_MESSAGE_LEN];

        esp_task_wdt_reset();
        Serial.println("Received a message");
        std::sprintf(buf_, "%s", reinterpret_cast<char*>(_msgRcvBuf));


        // msgRcv = (struct Measurement)buf_; // should be able to set it to this
        Measurement* received = reinterpret_cast<Measurement*>(&buf_);

        // do something with message, for example pass it through a callback
        Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", _msgFrom,
            received, RFM95Modem_.lastRssi());

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
}

void runSender(String packetInfo, uint8_t targetAddress_, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    // Need to look into sending structs over this
    int arr[7] = { 1,2,3,4,67, 5, 4 };
    // char* test = "Hello World!";
    // std::string test = "Hello World!";
    std::string test = packetInfo.c_str();
    std::vector<uint8_t> bytes(test.begin(), test.end());
    uint8_t* data = bytes.data();
    uint8_t _err =
        RHMeshManager_.sendtoWait(data,
            sizeof(bytes), targetAddress_);
    // Serial.println(String(sizeof(arr)));
    Serial.println(String(&packetInfo[0]));
    Serial.println(String(packetInfo.length()));
    // Serial.println(String(packetInfo));
    // Serial.println(String(*packetInfo));
    // Serial.println(sizeof(measurements));
    // Serial.println(sizeof(reinterpret_cast<uint8_t*>(measurements)));
    // for (int i = 0; i < sizeof(measurements) / sizeof(Measurement); i++) {
    //     printMeasurement(measurements[i]);
    //     // postData(packetInfo[i]);
    // }
    if (_err == RH_ROUTER_ERROR_NONE) {
        // message successfully be sent to the target node, or next neighboring
        // expecting to recieve a simple reply from the target node
        esp_task_wdt_reset();
        Serial.printf(" successfull! Awaiting for Reply\n");

        if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, 3000, _msgFrom)) {
            char buf_[RH_MESH_MAX_MESSAGE_LEN];

            std::sprintf(buf_, "%s", reinterpret_cast<char*>(_msgRcvBuf));
            Measurement* received = reinterpret_cast<Measurement*>(&buf_);
            Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", *_msgFrom,
                received, RFM95Modem_.lastRssi());
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
}

void runGatewayReceiver(int wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {

    Serial.println("Receiving mode active");
    uint64_t start = millis();

    // For wait_time, the gateway will wait for messages (multiple), and then post them to the backend
    while (millis() - start < wait_time) {
        if (RHMeshManager_.recvfromAck(_msgRcvBuf, _msgRcvBufLen, _msgFrom)) {
            char buf_[RH_MESH_MAX_MESSAGE_LEN];

            // esp_task_wdt_reset();
            Serial.println("Received a message");
            std::sprintf(buf_, "%s", reinterpret_cast<char*>(_msgRcvBuf));
            std::string rcvMsg = String(buf_).c_str();
            Serial.printf("%s\n", rcvMsg);
            Serial.println(String(*_msgRcvBufLen));
            // Measurement* received = reinterpret_cast<Measurement*>(buf_); // theoretically able to set it to this
            // int* received = reinterpret_cast<int*>(_msgRcvBuf);
            // Serial.println(String(*_msgRcvBufLen));
            // Serial.println(String(*_msgRcvBufLen));
            // memcpy(intRcv, _msgRcvBuf, sizeof(intRcv));
            // for (int i = 0; i < sizeof(intRcv) / sizeof(int); i++) {
            //     Serial.printf("%d\n", intRcv[i]);
            // }
            // std::sprintf(received, )

            // FIXME: Probably need to do a for loop (not for each)
            // for (struct Measurement m : &received)
            // {
            //     postData(m);
            // }

            // In here, we would want to take that message and do an HTTP POST to the backend with an auth key
            // instead of just sending a reply
            // Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", _msgFrom,
            //     received, RFM95Modem_.lastRssi());
            // Serial.println(String(sizeof(*received)));
            // Serial.println(String(sizeof(*_msgRcvBuf)));
            // Serial.println(buf_);
            // for (int i = 0; i < sizeof(*received) / sizeof(int); i++) {
            //     // printMeasurement(received[i]);
            //     Serial.printf("%d\n", received[i]);
            //     // postData(received[i]);
            // }

            // Resets msgRcv
            // memset(msgRcv, 0, sizeof(msgRcv));

            std::string _msgRply = String("Hi node " + String(*_msgFrom) + ", got the message!").c_str();
            uint8_t _err = RHMeshManager_.sendtoWait(
                reinterpret_cast<uint8_t*>(&_msgRply[0]), _msgRply.size(), *_msgFrom);
            if (_err != RH_ROUTER_ERROR_NONE) {
                Serial.println("Fail to send reply...");
            }
            esp_task_wdt_reset();
        }
        esp_task_wdt_reset();
    }
}
// struct Measurement* packetInfo, , uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_
void runGatewaySender(std::string settings, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    uint8_t _err =
        RHMeshManager_.sendtoWait(reinterpret_cast<uint8_t*>(&settings), sizeof(settings), RH_BROADCAST_ADDRESS);
    if (_err == RH_ROUTER_ERROR_NONE) {
        // message successfully be sent to the target node, or next neighboring
        // expecting to recieve a simple reply from the target node
        esp_task_wdt_reset();
        Serial.printf(" successfull! Awaiting for Reply\n");

        // This eventually will need to be replaced
        // if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, 3000, _msgFrom)) {
        //     char buf_[RH_MESH_MAX_MESSAGE_LEN];

        //     std::sprintf(buf_, "%s", reinterpret_cast<char*>(_msgRcvBuf));
        //     // Measurement *received = reinterpret_cast<Measurement *>(buf_);
        //     String received = String(buf_);
        //     Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", *_msgFrom,
        //         received, RFM95Modem_.lastRssi());
        // }
        // else {
        //     Serial.println("No reply, is the target node running?");
        // }

        esp_task_wdt_reset();
    }
    else {
        Serial.println(
            "sendtoWait failed. No response from intermediary node, are they "
            "running?");

        // TODO: If nobody received, up power? Maybe a function to do default power applied (starts at low value and increases, then stays at whatever value works)
    }
}