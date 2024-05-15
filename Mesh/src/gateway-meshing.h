void runGatewayReceiver(int wait_time, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {

    Serial.println("Receiving mode active");
    uint64_t start = millis();

    // For wait_time, the gateway will wait for messages (multiple), and then post them to the backend
    while (millis() - start < wait_time) {
        if (RHMeshManager_.recvfromAck(_msgRcvBuf, _msgRcvBufLen, _msgFrom)) {
            char buf_[RH_MESH_MAX_MESSAGE_LEN];

            esp_task_wdt_reset();
            Serial.println("Received a message");

            Packet* received = reinterpret_cast<Packet*>(_msgRcvBuf);

            String message = getMessage(*received, cur_times);
            Serial.printf("MESSAGE: %s\n", message.c_str());

            // memset(msgRcv, 0, sizeof(msgRcv));
            esp_task_wdt_reset();
            // std::string _msgRply = String("Hi node " + String(*_msgFrom) + ", got the message!").c_str();
            // uint8_t _err = RHMeshManager_.sendtoWait(
            //     reinterpret_cast<uint8_t*>(&_msgRply[0]), _msgRply.size(), *_msgFrom);
            // if (_err != RH_ROUTER_ERROR_NONE) {
            //     Serial.println("Fail to send reply...");
            // }
            esp_task_wdt_reset();
        }
        esp_task_wdt_reset();
    }
}
// struct Measurement* packetInfo, , uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_
void runGatewaySender(String settings, uint8_t* _msgRcvBuf, uint8_t* _msgRcvBufLen, uint8_t* _msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_) {
    uint8_t _err = RHMeshManager_.sendtoWait(reinterpret_cast<uint8_t*>(&settings[0]), settings.length(), RH_BROADCAST_ADDRESS);
    if (_err == RH_ROUTER_ERROR_NONE) {
        // message successfully be sent to the target node, or next neighboring
        // expecting to recieve a simple reply from the target node
        esp_task_wdt_reset();
        Serial.printf(" successfull! Awaiting for Reply\n");

        // This eventually will need to be replaced
        if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, 250, _msgFrom)) {
            char buf_[RH_MESH_MAX_MESSAGE_LEN];

            std::sprintf(buf_, "%s", reinterpret_cast<char*>(_msgRcvBuf));
            //     // Measurement *received = reinterpret_cast<Measurement *>(buf_);
            String received = String(buf_).c_str();
            // Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", *_msgFrom,
            //     received.c_str(), RFM95Modem_.lastRssi());
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

        // TODO: If nobody received, up power? Maybe a function to do default power applied (starts at low value and increases, then stays at whatever value works)
    }
    esp_task_wdt_reset();
}