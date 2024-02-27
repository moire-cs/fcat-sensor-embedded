void runReceiver(uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_)
{
    // while at it, wait for a message from other nodes
    Serial.println("Receiving mode active");

    if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, 3000, _msgFrom))
    {
        char buf_[RH_MESH_MAX_MESSAGE_LEN];

        esp_task_wdt_reset();
        Serial.println("Received a message");
        std::sprintf(buf_, "%s", reinterpret_cast<char *>(_msgRcvBuf));
        msgRcv = std::string(buf_);

        // do something with message, for example pass it through a callback
        Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", _msgFrom,
                      msgRcv.c_str(), RFM95Modem_.lastRssi());

        msgRcv = "";

        std::string _msgRply = String("Hi node " + String(*_msgFrom) + ", got the message!").c_str();
        uint8_t _err = RHMeshManager_.sendtoWait(
            reinterpret_cast<uint8_t *>(&_msgRply[0]), _msgRply.size(), *_msgFrom);
        if (_err != RH_ROUTER_ERROR_NONE)
        {
            Serial.println("Fail to send reply...");
        }
        esp_task_wdt_reset();
    }
}

void runSending(String packetInfo, uint8_t targetAddress_, uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_)
{
    uint8_t _err =
        RHMeshManager_.sendtoWait(reinterpret_cast<uint8_t *>(&packetInfo[0]),
                                  packetInfo.length(), targetAddress_);
    if (_err == RH_ROUTER_ERROR_NONE)
    {
        // message successfully be sent to the target node, or next neighboring
        // expecting to recieve a simple reply from the target node
        esp_task_wdt_reset();
        Serial.printf(" successfull! Awaiting for Reply\n");

        if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, 3000, _msgFrom))
        {
            char buf_[RH_MESH_MAX_MESSAGE_LEN];

            std::sprintf(buf_, "%s", reinterpret_cast<char *>(_msgRcvBuf));
            msgRcv = std::string(buf_);
            Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", *_msgFrom,
                          msgRcv.c_str(), RFM95Modem_.lastRssi());
        }
        else
        {
            Serial.println("No reply, is the target node running?");
        }

        esp_task_wdt_reset();
    }
    else
    {
        Serial.println(
            "sendtoWait failed. No response from intermediary node, are they "
            "running?");

        // TODO: If nobody received, up power? Maybe a function to do default power applied (starts at low value and increases, then stays at whatever value works)
    }
}