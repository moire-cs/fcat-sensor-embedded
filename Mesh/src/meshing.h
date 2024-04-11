#define gateway_wait 1000
void postData(struct Measurement m);

void runReceiver(uint16_t wait_time, uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_)
{
    // while at it, wait for a message from other nodes
    // TODO: I don't believe this node passes on the message

    if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, wait_time, _msgFrom))
    {

        // char buf_[RH_MESH_MAX_MESSAGE_LEN];
        char buf_[RH_MESH_MAX_MESSAGE_LEN];

        esp_task_wdt_reset();
        Serial.println("Received a message");
        std::sprintf(buf_, "%s", reinterpret_cast<char *>(_msgRcvBuf));
        // msgRcv = (struct Measurement)buf_; // should be able to set it to this
        Measurement *received = reinterpret_cast<Measurement *>(&buf_);

        // do something with message, for example pass it through a callback
        Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", _msgFrom,
                      received, RFM95Modem_.lastRssi());

        // clears msgRcv
        // memset(msgRcv, 0, sizeof(msgRcv));

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

void runSender(struct Measurement *packetInfo, uint8_t targetAddress_, uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_)
{
    // Need to look into sending structs over this
    uint8_t _err =
        RHMeshManager_.sendtoWait(reinterpret_cast<uint8_t *>(packetInfo),
                                  sizeof(packetInfo), targetAddress_);
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
            Measurement *received = reinterpret_cast<Measurement *>(&buf_);
            Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", *_msgFrom,
                          received, RFM95Modem_.lastRssi());
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

void runGatewayReceiver(int wait_time, uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_)
{

    Serial.println("Receiving mode active");
    uint64_t start = millis();

    // For wait_time, the gateway will wait for messages (multiple), and then post them to the backend
    while (millis() - start < wait_time)
    {
        if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, gateway_wait, _msgFrom))
        {
            char buf_[RH_MESH_MAX_MESSAGE_LEN];

            esp_task_wdt_reset();
            Serial.println("Received a message");
            std::sprintf(buf_, "%s", reinterpret_cast<char *>(_msgRcvBuf));
            Measurement *received = reinterpret_cast<Measurement *>(&buf_); // theoretically able to set it to this

            // FIXME: Probably need to do a for loop (not for each)
            // for (struct Measurement m : &received)
            // {
            //     postData(m);
            // }

            // In here, we would want to take that message and do an HTTP POST to the backend with an auth key
            // instead of just sending a reply
            Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", _msgFrom,
                          received, RFM95Modem_.lastRssi());

            // Resets msgRcv
            // memset(msgRcv, 0, sizeof(msgRcv));

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
}

void runGatewaySender(unsigned int *settings, uint8_t *_msgRcvBuf, uint8_t *_msgRcvBufLen, uint8_t *_msgFrom, RH_RF95 RFM95Modem_, RHMesh RHMeshManager_)
{
    uint8_t _err =
        RHMeshManager_.sendtoWait(reinterpret_cast<uint8_t *>(settings),
                                  sizeof(settings), 255);
    if (_err == RH_ROUTER_ERROR_NONE)
    {
        // message successfully be sent to the target node, or next neighboring
        // expecting to recieve a simple reply from the target node
        esp_task_wdt_reset();
        Serial.printf(" successfull! Awaiting for Reply\n");

        // This eventually will need to be replaced
        if (RHMeshManager_.recvfromAckTimeout(_msgRcvBuf, _msgRcvBufLen, 3000, _msgFrom))
        {
            char buf_[RH_MESH_MAX_MESSAGE_LEN];

            std::sprintf(buf_, "%s", reinterpret_cast<char *>(_msgRcvBuf));
            // Measurement *received = reinterpret_cast<Measurement *>(buf_);
            String received = String(buf_);
            Serial.printf("[%d] \"%s\" (%d). Sending a reply...\n", *_msgFrom,
                          received, RFM95Modem_.lastRssi());
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