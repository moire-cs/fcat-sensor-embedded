#include <RH_RF95.h>

// Flash Memory Allocation
#define EEPROM_SIZE 120 // 5 readings * 8 bytes (double) * 3 packets = 120 bytes

// For Measurements

#define MAX_MEASUREMENTS 24 // max number of measurements to take in a set time

// For Device Sleep
uint64_t time_period = 0;                             // 40 seconds
#define num_measurements 4                             // num measurements to take in a set time
uint64_t timer = time_period / (num_measurements - 1); // (equally spaces out measurements) converted to microseconds in code
#define microseconds 1000000                           // 1 second in microseconds
#define hours_to_seconds 3600                          // 1 hour in seconds

// Radio Constants
#define RF95_FREQ 915.0 // USA and Ecuador
#define WDT_TIMEOUT 15

#if defined(RFM95_CS) && defined(RFM95_RST) && defined(RFM95_INT)
#else

// Board pinout (from mesh)
#define RFM95_CS 5
#define RFM95_RST 14
#define RFM95_INT 13
#endif

#define SENDING_MODE 0
#define RECEIVING_MODE 1
#define SENSING_MODE 2

#if defined(SELF_ADDRESS) && defined(TARGET_ADDRESS)
const uint8_t selfAddress_ = SELF_ADDRESS;
const uint8_t targetAddress_ = TARGET_ADDRESS;
#else
// Topology
// define the node address
#define NODE_ADDRESS 3
#define ENDNODE_ADDRESS 255 // purposefully using the last number
// TODO: according to this, we might have a max of 256 nodes in one mesh
// selfAddress is node
// targetAddress will be our gateway
const uint8_t selfAddress_ = NODE_ADDRESS;      // CHANGE THIS!!!
const uint8_t targetAddress_ = ENDNODE_ADDRESS; // integer value
#endif

// radio driver & message mesh delivery/receipt manager
RH_RF95 RFM95Modem_(RFM95_CS, RFM95_INT);
RHMesh RHMeshManager_(RFM95Modem_, selfAddress_);
uint8_t mode_ = SENSING_MODE;

// these are expected to be global/externally exposed variables, if you plan to
// make a class to wrap this
std::string msgSend =
    String("Hello from node " + String(selfAddress_) + "!").c_str();
std::string msgRcv;

void rhSetup()
{
    if (!RHMeshManager_.init())
        Serial.println("init failed");
    RFM95Modem_.setTxPower(17, false);
    RFM95Modem_.setFrequency(RF95_FREQ);
    RFM95Modem_.setCADTimeout(500);
}
