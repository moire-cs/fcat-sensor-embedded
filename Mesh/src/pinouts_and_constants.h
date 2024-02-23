#include <RH_RF95.h>

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

// TODO: Define sensor pinouts

// Sensor pins
#define battery 25 // FIXME: Do we need this either if we're using ADC
#define moisture 34
#define light 35

#define clockPin 18
#define dataPin 21

// pins for radio
#define misoPin 23
#define mosiPin 19
#define nssPin 5
#define resetPin 14
#define dio0Pin 13
#define dio1Pin 12

// For Pulse Counter
#define SOIL_PULSE_COUNT_DELAY 500
#define MIN -21300 * WAIT_TIME / 1000 // will change based on soil moisture measurements
#define MAX -740 * WAIT_TIME / 1000   // will change based on soil moisture measurements
int16_t moisture_count = 0x10;        // Do we need 16? or is 0x00 ok
int counter = 0;
//  M  T  H  L  B
double readings[5] = {0, 0, 0, 0, 0};

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
const uint8_t selfAddress_ = ENDNODE_ADDRESS; // CHANGE THIS!!!
const uint8_t targetAddress_ = NODE_ADDRESS;  // integer value
#endif

// radio driver & message mesh delivery/receipt manager
RH_RF95 RFM95Modem_(RFM95_CS, RFM95_INT);
RHMesh RHMeshManager_(RFM95Modem_, selfAddress_);
uint8_t mode_ = RECEIVING_MODE;

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