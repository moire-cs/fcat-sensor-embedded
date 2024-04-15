// For Measurements

#define MAX_MEASUREMENTS 24                       // max number of measurements to take in a set time
RTC_DATA_ATTR unsigned int measurement_count = 1; // unsure if this will just stay as 0, so we need to check

// Using a struct saves memory
struct Measurement // 16 bytes
{
    unsigned int timestamp;        // 2
    unsigned int moisture_percent; // 2
    float temperature;             // 4
    float humidity;                // 4
    unsigned int light_level;      // 2
    unsigned int battery_level;    // 2
};

RTC_DATA_ATTR struct Measurement measurements[MAX_MEASUREMENTS]; // measurements stored in RTC memory

// For Device Sleep
RTC_DATA_ATTR uint64_t duration = 0;                              // hours
RTC_DATA_ATTR unsigned int num_measurements = 4;                             // num measurements to take in a set time
RTC_DATA_ATTR unsigned int time_sync_tolerance = 5;
RTC_DATA_ATTR unsigned int mesh_sync_tolerance = 5;

#define microseconds 1000000                           // 1 second in microseconds
#define hours_to_seconds 3600

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


// For State Machine
RTC_DATA_ATTR boolean isFull;

#define WAITING 0
#define SENSING 1
#define RECEIVING 2
#define SENDING 3
// #define SLEEPING 4

RTC_DATA_ATTR unsigned int state = WAITING;

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

std::string timeSyncRcv;
struct Measurement msgRcv[MAX_MEASUREMENTS];

void rhSetup() {
    if (!RHMeshManager_.init())
        Serial.println("init failed");
    RFM95Modem_.setTxPower(17, false);
    RFM95Modem_.setFrequency(RF95_FREQ);
    RFM95Modem_.setCADTimeout(500);
}

std::string* splitn(std::string s, std::string delimiter, int n) {
    size_t pos = 0;
    std::string* tokens = new std::string[n];
    int i = 0;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        tokens[i++] = s.substr(0, pos);
        s.erase(0, pos + delimiter.length());
    }

    return tokens;
}
