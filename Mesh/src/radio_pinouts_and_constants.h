// For Measurements

#define MAX_MEASUREMENTS 5                       // max number of measurements to take in a set time
RTC_DATA_ATTR unsigned int measurement_count = 0; // unsure if this will just stay as 0, so we need to check

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

struct Packet {
    unsigned int node_number;
    uint8_t sensors[4];
    struct Measurement data[MAX_MEASUREMENTS];
};

void printMeasurement(struct Measurement m) {
    // Serial.println("Measurement Number: " + String(NODE_ADDRESS) + ":" + String(m.measurement_num));
    Serial.println("Moisture: " + String(m.moisture_percent) + "%");
    Serial.println("Temperature: " + String(m.temperature) + "F");
    Serial.println("Humidity: " + String(m.humidity) + "%");
    Serial.println("Light Level: " + String(m.light_level));
    Serial.println("Battery Level: " + String(m.battery_level) + "mV");
}

void printMeasurements(struct Measurement m[]) {
    for (int i = 0; i < MAX_MEASUREMENTS; i++) {
        if (m[i].moisture_percent == 0)
            break;
        printMeasurement(m[i]);
    }
}

void printPacket(struct Packet p) {
    Serial.printf("\n==============================================\n");
    Serial.printf("Node Number: %d\n", p.node_number);
    Serial.printf("Sensors: %d, %d, %d, %d\n", p.sensors[0], p.sensors[1], p.sensors[2], p.sensors[3]);

    printMeasurements(p.data);
}

RTC_DATA_ATTR struct Measurement measurements[MAX_MEASUREMENTS]; // measurements stored in RTC memory

// For Device Sleep
RTC_DATA_ATTR float duration = 0.02;                              // hours
RTC_DATA_ATTR unsigned int num_measurements = 2;                             // num measurements to take in a set time
RTC_DATA_ATTR float time_sync_tolerance = 0.005; // factor
RTC_DATA_ATTR float mesh_sync_tolerance = 0.005; // factor

#define microseconds 1000000                           // 1 second in microseconds
#define hours_to_seconds 3600

RTC_DATA_ATTR uint64_t timer = duration * hours_to_seconds / (num_measurements); // (equally spaces out measurements) converted to microseconds in code



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

// #define SENDING_MODE 0
// #define RECEIVING_MODE 1
// #define SENSING_MODE 2


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
#define NODE_ADDRESS_1 0 // 
#define NODE_ADDRESS_2 1 //
#define NODE_ADDRESS_3 2 //
#define ENDNODE_ADDRESS 254 // purposefully using the last number (0-254, 255 is broadcast)
// TODO: according to this, we might have a max of 256 nodes in one mesh
// selfAddress is node
// targetAddress will be our gateway
const uint8_t selfAddress_ = NODE_ADDRESS_2;      // CHANGE THIS!!!
const uint8_t targetAddress_ = ENDNODE_ADDRESS; // integer value
#endif

// radio driver & message mesh delivery/receipt manager
RH_RF95 RFM95Modem_(RFM95_CS, RFM95_INT);
RHMesh RHMeshManager_(RFM95Modem_, selfAddress_);
// uint8_t mode_ = SENSING_MODE;

// these are expected to be global/externally exposed variables, if you plan to
// make a class to wrap this
std::string msgSend =
String("Hello from node " + String(selfAddress_) + "!").c_str();

String timeSyncRcv;
std::string rcvMsg;
struct Measurement msgRcv[MAX_MEASUREMENTS];
int intRcv[MAX_MEASUREMENTS];

uint8_t _msgRcvBuf[RH_MESH_MAX_MESSAGE_LEN];
uint8_t _timeSyncRcvBuf[RH_MESH_MAX_MESSAGE_LEN];

void rhSetup() {
    if (!RHMeshManager_.init())
        Serial.println("init failed");
    RFM95Modem_.setTxPower(17, false);
    RFM95Modem_.setFrequency(RF95_FREQ);
    RFM95Modem_.setCADTimeout(500);
}

void splitn(float* tokens, std::string s, std::string delimiter, int n) {
    size_t pos = 0;
    // Serial.printf("Splitting string: %s\n", s.c_str());
    // Serial.printf("Delimiter: %s\n", delimiter.c_str());
    int i = 0;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        // Serial.printf("Left: %s\n", s.c_str());
        // Serial.printf("%s\n", s.substr(0, pos).c_str());
        // Serial.printf("%0.3f\n", std::stof(s.substr(0, pos)));
        tokens[i] = std::stof(s.substr(0, pos));
        i++;
        // Serial.println("n: " + String(i));
        // Serial.println("Token: " + String(tokens[i]));

        s.erase(0, pos + delimiter.length());
        // Serial.printf("Right: %s\n", s.c_str());
    }
    // Serial.println("END");
    tokens[n - 1] = std::stof(s);
    // for (int k = 0; k < n; k++)
    //     Serial.printf("Token: %0.3f\n", tokens[k]);
}
