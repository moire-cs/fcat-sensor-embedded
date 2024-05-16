// For Measurements
#include <vector>

#define MAX_MEASUREMENTS 14                       // max number of measurements to take in a set time
RTC_DATA_ATTR unsigned int measurement_count = 0; // unsure if this will just stay as 0, so we need to check

// For Device Sleep
RTC_DATA_ATTR float duration = 0.02;                              // hours
RTC_DATA_ATTR unsigned int num_measurements = 2;                             // num measurements to take in a set time
RTC_DATA_ATTR float time_sync_tolerance = 0.005; // factor
RTC_DATA_ATTR int sync_duration = 30 * 1000;

#define microseconds 1000000                           // 1 second in microseconds
#define hours_to_seconds 3600

RTC_DATA_ATTR uint64_t timer = duration * hours_to_seconds / (num_measurements); // (equally spaces out measurements) converted to microseconds in code

// Radio Constants
#define RF95_FREQ 915.0 // USA and Ecuador
#define WDT_TIMEOUT 65

#if defined(RFM95_CS) && defined(RFM95_RST) && defined(RFM95_INT)
#else

// Board pinout (from mesh)
#define RFM95_CS 5
#define RFM95_RST 14
#define RFM95_INT 13
#endif

// For State Machine
RTC_DATA_ATTR boolean isFull;

#define WAITING 0
#define SENSING 1
#define RECEIVING 2
#define SENDING 3
// #define SLEEPING 4

RTC_DATA_ATTR unsigned int state = WAITING;

// radio driver & message mesh delivery/receipt manager
RH_RF95 RFM95Modem_(RFM95_CS, RFM95_INT);
RHMesh RHMeshManager_(RFM95Modem_, selfAddress_);

// these are expected to be global/externally exposed variables, if you plan to
// make a class to wrap this
std::string msgSend =
String("Hello from node " + String(selfAddress_) + "!").c_str();

String timeSyncRcv;

uint8_t _msgRcvBuf[RH_MESH_MAX_MESSAGE_LEN];
uint8_t _timeSyncRcvBuf[RH_MESH_MAX_MESSAGE_LEN];

void rhSetup() {
    if (!RHMeshManager_.init())
        Serial.println("init failed");
    RFM95Modem_.setTxPower(20, false); // 120 mA for transmitting
    RFM95Modem_.setFrequency(RF95_FREQ);
    RFM95Modem_.setCADTimeout(500);
}

RTC_DATA_ATTR uint64_t curr_time = 0;
RTC_DATA_ATTR uint64_t last_time = 0;

void splitn(float* tokens, std::string s, std::string delimiter, int n) {
    size_t pos = 0;
    int i = 0;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        tokens[i] = std::stof(s.substr(0, pos));
        i++;
        s.erase(0, pos + delimiter.length());
    }
    last_time = curr_time;
    curr_time = std::stoull(s);
}

struct __attribute__((__packed__)) Measurement // 16 bytes
{
    uint16_t timestamp;        // 2
    uint16_t moisture_percent; // 2
    float temperature;             // 4
    float humidity;                // 4
    uint16_t light_level;      // 2
    uint16_t battery_level;    // 2
};

struct Packet {
    unsigned int node_number;
    // std::vector<uint8_t> sensors;
    uint8_t sensors[5];
    struct Measurement data[MAX_MEASUREMENTS];
};

void __attribute__((__packed__)) printMeasurement(struct Measurement m) {
    Serial.println("Measurement Number: " + String(selfAddress_));
    Serial.println("Moisture: " + String(m.moisture_percent) + "%");
    Serial.println("Temperature: " + String(m.temperature) + "F");
    Serial.println("Humidity: " + String(m.humidity) + "%");
    Serial.println("Light Level: " + String(m.light_level));
    Serial.println("Battery Level: " + String(m.battery_level) + "mV");   
}

double getMoistureConvert(int moisture) {
    // return (-11.8571 + 1/(0.0007*sqrt(moisture)));
    // return 54.1 -0.0219*moisture + 2.54*pow(10, -6)*pow(moisture, 2);
    // return pow(14221*moisture, -0.895);
    return 57*exp(-5.05 * moisture * pow(10, -4));
}
String measurementToArray(struct Measurement m) {
    
    return "[" + String((m.moisture_percent)) + "," + String(m.temperature) + "," + String(m.humidity) + "," + String(m.light_level) + "," + String(m.battery_level) + "]";
}

String allMeasurementsToArray(struct Measurement m[]) {
    String arr = "[";
    for (int i = 0; i < MAX_MEASUREMENTS; i++) {
        if (m[i].moisture_percent == 0)
            break;
        arr = arr + measurementToArray(m[i]);
        if (i < MAX_MEASUREMENTS && m[i+1].moisture_percent != 0)
            arr = arr + ",";
    }
    arr = arr + "]";
    return arr;
}

String getMessage(Packet p, String times) {
    String sensors = "[";
    int lim = sizeof(p.sensors)/sizeof(uint8_t);
    for (int i = 0; i < lim; i++) {
        sensors = sensors + String(p.sensors[i]);
        if (i < lim - 1)
            sensors = sensors + ",";
    }
    sensors = sensors + "]";

    String message = "{ \"nodeId\" : " + String(p.node_number) + ",";
    message = message + "\"times\" : "  + String(times) + ",";
    message = message + "\"sensors\" : " + String(sensors.c_str()) + ",";
    message = message + "\"messages\" : " + String(allMeasurementsToArray(p.data)) + " }";
    return message;
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
    Serial.printf("Sensors: %d, %d, %d, %d %d\n", p.sensors[0], p.sensors[1], p.sensors[2], p.sensors[3], p.sensors[4]);

    printMeasurements(p.data);
}
RTC_DATA_ATTR struct Measurement measurements[MAX_MEASUREMENTS];