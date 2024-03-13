// TODO: Define sensor pinouts

#include <cstdint>
#define power_rail 26

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
// double readings[5] = {0, 0, 0, 0, 0};

void ENABLE_ACC_RAIL()
{
    digitalWrite(power_rail, HIGH);
}
void DISABLE_ACC_RAIL()
{
    digitalWrite(power_rail, LOW);
}