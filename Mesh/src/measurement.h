#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "ClosedCube_HDC1080.h"
#include "esp_wifi.h"
#include <driver/adc.h>
#include "sensor_pinouts.h"
#include <Wire.h>
#include <driver/adc.h>
#include <EEPROM.h>
// #include "pinouts_and_constants.h"

ClosedCube_HDC1080 hdc1080;

int measurement_memory_start = 8; // first 8 bytes are reserved

class Measurement
{
public:
    unsigned int moisture_percent;
    double temperature;
    double humidity;
    unsigned int light_level;
    unsigned int battery_level;
    void printMeasurement()
    {
        Serial.println("Moisture: " + String(moisture_percent) + "%");
        Serial.println("Temperature: " + String(temperature) + "F");
        Serial.println("Humidity: " + String(humidity) + "%");
        Serial.println("Light Level: " + String(light_level));
        Serial.println("Battery Level: " + String(battery_level) + "mV");
    }
};

void measureSetup()
{
    ENABLE_ACC_RAIL();

    pcnt_unit_t unit = PCNT_UNIT_0;

    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control
        .pulse_gpio_num = moisture,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        .pos_mode = PCNT_COUNT_DIS,      // Count up on the positive edge
        .neg_mode = PCNT_COUNT_INC,      // Keep the counter value on the negative edge
        .unit = unit,
        .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&pcnt_config);

    hdc1080.begin(0x40);
}
Measurement getReadings()
{

    // Enable the Power Rail
    ENABLE_ACC_RAIL();

    // Create measurement object
    Measurement m;

    // Moisture Reading
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    // TODO: Take time here
    // FIXME: We need to delay but we also need to be taking the time for this
    delay(SOIL_PULSE_COUNT_DELAY);
    pcnt_get_counter_value(PCNT_UNIT_0, &moisture_count);
    pcnt_counter_pause(PCNT_UNIT_0);
    // TODO: Take time here
    // TODO: calculate difference
    m.moisture_percent = moisture_count;

    // Light Level
    m.light_level = analogRead(light); // this will be out of 4095

    // Temperature/Humidity Reading
    hdc1080.begin(0x40);
    // TODO: Do we need to begin this every time
    delay(100);
    m.temperature = hdc1080.readTemperature() * 1.8 + 32;
    m.humidity = hdc1080.readHumidity();

    // Battery Reading
    int battery_level;
    esp_err_t r = adc2_get_raw(ADC2_CHANNEL_8, ADC_WIDTH_12Bit, &battery_level); // this will be out of 4095
    m.battery_level = battery_level;                                             // Can't put unsigned int into function above

    // Disable the Power Rail
    DISABLE_ACC_RAIL();
    // saveReadings(m);
    return m;
};

// writes sensor readings to flash memory
boolean saveReading(Measurement m)
{
    unsigned int measurement_progress = EEPROM.readInt(0);                                 // get number of readings taken so far
    int base_addr = measurement_progress * sizeof(Measurement) + measurement_memory_start; // base address for the next reading
    measurement_progress++;                                                                // increment the number of readings taken so far
    // Commit to memory

    EEPROM.writeInt(base_addr, m.moisture_percent);
    EEPROM.writeDouble(base_addr + sizeof(unsigned int), m.temperature);
    EEPROM.writeDouble(base_addr + sizeof(unsigned int) + sizeof(double), m.humidity);
    EEPROM.writeInt(base_addr + sizeof(unsigned int) + 2 * sizeof(double), m.light_level);
    EEPROM.writeInt(base_addr + 2 * sizeof(unsigned int) + 2 * sizeof(double), m.battery_level);

    EEPROM.writeInt(0, measurement_progress); // update the number of readings taken so far

    EEPROM.commit();

    return measurement_progress == num_measurements; // returns true if measurement count is over
    // might need a consideration for the time
}

// prints all stored readings
void printReadings()
{
    for (int numReadings = 0; numReadings < 3; numReadings++)
    {
        Measurement m;
        int base_addr = measurement_memory_start + numReadings * sizeof(Measurement);
        m.moisture_percent = EEPROM.readInt(base_addr);
        m.temperature = EEPROM.readDouble(base_addr + sizeof(unsigned int));
        m.humidity = EEPROM.readDouble(base_addr + sizeof(unsigned int) + sizeof(double));
        m.light_level = EEPROM.readInt(base_addr + sizeof(unsigned int) + 2 * sizeof(double));
        m.battery_level = EEPROM.readInt(base_addr + 2 * sizeof(unsigned int) + 2 * sizeof(double));
        m.printMeasurement();
        delete &m;
    }
}

// clears all readings from memory (sets them to 0)
void clearReadings()
{
    for (int i = 0; i < 64; i++)
        EEPROM.writeInt(i * sizeof(double), 0);
    EEPROM.commit();
}