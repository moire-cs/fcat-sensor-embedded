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

void printMeasurement(struct Measurement m)
{
    // Serial.println("Measurement Number: " + String(NODE_ADDRESS) + ":" + String(m.measurement_num));
    Serial.println("Moisture: " + String(m.moisture_percent) + "%");
    Serial.println("Temperature: " + String(m.temperature) + "F");
    Serial.println("Humidity: " + String(m.humidity) + "%");
    Serial.println("Light Level: " + String(m.light_level));
    Serial.println("Battery Level: " + String(m.battery_level) + "mV");
}
void measureSetup()
{
    ENABLE_ACC_RAIL();

    pinMode(moisture, INPUT);
    pinMode(light, INPUT);
    pinMode(battery, INPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(misoPin, INPUT);
    pinMode(mosiPin, OUTPUT);
    pinMode(power_rail, OUTPUT);

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
unsigned int readMoisture()
{

    // Moisture Reading
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    delay(SOIL_PULSE_COUNT_DELAY);
    pcnt_get_counter_value(PCNT_UNIT_0, &moisture_count);
    pcnt_counter_pause(PCNT_UNIT_0);

    return abs(moisture_count);
}
Measurement getReadings()
{

    // Enable the Power Rail
    ENABLE_ACC_RAIL();

    // Create measurement object
    struct Measurement m;

    m.moisture_percent = readMoisture();

    // Light Level
    m.light_level = analogRead(light) * 4095; // this will be out of 4095
    printf("\n\n%0.5f\n\n", analogRead(light));
    hdc1080.begin(0x40);
    // Temperature/Humidity Reading
    m.temperature = hdc1080.readTemperature() * 1.8 + 32;
    m.humidity = hdc1080.readHumidity();

    // Battery Reading
    int battery_level;
    esp_err_t r = adc2_get_raw(ADC2_CHANNEL_8, ADC_WIDTH_12Bit, &battery_level); // this will be out of 4095
    m.battery_level = battery_level;                                             // Can't put unsigned int into function above

    // Disable the Power Rail
    DISABLE_ACC_RAIL();

    return m;
};

// writes sensor readings to RTC memory
boolean saveReading(struct Measurement m)
{
    // Sets the struct object to be equal

    measurements[measurement_count - 1].battery_level = m.battery_level;
    measurements[measurement_count - 1].humidity = m.humidity;
    measurements[measurement_count - 1].light_level = m.light_level;
    measurements[measurement_count - 1].moisture_percent = m.moisture_percent;
    measurements[measurement_count - 1].temperature = m.temperature;
    measurements[measurement_count - 1].timestamp = m.timestamp;

    measurement_count++; // increment the number of readings taken so far

    return measurement_count == num_measurements; // returns true if measurement count is over
    // might need a consideration for the time
}

// prints all stored readings
void printReadings()
{
    int i = 0;
    for (struct Measurement m : measurements)
    {
        i++;
        if (i > measurement_count)
            break;

        printMeasurement(m);
    }
}

// clears all readings from memory (sets them to 0)
void clearReadings()
{
    memset(measurements, 0, sizeof(measurements));
    measurement_count = 1;
}