#include <Arduino.h>
#include "driver/pcnt.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ModbusRTU.h>

// Define ESP32 pins for A02YYUW sensor
#define A02YYUW_TX 25  // ESP32 TX → Sensor RX
#define A02YYUW_RX 26  // ESP32 RX ← Sensor TX

#define OUTPUT_PIN 33     // GPIO33 -> set HIGH
#define INPUT_PIN  14     // GPIO14 -> square wave input
#define PCNT_UNIT  PCNT_UNIT_0
#define PCNT_H_LIM 32767
#define PCNT_L_LIM -32768
#define GATE_TIME_MS 100   // measurement window in ms

#define SLAVE_ID 1
#define rx 17
#define tx 16

// Watchdog and reset configuration
#define WDT_TIMEOUT_SECONDS 30  // Watchdog timeout in seconds
#define RESET_INTERVAL_HOURS 1  // Reset ESP32 every 1 hour
#define MILLIS_PER_HOUR (3600UL * 1000UL)  // Milliseconds in an hour

// Modbus register definitions
#define REG_TEMPERATURE 0
#define REG_DISTANCE 1
#define REG_FREQUENCY_HI 2    // Frequency high 16 bits
#define REG_FREQUENCY_LO 3    // Frequency low 16 bits

ModbusRTU mb;

// Initialize hardware serial port (UART2)
HardwareSerial mySerial(1);

// GPIO where the DS18B20 is connected to
const int oneWireBus = 19;
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// RAW measurements (unfiltered)
int16_t pulse_count = 0;
uint32_t raw_frequency = 0;
float raw_temperatureC = 0;
uint16_t raw_distance = 0;

// FILTERED measurements (for Modbus)
uint32_t filtered_frequency = 0;
float filtered_temperatureC = 0;
uint16_t filtered_distance = 0;

// Watchdog timer variables
hw_timer_t *watchdogTimer = NULL;
volatile bool watchdogTriggered = false;
uint32_t lastResetTime = 0;

// EMA Filter class
class EMAFilter {
private:
    float filteredValue;
    float alpha;
    bool initialized;
    
public:
    EMAFilter(float alpha = 0.15) : alpha(alpha), initialized(false), filteredValue(0.0f) {}
    
    float update(float newValue) {
        if (!initialized) {
            filteredValue = newValue;
            initialized = true;
        } else {
            filteredValue = alpha * newValue + (1.0f - alpha) * filteredValue;
        }
        return filteredValue;
    }
    
    float getValue() const {
        return filteredValue;
    }
    
    void reset() {
        initialized = false;
        filteredValue = 0.0f;
    }
    
    bool isInitialized() const {
        return initialized;
    }
};

// Create EMA filters
EMAFilter freqFilter(0.15f);        // Alpha = 0.15 for frequency
EMAFilter tempFilter(0.1f);         // Alpha = 0.1 for temperature (slower response)
EMAFilter distanceFilter(0.2f);     // Alpha = 0.2 for distance

// Function to split 32-bit value into two 16-bit parts
void split32to16(uint32_t value, uint16_t &high, uint16_t &low) {
    high = (uint16_t)(value >> 16);  // High 16 bits
    low = (uint16_t)(value & 0xFFFF); // Low 16 bits
}

// Watchdog Timer ISR
void IRAM_ATTR watchdogTimerISR() {
    watchdogTriggered = true;
}

// Initialize watchdog timer
void setupWatchdog() {
    // Create and configure timer (Timer 0, 80 MHz / 80 = 1 MHz)
    watchdogTimer = timerBegin(0, 80, true);
    
    // Attach the ISR
    timerAttachInterrupt(watchdogTimer, &watchdogTimerISR, true);
    
    // Set alarm for timeout (in microseconds)
    timerAlarmWrite(watchdogTimer, WDT_TIMEOUT_SECONDS * 1000000, true);
    
    // Enable auto-reload and start
    timerAlarmEnable(watchdogTimer);
    
    Serial.print("Watchdog timer initialized with ");
    Serial.print(WDT_TIMEOUT_SECONDS);
    Serial.println(" second timeout");
}

// Feed the watchdog
void feedWatchdog() {
    if (watchdogTimer) {
        timerWrite(watchdogTimer, 0);  // Reset the timer
    }
}

// Check if hourly reset is needed
bool checkHourlyReset() {
    uint32_t currentTime = millis();
    
    // Handle millis() overflow (every ~50 days)
    if (currentTime < lastResetTime) {
        lastResetTime = currentTime;
        return false;
    }
    
    // Check if 1 hour has passed
    if ((currentTime - lastResetTime) >= MILLIS_PER_HOUR) {
        Serial.println("Hourly reset triggered");
        return true;
    }
    
    return false;
}

// Perform a controlled reset
void performReset() {
    Serial.println("Performing hourly system reset...");
    delay(100);  // Allow serial to flush
    ESP.restart();
}

// Declare task handles
TaskHandle_t CapacitiveHandle = NULL;
TaskHandle_t TemperatureHandle = NULL;
TaskHandle_t UltrasonicHandle = NULL;
TaskHandle_t ModbusHandle = NULL;

void setup_pcnt() {
    pcnt_config_t pcnt_config;

    // Assign each field explicitly (avoids "designator order" errors)
    pcnt_config.pulse_gpio_num = INPUT_PIN;
    pcnt_config.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
    pcnt_config.unit           = PCNT_UNIT;
    pcnt_config.channel        = PCNT_CHANNEL_0;
    pcnt_config.pos_mode       = PCNT_COUNT_INC;   // Count rising edges
    pcnt_config.neg_mode       = PCNT_COUNT_DIS;   // Ignore falling edges
    pcnt_config.lctrl_mode     = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;
    pcnt_config.counter_h_lim  = PCNT_H_LIM;
    pcnt_config.counter_l_lim  = PCNT_L_LIM;

    pcnt_unit_config(&pcnt_config);

    // Reset and start counter
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
}

void setup_mod() {
    Serial2.begin(9600, SERIAL_8N1, rx, tx);
#if defined(ESP32) || defined(ESP8266)
    mb.begin(&Serial2);
#else
    mb.begin(&Serial2);
    mb.setBaudrate(9600);
#endif
    mb.slave(SLAVE_ID);
    
    // Add Modbus registers - only frequency uses high/low
    mb.addHreg(REG_TEMPERATURE, 0);
    mb.addHreg(REG_DISTANCE, 0);
    mb.addHreg(REG_FREQUENCY_HI, 0);
    mb.addHreg(REG_FREQUENCY_LO, 0);
}

void Capacitive(void *parameter) {
    for (;;) {
        // Feed watchdog
        feedWatchdog();
        
        // Clear counter
        pcnt_counter_clear(PCNT_UNIT);

        // Wait gate time
        vTaskDelay(GATE_TIME_MS / portTICK_PERIOD_MS);
        
        // Read pulse count
        pcnt_get_counter_value(PCNT_UNIT, &pulse_count);

        // Convert to Hz (scale by gate time)
        // Use 32-bit calculation to prevent overflow
        raw_frequency = ((uint32_t)pulse_count * 1000) / GATE_TIME_MS;
        
        // Apply EMA filter to frequency
        filtered_frequency = (uint32_t)freqFilter.update((float)raw_frequency);

        // Print results (for debugging)
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 2000) {
            lastPrint = millis();
            Serial.print("Frequency - Raw: ");
            Serial.print(raw_frequency);
            Serial.print(" Hz, Filtered: ");
            Serial.print(filtered_frequency);
            Serial.println(" Hz");
        }

        // Check if watchdog was triggered
        if (watchdogTriggered) {
            Serial.println("Watchdog triggered! System will reset...");
            delay(1000);
            ESP.restart();
        }
    }
}

void Temperature(void *parameter) {
    for (;;) {
        // Feed watchdog
        feedWatchdog();
        
        sensors.requestTemperatures(); 
        raw_temperatureC = sensors.getTempCByIndex(0);
        
        // Apply EMA filter to temperature
        filtered_temperatureC = tempFilter.update(raw_temperatureC);
        
        // Print results (for debugging)
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 5000) {
            lastPrint = millis();
            Serial.print("Temperature - Raw: ");
            Serial.print(raw_temperatureC, 1);
            Serial.print("°C, Filtered: ");
            Serial.print(filtered_temperatureC, 1);
            Serial.println("°C");
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Read every 1 second
        
        // Check if watchdog was triggered
        if (watchdogTriggered) {
            Serial.println("Watchdog triggered! System will reset...");
            delay(1000);
            ESP.restart();
        }
    }
}

void Ultrasonic(void *parameter) {
    for (;;) {
        // Feed watchdog
        feedWatchdog();
        
        // Clear any old data
        while (mySerial.available()) {
            mySerial.read();
        }
        
        // Wait for data with timeout
        unsigned long start = millis();
        uint8_t data[4] = {0};
        int bytesRead = 0;
        
        while (millis() - start < 200 && bytesRead < 4) {
            if (mySerial.available()) {
                data[bytesRead] = mySerial.read();
                bytesRead++;
            }
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        
        if (bytesRead == 4) {
            if (data[0] == 0xFF) {
                uint8_t sum = (data[0] + data[1] + data[2]) & 0xFF;
                if (sum == data[3]) {
                    raw_distance = (data[1] << 8) | data[2];
                    
                    // Only use valid distances
                    if (raw_distance > 30 && raw_distance < 4500) {
                        // Apply EMA filter to distance
                        filtered_distance = (uint16_t)distanceFilter.update((float)raw_distance);
                        
                        // Print results (for debugging)
                        static unsigned long lastPrint = 0;
                        if (millis() - lastPrint > 3000) {
                            lastPrint = millis();
                            Serial.print("Distance - Raw: ");
                            Serial.print(raw_distance);
                            Serial.print(" mm, Filtered: ");
                            Serial.print(filtered_distance);
                            Serial.println(" mm");
                        }
                    }
                }
            }
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        // Check if watchdog was triggered
        if (watchdogTriggered) {
            Serial.println("Watchdog triggered! System will reset...");
            delay(1000);
            ESP.restart();
        }
    }
}

void Modbus(void *parameter) {
    for (;;) {
        // Feed watchdog
        feedWatchdog();
        
        // Update temperature (scaled to integer ×10 for 0.1°C resolution)
        mb.Hreg(REG_TEMPERATURE, (uint16_t)(filtered_temperatureC * 10.0f));
        
        // Update distance (use filtered distance)
        mb.Hreg(REG_DISTANCE, filtered_distance);
        
        // Split FILTERED 32-bit frequency into two 16-bit registers
        uint16_t freq_hi, freq_lo;
        split32to16(filtered_frequency, freq_hi, freq_lo);
        mb.Hreg(REG_FREQUENCY_HI, freq_hi);
        mb.Hreg(REG_FREQUENCY_LO, freq_lo);
        
        // Process Modbus requests
        mb.task();
        yield();
        
        // Check for hourly reset
        if (checkHourlyReset()) {
            performReset();
        }
        
        // Optional: Print Modbus values occasionally
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 10000) {
            lastPrint = millis();
            Serial.println("=== Modbus Registers ===");
            Serial.print("Temperature: ");
            Serial.print(filtered_temperatureC, 1);
            Serial.println("°C");
            Serial.print("Distance: ");
            Serial.print(filtered_distance);
            Serial.println(" mm");
            Serial.print("Frequency: ");
            Serial.print(filtered_frequency);
            Serial.println(" Hz");
            Serial.println("========================");
        }
        
        // Check if watchdog was triggered
        if (watchdogTriggered) {
            Serial.println("Watchdog triggered! System will reset...");
            delay(1000);
            ESP.restart();
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup() {
    // Start the Serial Monitor
    Serial.begin(115200);
    
    // Initialize ultrasonic sensor serial
    mySerial.begin(9600, SERIAL_8N1, A02YYUW_RX, A02YYUW_TX);
    
    // Start the DS18B20 sensor
    sensors.begin();
    
    // Set GPIO33 HIGH for capacitive sensor
    pinMode(OUTPUT_PIN, OUTPUT);
    digitalWrite(OUTPUT_PIN, HIGH);
    
    // Initialize PCNT
    setup_pcnt();
    
    // Initialize Modbus RTU
    setup_mod();
    
    // Initialize watchdog timer
    setupWatchdog();
    
    // Set initial reset time
    lastResetTime = millis();
    
    Serial.println("System Starting...");
    Serial.println("Sensors will stabilize in 5 seconds...");
    
    // Create tasks
    xTaskCreatePinnedToCore(
        Capacitive,
        "Capacitive",
        10000,
        NULL,
        1,
        &CapacitiveHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        Temperature,
        "Temperature",
        10000,
        NULL,
        1,
        &TemperatureHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        Ultrasonic,
        "Ultrasonic",
        10000,
        NULL,
        1,
        &UltrasonicHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        Modbus,
        "Modbus",
        10000,
        NULL,
        1,
        &ModbusHandle,
        0
    );
    
    // Let filters stabilize
    delay(5000);
    Serial.println("System Ready - Providing filtered sensor data via Modbus RTU");
    Serial.print("System will automatically reset every ");
    Serial.print(RESET_INTERVAL_HOURS);
    Serial.println(" hour(s)");
}

void loop() {
    // Feed watchdog as a backup
    feedWatchdog();
    
    // Check for watchdog trigger in main loop too
    if (watchdogTriggered) {
        Serial.println("Watchdog triggered in main loop! System will reset...");
        delay(1000);
        ESP.restart();
    }
    
    // Minimal delay
    delay(1000);
}





















/*#include <Arduino.h>
#include "driver/pcnt.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ModbusRTU.h>

// Define ESP32 pins for A02YYUW sensor
#define A02YYUW_TX 25  // ESP32 TX → Sensor RX
#define A02YYUW_RX 26  // ESP32 RX ← Sensor TX

#define OUTPUT_PIN 33     // GPIO33 -> set HIGH
#define INPUT_PIN  14     // GPIO14 -> square wave input
#define PCNT_UNIT  PCNT_UNIT_0
#define PCNT_H_LIM 32767
#define PCNT_L_LIM -32768
#define GATE_TIME_MS 100   // measurement window in ms

#define SLAVE_ID 1
#define rx 17
#define tx 16

// Modbus register definitions
#define REG_TEMPERATURE 0
#define REG_DISTANCE 1
#define REG_FREQUENCY_HI 2    // Frequency high 16 bits
#define REG_FREQUENCY_LO 3    // Frequency low 16 bits

ModbusRTU mb;

// Initialize hardware serial port (UART2)
HardwareSerial mySerial(1);

// GPIO where the DS18B20 is connected to
const int oneWireBus = 19;
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// RAW measurements (unfiltered)
int16_t pulse_count = 0;
uint32_t raw_frequency = 0;
float raw_temperatureC = 0;
uint16_t raw_distance = 0;

// FILTERED measurements (for Modbus)
uint32_t filtered_frequency = 0;
float filtered_temperatureC = 0;
uint16_t filtered_distance = 0;

// EMA Filter class
class EMAFilter {
private:
    float filteredValue;
    float alpha;
    bool initialized;
    
public:
    EMAFilter(float alpha = 0.15) : alpha(alpha), initialized(false), filteredValue(0.0f) {}
    
    float update(float newValue) {
        if (!initialized) {
            filteredValue = newValue;
            initialized = true;
        } else {
            filteredValue = alpha * newValue + (1.0f - alpha) * filteredValue;
        }
        return filteredValue;
    }
    
    float getValue() const {
        return filteredValue;
    }
    
    void reset() {
        initialized = false;
        filteredValue = 0.0f;
    }
    
    bool isInitialized() const {
        return initialized;
    }
};

// Create EMA filters
EMAFilter freqFilter(0.15f);        // Alpha = 0.15 for frequency
EMAFilter tempFilter(0.1f);         // Alpha = 0.1 for temperature (slower response)
EMAFilter distanceFilter(0.2f);     // Alpha = 0.2 for distance

// Function to split 32-bit value into two 16-bit parts
void split32to16(uint32_t value, uint16_t &high, uint16_t &low) {
    high = (uint16_t)(value >> 16);  // High 16 bits
    low = (uint16_t)(value & 0xFFFF); // Low 16 bits
}

// Declare task handles
TaskHandle_t CapacitiveHandle = NULL;
TaskHandle_t TemperatureHandle = NULL;
TaskHandle_t UltrasonicHandle = NULL;
TaskHandle_t ModbusHandle = NULL;

void setup_pcnt() {
    pcnt_config_t pcnt_config;

    // Assign each field explicitly (avoids "designator order" errors)
    pcnt_config.pulse_gpio_num = INPUT_PIN;
    pcnt_config.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
    pcnt_config.unit           = PCNT_UNIT;
    pcnt_config.channel        = PCNT_CHANNEL_0;
    pcnt_config.pos_mode       = PCNT_COUNT_INC;   // Count rising edges
    pcnt_config.neg_mode       = PCNT_COUNT_DIS;   // Ignore falling edges
    pcnt_config.lctrl_mode     = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;
    pcnt_config.counter_h_lim  = PCNT_H_LIM;
    pcnt_config.counter_l_lim  = PCNT_L_LIM;

    pcnt_unit_config(&pcnt_config);

    // Reset and start counter
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
}

void setup_mod() {
    Serial2.begin(9600, SERIAL_8N1, rx, tx);
#if defined(ESP32) || defined(ESP8266)
    mb.begin(&Serial2);
#else
    mb.begin(&Serial2);
    mb.setBaudrate(9600);
#endif
    mb.slave(SLAVE_ID);
    
    // Add Modbus registers - only frequency uses high/low
    mb.addHreg(REG_TEMPERATURE, 0);
    mb.addHreg(REG_DISTANCE, 0);
    mb.addHreg(REG_FREQUENCY_HI, 0);
    mb.addHreg(REG_FREQUENCY_LO, 0);
}

void Capacitive(void *parameter) {
    for (;;) {
        // Clear counter
        pcnt_counter_clear(PCNT_UNIT);

        // Wait gate time
        vTaskDelay(GATE_TIME_MS / portTICK_PERIOD_MS);
        
        // Read pulse count
        pcnt_get_counter_value(PCNT_UNIT, &pulse_count);

        // Convert to Hz (scale by gate time)
        // Use 32-bit calculation to prevent overflow
        raw_frequency = ((uint32_t)pulse_count * 1000) / GATE_TIME_MS;
        
        // Apply EMA filter to frequency
        filtered_frequency = (uint32_t)freqFilter.update((float)raw_frequency);

        // Print results (for debugging)
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 2000) {
            lastPrint = millis();
            Serial.print("Frequency - Raw: ");
            Serial.print(raw_frequency);
            Serial.print(" Hz, Filtered: ");
            Serial.print(filtered_frequency);
            Serial.println(" Hz");
        }

        // Serial.print("Capacitive running on core ");
        // Serial.println(xPortGetCoreID());
    }
}

void Temperature(void *parameter) {
    for (;;) {
        sensors.requestTemperatures(); 
        raw_temperatureC = sensors.getTempCByIndex(0);
        
        // Apply EMA filter to temperature
        filtered_temperatureC = tempFilter.update(raw_temperatureC);
        
        // Print results (for debugging)
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 5000) {
            lastPrint = millis();
            Serial.print("Temperature - Raw: ");
            Serial.print(raw_temperatureC, 1);
            Serial.print("°C, Filtered: ");
            Serial.print(filtered_temperatureC, 1);
            Serial.println("°C");
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Read every 1 second
    }
}

void Ultrasonic(void *parameter) {
    for (;;) {
        // Clear any old data
        while (mySerial.available()) {
            mySerial.read();
        }
        
        // Wait for data with timeout
        unsigned long start = millis();
        uint8_t data[4] = {0};
        int bytesRead = 0;
        
        while (millis() - start < 200 && bytesRead < 4) {
            if (mySerial.available()) {
                data[bytesRead] = mySerial.read();
                bytesRead++;
            }
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        
        if (bytesRead == 4) {
            if (data[0] == 0xFF) {
                uint8_t sum = (data[0] + data[1] + data[2]) & 0xFF;
                if (sum == data[3]) {
                    raw_distance = (data[1] << 8) | data[2];
                    
                    // Only use valid distances
                    if (raw_distance > 30 && raw_distance < 4500) {
                        // Apply EMA filter to distance
                        filtered_distance = (uint16_t)distanceFilter.update((float)raw_distance);
                        
                        // Print results (for debugging)
                        static unsigned long lastPrint = 0;
                        if (millis() - lastPrint > 3000) {
                            lastPrint = millis();
                            Serial.print("Distance - Raw: ");
                            Serial.print(raw_distance);
                            Serial.print(" mm, Filtered: ");
                            Serial.print(filtered_distance);
                            Serial.println(" mm");
                        }
                    }
                }
            }
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void Modbus(void *parameter) {
    for (;;) {
        // Update temperature (scaled to integer ×10 for 0.1°C resolution)
        mb.Hreg(REG_TEMPERATURE, (uint16_t)(filtered_temperatureC * 10.0f));
        
        // Update distance (use filtered distance)
        mb.Hreg(REG_DISTANCE, filtered_distance);
        
        // Split FILTERED 32-bit frequency into two 16-bit registers
        uint16_t freq_hi, freq_lo;
        split32to16(filtered_frequency, freq_hi, freq_lo);
        mb.Hreg(REG_FREQUENCY_HI, freq_hi);
        mb.Hreg(REG_FREQUENCY_LO, freq_lo);
        
        // Process Modbus requests
        mb.task();
        yield();
        
        // Optional: Print Modbus values occasionally
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 10000) {
            lastPrint = millis();
            Serial.println("=== Modbus Registers ===");
            Serial.print("Temperature: ");
            Serial.print(filtered_temperatureC, 1);
            Serial.println("°C");
            Serial.print("Distance: ");
            Serial.print(filtered_distance);
            Serial.println(" mm");
            Serial.print("Frequency: ");
            Serial.print(filtered_frequency);
            Serial.println(" Hz");
            Serial.println("========================");
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup() {
    // Start the Serial Monitor
    Serial.begin(115200);
    
    // Initialize ultrasonic sensor serial
    mySerial.begin(9600, SERIAL_8N1, A02YYUW_RX, A02YYUW_TX);
    
    // Start the DS18B20 sensor
    sensors.begin();
    
    // Set GPIO33 HIGH for capacitive sensor
    pinMode(OUTPUT_PIN, OUTPUT);
    digitalWrite(OUTPUT_PIN, HIGH);
    
    // Initialize PCNT
    setup_pcnt();
    
    // Initialize Modbus RTU
    setup_mod();
    
    Serial.println("System Starting...");
    Serial.println("Sensors will stabilize in 5 seconds...");
    
    // Create tasks
    xTaskCreatePinnedToCore(
        Capacitive,
        "Capacitive",
        10000,
        NULL,
        1,
        &CapacitiveHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        Temperature,
        "Temperature",
        10000,
        NULL,
        1,
        &TemperatureHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        Ultrasonic,
        "Ultrasonic",
        10000,
        NULL,
        1,
        &UltrasonicHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        Modbus,
        "Modbus",
        10000,
        NULL,
        1,
        &ModbusHandle,
        0
    );
    
    // Let filters stabilize
    delay(5000);
    Serial.println("System Ready - Providing filtered sensor data via Modbus RTU");
}

void loop() {
    // Empty loop - everything is handled by FreeRTOS tasks
}

*/




















/*#include <Arduino.h>
#include "driver/pcnt.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ModbusRTU.h>

// Define ESP32 pins for A02YYUW sensor
#define A02YYUW_TX 25  // ESP32 TX → Sensor RX
#define A02YYUW_RX 26  // ESP32 RX ← Sensor TX

#define OUTPUT_PIN 33     // GPIO33 -> set HIGH
#define INPUT_PIN  14     // GPIO14 -> square wave input
#define PCNT_UNIT  PCNT_UNIT_0
#define PCNT_H_LIM 32767
#define PCNT_L_LIM -32768
#define GATE_TIME_MS 100   // measurement window in ms

#define SLAVE_ID 1
#define rx 17
#define tx 16

// Modbus register definitions - using your original register numbers
#define REG_TEMPERATURE 0
#define REG_DISTANCE 1
#define REG_FREQUENCY_HI 2    // Frequency high 16 bits
#define REG_FREQUENCY_LO 3    // Frequency low 16 bits

ModbusRTU mb;

// Initialize hardware serial port (UART2)
HardwareSerial mySerial(1);

// GPIO where the DS18B20 is connected to
const int oneWireBus = 19;
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

int16_t pulse_count = 0;
uint32_t frequency = 0;        // Changed to 32-bit to handle larger values
float temperatureC = 0;
uint16_t distance = 0;

// Function to split 32-bit value into two 16-bit parts
void split32to16(uint32_t value, uint16_t &high, uint16_t &low) {
    high = (uint16_t)(value >> 16);  // High 16 bits
    low = (uint16_t)(value & 0xFFFF); // Low 16 bits
}

// Declare task handle
TaskHandle_t CapacitiveHandle = NULL;
TaskHandle_t TemperatureHandle = NULL;
TaskHandle_t UltrasonicHandle = NULL;
TaskHandle_t ModbusHandle = NULL;

void setup_pcnt() {
    pcnt_config_t pcnt_config;

    // Assign each field explicitly (avoids "designator order" errors)
    pcnt_config.pulse_gpio_num = INPUT_PIN;
    pcnt_config.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
    pcnt_config.unit           = PCNT_UNIT;
    pcnt_config.channel        = PCNT_CHANNEL_0;
    pcnt_config.pos_mode       = PCNT_COUNT_INC;   // Count rising edges
    pcnt_config.neg_mode       = PCNT_COUNT_DIS;   // Ignore falling edges
    pcnt_config.lctrl_mode     = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;
    pcnt_config.counter_h_lim  = PCNT_H_LIM;
    pcnt_config.counter_l_lim  = PCNT_L_LIM;

    pcnt_unit_config(&pcnt_config);

    // Reset and start counter
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
}

void setup_mod() {
    Serial2.begin(9600, SERIAL_8N1, rx, tx);
#if defined(ESP32) || defined(ESP8266)
    mb.begin(&Serial2);
#else
    mb.begin(&Serial2);
    mb.setBaudrate(9600);
#endif
    mb.slave(SLAVE_ID);
    
    // Add Modbus registers - only frequency uses high/low
    mb.addHreg(REG_TEMPERATURE, 0);
    mb.addHreg(REG_DISTANCE, 0);
    mb.addHreg(REG_FREQUENCY_HI, 0);
    mb.addHreg(REG_FREQUENCY_LO, 0);
}

void Capacitive(void *parameter) {
    for (;;) { // Infinite loop
        // Clear counter
        pcnt_counter_clear(PCNT_UNIT);

        // Wait gate time
        vTaskDelay(GATE_TIME_MS / portTICK_PERIOD_MS);
        
        // Read pulse count
        pcnt_get_counter_value(PCNT_UNIT, &pulse_count);

        // Convert to Hz (scale by gate time)
        // Use 32-bit calculation to prevent overflow
        frequency = ((uint32_t)pulse_count * 1000) / GATE_TIME_MS;

        // Print result
        Serial.print("Frequency: ");
        Serial.print(frequency);
        Serial.println(" Hz");

        Serial.print("Capacitive running on core ");
        Serial.println(xPortGetCoreID());
        Serial.printf("Capacitive Stack Free: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
    }
}

void Temperature(void *parameter) {
    for (;;) { // Infinite loop
        sensors.requestTemperatures(); 
        temperatureC = sensors.getTempCByIndex(0);
        Serial.print(temperatureC);
        Serial.println("ºC");
        
        vTaskDelay(500 / portTICK_PERIOD_MS); // 500ms
        Serial.print("Temperature running on core ");
        Serial.println(xPortGetCoreID());
        Serial.printf("Temperature Stack Free: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
    }
}

void Ultrasonic(void *parameter) {
    for (;;) { // Infinite loop
        while (mySerial.available() >= 4) {
            if (mySerial.read() == 0xFF) {  // Look for start byte
                uint8_t high = mySerial.read();
                uint8_t low  = mySerial.read();
                uint8_t sum  = mySerial.read();

                // Validate checksum
                if (((0xFF + high + low) & 0xFF) == sum) {
                    distance = (high << 8) | low;  // Distance in mm

                    if (distance > 30) {  // Ignore below sensor limit
                        Serial.print("Distance: ");
                        Serial.print(distance / 10.0); // Convert mm → cm
                        Serial.println(" cm");
                    } else {
                        Serial.println("Below the lower limit");
                    }
                } else {
                    Serial.println("Checksum error");
                }
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        Serial.print("Ultrasonic running on core ");
        Serial.println(xPortGetCoreID());
        Serial.printf("Ultrasonic Stack Free: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
    }
}

void Modbus(void *parameter) {
    for (;;) { // Infinite loop
        // Update temperature (scaled to integer, e.g., multiplied by 10 for 0.1°C resolution)
        mb.Hreg(REG_TEMPERATURE, (uint16_t)(temperatureC * 10));
        
        // Update distance (already 16-bit)
        mb.Hreg(REG_DISTANCE, distance);
        
        // Split 32-bit frequency into two 16-bit registers
        uint16_t freq_hi, freq_lo;
        split32to16(frequency, freq_hi, freq_lo);
        mb.Hreg(REG_FREQUENCY_HI, freq_hi);
        mb.Hreg(REG_FREQUENCY_LO, freq_lo);
        
        mb.task();
        yield();
        
        // Serial.print("Modbus running on core ");
        // Serial.println(xPortGetCoreID());
        // Serial.printf("Modbus Stack Free: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
        
        // Delay to prevent too frequent updates
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup() {
    // Start the Serial Monitor
    Serial.begin(115200);
    mySerial.begin(9600, SERIAL_8N1, A02YYUW_RX, A02YYUW_TX);

    // Start the DS18B20 sensor
    sensors.begin();

    // Set GPIO33 HIGH
    pinMode(OUTPUT_PIN, OUTPUT);
    digitalWrite(OUTPUT_PIN, HIGH);

    // Initialize PCNT
    setup_pcnt();

    // Initialize modbus RTU
    setup_mod();

    // Create tasks
    xTaskCreatePinnedToCore(
        Capacitive,
        "Capacitive",
        10000,
        NULL,
        1,
        &CapacitiveHandle,
        0
    );

    xTaskCreatePinnedToCore(
        Temperature,
        "Temperature",
        10000,
        NULL,
        1,
        &TemperatureHandle,
        0
    );

    xTaskCreatePinnedToCore(
        Ultrasonic,
        "Ultrasonic",
        10000,
        NULL,
        1,
        &UltrasonicHandle,
        0
    );

    xTaskCreatePinnedToCore(
        Modbus,
        "Modbus",
        10000,
        NULL,
        1,
        &ModbusHandle,
        0
    );
}

void loop() {
    // Empty loop - everything is handled by FreeRTOS tasks
}*/