#include <ModbusRTU.h>
//#include "definitions.h"

// Modbus RTU Configuration
ModbusRTU mbRTU;
HardwareSerial modbusSerial(2);  // Use Serial2 for Modbus communication

// Slave configuration
#define MODBUS_SLAVE_ID 1
#define MODBUS_POLL_INTERVAL 500  // Poll every 500ms

// Register addresses (match with slave)
#define REG_TEMPERATURE    0
#define REG_DISTANCE       1
#define REG_FREQUENCY_HI   2
#define REG_FREQUENCY_LO   3
#define NUM_MODBUS_REGS    4

// Modbus data storage
uint16_t sensorVals[NUM_MODBUS_REGS];
unsigned long lastModbusPoll = 0;
bool modbusSlaveAvailable = true;
uint8_t modbusRetryCount = 0;
const uint8_t MAX_MODBUS_RETRIES = 3;
unsigned long lastModbusReconnectTry = 0;

// Global Modbus data
float modbusTemperature = 0.0f;
uint32_t modbusDistance = 0;
uint32_t modbusFrequency = 0;
bool modbusDataValid = false;

// Callback function for Modbus response
bool cbModbusSlave(Modbus::ResultCode event, uint16_t transactionId, void* data) {
    if (event != Modbus::EX_SUCCESS) {
        Serial.print("Modbus Slave ");
        Serial.print(MODBUS_SLAVE_ID);
        Serial.print(" read failed, code: ");
        Serial.println(event);
        
        // Update retry count and availability
        modbusRetryCount++;
        if (modbusRetryCount >= MAX_MODBUS_RETRIES) {
            modbusSlaveAvailable = false;
            Serial.println("Modbus Slave disconnected");
            modbusDataValid = false;
        }
        return true;
    }
    
    // Reset retry count on success
    modbusRetryCount = 0;
    modbusSlaveAvailable = true;
    
    // Process the data
    // Temperature is stored as integer × 10 (for 0.1°C resolution)
    modbusTemperature = sensorVals[REG_TEMPERATURE] / 10.0f;
    
    // Distance is in mm from ultrasonic sensor
    modbusDistance = sensorVals[REG_DISTANCE];
    
    // Frequency: combine high and low 16-bit registers into 32-bit value
    modbusFrequency = ((uint32_t)sensorVals[REG_FREQUENCY_HI] << 16) | sensorVals[REG_FREQUENCY_LO];
    
    // Mark data as valid
    modbusDataValid = true;
    
    // Debug output
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 5000) {
        lastDebug = millis();
        Serial.print("[Modbus Slave ");
        Serial.print(MODBUS_SLAVE_ID);
        Serial.print("] ");
        Serial.print("Temperature: ");
        Serial.print(modbusTemperature, 1);
        Serial.print("°C | Distance: ");
        Serial.print(modbusDistance / 10.0f, 1);
        Serial.print(" cm | Frequency: ");
        Serial.print(modbusFrequency);
        Serial.println(" Hz");
    }
    
    return true;
}

// Initialize Modbus RTU
void setupModbusRTU() {
    // Initialize Serial2 for Modbus communication
    modbusSerial.begin(9600, SERIAL_8N1, 17, 16); // RX=17, TX=16
    
    // Initialize Modbus RTU
    mbRTU.begin(&modbusSerial);
    mbRTU.master();
    
    // Initialize variables
    modbusTemperature = 0.0f;
    modbusDistance = 0;
    modbusFrequency = 0;
    modbusDataValid = false;
    
    Serial.println("✅ Modbus RTU Master Initialized");
    Serial.println("Register Map:");
    Serial.println("  0: Temperature (×10)");
    Serial.println("  1: Distance (mm)");
    Serial.println("  2: Frequency High 16 bits");
    Serial.println("  3: Frequency Low 16 bits");
    Serial.println("--------------------------------------------");
}

// Update Modbus data
void updateModbusData() {
    // Process Modbus tasks
    mbRTU.task();
    
    // Poll at regular intervals
    if (millis() - lastModbusPoll >= MODBUS_POLL_INTERVAL) {
        lastModbusPoll = millis();
        
        // Try to reconnect if disconnected
        if (!modbusSlaveAvailable && (millis() - lastModbusReconnectTry > 10000)) {
            lastModbusReconnectTry = millis();
            Serial.println("Trying to reconnect Modbus Slave...");
            modbusSlaveAvailable = true;
            modbusRetryCount = 0;
        }
        
        // Send request if slave is available
        if (modbusSlaveAvailable) {
            // Read all 4 registers starting from REG_TEMPERATURE
            bool success = mbRTU.readHreg(MODBUS_SLAVE_ID, REG_TEMPERATURE, sensorVals, NUM_MODBUS_REGS, cbModbusSlave);
            
            if (!success) {
                Serial.println("Failed to send Modbus request to slave");
                modbusDataValid = false;
            }
        } else {
            Serial.println("Modbus Slave unavailable - skipping poll");
            modbusDataValid = false;
        }
    }
}

// Get temperature from Modbus
float getModbusTemperature() {
    return modbusTemperature;
}

// Get distance from Modbus (in mm)
uint32_t getModbusDistance() {
    return modbusDistance;
}

// Get frequency from Modbus (in Hz)
uint32_t getModbusFrequency() {
    return modbusFrequency;
}

// Check if Modbus data is valid
bool isModbusDataValid() {
    return modbusDataValid && modbusSlaveAvailable;
}

// Get Modbus connection status
bool isModbusConnected() {
    return modbusSlaveAvailable;
}