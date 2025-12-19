#include <ModbusRTU.h>

// Modbus RTU Configuration
ModbusRTU mbRTU;
HardwareSerial modbusSerial(2);  // Use Serial2 for Modbus communication

// Slave configuration
#define MODBUS_SLAVE_ID 1
#define MODBUS_POLL_INTERVAL 500  // Poll every 500ms
#define MODBUS_TIMEOUT_MS 10000   // 10 seconds timeout
#define ERROR_DISPLAY_DURATION 30000  // Show error for 30 seconds

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
unsigned long lastModbusSuccess = 0;
String lastModbusError = "";

// Error display management
bool modbusCriticalError = false;
unsigned long modbusErrorStartTime = 0;

// Global Modbus data
float modbusTemperature = 0.0f;
uint32_t modbusDistance = 0;
uint32_t modbusFrequency = 0;
bool modbusDataValid = false;

// Helper function to convert error codes to strings
String getSimpleModbusError(uint8_t event) {
    if (event == 0x00) return "Success";
    if (event == 0xE0 || event == 0xFF) return "Timeout";
    if (event == 0xE2) return "CRC Error";
    if (event == 0x01) return "Illegal Function";
    if (event == 0x02) return "Illegal Address";
    if (event == 0x03) return "Illegal Data";
    if (event == 0x04) return "Slave Failure";
    return "Error Code: " + String(event);
}

// Callback function for Modbus response
bool cbModbusSlave(Modbus::ResultCode event, uint16_t transactionId, void* data) {
    uint8_t errorCode = (uint8_t)event;
    
    if (errorCode != 0x00) {  // Not success
        lastModbusError = getSimpleModbusError(errorCode);
        modbusRetryCount++;
        modbusCriticalError = true;
        modbusErrorStartTime = millis();
        modbusDataValid = false;
        
        Serial.print("🚨 MODBUS ERROR: ");
        Serial.print(lastModbusError);
        Serial.print(" (Code: 0x");
        Serial.print(errorCode, HEX);
        Serial.print(", Retry: ");
        Serial.print(modbusRetryCount);
        Serial.println(")");
        
        if (modbusRetryCount >= MAX_MODBUS_RETRIES) {
            modbusSlaveAvailable = false;
            Serial.println("🔴 Modbus Slave marked as unavailable");
        }
        return true;
    }
    
    // Success
    lastModbusSuccess = millis();
    lastModbusError = "";
    modbusRetryCount = 0;
    modbusSlaveAvailable = true;
    
    if (modbusCriticalError) {
        modbusCriticalError = false;
        Serial.println("✅ Modbus recovered from error");
    }
    
    // Process the data
    modbusTemperature = sensorVals[REG_TEMPERATURE] / 10.0f;
    modbusDistance = sensorVals[REG_DISTANCE];
    modbusFrequency = ((uint32_t)sensorVals[REG_FREQUENCY_HI] << 16) | sensorVals[REG_FREQUENCY_LO];
    modbusDataValid = true;
    
    // Debug output (less frequent)
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 5000) {
        lastDebug = millis();
        Serial.print("[Modbus] ");
        Serial.print("Temp: ");
        Serial.print(modbusTemperature, 1);
        Serial.print("°C | Dist: ");
        Serial.print(modbusDistance);
        Serial.print(" mm | Freq: ");
        Serial.print(modbusFrequency);
        Serial.println(" Hz");
    }
    
    return true;
}

// Check if Modbus has timed out
bool isModbusTimedOut() {
    if (lastModbusSuccess > 0) {
        bool timedOut = (millis() - lastModbusSuccess > MODBUS_TIMEOUT_MS);
        if (timedOut && !modbusCriticalError) {
            modbusCriticalError = true;
            modbusErrorStartTime = millis();
            lastModbusError = "Timeout";
            modbusDataValid = false;
            Serial.println("🚨 Modbus timeout - critical error");
        }
        return timedOut;
    }
    return false;
}

// Check if we should display Modbus error
// bool shouldDisplayModbusError() {
//     if (!modbusCriticalError) return false;
    
//     // Check if error display duration has expired
//     if (millis() - modbusErrorStartTime > ERROR_DISPLAY_DURATION) {
//         // Auto-clear after duration
//         modbusCriticalError = false;
//         Serial.println("⚠️ Modbus error auto-cleared after timeout");
//         return false;
//     }
    
//     return true;
// }

// Add this function to control when errors are displayed
bool shouldDisplayModbusError() {
  if (!modbusCriticalError) return false;
  
  // Only display error if we've been in error state for at least 2 seconds
  // This prevents brief glitches from taking over the display
  if (millis() - modbusErrorStartTime < 2000) {
    return false;
  }
  
  // Check if error display duration has expired
  if (millis() - modbusErrorStartTime > ERROR_DISPLAY_DURATION) {
    // Auto-clear after duration
    modbusCriticalError = false;
    Serial.println("⚠️ Modbus error auto-cleared after timeout");
    return false;
  }
  
  // Check if we've had any successful communication recently
  // If we have, don't show error (recovery in progress)
  if (millis() - lastModbusSuccess < 5000) {
    modbusCriticalError = false;
    return false;
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
    lastModbusSuccess = 0;
    lastModbusError = "";
    modbusCriticalError = false;
    modbusErrorStartTime = 0;
    
    Serial.println("✅ Modbus RTU Master Initialized");
}

// Update Modbus data
void updateModbusData() {
    // Process Modbus tasks
    mbRTU.task();
    
    // Check for timeout
    if (modbusSlaveAvailable && isModbusTimedOut()) {
        lastModbusError = "Timeout";
        modbusSlaveAvailable = false;
        modbusDataValid = false;
    }
    
    // Poll at regular intervals
    if (millis() - lastModbusPoll >= MODBUS_POLL_INTERVAL) {
        lastModbusPoll = millis();
        
        // Try to reconnect if disconnected
        if (!modbusSlaveAvailable && (millis() - lastModbusReconnectTry > 10000)) {
            lastModbusReconnectTry = millis();
            Serial.println("🔄 Trying to reconnect Modbus Slave...");
            modbusSlaveAvailable = true;
            modbusRetryCount = 0;
        }
        
        // Send request if slave is available
        if (modbusSlaveAvailable) {
            // Read all 4 registers starting from REG_TEMPERATURE
            bool success = mbRTU.readHreg(MODBUS_SLAVE_ID, REG_TEMPERATURE, sensorVals, NUM_MODBUS_REGS, cbModbusSlave);
            
            if (!success) {
                lastModbusError = "Send Failed";
                modbusDataValid = false;
            }
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
    return modbusDataValid && modbusSlaveAvailable && !isModbusTimedOut();
}

// Get Modbus connection status
bool isModbusConnected() {
    return modbusSlaveAvailable && !isModbusTimedOut();
}

// Get last Modbus error message
String getLastModbusError() {
    return lastModbusError;
}
















/*#include <ModbusRTU.h>
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
*/