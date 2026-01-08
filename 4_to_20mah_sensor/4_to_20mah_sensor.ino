
/*
 * ESP32 Fuel Tank Level Monitor with 4-20mA Sensor + Modbus TCP + LCD Display
 * Using 120Ω resistor with hardcoded calibration
 */

// ===================== INCLUDES & DEFINITIONS =====================
#include <WiFi.h>
#include <WiFiManager.h>
#include <ModbusIP_ESP8266.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>

// LCD Configuration
#define LCD_I2C_ADDR 0x27
#define LCD_COLUMNS 20
#define LCD_ROWS 4
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLUMNS, LCD_ROWS);

// WiFi Manager
Preferences preferences;
#define CONFIG_AP_SSID "FuelTankMonitor"
#define CONFIG_AP_PASS "12345678"
#define CONFIG_TIMEOUT 30

// Sensor Configuration
#define ADC_PIN          34      // GPIO34 is ADC input for 4-20mA
#define RESISTOR_VALUE   120.0   // 120Ω resistor
#define VREF             3.3     // ESP32 reference voltage
#define ADC_RESOLUTION   4095    // 12-bit ADC
#define SAMPLE_COUNT     100     // Samples for averaging
#define READ_INTERVAL    2000    // Read every 2 seconds

// HARDCODED TANK CALIBRATION (ADJUST THESE VALUES FOR YOUR TANK)
// -------------------------------------------------------------
#define TANK_HEIGHT_MM    2000.0   // Tank height in mm
#define TANK_LENGTH_MM    1500.0   // Tank length in mm  
#define TANK_WIDTH_MM     1000.0   // Tank width in mm
#define TANK_CAPACITY_L   3000.0   // Tank capacity in liters

// 4-20mA Sensor Calibration (ADJUST THESE AFTER INSTALLATION)
#define SENSOR_MIN_MA     4.0      // 4mA = Empty tank (0mm)
#define SENSOR_MAX_MA     20.0     // 20mA = Full tank (TANK_HEIGHT_MM)
#define CALIBRATION_OFFSET 0.0     // Offset adjustment if needed
#define CALIBRATION_GAIN   1.0     // Gain adjustment if needed

// Valve Control
#define VALVE_PIN         19
bool valveOpen = false;
bool previousValveState = false;
#define VALVE_CLOSE_LEVEL 80       // Close valve at 80% fill
#define VALVE_OPEN_LEVEL  50       // Open valve at 50% fill

// Modbus TCP Configuration
ModbusIP mb;
const int REG_BASE = 0;
const int REG_COUNT = 3;

// Network Configuration
#ifdef USE_DHCP
  // DHCP Mode
#else
  // Static IP Mode (Default)
  #define DEFAULT_LOCAL_IP    IPAddress(192, 168, 1, 76)
  #define DEFAULT_GATEWAY     IPAddress(192, 168, 1, 1)
  #define DEFAULT_SUBNET      IPAddress(255, 255, 255, 0)
#endif

// Global Variables
float currentMA = 0;
float fuelHeightMM = 0;
float fuelVolumeL = 0;
float fuelPercent = 0;
String ipAddress = "0.0.0.0";
bool wifiConnected = false;
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 2000;

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("FUEL TANK LEVEL MONITOR - 4-20mA Sensor");
  Serial.println("With Modbus TCP & LCD Display");
  Serial.println("========================================\n");
  
  // Initialize LCD
  lcdInit();
  displayMessage("System Starting", "Initializing...");
  
  // Configure ADC for 4-20mA sensor
  setupSensor();
  
  // Initialize valve control
  initValveControl();
  
  // Setup WiFi Configuration Portal
  setupWiFiConfig();
  
  // Connect to WiFi
  connectToWiFi();
  
  // Setup Modbus TCP
  setupModbusTCP();
  
  // Display tank info
  displayTankInfo();
  
  delay(2000);
  displayMessage("System Ready", "Monitoring...");
}

// ===================== MAIN LOOP =====================
void loop() {
  // 1. Read sensor and calculate values
  readSensorData();
  
  // 2. Update LCD display periodically
  updateDisplay();
  
  // 3. Update Modbus TCP registers
  updateModbusRegisters();
  
  // 4. Handle valve control
  controlValve();
  
  // 5. Handle Modbus TCP requests
  mb.task();
  
  delay(100);
}

// ===================== SENSOR FUNCTIONS =====================
void setupSensor() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(ADC_PIN, INPUT);
  
  Serial.println("✅ 4-20mA Sensor initialized");
  Serial.print("Tank Height: ");
  Serial.print(TANK_HEIGHT_MM);
  Serial.println(" mm");
  Serial.print("Tank Capacity: ");
  Serial.print(TANK_CAPACITY_L);
  Serial.println(" L");
}

int readRawADC() {
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += analogRead(ADC_PIN);
    delayMicroseconds(50);
  }
  return sum / SAMPLE_COUNT;
}

float readCurrentMA() {
  int rawADC = readRawADC();
  float voltage = (rawADC * VREF) / ADC_RESOLUTION;
  float current = (voltage / RESISTOR_VALUE) * 1000.0;  // Convert to mA
  
  // Apply calibration
  current = (current + CALIBRATION_OFFSET) * CALIBRATION_GAIN;
  
  // Ensure within reasonable 4-20mA range
  current = constrain(current, 0.0, 25.0);
  
  return current;
}

void readSensorData() {
  static unsigned long lastRead = 0;
  
  if (millis() - lastRead >= READ_INTERVAL) {
    lastRead = millis();
    
    // Read current from 4-20mA sensor
    currentMA = readCurrentMA();
    
    // Calculate fuel height (linear conversion from mA to mm)
    if (currentMA <= SENSOR_MIN_MA) {
      fuelHeightMM = 0;
    } else if (currentMA >= SENSOR_MAX_MA) {
      fuelHeightMM = TANK_HEIGHT_MM;
    } else {
      // Linear interpolation: mA to mm
      fuelHeightMM = ((currentMA - SENSOR_MIN_MA) * TANK_HEIGHT_MM) / 
                     (SENSOR_MAX_MA - SENSOR_MIN_MA);
    }
    
    // Calculate volume (rectangular tank)
    fuelVolumeL = (fuelHeightMM * TANK_LENGTH_MM * TANK_WIDTH_MM) / 1000000.0;
    
    // Calculate percentage
    fuelPercent = (fuelHeightMM / TANK_HEIGHT_MM) * 100.0;
    
    // Debug output
    Serial.printf("Current: %.2f mA | Height: %.1f mm | Volume: %.1f L | Fill: %.1f%%\n",
                  currentMA, fuelHeightMM, fuelVolumeL, fuelPercent);
  }
}

// ===================== LCD DISPLAY FUNCTIONS =====================
void lcdInit() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

void displayMessage(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

// Clear a specific line on LCD
void clearLCDLine(int line) {
  lcd.setCursor(0, line);
  for (int i = 0; i < LCD_COLUMNS; i++) {
    lcd.print(" ");
  }
}

// Clear a specific area on LCD
void clearLCDArea(int line, int startCol, int endCol) {
  for (int i = startCol; i <= endCol; i++) {
    lcd.setCursor(i, line);
    lcd.print(" ");
  }
}

void updateDisplay() {
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = millis();
    
    // Clear all lines before updating
    clearLCDLine(0);
    clearLCDLine(1);
    clearLCDLine(2);
    clearLCDLine(3);
    
    // Line 0: Level (left) + Valve (right)
    lcd.setCursor(0, 0);
    lcd.print("Lev :");
    lcd.setCursor(6, 0);
    
    if (fuelHeightMM >= 0) {
      lcd.print(fuelHeightMM / 1000.0, 2);
      lcd.print(" M");
    } else {
      lcd.print("---");
    }
    
    // Valve status on right side
    lcd.setCursor(14, 0);
    lcd.print("V:");
    lcd.setCursor(17, 0);
    lcd.print(valveOpen ? "ON " : "OFF");
    
    // Line 1: Volume
    lcd.setCursor(0, 1);
    lcd.print("Vol :");
    lcd.setCursor(6, 1);
    
    if (fuelVolumeL >= 0) {
      lcd.print(fuelVolumeL, 1);
      lcd.print(" L");
    } else {
      lcd.print("---");
    }
    
    // Line 2: Fill Percentage
    lcd.setCursor(0, 2);
    lcd.print("Fill:");
    lcd.setCursor(6, 2);
    
    if (fuelPercent >= 0 && fuelPercent <= 100) {
      lcd.print(fuelPercent, 1);
      lcd.print("%");
    } else {
      lcd.print("---");
    }
    
    // Line 3: Network Status
    lcd.setCursor(0, 3);
    lcd.print("IP  :");
    
    if (WiFi.status() == WL_CONNECTED) {
      lcd.setCursor(5, 3);
      lcd.print(ipAddress);
    } else {
      lcd.setCursor(5, 3);
      lcd.print("Disconnected");
    }
  }
}

void displayTankInfo() {
  Serial.println("\n=== TANK CONFIGURATION ===");
  Serial.printf("Height: %.0f mm\n", TANK_HEIGHT_MM);
  Serial.printf("Length: %.0f mm\n", TANK_LENGTH_MM);
  Serial.printf("Width: %.0f mm\n", TANK_WIDTH_MM);
  Serial.printf("Capacity: %.0f L\n", TANK_CAPACITY_L);
  Serial.printf("4-20mA Range: %.1f-%.1f mA\n", SENSOR_MIN_MA, SENSOR_MAX_MA);
  Serial.println("===========================\n");
}

// ===================== VALVE CONTROL =====================
void initValveControl() {
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(VALVE_PIN, HIGH);  // Start with valve closed
  Serial.println("✅ Valve control initialized (default: CLOSED)");
}

void controlValve() {
  // Valve logic: Close at 80%, Open at 50%
  if (fuelPercent >= VALVE_CLOSE_LEVEL && valveOpen) {
    Serial.println("📈 Fill ≥ 80% → Closing valve");
    valveOpen = false;
    digitalWrite(VALVE_PIN, HIGH);
  } 
  else if (fuelPercent <= VALVE_OPEN_LEVEL && !valveOpen) {
    Serial.println("📉 Fill ≤ 50% → Opening valve");
    valveOpen = true;
    digitalWrite(VALVE_PIN, LOW);
  }
  
  // Update display if state changed
  if (valveOpen != previousValveState) {
    previousValveState = valveOpen;
    Serial.printf("✅ Valve now %s\n", valveOpen ? "OPEN" : "CLOSED");
  }
}

// ===================== NETWORK & MODBUS FUNCTIONS =====================
void setupWiFiConfig() {
  preferences.begin("netcfg", false);
  
  // Default AP credentials
  WiFiManager wm;
  wm.setConfigPortalTimeout(CONFIG_TIMEOUT);
  
  // Display WiFi portal info on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("   Connect WiFi    ");
  lcd.setCursor(2, 1);
  lcd.print(CONFIG_AP_SSID);
  lcd.setCursor(0, 2);
  lcd.print("  Pass: " CONFIG_AP_PASS);
  lcd.setCursor(0, 3);
  lcd.print("  Timeout: ");
  lcd.print(CONFIG_TIMEOUT);
  
  // Start config portal
  if (!wm.startConfigPortal(CONFIG_AP_SSID, CONFIG_AP_PASS)) {
    Serial.println("⏱️ Config portal timeout - using defaults");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Using defaults");
    delay(2000);
  }
  
  preferences.end();
}

void connectToWiFi() {
  preferences.begin("netcfg", true);
  
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("pass", "");
  
#ifdef USE_DHCP
  // DHCP Mode
  WiFi.begin(ssid.c_str(), password.c_str());
#else
  // Static IP Mode
  WiFi.config(DEFAULT_LOCAL_IP, DEFAULT_GATEWAY, DEFAULT_SUBNET);
  WiFi.begin(ssid.c_str(), password.c_str());
#endif
  
  preferences.end();
  
  Serial.print("Connecting to WiFi");
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
    Serial.print(".");
    delay(500);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    ipAddress = WiFi.localIP().toString();
    
    Serial.println("\n✅ WiFi connected");
    Serial.print("IP Address: ");
    Serial.println(ipAddress);
    
    displayMessage("WiFi Connected", ipAddress);
    delay(2000);
  } else {
    Serial.println("\n❌ WiFi connection failed");
    displayMessage("WiFi Failed", "Check connection");
    delay(2000);
  }
}

void setupModbusTCP() {
  if (!wifiConnected) {
    Serial.println("❌ Cannot setup Modbus TCP - WiFi not connected");
    return;
  }
  
  mb.server();
  
  // Initialize Modbus registers
  for (int i = 0; i < REG_COUNT; i++) {
    mb.addHreg(REG_BASE + i);
    mb.Hreg(REG_BASE + i, 0);
  }
  
  Serial.println("✅ Modbus TCP server started");
  Serial.print("Modbus port: ");
  Serial.println(502);
}

// void updateModbusRegisters() {
//   static unsigned long lastModbusUpdate = 0;
  
//   if (millis() - lastModbusUpdate >= 5000) {
//     lastModbusUpdate = millis();
    
//     if (wifiConnected) {
//       // Update Modbus registers with current values
//       mb.Hreg(REG_BASE + 0, (uint16_t)(fuelHeightMM));      // Height in mm (0-65535)
//       mb.Hreg(REG_BASE + 1, (uint16_t)(fuelVolumeL));       // Volume in L
//       mb.Hreg(REG_BASE + 2, (uint16_t)(fuelPercent * 10));  // Percentage * 10 (0-1000)
//       mb.Hreg(REG_BASE + 3, (uint16_t)(currentMA * 10));    // Current * 10 mA
      
//       // Valve status (0=closed, 1=open)
//       mb.Hreg(REG_BASE + 4, valveOpen ? 1 : 0);
      
//       // Tank dimensions (for reference)
//       mb.Hreg(REG_BASE + 5, (uint16_t)(TANK_HEIGHT_MM / 100));  // Height in dm
//       mb.Hreg(REG_BASE + 6, (uint16_t)(TANK_CAPACITY_L));       // Capacity in L
      
//       Serial.println("📤 Modbus registers updated");
//     }
//   }
// }

void updateModbusRegisters() {
  static unsigned long lastModbusUpdate = 0;
  
  if (millis() - lastModbusUpdate >= 1000) {  // Update every second
    lastModbusUpdate = millis();
    
    if (wifiConnected) {
      // REGISTER 0: Fuel Height in mm (0-65535)
      // Constrain to valid range for 16-bit register
      uint16_t heightValue = constrain((uint16_t)fuelHeightMM, 0, 65535);
      mb.Hreg(REG_BASE + 0, heightValue);
      
      // REGISTER 1: Fuel Volume in L (0-65535)
      // Constrain to valid range for 16-bit register
      uint16_t volumeValue = constrain((uint16_t)fuelVolumeL, 0, 65535);
      mb.Hreg(REG_BASE + 1, volumeValue);
      
      // REGISTER 2: Fill Percentage * 10 (0-1000)
      // Multiply by 10 to get 0.1% resolution (0.0% to 100.0%)
      uint16_t percentValue = constrain((uint16_t)(fuelPercent * 10.0), 0, 1000);
      mb.Hreg(REG_BASE + 2, percentValue);
      
      // Optional: Debug output
      static int debugCounter = 0;
      if (debugCounter++ % 10 == 0) {  // Print every 10 seconds
        Serial.printf("📤 Modbus: H=%u(mm) V=%u(L) P=%u(0.1%%)\n", 
                     heightValue, volumeValue, percentValue);
      }
    }
  }
}


// ===================== UTILITY FUNCTIONS =====================
float calculateCylindricalVolume(float height) {
  // For cylindrical tanks
  float radius = TANK_WIDTH_MM / 2.0;  // Assuming width = diameter
  float volume_mm3 = 3.14159 * radius * radius * height;
  return volume_mm3 / 1000000.0;  // Convert to liters
}

float calculateRectangularVolume(float height) {
  // For rectangular tanks
  float volume_mm3 = TANK_LENGTH_MM * TANK_WIDTH_MM * height;
  return volume_mm3 / 1000000.0;  // Convert to liters
}

// ===================== MODBUS REGISTER MAP =====================
/*
Register Map (Holding Registers starting at address 0):
0: Fuel Height (mm) - 0 to 65535
1: Fuel Volume (L) - 0 to 65535
2: Fill Percentage * 10 (0-1000 = 0.0% to 100.0%)
*/
