/*
 * ESP32 Fuel Tank Level Monitor with 4-20mA Sensor
 * Configurable calibration via WiFi Manager
 * Modbus TCP with 3 registers: Height, Volume, Fill %
 */

// ===================== INCLUDES =====================
#include <WiFi.h>
#include <WiFiManager.h>
#include <ModbusIP_ESP8266.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ===================== LCD CONFIGURATION =====================
#define LCD_I2C_ADDR 0x27
#define LCD_COLUMNS 20
#define LCD_ROWS 4
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLUMNS, LCD_ROWS);

// ===================== WIFI MANAGER =====================
Preferences preferences;
#define CONFIG_AP_SSID "FuelTankMonitor"
#define CONFIG_AP_PASS "12345678"
#define CONFIG_TIMEOUT 60  // 5 minutes for configuration

// ===================== SENSOR CONFIGURATION =====================
#define ADC_PIN          34      // GPIO34 is ADC input for 4-20mA
#define RESISTOR_VALUE   120.0   // 120Ω resistor
#define VREF             3.3     // ESP32 reference voltage
#define ADC_RESOLUTION   4095    // 12-bit ADC
#define SAMPLE_COUNT     100     // Samples for averaging
#define READ_INTERVAL    2000    // Read every 2 seconds

// ===================== DEFAULT CALIBRATION VALUES =====================
float TANK_HEIGHT_MM = 2000.0;      // Tank height in mm
float TANK_LENGTH_MM = 1500.0;      // Tank length in mm  
float TANK_WIDTH_MM = 1000.0;       // Tank width in mm
float TANK_CAPACITY_L = 3000.0;     // Tank capacity in liters

float SENSOR_MIN_MA = 4.0;          // 4mA = Empty tank (0mm)
float SENSOR_MAX_MA = 20.0;         // 20mA = Full tank (TANK_HEIGHT_MM)
float CALIBRATION_OFFSET = 0.0;     // Offset adjustment
float CALIBRATION_GAIN = 1.0;       // Gain adjustment
String TANK_TYPE = "rectangular";   // "rectangular" or "cylindrical"
float TANK_RADIUS_MM = 750.0;       // For cylindrical tanks

// ===================== VALVE CONTROL =====================
#define VALVE_PIN         19
bool valveOpen = false;
bool previousValveState = false;
int VALVE_CLOSE_LEVEL = 80;       // Close valve at 80% fill
int VALVE_OPEN_LEVEL = 50;        // Open valve at 50% fill

// ===================== MODBUS TCP =====================
ModbusIP mb;
const int REG_BASE = 0;
const int REG_COUNT = 3;  // ONLY 3 REGISTERS: Height, Volume, Fill%

// ===================== NETWORK CONFIGURATION =====================
IPAddress DEFAULT_LOCAL_IP(192, 168, 1, 76);     // FIXED IP: 192.168.1.76
IPAddress DEFAULT_GATEWAY(192, 168, 1, 1);       // FIXED Gateway
IPAddress DEFAULT_SUBNET(255, 255, 255, 0);      // FIXED Subnet

// ===================== WATCHDOG TIMER =====================
#define WDT_TIMEOUT 120  // 2 minutes
TaskHandle_t BackgroundTaskHandle;

// ===================== GLOBAL VARIABLES =====================
float currentMA = 0;
float fuelHeightMM = 0;
float fuelVolumeL = 0;
float fuelPercent = 0;
String ipAddress = "192.168.1.76";  // Default IP
bool wifiConnected = false;
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 2000;

// ===================== WIFI MANAGER CUSTOM PARAMETERS =====================
WiFiManagerParameter custom_tank_height("tank_height", "Tank Height (mm)", "2000", 10);
WiFiManagerParameter custom_tank_length("tank_length", "Tank Length (mm)", "1500", 10);
WiFiManagerParameter custom_tank_width("tank_width", "Tank Width (mm)", "1000", 10);
WiFiManagerParameter custom_tank_capacity("tank_capacity", "Tank Capacity (L)", "3000", 10);
WiFiManagerParameter custom_tank_type("tank_type", "Tank Type (rect/cyl)", "rectangular", 20);
WiFiManagerParameter custom_tank_radius("tank_radius", "Tank Radius (mm)", "750", 10);

WiFiManagerParameter custom_sensor_min("sensor_min", "Sensor Min (mA)", "4.0", 10);
WiFiManagerParameter custom_sensor_max("sensor_max", "Sensor Max (mA)", "20.0", 10);
WiFiManagerParameter custom_cal_offset("cal_offset", "Calibration Offset", "0.0", 10);
WiFiManagerParameter custom_cal_gain("cal_gain", "Calibration Gain", "1.0", 10);

WiFiManagerParameter custom_valve_close("valve_close", "Close Valve at %", "80", 5);
WiFiManagerParameter custom_valve_open("valve_open", "Open Valve at %", "50", 5);

// Static IP configuration parameters - FIXED VALUES
WiFiManagerParameter custom_local_ip("local_ip", "Static IP (xxx.xxx.xxx.xxx)", "192.168.1.76", 16);
WiFiManagerParameter custom_gateway("gateway", "Gateway (xxx.xxx.xxx.xxx)", "192.168.1.1", 16);
WiFiManagerParameter custom_subnet("subnet", "Subnet Mask (xxx.xxx.xxx.xxx)", "255.255.255.0", 16);

// ===================== WATCHDOG & RESTART FUNCTIONS =====================
void initWatchdog(int timeoutSeconds = WDT_TIMEOUT) {
  esp_task_wdt_init(timeoutSeconds, true);
  Serial.print("🛡️ Watchdog initialized: ");
  Serial.print(timeoutSeconds);
  Serial.println(" seconds");
  
  esp_task_wdt_add(NULL);
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("FUEL TANK LEVEL MONITOR - 4-20mA Sensor");
  Serial.println("========================================\n");
  
  // Initialize Watchdog Timer
  initWatchdog();
  
  // Initialize LCD
  lcdInit();
  displayMessage("System Starting", "Loading settings...");
  
  // Load saved calibration
  loadCalibration();
  
  // Configure ADC for 4-20mA sensor
  setupSensor();
  
  // Initialize valve control
  initValveControl();
  
  // Display loaded settings
  displayTankInfo();
  
  // ===================== ALWAYS OPEN CONFIG PORTAL ON EVERY START =====================
  Serial.println("🔧 ALWAYS OPENING CONFIG PORTAL ON STARTUP");
  Serial.println("⚠️ Connect to configure or wait 5 minutes to continue");
  displayMessage("Config Portal", "Connect to configure");
  delay(2000);
  
  // Setup WiFi Configuration Portal - ALWAYS opens on startup
  bool configSaved = setupWiFiConfig();
  
  if (configSaved) {
    Serial.println("✅ Configuration SAVED - connecting to WiFi...");
    displayMessage("Config Saved", "Connecting WiFi...");
    
    // Always use FIXED static IP: 192.168.1.76
    WiFi.config(DEFAULT_LOCAL_IP, DEFAULT_GATEWAY, DEFAULT_SUBNET);
    
    // Try to connect to WiFi with saved credentials
    if (WiFi.status() != WL_CONNECTED) {
      connectToWiFi();
    } else {
      // Already connected from portal
      wifiConnected = true;
      ipAddress = WiFi.localIP().toString();
      Serial.println("✅ Connected to WiFi from portal");
      Serial.print("IP: ");
      Serial.println(ipAddress);
      displayMessage("WiFi Connected", ipAddress);
      delay(2000);
    }
  } else {
    // Config portal timed out - use previously saved settings
    Serial.println("⏱️ Config portal timeout - using PREVIOUSLY SAVED settings");
    displayMessage("Using Saved", "Settings");
    
    // Try to connect with previously saved WiFi
    if (WiFi.status() != WL_CONNECTED) {
      connectWithSavedWiFi();
    }
    
    delay(2000);
  }
  
  // Setup Modbus TCP if WiFi is connected
  if (wifiConnected) {
    setupModbusTCP();
  }
  
  // Display network info
  Serial.println("\n=== NETWORK SETTINGS ===");
  Serial.printf("Static IP: %s\n", DEFAULT_LOCAL_IP.toString().c_str());
  Serial.printf("Gateway: %s\n", DEFAULT_GATEWAY.toString().c_str());
  Serial.printf("Subnet: %s\n", DEFAULT_SUBNET.toString().c_str());
  if (wifiConnected) {
    Serial.printf("Modbus TCP: %s:502 (ENABLED)\n", DEFAULT_LOCAL_IP.toString().c_str());
  } else {
    Serial.println("Modbus TCP: DISABLED (No WiFi connection)");
  }
  Serial.println("==========================\n");
  
  // Start hourly restart task
  hourlyResetSetup();
  
  delay(2000);
  displayMessage("System Ready", "Monitoring...");
  Serial.println("✅ System ready for monitoring");
  
  esp_task_wdt_reset();
}

// ===================== MAIN LOOP =====================
void loop() {
  // 1. Read sensor and calculate values
  readSensorData();
  
  // 2. Update LCD display periodically
  updateDisplay();
  
  // 3. Update ONLY 3 Modbus TCP registers (if WiFi connected)
  if (wifiConnected) {
    updateModbusRegisters();
  }
  
  // 4. Handle valve control
  controlValve();
  
  // 5. Handle Modbus TCP requests (if WiFi connected)
  if (wifiConnected) {
    mb.task();
  }
  
  // 6. Check for serial commands
  checkSerialCommands();
  
  // 7. Feed the watchdog
  esp_task_wdt_reset();
  
  delay(100);
}

// ===================== SERIAL COMMANDS =====================
void checkSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "config" || command == "reconfig") {
      Serial.println("🔄 Re-entering Configuration Mode...");
      displayMessage("Reconfiguring", "Restarting...");
      delay(2000);
      ESP.restart();  // Restart will open config portal
    }
    
    if (command == "factoryreset") {
      Serial.println("🧹 Factory Reset - Clearing ALL settings!");
      displayMessage("Factory Reset", "Clearing ALL...");
      delay(2000);
      
      preferences.begin("netcfg", false);
      preferences.clear();
      preferences.end();
      
      preferences.begin("tank_cal", false);
      preferences.clear();
      preferences.end();
      
      Serial.println("✅ All settings cleared");
      displayMessage("Settings Cleared", "Restarting...");
      delay(2000);
      
      ESP.restart();
    }
    
    if (command == "status") {
      Serial.println("\n=== SYSTEM STATUS ===");
      Serial.printf("WiFi: %s\n", wifiConnected ? "Connected" : "Disconnected");
      Serial.printf("IP: %s\n", ipAddress.c_str());
      Serial.printf("Sensor: %.2f mA\n", currentMA);
      Serial.printf("Height: %.0f mm\n", fuelHeightMM);
      Serial.printf("Volume: %.0f L\n", fuelVolumeL);
      Serial.printf("Fill: %.1f%%\n", fuelPercent);
      Serial.printf("Valve: %s\n", valveOpen ? "OPEN" : "CLOSED");
      Serial.println("\n=== TANK SETTINGS ===");
      Serial.printf("Type: %s\n", TANK_TYPE.c_str());
      Serial.printf("Height: %.0f mm\n", TANK_HEIGHT_MM);
      if (TANK_TYPE == "rectangular") {
        Serial.printf("Length: %.0f mm, Width: %.0f mm\n", TANK_LENGTH_MM, TANK_WIDTH_MM);
      } else {
        Serial.printf("Radius: %.0f mm\n", TANK_RADIUS_MM);
      }
      Serial.printf("4-20mA: %.1f-%.1f mA\n", SENSOR_MIN_MA, SENSOR_MAX_MA);
      Serial.printf("Valve: Close at %d%%, Open at %d%%\n", VALVE_CLOSE_LEVEL, VALVE_OPEN_LEVEL);
      Serial.println("===================\n");
    }
    
    if (command == "network") {
      Serial.println("\n=== NETWORK INFO ===");
      Serial.printf("Static IP: 192.168.1.76 (FIXED)\n");
      Serial.printf("Gateway: 192.168.1.1\n");
      Serial.printf("Subnet: 255.255.255.0\n");
      if (wifiConnected) {
        Serial.printf("Current IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("Modbus TCP Port: 502 (ENABLED)\n");
      } else {
        Serial.printf("Current IP: N/A (WiFi Disconnected)\n");
        Serial.printf("Modbus TCP Port: 502 (DISABLED)\n");
      }
      Serial.println("=====================\n");
    }
    
    if (command == "wifi") {
      Serial.println("Attempting to reconnect WiFi...");
      displayMessage("Reconnecting", "WiFi...");
      connectToWiFi();
    }
    
    if (command == "save") {
      Serial.println("💾 Manually saving current settings...");
      saveCalibration();
      displayMessage("Settings Saved", "to storage");
      delay(2000);
    }
  }
}

// ===================== HOURLY RESET TASK =====================
void hourlyResetTask(void* pvParameters) {
  while (true) {
    vTaskDelay(3600000 / portTICK_PERIOD_MS);  // 1 hour delay
    
    Serial.println("🔁 Hourly restart - will open config portal...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Hourly Restart...");
    lcd.setCursor(0, 1);
    lcd.print("Config portal will");
    lcd.setCursor(0, 2);
    lcd.print("open on restart");
    delay(3000);
    
    ESP.restart();  // Restart will open config portal
  }
}

void hourlyResetSetup() {
  xTaskCreatePinnedToCore(hourlyResetTask, "HourlyReset", 2048, NULL, 1, &BackgroundTaskHandle, 1);
  Serial.println("✅ Hourly restart task started");
}

// ===================== CALIBRATION LOAD/SAVE =====================
void loadCalibration() {
  preferences.begin("tank_cal", true);
  
  TANK_HEIGHT_MM = preferences.getFloat("height", 2000.0);
  TANK_LENGTH_MM = preferences.getFloat("length", 1500.0);
  TANK_WIDTH_MM = preferences.getFloat("width", 1000.0);
  TANK_CAPACITY_L = preferences.getFloat("capacity", 3000.0);
  TANK_TYPE = preferences.getString("type", "rectangular");
  TANK_RADIUS_MM = preferences.getFloat("radius", 750.0);
  
  SENSOR_MIN_MA = preferences.getFloat("sensor_min", 4.0);
  SENSOR_MAX_MA = preferences.getFloat("sensor_max", 20.0);
  CALIBRATION_OFFSET = preferences.getFloat("offset", 0.0);
  CALIBRATION_GAIN = preferences.getFloat("gain", 1.0);
  
  VALVE_CLOSE_LEVEL = preferences.getInt("valve_close", 80);
  VALVE_OPEN_LEVEL = preferences.getInt("valve_open", 50);
  
  preferences.end();
  
  Serial.println("📂 Loaded calibration from storage");
}

void saveCalibration() {
  preferences.begin("tank_cal", false);
  
  preferences.putFloat("height", TANK_HEIGHT_MM);
  preferences.putFloat("length", TANK_LENGTH_MM);
  preferences.putFloat("width", TANK_WIDTH_MM);
  preferences.putFloat("capacity", TANK_CAPACITY_L);
  preferences.putString("type", TANK_TYPE);
  preferences.putFloat("radius", TANK_RADIUS_MM);
  
  preferences.putFloat("sensor_min", SENSOR_MIN_MA);
  preferences.putFloat("sensor_max", SENSOR_MAX_MA);
  preferences.putFloat("offset", CALIBRATION_OFFSET);
  preferences.putFloat("gain", CALIBRATION_GAIN);
  
  preferences.putInt("valve_close", VALVE_CLOSE_LEVEL);
  preferences.putInt("valve_open", VALVE_OPEN_LEVEL);
  
  preferences.end();
  
  Serial.println("💾 Saved calibration to storage");
}

// ===================== SENSOR FUNCTIONS =====================
void setupSensor() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(ADC_PIN, INPUT);
  Serial.println("✅ 4-20mA Sensor initialized");
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
  float current = (voltage / RESISTOR_VALUE) * 1000.0;
  current = (current + CALIBRATION_OFFSET) * CALIBRATION_GAIN;
  return constrain(current, 0.0, 25.0);
}

void readSensorData() {
  static unsigned long lastRead = 0;
  
  if (millis() - lastRead >= READ_INTERVAL) {
    lastRead = millis();
    
    // Read current from sensor
    currentMA = readCurrentMA();
    
    // Calculate fuel height
    if (currentMA <= SENSOR_MIN_MA) {
      fuelHeightMM = 0;
    } else if (currentMA >= SENSOR_MAX_MA) {
      fuelHeightMM = TANK_HEIGHT_MM;
    } else {
      fuelHeightMM = ((currentMA - SENSOR_MIN_MA) * TANK_HEIGHT_MM) / 
                     (SENSOR_MAX_MA - SENSOR_MIN_MA);
    }
    
    // Calculate volume based on tank type
    if (TANK_TYPE == "cylindrical") {
      fuelVolumeL = (3.14159 * TANK_RADIUS_MM * TANK_RADIUS_MM * fuelHeightMM) / 1000000.0;
    } else {
      fuelVolumeL = (fuelHeightMM * TANK_LENGTH_MM * TANK_WIDTH_MM) / 1000000.0;
    }
    
    // Calculate percentage
    fuelPercent = (fuelHeightMM / TANK_HEIGHT_MM) * 100.0;
    
    // Constrain to valid ranges
    fuelHeightMM = constrain(fuelHeightMM, 0.0, TANK_HEIGHT_MM);
    fuelVolumeL = constrain(fuelVolumeL, 0.0, TANK_CAPACITY_L);
    fuelPercent = constrain(fuelPercent, 0.0, 100.0);
    
    Serial.printf("Sensor: %.2f mA | Height: %.0f mm | Volume: %.0f L | Fill: %.0f%%\n",
                  currentMA, fuelHeightMM, fuelVolumeL, fuelPercent);
    
    esp_task_wdt_reset();
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

void updateDisplay() {
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = millis();
    
    // Clear lines
    for (int line = 0; line < 4; line++) {
      lcd.setCursor(0, line);
      for (int i = 0; i < LCD_COLUMNS; i++) lcd.print(" ");
    }
    
    // Line 0: Level + Valve
    lcd.setCursor(0, 0);
    lcd.print("Lev:");
    lcd.setCursor(5, 0);
    lcd.print(fuelHeightMM / 1000.0, 2);
    lcd.print(" M");
    
    lcd.setCursor(14, 0);
    lcd.print("V:");
    lcd.setCursor(17, 0);
    lcd.print(valveOpen ? "ON " : "OFF");
    
    // Line 1: Volume
    lcd.setCursor(0, 1);
    lcd.print("Vol:");
    lcd.setCursor(5, 1);
    lcd.print(fuelVolumeL, 0);
    lcd.print(" L");
    
    // Line 2: Fill Percentage
    lcd.setCursor(0, 2);
    lcd.print("Fill:");
    lcd.setCursor(6, 2);
    lcd.print(fuelPercent, 0);
    lcd.print("%");
    
    // Line 3: Network Status - ALWAYS show 192.168.1.76
    lcd.setCursor(0, 3);
    lcd.print("IP:");
    lcd.setCursor(4, 3);
    
    if (wifiConnected) {
      // Show the FIXED IP: 192.168.1.76
      lcd.print("192.168.1.76");
    } else {
      lcd.print("192.168.1.76");
      lcd.setCursor(16, 3);
      lcd.print("NC");  // NC = Not Connected
    }
    
    esp_task_wdt_reset();
  }
}

void displayTankInfo() {
  Serial.println("\n=== LOADED TANK SETTINGS ===");
  Serial.printf("Type: %s\n", TANK_TYPE.c_str());
  Serial.printf("Height: %.0f mm\n", TANK_HEIGHT_MM);
  
  if (TANK_TYPE == "rectangular") {
    Serial.printf("Length: %.0f mm, Width: %.0f mm\n", TANK_LENGTH_MM, TANK_WIDTH_MM);
  } else {
    Serial.printf("Radius: %.0f mm\n", TANK_RADIUS_MM);
  }
  
  Serial.printf("Capacity: %.0f L\n", TANK_CAPACITY_L);
  Serial.printf("4-20mA: %.1f-%.1f mA\n", SENSOR_MIN_MA, SENSOR_MAX_MA);
  Serial.printf("Valve: Close at %d%%, Open at %d%%\n", VALVE_CLOSE_LEVEL, VALVE_OPEN_LEVEL);
  Serial.println("===================================\n");
}

// ===================== VALVE CONTROL =====================
void initValveControl() {
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(VALVE_PIN, HIGH);  // Start with valve closed
  Serial.println("✅ Valve control initialized");
}

void controlValve() {
  if (fuelPercent >= VALVE_CLOSE_LEVEL && valveOpen) {
    Serial.printf("📈 Fill ≥ %d%% → Closing valve\n", VALVE_CLOSE_LEVEL);
    valveOpen = false;
    digitalWrite(VALVE_PIN, HIGH);
  } 
  else if (fuelPercent <= VALVE_OPEN_LEVEL && !valveOpen) {
    Serial.printf("📉 Fill ≤ %d%% → Opening valve\n", VALVE_OPEN_LEVEL);
    valveOpen = true;
    digitalWrite(VALVE_PIN, LOW);
  }
  
  if (valveOpen != previousValveState) {
    previousValveState = valveOpen;
    Serial.printf("✅ Valve now %s\n", valveOpen ? "OPEN" : "CLOSED");
  }
}

// ===================== WIFI CONFIGURATION =====================
bool setupWiFiConfig() {
  // This function ALWAYS opens config portal on startup
  // Returns true if new settings were saved, false if timeout
  
  preferences.begin("netcfg", false);
  
  WiFiManager wm;
  wm.setConfigPortalTimeout(CONFIG_TIMEOUT);
  wm.setConnectTimeout(30);
  
  // Don't auto-connect - always show portal
  wm.setConnectRetries(0);
  
  // Set current values in parameters (load from saved settings)
  char tankHeightStr[10], tankLengthStr[10], tankWidthStr[10], tankCapacityStr[10];
  char tankRadiusStr[10], sensorMinStr[10], sensorMaxStr[10], calOffsetStr[10], calGainStr[10];
  char valveCloseStr[10], valveOpenStr[10];
  
  dtostrf(TANK_HEIGHT_MM, 1, 0, tankHeightStr);
  dtostrf(TANK_LENGTH_MM, 1, 0, tankLengthStr);
  dtostrf(TANK_WIDTH_MM, 1, 0, tankWidthStr);
  dtostrf(TANK_CAPACITY_L, 1, 0, tankCapacityStr);
  dtostrf(TANK_RADIUS_MM, 1, 0, tankRadiusStr);
  dtostrf(SENSOR_MIN_MA, 1, 1, sensorMinStr);
  dtostrf(SENSOR_MAX_MA, 1, 1, sensorMaxStr);
  dtostrf(CALIBRATION_OFFSET, 1, 2, calOffsetStr);
  dtostrf(CALIBRATION_GAIN, 1, 2, calGainStr);
  sprintf(valveCloseStr, "%d", VALVE_CLOSE_LEVEL);
  sprintf(valveOpenStr, "%d", VALVE_OPEN_LEVEL);
  
  // Update parameter values with current settings
  custom_tank_height.setValue(tankHeightStr, 10);
  custom_tank_length.setValue(tankLengthStr, 10);
  custom_tank_width.setValue(tankWidthStr, 10);
  custom_tank_capacity.setValue(tankCapacityStr, 10);
  custom_tank_type.setValue(TANK_TYPE.c_str(), 20);
  custom_tank_radius.setValue(tankRadiusStr, 10);
  custom_sensor_min.setValue(sensorMinStr, 10);
  custom_sensor_max.setValue(sensorMaxStr, 10);
  custom_cal_offset.setValue(calOffsetStr, 10);
  custom_cal_gain.setValue(calGainStr, 10);
  custom_valve_close.setValue(valveCloseStr, 5);
  custom_valve_open.setValue(valveOpenStr, 5);
  
  // Add custom parameters
  wm.addParameter(&custom_tank_height);
  wm.addParameter(&custom_tank_length);
  wm.addParameter(&custom_tank_width);
  wm.addParameter(&custom_tank_capacity);
  wm.addParameter(&custom_tank_type);
  wm.addParameter(&custom_tank_radius);
  wm.addParameter(&custom_sensor_min);
  wm.addParameter(&custom_sensor_max);
  wm.addParameter(&custom_cal_offset);
  wm.addParameter(&custom_cal_gain);
  wm.addParameter(&custom_valve_close);
  wm.addParameter(&custom_valve_open);
  
  // Add static IP parameters - FIXED VALUES
  wm.addParameter(&custom_local_ip);
  wm.addParameter(&custom_gateway);
  wm.addParameter(&custom_subnet);
  
  bool settingsSaved = false;
  
  // Save callback
  // wm.setSaveConfigCallback([&settingsSaved]() {
  //   Serial.println("💾 SAVING NEW SETTINGS...");
    
  //   // Get new values from portal
  //   TANK_HEIGHT_MM = atof(custom_tank_height.getValue());
  //   TANK_LENGTH_MM = atof(custom_tank_length.getValue());
  //   TANK_WIDTH_MM = atof(custom_tank_width.getValue());
  //   TANK_CAPACITY_L = atof(custom_tank_capacity.getValue());
  //   TANK_TYPE = String(custom_tank_type.getValue());
  //   TANK_RADIUS_MM = atof(custom_tank_radius.getValue());
    
  //   SENSOR_MIN_MA = atof(custom_sensor_min.getValue());
  //   SENSOR_MAX_MA = atof(custom_sensor_max.getValue());
  //   CALIBRATION_OFFSET = atof(custom_cal_offset.getValue());
  //   CALIBRATION_GAIN = atof(custom_cal_gain.getValue());
    
  //   VALVE_CLOSE_LEVEL = atoi(custom_valve_close.getValue());
  //   VALVE_OPEN_LEVEL = atoi(custom_valve_open.getValue());
    
  //   // Save WiFi credentials
  //   preferences.begin("netcfg", false);
  //   preferences.putString("ssid", WiFi.SSID());
  //   preferences.putString("pass", WiFi.psk());
  //   preferences.end();
    
  //   // Save calibration settings
  //   saveCalibration();
    
  //   settingsSaved = true;
  //   Serial.println("✅ NEW SETTINGS SAVED TO ESP!");
    
  //   Serial.println("\n=== NEW TANK SETTINGS ===");
  //   Serial.printf("Type: %s\n", TANK_TYPE.c_str());
  //   Serial.printf("Height: %.0f mm\n", TANK_HEIGHT_MM);
  //   Serial.printf("4-20mA: %.1f-%.1f mA\n", SENSOR_MIN_MA, SENSOR_MAX_MA);
  //   Serial.printf("Valve: Close at %d%%, Open at %d%%\n", VALVE_CLOSE_LEVEL, VALVE_OPEN_LEVEL);
  //   Serial.println("===========================\n");
  // });
  
// In setupWiFiConfig() function, update the save callback:
// Save callback
wm.setSaveConfigCallback([&settingsSaved]() {
  Serial.println("💾 SAVING NEW SETTINGS...");
  
  // Get new values from portal
  TANK_HEIGHT_MM = atof(custom_tank_height.getValue());
  TANK_LENGTH_MM = atof(custom_tank_length.getValue());
  TANK_WIDTH_MM = atof(custom_tank_width.getValue());
  TANK_CAPACITY_L = atof(custom_tank_capacity.getValue());
  TANK_TYPE = String(custom_tank_type.getValue());
  TANK_RADIUS_MM = atof(custom_tank_radius.getValue());
  
  SENSOR_MIN_MA = atof(custom_sensor_min.getValue());
  SENSOR_MAX_MA = atof(custom_sensor_max.getValue());
  CALIBRATION_OFFSET = atof(custom_cal_offset.getValue());
  CALIBRATION_GAIN = atof(custom_cal_gain.getValue());
  
  VALVE_CLOSE_LEVEL = atoi(custom_valve_close.getValue());
  VALVE_OPEN_LEVEL = atoi(custom_valve_open.getValue());
  
  // Save WiFi credentials
  preferences.begin("netcfg", false);
  preferences.putString("ssid", WiFi.SSID());
  preferences.putString("pass", WiFi.psk());
  preferences.end();
  
  // Save calibration settings
  saveCalibration();
  
  settingsSaved = true;
  Serial.println("✅ NEW SETTINGS SAVED TO ESP!");
  
  // ===================== FIXED: DISPLAY ALL TANK SETTINGS =====================
  Serial.println("\n=== NEW TANK SETTINGS ===");
  Serial.printf("Type: %s\n", TANK_TYPE.c_str());
  Serial.printf("Height: %.0f mm\n", TANK_HEIGHT_MM);
  
  if (TANK_TYPE == "rectangular") {
    Serial.printf("Length: %.0f mm\n", TANK_LENGTH_MM);
    Serial.printf("Width: %.0f mm\n", TANK_WIDTH_MM);
  } else {
    Serial.printf("Radius: %.0f mm\n", TANK_RADIUS_MM);
  }
  
  Serial.printf("Capacity: %.0f L\n", TANK_CAPACITY_L);
  Serial.printf("4-20mA: %.1f-%.1f mA\n", SENSOR_MIN_MA, SENSOR_MAX_MA);
  Serial.printf("Calibration Offset: %.2f\n", CALIBRATION_OFFSET);
  Serial.printf("Calibration Gain: %.2f\n", CALIBRATION_GAIN);
  Serial.printf("Valve: Close at %d%%, Open at %d%%\n", VALVE_CLOSE_LEVEL, VALVE_OPEN_LEVEL);
  Serial.println("===========================\n");
  // ============================================================================
});


  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CONFIG PORTAL OPEN");
  lcd.setCursor(0, 1);
  lcd.print("SSID:" CONFIG_AP_SSID);
  lcd.setCursor(0, 2);
  lcd.print("IP: 192.168.4.1");
  lcd.setCursor(0, 3);
  lcd.print("Timeout:60 sec");
  
  Serial.println("\n🔧 CONFIGURATION PORTAL OPEN");
  Serial.println("========================================");
  Serial.println("⚠️ CONFIG PORTAL OPENS ON EVERY STARTUP");
  Serial.println("Connect to WiFi: " CONFIG_AP_SSID);
  Serial.println("Password: " CONFIG_AP_PASS);
  Serial.println("Open browser: 192.168.4.1");
  Serial.println("\n📝 Configure or change settings:");
  Serial.println("- WiFi Network & Password");
  Serial.println("- Tank Calibration");
  Serial.println("- Sensor Settings");
  Serial.println("- Valve Control Levels");
  Serial.println("\n💾 Click 'SAVE' to save new settings to ESP");
  Serial.println("⏱️ Wait 5 minutes to continue with current settings");
  Serial.println("========================================\n");
  
  // Start config portal - ALWAYS on startup
  bool portalStarted = wm.startConfigPortal(CONFIG_AP_SSID, CONFIG_AP_PASS);
  
  preferences.end();
  esp_task_wdt_reset();
  
  return settingsSaved;  // Return true if new settings were saved
}

void connectToWiFi() {
  // Check if already connected
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    ipAddress = "192.168.1.76";
    Serial.println("✅ Already connected to WiFi");
    return;
  }
  
  preferences.begin("netcfg", true);
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("pass", "");
  preferences.end();
  
  if (ssid == "") {
    Serial.println("⚠️ No WiFi credentials saved");
    displayMessage("No WiFi Config", "Enter portal to set");
    delay(2000);
    return;
  }
  
  // ALWAYS use FIXED static IP: 192.168.1.76
  WiFi.config(DEFAULT_LOCAL_IP, DEFAULT_GATEWAY, DEFAULT_SUBNET);
  
  WiFi.disconnect(true);
  delay(1000);
  
  WiFi.begin(ssid.c_str(), password.c_str());
  
  Serial.print("Connecting to saved WiFi: ");
  Serial.print(ssid);
  Serial.print(" with FIXED IP: ");
  Serial.println(DEFAULT_LOCAL_IP.toString());
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) {
    Serial.print(".");
    delay(500);
    esp_task_wdt_reset();
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    ipAddress = "192.168.1.76";
    Serial.println("\n✅ WiFi connected");
    Serial.print("FIXED IP: ");
    Serial.println(ipAddress);
    displayMessage("WiFi Connected", ipAddress);
    delay(2000);
  } else {
    wifiConnected = false;
    Serial.println("\n❌ WiFi connection failed");
    displayMessage("WiFi Failed", "Check settings");
    delay(2000);
  }
  
  esp_task_wdt_reset();
}

void connectWithSavedWiFi() {
  // Similar to connectToWiFi but for timeout scenario
  preferences.begin("netcfg", true);
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("pass", "");
  preferences.end();
  
  if (ssid == "") {
    return;  // No saved credentials
  }
  
  WiFi.config(DEFAULT_LOCAL_IP, DEFAULT_GATEWAY, DEFAULT_SUBNET);
  WiFi.begin(ssid.c_str(), password.c_str());
  
  Serial.print("Trying saved WiFi: ");
  Serial.println(ssid);
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    delay(500);
    esp_task_wdt_reset();
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("✅ Connected with saved WiFi");
  }
}

// ===================== MODBUS TCP =====================
void setupModbusTCP() {
  if (!wifiConnected) {
    return;
  }
  
  mb.server();
  
  for (int i = 0; i < REG_COUNT; i++) {
    mb.addHreg(REG_BASE + i);
    mb.Hreg(REG_BASE + i, 0);
  }
  
  Serial.println("✅ Modbus TCP server started");
  Serial.print("📡 Modbus Address: ");
  Serial.print(DEFAULT_LOCAL_IP.toString());
  Serial.println(":502");
  Serial.println("📊 Registers: 0=Height(mm), 1=Volume(L), 2=Fill%(0.1%)");
  
  esp_task_wdt_reset();
}

void updateModbusRegisters() {
  static unsigned long lastModbusUpdate = 0;
  
  if (millis() - lastModbusUpdate >= 1000) {
    lastModbusUpdate = millis();
    
    if (wifiConnected) {
      mb.Hreg(REG_BASE + 0, (uint16_t)constrain(fuelHeightMM, 0, 65535));
      mb.Hreg(REG_BASE + 1, (uint16_t)constrain(fuelVolumeL, 0, 65535));
      mb.Hreg(REG_BASE + 2, (uint16_t)constrain(fuelPercent * 10.0, 0, 1000));
      
      static int debugCounter = 0;
      if (debugCounter++ % 10 == 0) {
        Serial.printf("📤 Modbus@192.168.1.76:502 - H=%u(mm) V=%u(L) P=%u(0.1%%)\n", 
                     (uint16_t)fuelHeightMM, 
                     (uint16_t)fuelVolumeL, 
                     (uint16_t)(fuelPercent * 10.0));
      }
      
      esp_task_wdt_reset();
    }
  }
}