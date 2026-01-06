#include "definitions.h"
int Volume = 0;
int Fill_Percentage = 0;


TaskHandle_t BackgroundProcessor;
void initWatchdog(int timeoutSeconds = 600) {
  // Initialize the Task Watchdog Timer
  esp_task_wdt_init(timeoutSeconds, true);  // timeout, panic on trigger
  Serial.println("🛡️ Watchdog initialized");

  // Add the tasks you want to monitor
  esp_task_wdt_add(NULL);  // Add current task (usually loop or setup task)
}
/*
void setup() {
  Serial.begin(115200);
  initWatchdog();
  LCDsetup();
  StatusDisplay();
  initSD();

  // ---- Run WiFi Config Portal (always on reset) ----
  setupConfig();
  // ---- Initialize Modbus RTU Master ----
  setupModbusRTU();

  // ---- Continue with system init ----
  xTaskCreatePinnedToCore(wifiConnection, "wifi_Connection", 10000, NULL, 0, &BackgroundProcessor, 0);
  //initGpsHandler();
  initSettings();
  initCalibration();
  setupSensors();
  rtcTimeOk = loadRtcTimeOk();
  if (rtcTimeOk) Serial.println("RTC is SET");
  else Serial.println("RTC is NOT SET");

  // for (int i = 0; i < 10; i++) {
  //   freqHz = getAvgFrequency_EMA(15, 0.15);
  // }

  // ---- Apply network mode selection ----
  preferences.begin("netcfg", false);
  int netMode = preferences.getInt("netmode", 0);  // 0 = DHCP, 1 = Static
  preferences.end();

#ifdef USE_WIFI
  if (netMode == 0) {
    // DHCP → WiFi + MQTT
    while (!connectWiFiDHCP())
      ;
    if (mqttConnect()) {
      Serial.println("✅ Attributes sync done");
    }
  } else {
    // Static → WiFi only, no MQTT
    while (!connectWiFiStatic())
      ;
    Serial.println("🚫 MQTT skipped (Static IP mode)");
  }
#endif

#ifdef USE_GSM
  manageGSMConnectivity();
#endif

  hourlyResetSetup();
  Serial.println("✅ System Ready - Using Modbus RTU for sensor data");
}
*/

// In fuellevel_monitoring_withmodbus_TCP.ino, update the setup function:

void setup() {
  Serial.begin(115200);
  initWatchdog();
  LCDsetup();
  StatusDisplay();
  initSD();

  // ---- Show initial display while booting ----
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Starting...");

  // ---- Initialize Modbus RTU First (critical) ----
  setupModbusRTU();

  // ---- Run WiFi Config Portal (non-blocking) ----
  setupConfig();

  telemetryMutex = xSemaphoreCreateMutex();
  if (telemetryMutex == NULL) {
    Serial.println("❌ Failed to create telemetry mutex");
  } else {
    Serial.println("✅ Telemetry mutex created");
  }


  // ---- Continue with system init ----
  xTaskCreatePinnedToCore(wifiConnection, "wifi_Connection", 10000, NULL, 0, &BackgroundProcessor, 0);
  initSettings();
  initCalibration();
  setupSensors();

  rtcTimeOk = loadRtcTimeOk();
  if (rtcTimeOk) Serial.println("RTC is SET");
  else Serial.println("RTC is NOT SET");

  // ---- Apply network mode selection ----
  preferences.begin("netcfg", false);
  int netMode = preferences.getInt("netmode", 0);  // 0 = DHCP, 1 = Static
  preferences.end();

#ifdef USE_WIFI
  // Start WiFi connection in background, don't block
  if (netMode == 0) {
    // DHCP → WiFi + MQTT (connect in background)
    xTaskCreatePinnedToCore(connectWiFiDHCPTask, "WiFiDHCP", 4096, NULL, 1, NULL, 0);
  } else {
    // Static → WiFi only (connect in background)
    xTaskCreatePinnedToCore(connectWiFiStaticTask, "WiFiStatic", 4096, NULL, 1, NULL, 0);
  }
#endif

  hourlyResetSetup();

  // Show ready message
  lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print("System Ready");
  // lcd.setCursor(0, 1);
  // lcd.print("Mode: ");
  // lcd.print(netMode == 0 ? "DHCP" : "Static");
  // lcd.setCursor(0, 2);
  // lcd.print("Using Modbus RTU");

  Serial.println("✅ System Ready - Using Modbus RTU for sensor data");
  delay(2000);  // Show ready message briefly
}
















unsigned long tGsm;
unsigned long lastTelemetryTime;
bool startupTelemetry = true;

// void loop() {
//   // 1. Update Modbus data (gets filtered readings from slave)
//   updateModbusData();


//   // 2. Update global frequency variable if Modbus data is valid
//   if (isModbusDataValid()) {
//     freqHz = (float)getModbusFrequency();  // Update global freqHz
//   }

//   unsigned long currentTime = millis();

//   if (currentTime - lastTelemetryTime >= deviceSettings.telemetryInterval * 1000 || startupTelemetry) {
//     lastTelemetryTime = currentTime;

//     // Read network mode live
//     preferences.begin("netcfg", true);
//     int netMode = preferences.getInt("netmode", 0);  // 0 = DHCP, 1 = Static
//     preferences.end();

//     // Only send telemetry if DHCP mode and Wi-Fi connected
//     if (netMode == 0 && WiFi.status() == WL_CONNECTED) {
//       Serial.println("----------------Sending telemetry to server-----------------");
//       telemetryLoop();
//     } else if (netMode == 0 && WiFi.status() != WL_CONNECTED) {
//       Serial.println("----------------Telemetry skipped (DHCP, Wi-Fi disconnected)-----------------");
//     } else {
//       Serial.println("----------------Telemetry skipped (Static IP mode)-----------------");
//     }

//     startupTelemetry = false;
//   }

//   delay(100);
//   esp_task_wdt_reset();
// }

// In your loop(), ensure Modbus updates start immediately:

void loop() {
  // 1. Always update Modbus data first (works offline)
  updateModbusData();

  // 2. Update global frequency variable if Modbus data is valid
  if (isModbusDataValid()) {
    freqHz = (float)getModbusFrequency();
  } else {
    // If no valid data yet, try to initialize
    static bool modbusInitAttempted = false;
    if (!modbusInitAttempted) {
      setupModbusRTU();  // Try to reinitialize
      modbusInitAttempted = true;
    }
  }

  // 3. Process tank calculations even without WiFi
  static unsigned long lastCalculation = 0;
  if (millis() - lastCalculation >= 5000) {  // Every 5 seconds
    lastCalculation = millis();

    if (isModbusDataValid()) {
      // Process tank data even without WiFi
      float volume_liters = getLevelLiters();
      int volumeInt = (int)round(volume_liters);
      int heightInt = (int)round(height_mm);

      // Valve control works offline
      if (deviceSettings.tankHeight > 0) {
        int fillPercent = (int)round(((float)heightInt / deviceSettings.tankHeight) * 100.0);
        fillPercent = constrain(fillPercent, 0, 100);
        valveState(fillPercent);
      }
    }
  }

  // 4. Telemetry only if conditions met (non-blocking)
  unsigned long currentTime = millis();
  static unsigned long lastTelemetryTime = 0;

  if (currentTime - lastTelemetryTime >= deviceSettings.telemetryInterval * 1000) {
    lastTelemetryTime = currentTime;

    // Check WiFi status and mode
    preferences.begin("netcfg", true);
    int netMode = preferences.getInt("netmode", 0);
    preferences.end();

    if (netMode == 0 && WiFi.status() == WL_CONNECTED) {
      Serial.println("----------------Sending telemetry to server-----------------");
      telemetryLoop();
    } else {
      // Just log, don't block
      Serial.println("----------------Telemetry skipped-----------------");
    }
  }

  delay(100);
  esp_task_wdt_reset();
}









// void loop() {
//     updateUltrasonicDistance();

//     // Get frequency (EMA average)
//     for (int i = 0; i < 10; i++) {
//         freqHz = getAvgFrequency_EMA(15, 0.15);
//     }
//     Serial.print("Frequency: ");
//     Serial.println(freqHz);

//     unsigned long currentTime = millis();

//     if (currentTime - lastTelemetryTime >= deviceSettings.telemetryInterval * 1000 || startupTelemetry) {
//         lastTelemetryTime = currentTime;

//         // Read network mode live
//         preferences.begin("netcfg", true);
//         int netMode = preferences.getInt("netmode", 0);   // 0 = DHCP, 1 = Static
//         preferences.end();

//         // Only send telemetry if DHCP mode and Wi-Fi connected
//         if (netMode == 0 && WiFi.status() == WL_CONNECTED) {
//             Serial.println("----------------Sending telemetry to server-----------------");
//             telemetryLoop();
//         } else if (netMode == 0 && WiFi.status() != WL_CONNECTED) {
//             Serial.println("----------------Telemetry skipped (DHCP, Wi-Fi disconnected)-----------------");
//         } else {
//             Serial.println("----------------Telemetry skipped (Static IP mode)-----------------");
//         }

//         startupTelemetry = false;
//     }

//     delay(100);
//     esp_task_wdt_reset();
// }


//     Serial.println("----------------Sending telemetry to server-----------------");
//     telemetryLoop();
//     startupTelemetry = false;
//   }
//   delay(100);
//   esp_task_wdt_reset();  // Feed the watchdog
// }