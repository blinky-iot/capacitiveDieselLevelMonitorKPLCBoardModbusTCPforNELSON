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
  int netMode = preferences.getInt("netmode", 0);   // 0 = DHCP, 1 = Static
  preferences.end();

#ifdef USE_WIFI
  if (netMode == 0) {
    // DHCP → WiFi + MQTT
    while (!connectWiFiDHCP());
    if (mqttConnect()) {
      Serial.println("✅ Attributes sync done");
    }
  } else {
    // Static → WiFi only, no MQTT
    while (!connectWiFiStatic());
    Serial.println("🚫 MQTT skipped (Static IP mode)");
  }
#endif

#ifdef USE_GSM
  manageGSMConnectivity();
#endif

  hourlyResetSetup();
  Serial.println("✅ System Ready - Using Modbus RTU for sensor data");
}



unsigned long tGsm;
unsigned long lastTelemetryTime;
bool startupTelemetry = true;

void loop() {
    // 1. Update Modbus data (gets filtered readings from slave)
    updateModbusData();
    
    // 2. Update global frequency variable if Modbus data is valid
    if (isModbusDataValid()) {
        freqHz = (float)getModbusFrequency();  // Update global freqHz
    }
    
    unsigned long currentTime = millis();

    if (currentTime - lastTelemetryTime >= deviceSettings.telemetryInterval * 1000 || startupTelemetry) {
        lastTelemetryTime = currentTime;

        // Read network mode live
        preferences.begin("netcfg", true);
        int netMode = preferences.getInt("netmode", 0);   // 0 = DHCP, 1 = Static
        preferences.end();

        // Only send telemetry if DHCP mode and Wi-Fi connected
        if (netMode == 0 && WiFi.status() == WL_CONNECTED) {
            Serial.println("----------------Sending telemetry to server-----------------");
            telemetryLoop();
        } else if (netMode == 0 && WiFi.status() != WL_CONNECTED) {
            Serial.println("----------------Telemetry skipped (DHCP, Wi-Fi disconnected)-----------------");
        } else {
            Serial.println("----------------Telemetry skipped (Static IP mode)-----------------");
        }

        startupTelemetry = false;
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