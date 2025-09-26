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

// void setup() {
//   Serial.begin(115200);
//   initWatchdog();
//   LCDsetup();
//   StatusDisplay();
//   initSD();
//   xTaskCreatePinnedToCore(wifiConnection, "wifi_Connection", 10000, NULL, 0, &BackgroundProcessor, 0);
//   //displayTask();
//   initGpsHandler();
//   initSettings();
//   initCalibration();
//   setupSensors();

//   rtcTimeOk = loadRtcTimeOk();  // load saved value
//   if (rtcTimeOk) Serial.println("RTC is SET");
//   else Serial.println("RTC is NOT SET");

//   for (int i = 0; i < 10; i++) {
//     freqHz = getAvgFrequency_EMA(15, 0.15);
//   }

// #ifdef USE_WIFI
//   // 1️⃣ Step 1 → DHCP WiFi for MQTT + attributes
//   while (!connectWiFiDHCP());   // keep trying until connected
//   if (mqttConnect()) {
//     Serial.println("✅ Attributes sync done");
//   }

//   // 2️⃣ Step 2 → Restart WiFi with static IP + Modbus
//   //WiFi.disconnect(true);
//   delay(1000);
//   while (!connectWiFiStatic()); // keep trying until connected
// #endif

// #ifdef USE_GSM
//   manageGSMConnectivity();
// #endif

//   hourlyResetSetup();
// }

// void setup() {
//   Serial.begin(115200);
//   initWatchdog();
//   LCDsetup();
//   StatusDisplay();
//   initSD();
//   xTaskCreatePinnedToCore(wifiConnection, "wifi_Connection", 10000, NULL, 0, &BackgroundProcessor, 0);
//   initGpsHandler();
//   initSettings();
//   initCalibration();
//   setupSensors();

//   rtcTimeOk = loadRtcTimeOk();
//   if (rtcTimeOk) Serial.println("RTC is SET");
//   else Serial.println("RTC is NOT SET");

//   for (int i = 0; i < 10; i++) {
//     freqHz = getAvgFrequency_EMA(15, 0.15);
//   }

// #ifdef USE_WIFI
//   // Conditional IP configuration
//   #ifdef USE_DHCP
//     // Use DHCP
//     while (!connectWiFiDHCP());
//   #elif defined(USE_STATIC_IP)
//     // Use Static IP
//     while (!connectWiFiStatic());
//   #else
//     // Default to DHCP if neither is defined
//     while (!connectWiFiDHCP());
//   #endif

//   if (mqttConnect()) {
//     Serial.println("✅ Attributes sync done");
//   }
// #endif

// #ifdef USE_GSM
//   manageGSMConnectivity();
// #endif

//   hourlyResetSetup();
// }



void setup() {
  Serial.begin(115200);
  initWatchdog();
  LCDsetup();
  StatusDisplay();
  initSD();

  // ---- Run WiFi Config Portal (always on reset) ----
  setupConfig();

  // ---- Continue with system init ----
  xTaskCreatePinnedToCore(wifiConnection, "wifi_Connection", 10000, NULL, 0, &BackgroundProcessor, 0);
  initGpsHandler();
  initSettings();
  initCalibration();
  setupSensors();

  rtcTimeOk = loadRtcTimeOk();
  if (rtcTimeOk) Serial.println("RTC is SET");
  else Serial.println("RTC is NOT SET");

  for (int i = 0; i < 10; i++) {
    freqHz = getAvgFrequency_EMA(15, 0.15);
  }

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
}






/*
void setup() {
  Serial.begin(115200);
  initWatchdog();
  LCDsetup();
  StatusDisplay();
  initSD();
  setupConfig();
  xTaskCreatePinnedToCore(wifiConnection, "wifi_Connection", 10000, NULL, 0, &BackgroundProcessor, 0);
  initGpsHandler();
  initSettings();
  initCalibration();
  setupSensors();

  rtcTimeOk = loadRtcTimeOk();
  if (rtcTimeOk) Serial.println("RTC is SET");
  else Serial.println("RTC is NOT SET");

  for (int i = 0; i < 10; i++) {
    freqHz = getAvgFrequency_EMA(15, 0.15);
  }

#ifdef USE_WIFI
  #ifdef USE_DHCP
    // DHCP → WiFi + MQTT
    while (!connectWiFiDHCP());
    if (mqttConnect()) {
      Serial.println("✅ Attributes sync done");
    }

  #elif defined(USE_STATIC_IP)
    // Static → WiFi + Modbus only, no MQTT
    while (!connectWiFiStatic());
    Serial.println("🚫 MQTT skipped (Static IP mode)");

  #else
    // Default → DHCP
    while (!connectWiFiDHCP());
    if (mqttConnect()) {
      Serial.println("✅ Attributes sync done");
    }
  #endif
#endif

#ifdef USE_GSM
  manageGSMConnectivity();
#endif

  hourlyResetSetup();
}*/

unsigned long tGsm;
unsigned long lastTelemetryTime;
bool startupTelemetry = true;

// void loop() {

//   updateUltrasonicDistance();

//   //float freqHz = getAvgFrequency_SMA();     // Use simple average
//   for (int i = 0; i < 10; i++) {
//     freqHz = getAvgFrequency_EMA(15, 0.15);  // Or test exponential average
//   }
//   Serial.print("Frequency: ");
//   Serial.print(freqHz);

//   unsigned long currentTime = millis();


//   if (currentTime - lastTelemetryTime >= deviceSettings.telemetryInterval * 1000 || startupTelemetry) {
//     lastTelemetryTime = currentTime;  // Update timestamp first
//      if (shouldSendTelemetry()) {
//       Serial.println("----------------Sending telemetry to server-----------------");
//       telemetryLoop();
//     } else {
//       Serial.println("----------------Telemetry skipped (Static IP mode)-----------------");
//       // Optional: Local storage or other actions
//     }
    
//     startupTelemetry = false;
//   }
  
//  delay(100);

//   esp_task_wdt_reset();
// }

void loop() {
    updateUltrasonicDistance();

    // Get frequency (EMA average)
    for (int i = 0; i < 10; i++) {
        freqHz = getAvgFrequency_EMA(15, 0.15);
    }
    Serial.print("Frequency: ");
    Serial.println(freqHz);

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


//     Serial.println("----------------Sending telemetry to server-----------------");
//     telemetryLoop();
//     startupTelemetry = false;
//   }
//   delay(100);
//   esp_task_wdt_reset();  // Feed the watchdog
// }