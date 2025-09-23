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
  xTaskCreatePinnedToCore(wifiConnection, "wifi_Connection", 10000, NULL, 0, &BackgroundProcessor, 0);
  //displayTask();
  initGpsHandler();
  initSettings();
  initCalibration();
  setupSensors();

  rtcTimeOk = loadRtcTimeOk();  // load saved value
  if (rtcTimeOk) Serial.println("RTC is SET");
  else Serial.println("RTC is NOT SET");

  for (int i = 0; i < 10; i++) {
    freqHz = getAvgFrequency_EMA(15, 0.15);
  }

#ifdef USE_WIFI
  // 1️⃣ Step 1 → DHCP WiFi for MQTT + attributes
  while (!connectWiFiDHCP());   // keep trying until connected
  if (mqttConnect()) {
    Serial.println("✅ Attributes sync done");
  }

  // 2️⃣ Step 2 → Restart WiFi with static IP + Modbus
  WiFi.disconnect(true);
  delay(1000);
  while (!connectWiFiStatic()); // keep trying until connected
#endif

#ifdef USE_GSM
  manageGSMConnectivity();
#endif

  hourlyResetSetup();
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
//     freqHz = getAvgFrequency_EMA(15, 0.15);  // Or test exponential average
//   }


// #ifdef USE_WIFI
//   while (!connectWiFi())
//     ;
//   mqttConnect();
// #endif

// #ifdef USE_GSM
//   manageGSMConnectivity();
// #endif
//   hourlyResetSetup();
// }
unsigned long tGsm;
unsigned long lastTelemetryTime;
bool startupTelemetry = true;

void loop() {

  updateUltrasonicDistance();

  //float freqHz = getAvgFrequency_SMA();     // Use simple average
  for (int i = 0; i < 10; i++) {
    freqHz = getAvgFrequency_EMA(15, 0.15);  // Or test exponential average
  }
  Serial.print("Frequency: ");
  Serial.print(freqHz);

  unsigned long currentTime = millis();

  // if (currentTime - lastTelemetryTime >= deviceSettings.telemetryInterval * 1000 || startupTelemetry) {
  //   lastTelemetryTime = currentTime;  // Update timestamp first
  //   Serial.println("----------------Sending telemetry to server-----------------");
  //   telemetryLoop();
  //   startupTelemetry = false;
  // }


  delay(100);

  esp_task_wdt_reset();  // Feed the watchdog
}