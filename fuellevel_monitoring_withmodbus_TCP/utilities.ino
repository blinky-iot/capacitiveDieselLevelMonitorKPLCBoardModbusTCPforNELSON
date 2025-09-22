void HourlyResetTask(void* pvParameters) {
  while (true) {
    vTaskDelay(3600000 / portTICK_PERIOD_MS);  // 1 hour delay
    //vTaskDelay(60000 / portTICK_PERIOD_MS);  // 1 minute for testing
    Serial.println("🔁 Resetting ESP32 (hourly task)...");
    delay(100);
    ESP.restart();
  }
}

void hourlyResetSetup() {
  // In setup:
  xTaskCreatePinnedToCore(
    HourlyResetTask, "HourlyReset", 2048, NULL, 1, NULL, 1  // core 1 is typical for background
  );
}

/*
  utilities.ino – LittleFS Config Management for Device Settings
  ------------------------------------------------------------------------------

  Functions:
  - saveConfig(): Saves current device settings to LittleFS
  - readConfig(): Loads device settings from LittleFS (or applies defaults)
  - writeFile(): Utility to write a file to LittleFS
  - readFile(): Reads and returns content of a file as String

  DeviceSettings fields handled:
  - accessToken
  - server
  - port
  - telemetryInterval
  - measurementInterval
  - tankType
  - tankLength, tankWidth, tankHeight, tankRadius
*/

bool saveConfig(const char* filename) {
  StaticJsonDocument<1024> doc;
  doc["accessToken"] = deviceSettings.TOKEN;
  doc["server"] = deviceSettings.SERVER;
  doc["port"] = deviceSettings.port;
  doc["telemetryInterval"] = deviceSettings.telemetryInterval;
  doc["measurementInterval"] = deviceSettings.measurementInterval;
  doc["tankType"] = deviceSettings.tankType;
  doc["tankLength"] = deviceSettings.tankLength;
  doc["tankWidth"] = deviceSettings.tankWidth;
  doc["tankHeight"] = deviceSettings.tankHeight;
  doc["tankRadius"] = deviceSettings.tankRadius;
  doc["sensorType"] = deviceSettings.sensorType;
  doc["ultrasonicOffset"] = deviceSettings.ultrasonicOffset;
  String jsonStr;
  serializeJson(doc, jsonStr);
  writeFile(filename, jsonStr);
  return true;
}

bool readConfig(const char* filename) {
  Serial.println("📂 Reading config file: " + String(filename));

  String content = readFile(filename);
  if (content == "") {
    Serial.println("❌ Config file empty or missing");
    return false;
  }

  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, content);
  if (error) {
    Serial.print("❌ JSON parse error: ");
    Serial.println(error.c_str());
    return false;
  }

  // accessToken
  String tok = doc["accessToken"] | "default_token";
  tok.toCharArray(deviceSettings.TOKEN, sizeof(deviceSettings.TOKEN));
  Serial.println("🔐 accessToken: " + String(deviceSettings.TOKEN));

  // server
  String srv = doc["server"] | "telemetry.blinkelectrics.co.ke";
  srv.toCharArray(deviceSettings.SERVER, sizeof(deviceSettings.SERVER));
  Serial.println("🌐 server: " + String(deviceSettings.SERVER));

  // port
  deviceSettings.port = doc["port"] | 1883;
  Serial.println("🔌 port: " + String(deviceSettings.port));

  // telemetryInterval
  deviceSettings.telemetryInterval = doc["telemetryInterval"] | 60;
  Serial.println("📊 telemetryInterval: " + String(deviceSettings.telemetryInterval));

  // measurementInterval
  deviceSettings.measurementInterval = doc["measurementInterval"] | 30000;
  Serial.println("⏱️ measurementInterval: " + String(deviceSettings.measurementInterval));

  // Tank parameters
  String ttype = doc["tankType"] | "rectangular";
  ttype.toCharArray(deviceSettings.tankType, sizeof(deviceSettings.tankType));
  deviceSettings.tankLength = doc["tankLength"] | 2000.0;
  deviceSettings.tankWidth = doc["tankWidth"] | 2000.0;
  deviceSettings.tankHeight = doc["tankHeight"] | 2000.0;
  deviceSettings.tankRadius = doc["tankRadius"] | 2000.0;



  Serial.println("🛢️ Tank type: " + String(deviceSettings.tankType));
  Serial.println("📐 Dimensions - L: " + String(deviceSettings.tankLength) + " W: " + String(deviceSettings.tankWidth) + " H: " + String(deviceSettings.tankHeight) + " R: " + String(deviceSettings.tankRadius));


  // ✅ New: ultrasonicOffset
  deviceSettings.ultrasonicOffset = doc["ultrasonicOffset"] | 100;
  Serial.println("📏 ultrasonicOffset: " + String(deviceSettings.ultrasonicOffset));

  // ✅ New: sensorType
  String sensorType = doc["sensorType"] | "ultrasonic";
  sensorType.toCharArray(deviceSettings.sensorType, sizeof(deviceSettings.sensorType));
  Serial.println("🔧 sensorType: " + String(deviceSettings.sensorType));


  Serial.println("✅ Config loaded successfully.");
  return true;
}




void writeFile(const char* filename, String message) {
  File file = LittleFS.open(filename, "w");
  if (!file) {
    Serial.println("❌ Failed to open file for writing: " + String(filename));
    return;
  }
  file.print(message);
  file.close();
  Serial.println("💾 File written: " + String(filename));
}

String readFile(const char* filename) {
  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.println("❌ Failed to open file: " + String(filename));
    return "";
  }

  String content = file.readString();
  file.close();
  return content;
}
