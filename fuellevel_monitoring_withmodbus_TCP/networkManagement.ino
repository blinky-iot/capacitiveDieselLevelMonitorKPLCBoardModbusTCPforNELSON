

bool detectAndLockBaudRate() {
  Serial.println("🔍 Starting auto-baud detection...");

  // 1. Detect working baud using TinyGsmAutoBaud
  uint32_t detectedBaud = TinyGsmAutoBaud(SerialAT, 4800, 115200);

  if (detectedBaud == 0) {
    Serial.println("❌ No working baud rate detected.");
    return false;
  }

  Serial.println(String("✅ Detected baud rate: ") + detectedBaud);

  // 2. Set fixed baud rate to 115200 and save it
  SerialAT.println("AT+IPR=115200");  // Set baud
  delay(200);
  SerialAT.println("AT&W");  // Save to flash
  delay(200);

  // 3. Restart SerialAT at 115200
  SerialAT.end();
  delay(100);
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(300);

  // 4. Confirm modem is still responsive
  SerialAT.println("AT");
  if (SerialAT.find("OK")) {
    Serial.println("📡 Baud locked and confirmed at 115200.");
    return true;
  } else {
    Serial.println("⚠️ Baud set to 115200 but modem not responding.");
    return false;
  }
}

bool powerOnModem() {
#ifdef blinkBoard
  pinMode(MODEM_RST, OUTPUT);
  pinMode(PIN_SIM_PWR, OUTPUT);
  digitalWrite(PIN_SIM_PWR, HIGH);
  digitalWrite(MODEM_RST, LOW);
  delay(100);
  digitalWrite(MODEM_RST, HIGH);
  delay(5000);  // Allow modem to boot up
#endif

#ifdef powWaterBoard
  pinMode(MODEM_RST, OUTPUT);
  pinMode(PIN_SIM_PWR, OUTPUT);
  digitalWrite(PIN_SIM_PWR, HIGH);
  digitalWrite(MODEM_RST, HIGH);
  delay(100);
  digitalWrite(MODEM_RST, LOW);
  delay(5000);  // Allow modem to boot up
#endif

  if (detectAndLockBaudRate()) {
    Serial.println("🚀 Modem ready at 115200.");

    // 🟢 Attempt to unlock SIM card with PIN
    if (modem.simUnlock("6638")) {
      Serial.println("🔓 SIM unlocked successfully.");
    } else {
      Serial.println("🔐 SIM unlock failed or not required.");
    }

    return true;
  } else {
    Serial.println("💥 Failed to initialize modem.");
    return false;
  }
}


void powerDownModem() {
  pinMode(PIN_SIM_PWR, OUTPUT);
  digitalWrite(PIN_SIM_PWR, LOW);
  Serial.println("-----------------------GSM POWER OFF-----------------");
}


// --- GSM MQTT Maintenance Function
bool manageGSMConnectivity() {
  SerialMon.println("🔄 Starting GSM Connectivity Check...");

  SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);  // Give time for modem to wake

  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 10000;
  static bool modemInitialized = false;
  static bool onStartup = true;

  if (millis() - lastCheck > checkInterval || onStartup) {
    lastCheck = millis();
    onStartup = false;
  } else {
    SerialMon.println("⏳ Skipping GSM check. Waiting for next interval...");
    return false;
  }

  // (Re)initialize modem if needed
  if (!modemInitialized || !modem.isNetworkConnected()) {
    SerialMon.println("⚠️ GSM not connected or modem not initialized. Restarting modem...");

    powerOnModem();
    SerialMon.println("🔌 Powering on modem...");

    modemInitialized = true;

    if (!modem.waitForNetwork(60000L)) {
      SerialMon.println("❌ GSM Network not found within timeout.");
      modemInitialized = false;
      return false;
    }

    SerialMon.println("✅ GSM Network Connected");


    syncRtcOnce();  //update RTC date and time via network

  } else {
    SerialMon.println("📶 GSM already connected to network.");
  }

  // 2. GPRS check
  if (!modem.isGprsConnected()) {
    Serial.println("🌐 GPRS not connected. Attempting to reconnect...");

    const int gprsMaxRetries = 3;
    int gprsAttempts = 0;
    bool gprsConnected = false;

    while (gprsAttempts < gprsMaxRetries) {
      gprsAttempts++;
      Serial.print("🔁 GPRS attempt ");
      Serial.print(gprsAttempts);
      Serial.println("...");

      if (modem.gprsConnect(apn, gprsUser, gprsPass)) {
        gprsConnected = true;
        break;
      }

      delay(3000);  // wait before retry
    }

    if (!gprsConnected) {
      Serial.println("❌ GPRS connection failed after retries.");
      return false;
    }

    Serial.println("✅ GPRS connected.");
  }


  // MQTT connection
  if (ensureMQTT()) return true;
  else return false;

  mqttClient.loop();  // Always call loop to maintain MQTT connection
}

// --- MQTT Connection with Retry
bool ensureMQTT() {
  const int maxRetries = 5;
  const int retryDelay = 2000;  // milliseconds

  if (mqttClient.connected()) {
    SerialMon.println("📨 MQTT already connected.");
    return true;
  }

  SerialMon.println("🔌 MQTT not connected. Attempting reconnect...");

  for (int attempt = 1; attempt <= maxRetries; attempt++) {
    SerialMon.printf("🔁 MQTT connect attempt %d of %d...\n", attempt, maxRetries);
    if (mqttConnect()) {
      SerialMon.println("✅ MQTT connected successfully.");
      return true;
    } else {
      SerialMon.println("⚠️ MQTT connect failed.");
      delay(retryDelay);
    }
  }

  SerialMon.println("❌ MQTT connection failed after retries.");
  return false;
}

// --- WiFi Connection with 30s Timeout
bool connectWiFi() {
  WiFi.begin(ssid, password);
  networkConnected = false;
  Serial.print("Connecting to WiFi");

  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 30000;  // 30 seconds

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);

    if (millis() - startAttemptTime > timeout) {
      Serial.println("\n❌ WiFi connection timed out");
      return false;
    }
  }

  networkConnected = true;
  Serial.println("\n✅ WiFi connected");
  Serial.print("📡 IP address: ");
  Serial.println(WiFi.localIP());
  return true;
  // Start Modbus TCP server
  mb.server();

  // Add 10 Holding Registers
  for (int i = 0; i < REG_COUNT; i++) {
    mb.addHreg(REG_BASE + i);
    mb.Hreg(REG_BASE + i, i);  // Optional: preset values 0–9
  }
}

String interpretMqttState(int state) {
  switch (state) {
    case -4: return "Connection Timeout";
    case -3: return "Connection Lost";
    case -2: return "Connect Failed";
    case -1: return "Disconnected";
    case 0: return "Connected";
    case 1: return "Bad Protocol";
    case 2: return "Bad Client ID";
    case 3: return "Unavailable";
    case 4: return "Bad Credentials";
    case 5: return "Unauthorized";
    default: return "Unknown Error";
  }
}

/*
  ------------------------------------------------------------------------------
  mqttConnect() – MQTT Broker Connection Handler
  ------------------------------------------------------------------------------

  Description:
  This function manages the connection to an MQTT broker using credentials
  provided via the global `deviceSettings` structure.

  It attempts to establish a connection with the broker and sets the MQTT
  callback function for handling incoming messages.

  ------------------------------------------------------------------------------
  Function Behavior:
  - Initializes connection parameters (host, port, token)
  - Repeatedly attempts to connect to the broker (inside a `while` loop)
  - If connection is successful:
      - Logs success message to Serial
      - Sets `mqttConnected = true`
      - Triggers `requestSharedAttributes()` to fetch initial shared config
      - Returns `true`
  - If connection fails:
      - Logs the failure state using `mqttClient.state()`
      - Sets `mqttConnected = false`
      - Returns `false` immediately (no retry delay active)

  ------------------------------------------------------------------------------
  Dependencies:
  - `PubSubClient` for MQTT connection management
  - `deviceSettings` object must contain:
      - `SERVER` (String/IP of MQTT broker)
      - `port` (int)
      - `TOKEN` (auth token for ThingsBoard or similar)
  - `callback` function for message handling must be defined elsewhere
  - `requestSharedAttributes()` should be implemented to fetch shared attributes

  ------------------------------------------------------------------------------
  Global Variables Modified:
  - `mqttConnected` – Boolean flag representing connection status

  ------------------------------------------------------------------------------
  Suggested Improvements:
  - Add exponential backoff or retry delay logic for persistent retry
  - Improve security by using SSL/TLS if supported by the broker
  - Parameterize client ID or generate dynamically

  ------------------------------------------------------------------------------
*/

bool mqttConnect() {
  mqttConnected = false;
  mqttClient.setServer(deviceSettings.SERVER, deviceSettings.port);
  mqttClient.setCallback(callback);

  while (!mqttClient.connected()) {
    Serial.print("🔌 Connecting to MQTT broker at ");
    Serial.print(deviceSettings.SERVER);
    Serial.print(":");
    Serial.println(deviceSettings.port);

    if (mqttClient.connect("Client", deviceSettings.TOKEN, NULL)) {
      Serial.println("✅ MQTT connected");
      mqttConnected = true;

      // Request shared attributes
      requestSharedAttributes(mqttAttributes, 1);
      delay(500);
      requestSharedAttributes(tankAttributes, 2);
      delay(500);
      requestSharedAttributes(sensorAttributes, 3);
      return true;
    } else {
      int state = mqttClient.state();
      Serial.print("❌ MQTT connect failed. State [");
      Serial.print(state);
      Serial.print("]: ");
      Serial.println(interpretMqttState(state));

      mqttConnected = false;
      delay(2000);   // wait before retrying
      return false;  // remove this return if you want retries
    }
  }

  return false;  // fallback
}


// --- Request Shared Attributes

/*
  ------------------------------------------------------------------------------
  requestSharedAttributes() – Fetch Shared Attributes via MQTT
  ------------------------------------------------------------------------------

  Description:
  This function sends a request to the ThingsBoard platform (or compatible MQTT
  broker) to retrieve shared device attributes. These attributes may include
  configuration values such as:
    - Access token
    - Critical thresholds
    - Server address and port
    - Telemetry interval

  ------------------------------------------------------------------------------
  Behavior:
  1. Subscribes to two MQTT topics to receive attribute responses:
      - `v1/devices/me/attributes/response/+` for specific responses
      - `v1/devices/me/attributes` for general updates
  2. Constructs a JSON payload specifying the shared keys to request.
  3. Publishes the request to the topic:
      - `v1/devices/me/attributes/request/1`
  4. Waits up to 10 seconds (non-blocking loop) for the attributes response
      by continuously calling `mqttClient.loop()`.

  ------------------------------------------------------------------------------
  MQTT Topics:
  - **Subscribe**:
      - `v1/devices/me/attributes/response/+`
      - `v1/devices/me/attributes`
  - **Publish**:
      - `v1/devices/me/attributes/request/1` with payload like:
        `{"sharedKeys":"accessToken,High_Critical_Value,Low_Critical_Value,port,server,telemetryInterval"}`

  ------------------------------------------------------------------------------
  Output:
  - Logs success or failure of the publish operation to the serial console
  - Prints a message indicating that attribute request was sent

  ------------------------------------------------------------------------------
  Dependencies:
  - Requires an active MQTT connection (`mqttClient.connected() == true`)
  - `mqttClient` must be an instance of `PubSubClient` or compatible library
  - Shared attributes must be configured on the ThingsBoard device

  ------------------------------------------------------------------------------
  Suggestions:
  - Consider adding a timeout callback or flag to indicate success/failure
    in receiving attributes
  - Abstract the list of requested keys into a configurable variable or list

  ------------------------------------------------------------------------------
*/

void requestSharedAttributes(String keys, int requestId) {
  // Subscribe to response topics
  mqttClient.subscribe("v1/devices/me/attributes/response/+");

  // Build the request payload
  String payload = "{\"sharedKeys\":\"" + keys + "\"}";
  String topic = "v1/devices/me/attributes/request/" + String(requestId);

  // Send the request
  if (mqttClient.publish(topic.c_str(), payload.c_str())) {
    Serial.print("📡 Request sent on ID ");
    Serial.println(requestId);
  } else {
    Serial.print("❌ Request failed on ID ");
    Serial.println(requestId);
  }

  // Let the client listen for incoming response
  unsigned long start = millis();
  while (millis() - start < 5000) {
    mqttClient.loop();
  }
}

// --- Callback to handle attribute response

/*
  ------------------------------------------------------------------------------
  callback() – MQTT Attribute Response Handler
  ------------------------------------------------------------------------------

  Description:
  This function is triggered when an MQTT message is received on subscribed
  attribute topics. It is designed to process shared attribute updates sent
  from the ThingsBoard platform (or similar MQTT server) and apply them to the
  device’s runtime configuration.

  ------------------------------------------------------------------------------
  Topics Handled:
  - `v1/devices/me/attributes/response/+` (direct response to attribute request)
  - `v1/devices/me/attributes` (general shared attribute updates)

  ------------------------------------------------------------------------------
  Behavior:
  1. Converts the received payload (byte array) to a JSON string.
  2. Deserializes the JSON and extracts shared attributes, either flat or nested.
  3. Compares each attribute with the currently saved value.
  4. If differences are found:
     - Updates in-memory config values in `deviceSettings`
     - Flags `updated = true`
     - Flags `mqttNeedsRestart = true` for access token, server, or port changes
  5. If any config was updated:
     - Calls `saveConfig()` to persist changes to file
     - Calls `ESP.restart()` if a critical MQTT setting changed

  ------------------------------------------------------------------------------
  Attributes Handled:
  - `accessToken` → Updates `deviceSettings.TOKEN`
  - `server`      → Updates `deviceSettings.SERVER`
  - `port`        → Updates `deviceSettings.port`
  - `telemetryInterval` → Updates telemetry frequency in seconds
  - `type of the tank cylindlical or rectangular for cilindlical we have to use radius and height
  if its rectangular  we have to have length ,width and height remember height is the one we are converting the frequency

  ------------------------------------------------------------------------------
  Dependencies:
  - Global variables:
    - `deviceSettings` (configuration structure)
    - `high_critical_value`, `low_critical_value` (thresholds)
    - `config_filename` (used in saveConfig)
  - External functions:
    - `saveConfig(filename)` → saves updated config to persistent storage
  - Libraries:
    - ArduinoJson
    - PubSubClient (MQTT library)
    - ESP.restart()

  ------------------------------------------------------------------------------
  Notes:
  - The function handles both flat and nested JSON (e.g., wrapped under "shared")
  - Handles both numeric and string representations of numbers
  - Prints out all actions to Serial for debugging and traceability

  ------------------------------------------------------------------------------
  Suggestions:
  - Add fallback or retry logic for `saveConfig()` failure
  - Use a non-blocking approach for restart (e.g., scheduling via flag)
  - Add validation (e.g., value ranges) before applying attribute updates

  ------------------------------------------------------------------------------
*/

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("📥 MQTT Callback Triggered");
  Serial.print("📨 Topic: ");
  Serial.println(topic);

  String json;
  for (unsigned int i = 0; i < length; i++) {
    json += (char)payload[i];
  }
  Serial.print("📨 Payload: ");
  Serial.println(json);

  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.print("❌ JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  JsonObject root = doc.as<JsonObject>();
  JsonObject shared = root.containsKey("shared") ? root["shared"].as<JsonObject>() : root;

  bool updated = false;
  bool restartNeeded = false;

  //Handle Calibration from attributes
  calibrateFromAttributes(shared);

  // --- Access Token
  if (shared.containsKey("accessToken")) {
    String val = shared["accessToken"];
    if (val != String(deviceSettings.TOKEN)) {
      val.toCharArray(deviceSettings.TOKEN, sizeof(deviceSettings.TOKEN));
      updated = true;
      restartNeeded = true;
      Serial.println("🔐 Updated accessToken");
    }
  }

  // --- Server
  if (shared.containsKey("server")) {
    String val = shared["server"];
    if (val != String(deviceSettings.SERVER)) {
      val.toCharArray(deviceSettings.SERVER, sizeof(deviceSettings.SERVER));
      updated = true;
      restartNeeded = true;
      Serial.println("🌐 Updated server");
    }
  }

  // --- Port
  if (shared.containsKey("port")) {
    int newPort = shared["port"].as<int>();
    if (newPort != deviceSettings.port && newPort > 0) {
      deviceSettings.port = newPort;
      updated = true;
      restartNeeded = true;
      Serial.println("🔌 Updated port");
    }
  }

  // --- Telemetry Interval
  if (shared.containsKey("telemetryInterval")) {
    int newVal = shared["telemetryInterval"].as<int>();
    if (newVal != deviceSettings.telemetryInterval && newVal > 0) {
      deviceSettings.telemetryInterval = newVal;
      updated = true;
      Serial.println("📊 Updated telemetryInterval");
    }
  }

  // --- Measurement Interval
  if (shared.containsKey("measurementInterval")) {
    int newVal = shared["measurementInterval"].as<int>();
    if (newVal != deviceSettings.measurementInterval && newVal > 0) {
      deviceSettings.measurementInterval = newVal;
      updated = true;
      Serial.println("🧪 Updated measurementInterval (SAMPLE_INT_MS)");
    }
  }

  // --- Tank Type
  if (shared.containsKey("tankType")) {
    String t = shared["tankType"];
    if (t != String(deviceSettings.tankType)) {
      t.toCharArray(deviceSettings.tankType, sizeof(deviceSettings.tankType));
      updated = true;
      Serial.println("🏷️ Updated tankType: " + String(deviceSettings.tankType));
    }
  }

  // --- Tank Dimensions
  if (shared.containsKey("radius")) {
    deviceSettings.tankRadius = shared["radius"].as<float>();
    Serial.println("⚙️ radius: " + String(deviceSettings.tankRadius));
    updated = true;
  }

  if (shared.containsKey("height")) {
    deviceSettings.tankHeight = shared["height"].as<float>();
    Serial.println("⚙️ height: " + String(deviceSettings.tankHeight));
    updated = true;
  }

  if (shared.containsKey("length")) {
    deviceSettings.tankLength = shared["length"].as<float>();
    Serial.println("📏 length: " + String(deviceSettings.tankLength));
    updated = true;
  }

  if (shared.containsKey("width")) {
    deviceSettings.tankWidth = shared["width"].as<float>();
    Serial.println("📐 width: " + String(deviceSettings.tankWidth));
    updated = true;
  }

  // --- Save + Restart if needed
  if (updated) {
    Serial.println("💾 Saving updated attributes...");
    saveConfig(config_filename);
    if (restartNeeded) {
      Serial.println("🔄 Restarting due to MQTT critical updates...");
      delay(100);
      ESP.restart();
    } else {
      Serial.println("✅ Attributes updated, no restart required");
    }
  } else {
    Serial.println("✔️ Callback Exited");
  }
  //powerDownModem();
}


bool publishTelemetry(const String& jsonPayload) {
  if (!mqttClient.connected()) {
    SerialMon.println("❌ MQTT not connected. Telemetry not sent.");
    return false;
  }

  SerialMon.println("📤 Attempting to publish telemetry:");
  SerialMon.println("Length: " + String(jsonPayload.length()));

  if (jsonPayload.length() == 0) {
    SerialMon.println("❌ Payload is EMPTY. Nothing to send.");
    return false;
  }

  // Try parsing to ensure it's valid JSON (debug only)
  StaticJsonDocument<512> testDoc;
  DeserializationError err = deserializeJson(testDoc, jsonPayload);
  if (err) {
    SerialMon.print("❌ Payload is not valid JSON: ");
    SerialMon.println(err.c_str());
  } else {
    SerialMon.println("✅ Payload is valid JSON.");
  }

  bool success = mqttClient.publish("v1/devices/me/telemetry", jsonPayload.c_str());

  if (success) {
    SerialMon.println("✅ Telemetry published successfully.");
  } else {
    SerialMon.println("❌ Failed to publish telemetry.");
  }

  return success;
}

bool sendGsmTelemetry() {
  if (!mqttClient.connected()) {
    SerialMon.println("❌ MQTT not connected. Cannot send GSM telemetry.");
    return false;
  }

  // 📇 Modem Identifiers
  String ccid = modem.getSimCCID();
  String imei = modem.getIMEI();

  // 🔋 Battery Info
  int8_t chargeState = 0;
  int8_t batteryPercent = 0;
  int16_t batteryVoltage_mV = 0;
  float batteryVoltage = 0.0f;

  if (modem.getBattStats(chargeState, batteryPercent, batteryVoltage_mV)) {
    batteryVoltage = batteryVoltage_mV / 1000.0f;
  } else {
    SerialMon.println("⚠️ Failed to read battery stats.");
  }

  // 📶 Signal Quality (0–31 RSSI)
  int signalQuality = modem.getSignalQuality();

  // 📍 Last known GPS fix from SD
  GpsFix lastFix;
  bool hasFix = getLastGpsFromSD(lastFix);

  // 📦 Prepare telemetry JSON
  StaticJsonDocument<384> doc;
  doc["ccid"] = ccid;
  doc["imei"] = imei;
  doc["GSMbatteryV"] = roundf(batteryVoltage * 100) / 100.0f;
  doc["GSMbatteryLevel"] = batteryPercent;
  doc["rssi"] = signalQuality;

  if (hasFix) {
    doc["lat"] = roundf(lastFix.lat * 1000000) / 1000000.0;
    doc["lon"] = roundf(lastFix.lon * 1000000) / 1000000.0;
    doc["gpsTs"] = lastFix.timestamp;
  } else {
    SerialMon.println("⚠️ No GPS fix found in SD card.");
  }

  String json;
  serializeJson(doc, json);

  SerialMon.println("📡 Sending GSM telemetry:");
  SerialMon.println(json);

  // 🚀 Publish to ThingsBoard
  if (mqttClient.publish("v1/devices/me/telemetry", json.c_str())) {
    SerialMon.println("✅ GSM telemetry published successfully.");
    return true;
  } else {
    SerialMon.println("❌ Failed to publish GSM telemetry.");
    return false;
  }
}

// --- Loop
/*
  ------------------------------------------------------------------------------
  telemetryLoop() – Periodic MQTT Telemetry Publisher
  ------------------------------------------------------------------------------

  Description:
  This function is designed to be called repeatedly from the main loop.
  It handles maintaining network connectivity (Wi-Fi or GSM) and ensures
  that telemetry is sent at a defined interval via MQTT.

  ------------------------------------------------------------------------------
  Dependencies:
  - Global:
    - `deviceSettings.telemetryInterval` – defines telemetry interval in seconds
    - `telemetryPayload` – preformatted JSON string (from sensor reading task)
    - `mqttClient` – an instance of PubSubClient
  - Functions:
    - `maintainWiFiConnection()` – ensures Wi-Fi remains connected (if enabled)
    - `maintainGSMConnectivity()` – ensures GSM remains connected (if enabled)
    - `mqttConnect()` – attempts to (re)connect to the MQTT broker

  ------------------------------------------------------------------------------
  Behavior:
  1. Checks connectivity based on the active connection mode:
     - If `USE_WIFI` is defined, calls `maintainWiFiConnection()`
     - If `USE_GSM` is defined, calls `maintainGSMConnectivity()`
  2. If the time since `lastSent` exceeds the configured telemetry interval:
     - Checks if the MQTT client is connected
       - If yes and the telemetry payload is non-empty (`{}` or larger), publishes to:
         `v1/devices/me/telemetry`
       - If not connected, attempts to reconnect using `mqttConnect()`
     - Updates `lastSent` with the current timestamp (from `millis()`)

  3. Calls `mqttClient.loop()` every cycle to ensure MQTT events are processed

  ------------------------------------------------------------------------------
  Timing:
  - Uses `millis()` and `lastSent` to determine when to send telemetry
  - Telemetry is sent every `deviceSettings.telemetryInterval * 1000` ms

  ------------------------------------------------------------------------------
  Global Variables:
  - `unsigned long lastSent` – timestamp (in ms) of the last successful telemetry push

  ------------------------------------------------------------------------------
  Notes:
  - Avoids sending empty JSON payloads (`{}`) by checking payload length
  - Must be called inside the `loop()` function for continuous operation
  - Designed for use in systems that publish telemetry via MQTT to platforms
    like ThingsBoard, Home Assistant, or custom brokers

  ------------------------------------------------------------------------------
*/
void telemetryLoop() {
  bool netwokConnected = false;

#ifdef USE_WIFI
  netwokConnected = maintainWiFiConnection();
#endif

#ifdef USE_GSM
  netwokConnected = manageGSMConnectivity();
#endif
  if (netwokConnected) {
    publishTelemetry(filteredPayload);
    sendTelemetryFromSD();
    if (!gsmTelemetrySent && mqttClient.connected()) {
      if (sendGsmTelemetry()) {
        gsmTelemetrySent = true;
      }
    }
    mqttClient.loop();
    delay(2000);
  }
  powerDownModem();
}
bool maintainWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi disconnected. Reconnecting...");
    if (connectWiFi()) {
      // MQTT connection
      if (ensureMQTT()) return true;
      else {
        return false;
      }
    }
  }
}

// void sendTelemetryFromSD() {
//   bool allSent = true;
//   if (!sdReady) {
//     SerialMon.println("❌ SD not ready. Cannot send telemetry.");
//     return;
//   }

//   File file = SD.open("/telemetry.csv", FILE_READ);
//   if (!file) {
//     SerialMon.println("❌ Failed to open telemetry.csv");
//     return;
//   }

//   SerialMon.println("📤 Sending telemetry from SD (adjusting from UTC+3 to UTC)...");

//   bool isFirstLine = true;
//   while (file.available()) {
//     String line = file.readStringUntil('\n');
//     line.trim();
//     if (line.length() == 0) continue;

//     // ⏭️ Skip CSV header if present
//     if (isFirstLine) {
//       isFirstLine = false;
//       if (line.startsWith("timestamp")) continue;
//     }

//     uint64_t ts_local = 0;
//     float volume = 0.0, voltage = 0.0, freq = 0.0, height_mm = 0.0, rtcTemp = 0.0, Height_ultrsnc = 0.0, ultrasonicSensorStatus = 0.0;
//     char status[20] = { 0 };  // buffer for SensorStatus

//     // Updated: parse 7 values including temperature
//     //int matches = sscanf(line.c_str(), "%llu,%f,%f,%f,%19[^,],%f,%f,%f,%f", &ts_local, &volume, &voltage, &freq, status, &height_mm, &rtcTemp,&Height_ultrsnc,ultrasonicSensorStatus);
//     int matches = sscanf(line.c_str(), "%llu,%f,%f,%f,%19[^,],%f,%f,%f,%f",
//                          &ts_local, &volume, &voltage, &freq,
//                          status, &height_mm, &rtcTemp, &Height_ultrsnc, &ultrasonicSensorStatus);

//     if (matches != 9) {
//       SerialMon.print("⚠️ Expected 9 values, got ");
//       SerialMon.print(matches);
//       SerialMon.print(" → ");
//       SerialMon.println(line);
//       continue;
//     }

//     if (matches == 9) {
//       uint64_t ts_utc = ts_local - (3ULL * 60 * 60 * 1000);  // adjust to UTC

//       String payload = "{";
//       payload += "\"ts\":" + String(ts_utc) + ",";
//       payload += "\"values\":{";
//       payload += "\"volume\":" + String(volume, 2) + ",";
//       payload += "\"voltage\":" + String(voltage, 2) + ",";
//       payload += "\"fuelFrequency\":" + String(freq, 2) + ",";
//       payload += "\"SensorStatus\":\"" + String(status) + "\",";
//       payload += "\"Height\":" + String(height_mm) + ",";
//       payload += "\"rtcTemp\":" + String(rtcTemp, 1);

//       // Only add Height_ultrsnc if non-zero
//       if (Height_ultrsnc > 0.0f) {
//         payload += ",";
//         payload += "\"Height_ultrsnc\":" + String(Height_ultrsnc);
//       }

//       // Always add ultraSonicStatus (conditionally append comma if Height_ultrsnc was not added)
//       payload += ",";
//       payload += "\"ultraSonicStatus\":" + String(ultrasonicSensorStatus);

//       payload += "}}";  // Close JSON

//       // Send it
//       if (!publishTelemetry(payload)) {
//         allSent = false;  // At least one line failed
//       }

//     } else {
//       SerialMon.print("⚠️ Malformed line: ");
//       SerialMon.println(line);
//     }


//     delay(250);  // optional delay to avoid spamming broker
//   }

//   file.close();
//   SerialMon.println("✅ Done sending telemetry from SD");

//   if (allSent) {
//     if (SD.remove("/telemetry.csv")) {
//       SerialMon.println("🧹 Deleted telemetry.csv after successful sending");
//     } else {
//       SerialMon.println("⚠️ Could not delete telemetry.csv");
//     }
//   } else {
//     SerialMon.println("⚠️ Not all telemetry was sent. File not deleted.");
//   }
// }


void sendTelemetryFromSD() {
  bool allSent = true;
  if (!sdReady) {
    SerialMon.println("❌ SD not ready. Cannot send telemetry.");
    return;
  }

  File file = SD.open("/telemetry.csv", FILE_READ);
  if (!file) {
    SerialMon.println("❌ Failed to open telemetry.csv");
    return;
  }

  SerialMon.println("📤 Sending telemetry from SD (adjusting from UTC+3 to UTC)...");

  bool isFirstLine = true;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    if (isFirstLine) {
      isFirstLine = false;
      if (line.startsWith("timestamp")) continue;
    }

    uint64_t ts_local = 0;
    float volume = 0.0, voltage = 0.0, freq = 0.0, height_mm = 0.0, rtcTemp = 0.0, Height_ultrsnc = 0.0, ultrasonicSensorStatus = 0.0;
    char status[20] = { 0 };

    int matches = sscanf(line.c_str(), "%llu,%f,%f,%f,%19[^,],%f,%f,%f,%f",
                         &ts_local, &volume, &voltage, &freq,
                         status, &height_mm, &rtcTemp, &Height_ultrsnc, &ultrasonicSensorStatus);

    if (matches != 9) {
      SerialMon.print("⚠️ Expected 9 values, got ");
      SerialMon.print(matches);
      SerialMon.print(" → ");
      SerialMon.println(line);
      continue;
    }

    uint64_t ts_utc = ts_local - (3ULL * 60 * 60 * 1000);  // adjust to UTC

    String payload = "{";
    payload += "\"ts\":" + String(ts_utc) + ",";
    payload += "\"values\":{";

    // Add volume only if greater than 0
    bool valueStarted = false;
    if (volume > 0.0f) {
      payload += "\"volume\":" + String(volume, 2);
      valueStarted = true;
    }

    // Voltage
    if (valueStarted) payload += ",";
    payload += "\"voltage\":" + String(voltage, 2);
    valueStarted = true;

    // Fuel Frequency
    payload += ",";
    payload += "\"fuelFrequency\":" + String(freq, 2);

    // Sensor Status
    payload += ",";
    payload += "\"SensorStatus\":\"" + String(status) + "\"";

    // Height
    payload += ",";
    payload += "\"Height_cap\":" + String(height_mm);

    // RTC Temp
    payload += ",";
    payload += "\"rtcTemp\":" + String(rtcTemp, 1);

    // Optional: Ultrasonic height
    if (Height_ultrsnc > 0.0f) {
      payload += ",";
      payload += "\"Height_ultrsnc\":" + String(Height_ultrsnc);
    }

    // Always add ultrasonicSensorStatus
    payload += ",";
    payload += "\"ultraSonicStatus\":" + String(ultrasonicSensorStatus);

    payload += "}}";  // Close JSON

    if (!publishTelemetry(payload)) {
      allSent = false;
    }

    delay(250);  // optional delay
  }

  file.close();
  SerialMon.println("✅ Done sending telemetry from SD");

  if (allSent) {
    if (SD.remove("/telemetry.csv")) {
      SerialMon.println("🧹 Deleted telemetry.csv after successful sending");
    } else {
      SerialMon.println("⚠️ Could not delete telemetry.csv");
    }
  } else {
    SerialMon.println("⚠️ Not all telemetry was sent. File not deleted.");
  }
}
