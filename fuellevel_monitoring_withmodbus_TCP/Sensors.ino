
extern String telemetryPayload;
extern void setupUltrasonic();
extern float readUltrasonicCM();


void setupSensors() {
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  rtc.begin();
  setupPCNT();
  setupADC();
  initValveControl();
  setupUltrasonic();
  

  xTaskCreatePinnedToCore(
    SensorReading,
    "SensorTask",
    10000,
    NULL,
    1,
    &Task1,
    1);  // Core 1

  xTaskCreatePinnedToCore(
    GpsTask,
    "GpsTask",
    6000,
    NULL,
    1,
    NULL,
    0);  // Core 0


  // xTaskCreatePinnedToCore(
  //   ultrasonic,
  //   "SensorTask",
  //   10000,
  //   NULL,
  //   1,
  //   NULL,
  //   0);  // Core 1
}
void GpsTask(void* param) {
  for (;;) {
    handleGpsSession();              // Must run frequently
    vTaskDelay(pdMS_TO_TICKS(200));  // 200ms interval is ideal
  }
}

void ultrasonic(void* param) {
  float x;
  for (;;) {
    x = readUltrasonicCM();  // Must run frequently
    if (x != -1 && x != -2) Height_ultrsnc = x;
    else Serial.println(x);
    vTaskDelay(pdMS_TO_TICKS(300));  // 200ms interval is ideal
  }
}

float getAvgFrequency_EMA(uint8_t samples = 15, float alpha = 0.15) {
  float freq = measureFreqHz();  // Initial value
  for (uint8_t i = 1; i < samples; ++i) {
    float newFreq = measureFreqHz();
    freqHz = alpha * newFreq + (1 - alpha) * freqHz;  // EMA formula
    vTaskDelay(pdMS_TO_TICKS(4));
  }
  return freqHz;
}

// void SensorReading(void* pvParameters) {
//   for (;;) {
//     esp_task_wdt_reset();

//     DateTime now = rtc.now();
//     float rtcTemp = rtc.getTemperature();
//     uint64_t tsUtcMs = (uint64_t)now.unixtime() * 1000ULL;

//     float levelLiters = getLevelLiters();
//     float batteryVoltage = getBatteryVoltage();

//     Serial.print("Frequency: ");
//     Serial.print(freqHz);


//     FuelSensorStatus status = checkFuelSensor(freqHz);
//     const char* statusText[] = { "OK", "DISCONNECTED", "OUT_OF_RANGE" };

//     // 📦 Create full JSON document
//     StaticJsonDocument<1024> doc;
//     doc["volume"] = roundf(levelLiters * 10) / 10.0;
//     doc["voltage"] = roundf(batteryVoltage * 100) / 100.0;
//     doc["fuelHz"] = freqHz;
//     doc["SensorStatus"] = statusText[status];
//     doc["Height_cap"] = height_mm;
//     doc["rtcTemp"] = roundf(rtcTemp * 10) / 10.0;
//     doc["ultrasonicDistance"] = Height_ultrsnc;
//     doc["ultraSonicStatus"] = ultrasonicSensorStatus;

//     // // 💾 Log to SD only after RTC is synced
//     // if (rtcTimeOk) {
//     //   logToSD(tsUtcMs, levelLiters, batteryVoltage, freqHz, status, height_mm, rtcTemp, Height_ultrsnc, ultrasonicSensorStatus);
//     // } else {
//     //   Serial.println("⏳ Waiting for RTC time sync...");
//     // }


//     // 📝 Serialize full payload
//     serializeJson(doc, telemetryPayload);
//     Serial.println("📤 Telemetry updated (full): " + telemetryPayload);

//     // 🧹 Create filtered payload without ultrasonic fields if distance == 0
//     StaticJsonDocument<1024> filteredDoc;
//     filteredDoc.set(doc);  // deep copy

//     if (Height_ultrsnc == 0.0f) {
//       filteredDoc.remove("ultrasonicDistance");
//     }
//     if (levelLiters < 0.0f) {
//       filteredDoc.remove("volume");
//     }
//     filteredDoc["sdCardStatus"] = sdCardStatus;


//     serializeJson(filteredDoc, filteredPayload);
//     Serial.println("📤 Filtered telemetry: " + filteredPayload);

//     // // ⏱️ Wait for next cycle
//     vTaskDelay(pdMS_TO_TICKS(deviceSettings.measurementInterval));
//   }
// }

void SensorReading(void* pvParameters) {
  for (;;) {
    esp_task_wdt_reset();

    DateTime now = rtc.now();
    float rtcTemp = rtc.getTemperature();
    uint64_t tsUtcMs = (uint64_t)now.unixtime() * 1000ULL;

    float levelLiters = getLevelLiters();   // updates height_mm
    float batteryVoltage = getBatteryVoltage();

    // --- Calculate fill percentage ---
    int Fill_Percentage = 0;
    if (deviceSettings.tankHeight > 0) {
      Fill_Percentage = (int)((height_mm * 100.0f) / deviceSettings.tankHeight);
      if (Fill_Percentage > 100) Fill_Percentage = 100;
      if (Fill_Percentage < 0)   Fill_Percentage = 0;
    }

    Serial.printf("📦 Volume: %.1f L | 📊 Fill: %d%% | Height: %.1f mm\n",
                  levelLiters, Fill_Percentage, height_mm);

    // --- Valve control here ---
    valveState(Fill_Percentage);

    // --- JSON telemetry ---
    FuelSensorStatus status = checkFuelSensor(freqHz);
    const char* statusText[] = { "OK", "DISCONNECTED", "OUT_OF_RANGE" };

    StaticJsonDocument<1024> doc;
    doc["volume"] = roundf(levelLiters * 10) / 10.0;
    doc["voltage"] = roundf(batteryVoltage * 100) / 100.0;
    doc["fuelHz"] = freqHz;
    doc["SensorStatus"] = statusText[status];
    doc["Height_cap"] = height_mm;
    doc["Fill_Percentage"] = Fill_Percentage;   // ✅ add it to telemetry
    doc["rtcTemp"] = roundf(rtcTemp * 10) / 10.0;
    doc["ultrasonicDistance"] = Height_ultrsnc;
    doc["ultraSonicStatus"] = ultrasonicSensorStatus;

    serializeJson(doc, telemetryPayload);
    Serial.println("📤 Telemetry updated (full): " + telemetryPayload);

    // Filtered telemetry
    StaticJsonDocument<1024> filteredDoc;
    filteredDoc.set(doc);
    if (Height_ultrsnc == 0.0f) filteredDoc.remove("ultrasonicDistance");
    if (levelLiters < 0.0f)     filteredDoc.remove("volume");
    filteredDoc["sdCardStatus"] = sdCardStatus;

    serializeJson(filteredDoc, filteredPayload);
    Serial.println("📤 Filtered telemetry: " + filteredPayload);

    vTaskDelay(pdMS_TO_TICKS(deviceSettings.measurementInterval));
  }
}






void setupPCNT() {
  pcnt_config_t cfg = {
    .pulse_gpio_num = PIN_FREQ,
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,
    .counter_h_lim = 32767,
    .counter_l_lim = 0,
    .unit = PCNT_UNIT,
    .channel = PCNT_CH
  };
  pcnt_unit_config(&cfg);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
}
float measureFreqHz() {
  setupPCNT();
  pcnt_counter_clear(PCNT_UNIT);
  delay(PCNT_GATE_MS);
  int16_t pulses = 0;
  pcnt_get_counter_value(PCNT_UNIT, &pulses);
  return (pulses * 1000.0f) / PCNT_GATE_MS;
}

double frequencyHz;

float getAvgFrequency() {
  float sum = 0;
  for (uint8_t i = 0; i < NUM_SAMPLES; ++i) {
    sum += measureFreqHz();
    delay(2);
  }
  return sum / NUM_SAMPLES;
}

float frequencyToHeight(float freq) {
  freq = constrain(freq, calibrationData.fullHz, calibrationData.emptyHz);
  return (calibrationData.a * freq * freq + calibrationData.b * freq + calibrationData.c);
}

float getLevelLiters() {
  float volume_liters = 0.0;
  float freq = 0.0;
  height_mm = 0;

  if (strcmp(deviceSettings.sensorType, "ultrasonic") == 0) {
    // Ultrasonic sensor
    if (ultrasonicSensorStatus == 1) {
      height_mm = deviceSettings.tankHeight + deviceSettings.ultrasonicOffset - Height_ultrsnc;
      Serial.println("📡 Using ultrasonic sensor");
      Serial.printf("🟢 Ultrasonic height: %.1f mm (offset: %d mm)\n", height_mm, deviceSettings.ultrasonicOffset);
    }
  } else if (strcmp(deviceSettings.sensorType, "capacitive") == 0) {
    // Capacitive sensor
    //freq = getAvgFrequency();
    if (freqHz == 0) {
      return 0.0;
    } else height_mm = frequencyToHeight(freqHz);

    if (sensorDisconnected(freqHz)) {
      Serial.println("❌ Capacitive sensor disconnected");
      return 0.0;
    }

    Serial.println("🧪 Using capacitive sensor");
    Serial.printf("🔵 Frequency: %.2f Hz | Height: %.1f mm\n", freqHz, height_mm);

  } else {
    // Unsupported sensor type
    Serial.println("❌ Unknown sensor type: " + String(deviceSettings.sensorType));
    return 0.0;
  }

  // Compute volume
  volume_liters = calculateVolumeLiters(height_mm);
  Serial.printf("[Fuel] Volume: %.2f L\n", volume_liters);
  return volume_liters;
}

void setupADC() {
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ADC_BAT, ADC_11db);
}

float interpolateVoltage(int adcValue) {
  // Below lowest or above highest known point
  if (adcValue >= adcTable[0]) return voltageTable[0];
  if (adcValue <= adcTable[numPoints - 1]) return voltageTable[numPoints - 1];

  // Linear interpolation
  for (int i = 0; i < numPoints - 1; i++) {
    if (adcValue <= adcTable[i] && adcValue >= adcTable[i + 1]) {
      float slope = (voltageTable[i + 1] - voltageTable[i]) / (adcTable[i + 1] - adcTable[i]);
      return voltageTable[i] + slope * (adcValue - adcTable[i]);
    }
  }

  return 0.0;  // Fallback
}

float getBatteryVoltage() {
  int adcReading = analogRead(35);  // Replace with your ADC input
  float voltage = interpolateVoltage(adcReading);

  Serial.print("ADC: ");
  Serial.print(adcReading);
  Serial.print(" -> Voltage: ");
  Serial.println(voltage, 1);

  return voltage;
}

bool sensorDisconnected(float frequency) {
  float lowerBound = calibrationData.sensorDisconnectedHz * 0.90;
  float upperBound = calibrationData.sensorDisconnectedHz * 1.10;
  return (frequency >= lowerBound && frequency <= upperBound);
}

// FuelSensorStatus checkFuelSensor(float freqHz) {
//   const float margin = 0.20;
//   float disconnectThreshold = calibrationData.sensorDisconnectedHz * (1.0 + margin);

//   if (freqHz < disconnectThreshold) return SENSOR_DISCONNECTED;
//   if (freqHz > calibrationData.fullHz || freqHz < calibrationData.emptyHz) return SENSOR_OUT_OF_RANGE;
//   return SENSOR_OK;
// }
FuelSensorStatus checkFuelSensor(float freqHz) {
  const float disconnectMargin = 0.20;  // 20% margin for disconnection
  const float rangeMargin = 0.01;       // 1% tolerance on full/empty range

  float disconnectThreshold = calibrationData.sensorDisconnectedHz * (1.0 + disconnectMargin);

  if (freqHz < disconnectThreshold) return SENSOR_DISCONNECTED;

  float low = calibrationData.fullHz * (1.0 - rangeMargin);    // slightly below full
  float high = calibrationData.emptyHz * (1.0 + rangeMargin);  // slightly above empty

  if (freqHz < low || freqHz > high) return SENSOR_OUT_OF_RANGE;

  return SENSOR_OK;
}

void sendFuelSensorStatus(float freqHz) {
  FuelSensorStatus status = checkFuelSensor(freqHz);
  StaticJsonDocument<128> doc;
  const char* statusText[] = { "OK", "DISCONNECTED", "OUT_OF_RANGE" };

  doc["fuelSensorStatus"] = statusText[status];
  doc["fuelFrequency"] = freqHz;

  String payload;
  serializeJson(doc, payload);

  if (mqttClient.connected()) {
    mqttClient.publish("v1/devices/me/telemetry", payload.c_str());
    Serial.println("📤 Fuel sensor status sent: " + payload);
  } else {
    Serial.println("❌ MQTT not connected. Fuel sensor status not sent.");
  }
}