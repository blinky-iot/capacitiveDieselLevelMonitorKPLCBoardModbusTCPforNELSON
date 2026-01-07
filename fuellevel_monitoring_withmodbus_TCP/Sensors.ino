
extern String telemetryPayload;
extern void setupUltrasonic();
extern float readUltrasonicCM();


void setupSensors() {
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  rtc.begin();
  //setupPCNT();
  setupADC();
  initValveControl();
  //setupUltrasonic();


  xTaskCreatePinnedToCore(
    SensorReading,
    "SensorTask",
    10000,
    NULL,
    1,
    &Task1,
    1);  // Core 1
}


void SensorReading(void* pvParameters) {
  for (;;) {
    esp_task_wdt_reset();

    // 1. Wait for valid Modbus data
    if (!isModbusDataValid()) {
      Serial.println("⚠️ Waiting for valid Modbus data...");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    DateTime now = rtc.now();
    float rtcTemp = rtc.getTemperature();
    uint64_t tsUtcMs = (uint64_t)now.unixtime() * 1000ULL;

    // 2. GET DATA FROM MODBUS (filtered and stable)
    float modbusTemp = getModbusTemperature();
    uint32_t modbusDist = getModbusDistance();
    uint32_t modbusFreq = getModbusFrequency();

    // 3. Calculate level and volume based on sensor type
    float levelLiters = 0.0;
    int Fill_Percentage = 0;

    if (strcmp(deviceSettings.sensorType, "ultrasonic") == 0) {
      // Ultrasonic sensor - use Modbus distance WITH OFFSET
      height_mm = deviceSettings.tankHeight + deviceSettings.ultrasonicOffset - (float)modbusDist;

      Serial.printf("📡 Ultrasonic (Modbus): %u mm → Height: %.1f mm\n",
                    modbusDist, height_mm);

    } else if (strcmp(deviceSettings.sensorType, "capacitive") == 0) {
      // Capacitive sensor - use Modbus frequency
      height_mm = frequencyToHeight((float)modbusFreq);

      Serial.printf("🧪 Capacitive (Modbus): %u Hz → Height: %.1f mm\n",
                    modbusFreq, height_mm);
    } else {
      Serial.println("❌ Unknown sensor type");
      vTaskDelay(pdMS_TO_TICKS(deviceSettings.measurementInterval));
      continue;
    }

    // 4. Validate height - COMPREHENSIVE VALIDATION
    bool heightValid = true;

    if (height_mm <= 0) {
      Serial.printf("⚠️ Invalid height: %.1f mm\n", height_mm);
      heightValid = false;
      height_mm = 0.0;
    } 
    else if (height_mm > deviceSettings.tankHeight * 1.5) {
      Serial.printf("⚠️ Height exceeds reasonable max: %.1f > %.1f mm\n", 
                    height_mm, deviceSettings.tankHeight * 1.5);
      heightValid = false;
      height_mm = deviceSettings.tankHeight;
    }

    if (height_mm > deviceSettings.tankHeight) {
      Serial.printf("⚠️ Height exceeds tank: %.1f > %.1f, clamping\n",
                    height_mm, deviceSettings.tankHeight);
      height_mm = deviceSettings.tankHeight;
    }

    // 5. Compute volume and percentage with validation
    bool fillPercentValid = false;

    if (height_mm > 0 && heightValid) {
      // Truncate to nearest 10mm for stability
      int height_tens = (int)round(height_mm / 10.0f);
      float truncated_height = height_tens * 10.0f;
      
      levelLiters = calculateVolumeLiters(truncated_height);

      if (deviceSettings.tankHeight > 0) {
        Fill_Percentage = (int)((truncated_height * 100.0f) / deviceSettings.tankHeight);
        
        if (Fill_Percentage >= 0 && Fill_Percentage <= 100) {
          fillPercentValid = true;
        } else {
          Serial.printf("⚠️ Invalid fill percentage: %d%%\n", Fill_Percentage);
          Fill_Percentage = constrain(Fill_Percentage, 0, 100);
        }
      }
      
      Serial.printf("📦 Volume: %.1f L | 📊 Fill: %d%% | Height: %.1f mm\n",
                    levelLiters, Fill_Percentage, truncated_height);
    } else {
      Serial.println("📦 Tank is empty or invalid reading");
      levelLiters = 0.0;
      Fill_Percentage = 0;
    }

    float batteryVoltage = getBatteryVoltage();
    freqHz = (float)modbusFreq;

    // 6. Valve control
    valveState(Fill_Percentage);

    // 7. Create MAIN telemetry JSON
    FuelSensorStatus status = checkFuelSensor((float)modbusFreq);
    const char* statusText[] = { "OK", "DISCONNECTED", "OUT_OF_RANGE" };

    StaticJsonDocument<1024> doc;
    doc["volume"] = roundf(levelLiters * 10) / 10.0;
    doc["voltage"] = roundf(batteryVoltage * 100) / 100.0;
    doc["fuelHz"] = (float)modbusFreq;
    doc["SensorStatus"] = statusText[status];
    doc["Height_cap"] = height_mm;
    doc["Fill_Percentage"] = Fill_Percentage;
    doc["rtcTemp"] = roundf(rtcTemp * 10) / 10.0;
    doc["modbusTemperature"] = modbusTemp;
    doc["modbusDistance"] = modbusDist;
    doc["tankHeight"] = deviceSettings.tankHeight;
    doc["ultrasonicOffset"] = deviceSettings.ultrasonicOffset;

    serializeJson(doc, telemetryPayload);
    Serial.println("📤 Main telemetry updated");

    // 8. Create FILTERED telemetry - ONLY VALID FIELDS
    StaticJsonDocument<1024> filteredDoc;
    
    // Always include these
    filteredDoc["voltage"] = roundf(batteryVoltage * 100) / 100.0;
    filteredDoc["fuelHz"] = (float)modbusFreq;
    filteredDoc["SensorStatus"] = statusText[status];
    filteredDoc["rtcTemp"] = roundf(rtcTemp * 10) / 10.0;
    filteredDoc["modbusTemperature"] = modbusTemp;
    filteredDoc["modbusDistance"] = modbusDist;
    filteredDoc["tankHeight"] = deviceSettings.tankHeight;
    filteredDoc["ultrasonicOffset"] = deviceSettings.ultrasonicOffset;
    filteredDoc["sdCardStatus"] = sdCardStatus;

    // Conditionally include
    if (heightValid && height_mm > 0) {
        filteredDoc["Height_cap"] = height_mm;
    }
    
    if (fillPercentValid) {
        filteredDoc["Fill_Percentage"] = Fill_Percentage;
    }
    
    if (levelLiters > 0.0f && !isnan(levelLiters) && !isinf(levelLiters)) {
        filteredDoc["volume"] = roundf(levelLiters * 10) / 10.0;
    }
    
    if (strcmp(deviceSettings.sensorType, "ultrasonic") == 0) {
        filteredDoc["ultrasonicDistance"] = (float)modbusDist;
        filteredDoc["ultraSonicStatus"] = 1;
    } else {
        filteredDoc["ultraSonicStatus"] = 0;
    }

    // 9. MUTEX: Update filteredPayload safely
    char localBuffer[512];
    serializeJson(filteredDoc, localBuffer, sizeof(localBuffer));
    
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Critical section: Update global variable
        filteredPayload = localBuffer;  // Fast assignment
        
        xSemaphoreGive(telemetryMutex);  // Release immediately
        
        Serial.println("📤 Filtered telemetry updated with mutex: " + String(localBuffer));
    } else {
        Serial.println("⚠️ Couldn't update telemetry (mutex busy - sending in progress)");
        // Telemetry will be sent with slightly older data
    }

    // 10. Log to SD
    if (rtcTimeOk) {
      int ultrasonicStatus = (strcmp(deviceSettings.sensorType, "ultrasonic") == 0) ? 1 : 0;
      logToSD(tsUtcMs, levelLiters, batteryVoltage, (float)modbusFreq,
              status, height_mm, rtcTemp,
              (modbusDist / 1.0f), ultrasonicStatus);
    }

    vTaskDelay(pdMS_TO_TICKS(deviceSettings.measurementInterval));
  }
}


float frequencyToHeight(float freq) {
  freq = constrain(freq, calibrationData.fullHz, calibrationData.emptyHz);
  return (calibrationData.a * freq * freq + calibrationData.b * freq + calibrationData.c);
}


float getLevelLiters() {
  float volume_liters = 0.0;

  // 1. Check Modbus data validity
  if (!isModbusDataValid()) {
    Serial.println("❌ No valid Modbus data available");
    height_mm = 0.0;
    return 0.0;
  }

  // 2. Get data from Modbus
  uint32_t modbusDist = getModbusDistance();   // mm
  uint32_t modbusFreq = getModbusFrequency();  // Hz

  Serial.printf("[Modbus Raw] Distance: %u mm, Frequency: %u Hz\n", modbusDist, modbusFreq);

  // 3. Sensor-specific height calculation
  if (strcmp(deviceSettings.sensorType, "ultrasonic") == 0) {
    // ULTRASONIC SENSOR CALCULATION WITH OFFSET
    Serial.println("📡 Using Ultrasonic Sensor");
    
    // IMPORTANT: With offset
    // Fuel height = Tank height + offset - distance reading
    // This accounts for sensor mounting position
    height_mm = deviceSettings.tankHeight + deviceSettings.ultrasonicOffset - (float)modbusDist;
    
    Serial.printf("  Tank Height: %.1f mm\n", deviceSettings.tankHeight);
    Serial.printf("  Ultrasonic Offset: %d mm\n", deviceSettings.ultrasonicOffset);
    Serial.printf("  Measured Distance: %u mm\n", modbusDist);
    Serial.printf("  Calculated Fuel Height: %.1f mm\n", height_mm);
    
    // Debug: Show the calculation step by step
    Serial.printf("  Calculation: %.1f + %d - %u = %.1f mm\n",
                  deviceSettings.tankHeight,
                  deviceSettings.ultrasonicOffset,
                  modbusDist,
                  height_mm);
    
    // Ensure height is not negative
    if (height_mm < 0) {
      Serial.printf("⚠️ Warning: Negative height (%.1f mm), setting to 0\n", height_mm);
      Serial.println("   Check: tank height, offset, or distance might be wrong");
      height_mm = 0.0;
    }
    
    // Ensure height doesn't exceed tank height
    if (height_mm > deviceSettings.tankHeight) {
      Serial.printf("⚠️ Warning: Height exceeds tank (%.1f > %.1f), clamping\n", 
                    height_mm, deviceSettings.tankHeight);
      height_mm = deviceSettings.tankHeight;
    }
    
  } else if (strcmp(deviceSettings.sensorType, "capacitive") == 0) {
    // CAPACITIVE SENSOR CALCULATION
    Serial.println("🧪 Using Capacitive Sensor");
    
    if (modbusFreq == 0) {
      Serial.println("❌ Invalid frequency: 0 Hz");
      height_mm = 0.0;
      return 0.0;
    }

    height_mm = frequencyToHeight((float)modbusFreq);
    
    if (sensorDisconnected((float)modbusFreq)) {
      Serial.println("❌ Capacitive sensor disconnected");
      height_mm = 0.0;
      return 0.0;
    }
    
    Serial.printf("  Frequency: %u Hz\n", modbusFreq);
    Serial.printf("  Calculated Fuel Height: %.1f mm\n", height_mm);
    
  } else {
    Serial.println("❌ Unknown sensor type: " + String(deviceSettings.sensorType));
    height_mm = 0.0;
    return 0.0;
  }

  // 4. Validate final height
  if (height_mm <= 0 || height_mm > 10000) { // Reasonable max 10m
    Serial.printf("❌ Final height invalid: %.1f mm\n", height_mm);
    height_mm = 0.0;
    return 0.0;
  }

  // 5. Compute volume
  volume_liters = calculateVolumeLiters(height_mm);
  
  if (volume_liters < 0) {
    Serial.println("❌ Calculated volume is negative");
    height_mm = 0.0;
    return 0.0;
  }
  
  Serial.printf("[Fuel] Final Height: %.1f mm → Volume: %.2f L\n", height_mm, volume_liters);
  
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