
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

  // xTaskCreatePinnedToCore(
  // GpsTask,
  // "GpsTask",
  // 6000,
  // NULL,
  // 1,
  // NULL,
  // 0);  // Core 0


  // xTaskCreatePinnedToCore(
  //   ultrasonic,
  //   "SensorTask",
  //   10000,
  //   NULL,
  //   1,
  //   NULL,
  //   0);  // Core 1
}
// void GpsTask(void* param) {
//   for (;;) {
//     handleGpsSession();              // Must run frequently
//     vTaskDelay(pdMS_TO_TICKS(200));  // 200ms interval is ideal
//   }
// }

// void ultrasonic(void* param) {
//   float x;
//   for (;;) {
//     x = readUltrasonicCM();  // Must run frequently
//     if (x != -1 && x != -2) Height_ultrsnc = x;
//     else Serial.println(x);
//     vTaskDelay(pdMS_TO_TICKS(300));  // 200ms interval is ideal
//   }
// }

// float getAvgFrequency_EMA(uint8_t samples = 15, float alpha = 0.15) {
//   float freq = measureFreqHz();  // Initial value
//   for (uint8_t i = 1; i < samples; ++i) {
//     float newFreq = measureFreqHz();
//     freqHz = alpha * newFreq + (1 - alpha) * freqHz;  // EMA formula
//     vTaskDelay(pdMS_TO_TICKS(4));
//   }
//   return freqHz;
// }

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


// void SensorReading(void* pvParameters) {
//   for (;;) {
//     esp_task_wdt_reset();

//     // 1. Wait for valid Modbus data
//     if (!isModbusDataValid()) {
//       Serial.println("⚠️ Waiting for valid Modbus data...");
//       vTaskDelay(pdMS_TO_TICKS(1000));
//       continue;
//     }

//     DateTime now = rtc.now();
//     float rtcTemp = rtc.getTemperature();
//     uint64_t tsUtcMs = (uint64_t)now.unixtime() * 1000ULL;

//     // 2. GET DATA FROM MODBUS (filtered and stable)
//     float modbusTemp = getModbusTemperature();   // Already filtered on slave
//     uint32_t modbusDist = getModbusDistance();   // Already filtered on slave
//     uint32_t modbusFreq = getModbusFrequency();  // Already filtered on slave

//     // 3. Calculate level and volume
//     float levelLiters = 0.0;
//     int Fill_Percentage = 0;

//     if (strcmp(deviceSettings.sensorType, "ultrasonic") == 0) {
//       // Ultrasonic sensor - use Modbus distance
//       height_mm = deviceSettings.tankHeight + deviceSettings.ultrasonicOffset - (modbusDist / 10.0f);

//       Serial.printf("📡 Ultrasonic (Modbus): %.1f mm → Height: %.1f mm\n",
//                     modbusDist, height_mm);

//     } else if (strcmp(deviceSettings.sensorType, "capacitive") == 0) {
//       // Capacitive sensor - use Modbus frequency
//       height_mm = frequencyToHeight((float)modbusFreq);

//       Serial.printf("🧪 Capacitive (Modbus): %u Hz → Height: %.1f mm\n",
//                     modbusFreq, height_mm);

//       // Check sensor status
//       if (sensorDisconnected((float)modbusFreq)) {
//         Serial.println("❌ Capacitive sensor disconnected");
//         height_mm = 0;
//       }
//     } else {
//       Serial.println("❌ Unknown sensor type");
//       vTaskDelay(pdMS_TO_TICKS(deviceSettings.measurementInterval));
//       continue;
//     }

//     // 4. Compute volume and percentage
//     if (height_mm > 0) {
//       levelLiters = calculateVolumeLiters(height_mm);

//       if (deviceSettings.tankHeight > 0) {
//         Fill_Percentage = (int)((height_mm * 100.0f) / deviceSettings.tankHeight);
//         Fill_Percentage = constrain(Fill_Percentage, 0, 100);
//       }
//     }



//     // if (height_mm > 0) {
//     //   // Get the tens digit (792.2105103 → 79)
//     //   int height_tens = (int)(height_mm / 10.0f);

//     //   // If calculateVolumeLiters expects mm, multiply by 10
//     //   float height_for_calculation = height_tens * 10.0f;  // 790.0

//     //   levelLiters = calculateVolumeLiters(height_for_calculation);

//     //   if (deviceSettings.tankHeight > 0) {
//     //     Fill_Percentage = (int)((height_for_calculation * 100.0f) / deviceSettings.tankHeight);
//     //     Fill_Percentage = constrain(Fill_Percentage, 0, 100);
//     //   }

//     //   // Optional: Update display with tens value
//     //   Serial.printf("Height: %d tens (%.1f mm)\n", height_tens, height_for_calculation);
//     // }



//     float batteryVoltage = getBatteryVoltage();

//     // Update global frequency variable (if still used elsewhere)
//     freqHz = (float)modbusFreq;

//     Serial.printf("📦 Volume: %.1f L | 📊 Fill: %d%% | Height: %.1f mm\n",
//                   levelLiters, Fill_Percentage, height_mm);

//     // 5. Valve control
//     valveState(Fill_Percentage);

//     // 6. JSON telemetry
//     FuelSensorStatus status = checkFuelSensor((float)modbusFreq);
//     const char* statusText[] = { "OK", "DISCONNECTED", "OUT_OF_RANGE" };

//     StaticJsonDocument<1024> doc;
//     doc["volume"] = roundf(levelLiters * 10) / 10.0;
//     doc["voltage"] = roundf(batteryVoltage * 100) / 100.0;
//     doc["fuelHz"] = (float)modbusFreq;  // Filtered frequency from Modbus
//     doc["SensorStatus"] = statusText[status];
//     doc["Height_cap"] = height_mm;
//     doc["Fill_Percentage"] = Fill_Percentage;
//     doc["rtcTemp"] = roundf(rtcTemp * 10) / 10.0;
//     doc["modbusTemperature"] = modbusTemp;  // Temperature from Modbus
//     doc["modbusDistance"] = modbusDist;     // Distance from Modbus

//     serializeJson(doc, telemetryPayload);
//     Serial.println("📤 Telemetry updated (using Modbus data)");

//     // 7. Filtered telemetry (remove empty fields)
//     StaticJsonDocument<1024> filteredDoc;
//     filteredDoc.set(doc);
//     if (levelLiters < 0.0f) filteredDoc.remove("volume");
//     filteredDoc["sdCardStatus"] = sdCardStatus;

//     serializeJson(filteredDoc, filteredPayload);
//     Serial.println("📤 Filtered telemetry: " + filteredPayload);

//     // 8. Log to SD
//     if (rtcTimeOk) {
//       // Assuming you have ultrasonicSensorStatus variable
//       int ultrasonicStatus = (strcmp(deviceSettings.sensorType, "ultrasonic") == 0) ? 1 : 0;

//       logToSD(tsUtcMs, levelLiters, batteryVoltage, (float)modbusFreq,
//               status, height_mm, rtcTemp,
//               (modbusDist / 10.0f), ultrasonicStatus);
//     }

//     vTaskDelay(pdMS_TO_TICKS(deviceSettings.measurementInterval));
//   }
// }
/*
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
      Serial.printf("  Tank: %.1f + Offset: %d - Distance: %u = %.1f mm\n",
                    deviceSettings.tankHeight,
                    deviceSettings.ultrasonicOffset,
                    modbusDist,
                    height_mm);

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

    // 4. Validate height
    if (height_mm <= 0) {
      Serial.printf("⚠️ Invalid height: %.1f mm, setting to 0\n", height_mm);
      height_mm = 0.0;
    }
    
    if (height_mm > deviceSettings.tankHeight) {
      Serial.printf("⚠️ Height exceeds tank: %.1f > %.1f, clamping\n",
                    height_mm, deviceSettings.tankHeight);
      height_mm = deviceSettings.tankHeight;
    }

    // 5. Compute volume and percentage
    if (height_mm > 0) {
      // Truncate to nearest 10mm for stability
      int height_tens = (int)round(height_mm / 10.0f);
      float truncated_height = height_tens * 10.0f;
      
      levelLiters = calculateVolumeLiters(truncated_height);

      if (deviceSettings.tankHeight > 0) {
        Fill_Percentage = (int)((truncated_height * 100.0f) / deviceSettings.tankHeight);
        Fill_Percentage = constrain(Fill_Percentage, 0, 100);
      }
      
      Serial.printf("📦 Volume: %.1f L | 📊 Fill: %d%% | Height: %.1f mm (truncated from %.1f)\n",
                    levelLiters, Fill_Percentage, truncated_height, height_mm);
    } else {
      Serial.println("📦 Tank is empty");
      levelLiters = 0.0;
      Fill_Percentage = 0;
    }

    float batteryVoltage = getBatteryVoltage();

    // Update global frequency variable (if still used elsewhere)
    freqHz = (float)modbusFreq;

    // 6. Valve control
    valveState(Fill_Percentage);

    // 7. JSON telemetry
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
    doc["tankHeight"] = deviceSettings.tankHeight;  // For debugging
    doc["ultrasonicOffset"] = deviceSettings.ultrasonicOffset;  // For debugging

    serializeJson(doc, telemetryPayload);
    Serial.println("📤 Telemetry updated (using Modbus data)");

    // 8. Filtered telemetry
    StaticJsonDocument<1024> filteredDoc;
    filteredDoc.set(doc);
    if (levelLiters < 0.0f) filteredDoc.remove("volume");
    filteredDoc["sdCardStatus"] = sdCardStatus;

    serializeJson(filteredDoc, filteredPayload);
    Serial.println("📤 Filtered telemetry: " + filteredPayload);

    // 9. Log to SD
    if (rtcTimeOk) {
      int ultrasonicStatus = (strcmp(deviceSettings.sensorType, "ultrasonic") == 0) ? 1 : 0;
      logToSD(tsUtcMs, levelLiters, batteryVoltage, (float)modbusFreq,
              status, height_mm, rtcTemp,
              (modbusDist / 1.0f), ultrasonicStatus);
    }

    vTaskDelay(pdMS_TO_TICKS(deviceSettings.measurementInterval));
  }
}
*/


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





/*
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



// void setupPCNT() {
//   pcnt_config_t cfg = {
//     .pulse_gpio_num = PIN_FREQ,
//     .ctrl_gpio_num = PCNT_PIN_NOT_USED,
//     .lctrl_mode = PCNT_MODE_KEEP,
//     .hctrl_mode = PCNT_MODE_KEEP,
//     .pos_mode = PCNT_COUNT_INC,
//     .neg_mode = PCNT_COUNT_DIS,
//     .counter_h_lim = 32767,
//     .counter_l_lim = 0,
//     .unit = PCNT_UNIT,
//     .channel = PCNT_CH
//   };
//   pcnt_unit_config(&cfg);
//   pcnt_counter_clear(PCNT_UNIT);
//   pcnt_counter_resume(PCNT_UNIT);
// }
// float measureFreqHz() {
//   setupPCNT();
//   pcnt_counter_clear(PCNT_UNIT);
//   delay(PCNT_GATE_MS);
//   int16_t pulses = 0;
//   pcnt_get_counter_value(PCNT_UNIT, &pulses);
//   return (pulses * 1000.0f) / PCNT_GATE_MS;
// }

// double frequencyHz;

// float getAvgFrequency() {
//   float sum = 0;
//   for (uint8_t i = 0; i < NUM_SAMPLES; ++i) {
//     sum += measureFreqHz();
//     delay(2);
//   }
//   return sum / NUM_SAMPLES;
// }
*/

float frequencyToHeight(float freq) {
  freq = constrain(freq, calibrationData.fullHz, calibrationData.emptyHz);
  return (calibrationData.a * freq * freq + calibrationData.b * freq + calibrationData.c);
}

// float getLevelLiters() {
//   float volume_liters = 0.0;
//   float freq = 0.0;
//   height_mm = 0;

//   if (strcmp(deviceSettings.sensorType, "ultrasonic") == 0) {
//     // Ultrasonic sensor
//     if (ultrasonicSensorStatus == 1) {
//       height_mm = deviceSettings.tankHeight + deviceSettings.ultrasonicOffset - Height_ultrsnc;
//       Serial.println("📡 Using ultrasonic sensor");
//       Serial.printf("🟢 Ultrasonic height: %.1f mm (offset: %d mm)\n", height_mm, deviceSettings.ultrasonicOffset);
//     }
//   } else if (strcmp(deviceSettings.sensorType, "capacitive") == 0) {
//     // Capacitive sensor
//     //freq = getAvgFrequency();
//     if (freqHz == 0) {
//       return 0.0;
//     } else height_mm = frequencyToHeight(freqHz);

//     if (sensorDisconnected(freqHz)) {
//       Serial.println("❌ Capacitive sensor disconnected");
//       return 0.0;
//     }

//     Serial.println("🧪 Using capacitive sensor");
//     Serial.printf("🔵 Frequency: %.2f Hz | Height: %.1f mm\n", freqHz, height_mm);

//   } else {
//     // Unsupported sensor type
//     Serial.println("❌ Unknown sensor type: " + String(deviceSettings.sensorType));
//     return 0.0;
//   }

//   // Compute volume
//   volume_liters = calculateVolumeLiters(height_mm);
//   Serial.printf("[Fuel] Volume: %.2f L\n", volume_liters);
//   return volume_liters;
// }


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




/*
float getLevelLiters() {
  float volume_liters = 0.0;

  // 1. Check Modbus data validity
  if (!isModbusDataValid()) {
    Serial.println("❌ No valid Modbus data available");
    return 0.0;
  }

  // 2. Get data from Modbus
  uint32_t modbusDist = getModbusDistance();   // mm
  uint32_t modbusFreq = getModbusFrequency();  // Hz

  // 3. Sensor-specific height calculation
  if (strcmp(deviceSettings.sensorType, "ultrasonic") == 0) {
    // Ultrasonic sensor
    if (ultrasonicSensorStatus == 1) {
      height_mm = deviceSettings.tankHeight + deviceSettings.ultrasonicOffset - (modbusDist / 10.0f);
      Serial.println("📡 Using ultrasonic sensor (from Modbus)");
      Serial.printf("🟢 Ultrasonic height: %.1f mm (raw: %u mm, offset: %d mm)\n",
                    height_mm, modbusDist, deviceSettings.ultrasonicOffset);
    }
  } else if (strcmp(deviceSettings.sensorType, "capacitive") == 0) {
    // Capacitive sensor
    if (modbusFreq == 0) {
      return 0.0;
    }

    // Use filtered frequency from Modbus
    height_mm = frequencyToHeight((float)modbusFreq);

    if (sensorDisconnected((float)modbusFreq)) {
      Serial.println("❌ Capacitive sensor disconnected");
      return 0.0;
    }

    Serial.println("🧪 Using capacitive sensor (from Modbus)");
    Serial.printf("🔵 Frequency: %u Hz | Height: %.1f mm\n", modbusFreq, height_mm);
  } else {
    Serial.println("❌ Unknown sensor type: " + String(deviceSettings.sensorType));
    return 0.0;
  }

  // 4. Compute volume
  volume_liters = calculateVolumeLiters(height_mm);
  Serial.printf("[Fuel] Volume: %.2f L\n", volume_liters);
  return volume_liters;
}*/




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