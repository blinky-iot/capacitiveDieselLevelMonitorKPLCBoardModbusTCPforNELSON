// === Fit y = ax^2 + bx + c from 3 points ===
void calibrateFromThreePoints(Point p1, Point p2, Point p3, double &a, double &b, double &c) {
  double A[3][4] = {
    { p1.x * p1.x, p1.x, 1, p1.y },
    { p2.x * p2.x, p2.x, 1, p2.y },
    { p3.x * p3.x, p3.x, 1, p3.y }
  };

  gaussJordan(A);

  a = A[0][3];
  b = A[1][3];
  c = A[2][3];
}

// === Evaluate the polynomial for a given frequency ===
float calculateDieselHeight(unsigned long freq) {
  return a * freq * freq + b * freq + c;
}

// === Gauss-Jordan elimination for solving 3x3 ===
void gaussJordan(double m[3][4]) {
  for (int i = 0; i < 3; i++) {
    double diag = m[i][i];
    for (int j = 0; j < 4; j++) m[i][j] /= diag;

    for (int k = 0; k < 3; k++) {
      if (k != i) {
        double factor = m[k][i];
        for (int j = 0; j < 4; j++) {
          m[k][j] -= factor * m[i][j];
        }
      }
    }
  }
}


void requestCalibrationAttributes() {
  // mqttClient.subscribe("v1/devices/me/attributes/response/+");
  // mqttClient.subscribe("v1/devices/me/attributes");

  String keys = "emptyHz,emptyDistance,midHz,midDistance,fullHz,fullDistance";
  String payload = "{\"sharedKeys\":\"" + keys + "\"}";

  if (mqttClient.publish("v1/devices/me/attributes/request/1", payload.c_str())) {
    Serial.println("📡 Requested calibration attributes");
  } else {
    Serial.println("❌ Calibration attribute request failed");
  }

  // Optional: Wait up to 10s for the response (not strictly required)
  unsigned long start = millis();
  while (millis() - start < 10000) mqttClient.loop();
}




// bool   calibrateFromAttributes(JsonObject &shared) {
//   if (shared.containsKey("emptyHz") && shared.containsKey("emptyDistance") && shared.containsKey("midHz") && shared.containsKey("midDistance") && shared.containsKey("fullHz") && shared.containsKey("fullDistance")) {

//     Point p1 = { shared["emptyHz"].as<double>(), shared["emptyDistance"].as<double>() };
//     Point p2 = { shared["midHz"].as<double>(), shared["midDistance"].as<double>() };
//     Point p3 = { shared["fullHz"].as<double>(), shared["fullDistance"].as<double>() };

//     calibrationData.sensorDisconnectedHz = shared["sensorDisconnectedHz"].as<double>();

//     calibrateFromThreePoints(p1, p2, p3, a, b, c);

//     // Round b and c to 6 decimal places
//     double roundedB = round(b * 1000000.0) / 1000000.0;
//     double roundedC = round(c * 1000000.0) / 1000000.0;

//     Serial.println("🚀 Calibration Polynomial:");
//     Serial.printf("y = %.10f * x^2 + %.6f * x + %.6f\n", a, roundedB, roundedC);

//     // Test outputs
//     Serial.printf("📐 H1 = %.2f mm | H2 = %.2f mm | H3 = %.2f mm\n",
//                   a * p1.x * p1.x + b * p1.x + c,
//                   a * p2.x * p2.x + b * p2.x + c,
//                   a * p3.x * p3.x + b * p3.x + c);

//     bool changed = false;

//     // Raw calibration points
//     long newEmptyHz = shared["emptyHz"];
//     long newEmptyDistance = shared["emptyDistance"];
//     long newMidHz = shared["midHz"];
//     long newMidDistance = shared["midDistance"];
//     long newFullHz = shared["fullHz"];
//     long newFullDistance = shared["fullDistance"];
//     long newSensorDisconnectedHz = shared["sensorDisconnectedHz"];

//     // Check and update only if different
//     if (newEmptyHz != calibrationData.emptyHz) {
//       calibrationData.emptyHz = newEmptyHz;
//       changed = true;
//     }
//     if (newEmptyDistance != calibrationData.emptyDistance) {
//       calibrationData.emptyDistance = newEmptyDistance;
//       changed = true;
//     }
//     if (newMidHz != calibrationData.midHz) {
//       calibrationData.midHz = newMidHz;
//       changed = true;
//     }
//     if (newMidDistance != calibrationData.midDistance) {
//       calibrationData.midDistance = newMidDistance;
//       changed = true;
//     }
//     if (newFullHz != calibrationData.fullHz) {
//       calibrationData.fullHz = newFullHz;
//       changed = true;
//     }
//     if (newFullDistance != calibrationData.fullDistance) {
//       calibrationData.fullDistance = newFullDistance;
//       changed = true;
//     }

//     if (newSensorDisconnectedHz != calibrationData.sensorDisconnectedHz) {
//       calibrationData.sensorDisconnectedHz = newSensorDisconnectedHz;
//       changed = true;
//     }

//     const double epsilon = 0.000001;  // For 6 decimal places

//     bool coeffChanged = fabs(a - calibrationData.a) > epsilon || fabs(roundedB - calibrationData.b) > epsilon || fabs(roundedC - calibrationData.c) > epsilon;

//     if (coeffChanged) {
//       calibrationData.a = a;
//       calibrationData.b = roundedB;
//       calibrationData.c = roundedC;
//       changed = true;
//     }


//     // Save only if something changed
//     if (changed) {
//       saveCalibrationData(calibration_filename);
//       Serial.println("📁 Calibration file updated");
//     } else {
//       Serial.println("📂 Calibration unchanged — no write needed");
//     }

//     return true;
//   } else {
//     //Serial.println("⚠️ Calibration fields missing from shared attributes.");
//     return false;
//   }
// }

// bool calibrateFromAttributes(JsonObject &shared) {
//   if (!shared.containsKey("sensorType")) {
//     Serial.println("⚠️ Missing sensorType attribute");
//     return false;
//   }

//   String sensorType = shared["sensorType"].as<String>();

//   if (sensorType == "capacitive") {
//     if (shared.containsKey("emptyHz") && shared.containsKey("emptyDistance") &&
//         shared.containsKey("midHz") && shared.containsKey("midDistance") &&
//         shared.containsKey("fullHz") && shared.containsKey("fullDistance")) {

//       Point p1 = { shared["emptyHz"].as<double>(), shared["emptyDistance"].as<double>() };
//       Point p2 = { shared["midHz"].as<double>(), shared["midDistance"].as<double>() };
//       Point p3 = { shared["fullHz"].as<double>(), shared["fullDistance"].as<double>() };

//       calibrationData.sensorDisconnectedHz = shared["sensorDisconnectedHz"].as<double>();

//       calibrateFromThreePoints(p1, p2, p3, a, b, c);

//       // Round b and c to 6 decimal places
//       double roundedB = round(b * 1000000.0) / 1000000.0;
//       double roundedC = round(c * 1000000.0) / 1000000.0;

//       Serial.println("🚀 Calibration Polynomial:");
//       Serial.printf("y = %.10f * x^2 + %.6f * x + %.6f\n", a, roundedB, roundedC);

//       // Test outputs
//       Serial.printf("📐 H1 = %.2f mm | H2 = %.2f mm | H3 = %.2f mm\n",
//                     a * p1.x * p1.x + b * p1.x + c,
//                     a * p2.x * p2.x + b * p2.x + c,
//                     a * p3.x * p3.x + b * p3.x + c);

//       bool changed = false;

//       // Raw calibration points
//       long newEmptyHz = shared["emptyHz"];
//       long newEmptyDistance = shared["emptyDistance"];
//       long newMidHz = shared["midHz"];
//       long newMidDistance = shared["midDistance"];
//       long newFullHz = shared["fullHz"];
//       long newFullDistance = shared["fullDistance"];
//       long newSensorDisconnectedHz = shared["sensorDisconnectedHz"];

//       // Check and update only if different
//       if (newEmptyHz != calibrationData.emptyHz) {
//         calibrationData.emptyHz = newEmptyHz;
//         changed = true;
//       }
//       if (newEmptyDistance != calibrationData.emptyDistance) {
//         calibrationData.emptyDistance = newEmptyDistance;
//         changed = true;
//       }
//       if (newMidHz != calibrationData.midHz) {
//         calibrationData.midHz = newMidHz;
//         changed = true;
//       }
//       if (newMidDistance != calibrationData.midDistance) {
//         calibrationData.midDistance = newMidDistance;
//         changed = true;
//       }
//       if (newFullHz != calibrationData.fullHz) {
//         calibrationData.fullHz = newFullHz;
//         changed = true;
//       }
//       if (newFullDistance != calibrationData.fullDistance) {
//         calibrationData.fullDistance = newFullDistance;
//         changed = true;
//       }

//       if (newSensorDisconnectedHz != calibrationData.sensorDisconnectedHz) {
//         calibrationData.sensorDisconnectedHz = newSensorDisconnectedHz;
//         changed = true;
//       }

//       const double epsilon = 0.000001;

//       bool coeffChanged = fabs(a - calibrationData.a) > epsilon ||
//                           fabs(roundedB - calibrationData.b) > epsilon ||
//                           fabs(roundedC - calibrationData.c) > epsilon;

//       if (coeffChanged) {
//         calibrationData.a = a;
//         calibrationData.b = roundedB;
//         calibrationData.c = roundedC;
//         changed = true;
//       }

//       if (changed) {
//         saveCalibrationData(calibration_filename);
//         Serial.println("📁 Calibration file updated");
//       } else {
//         Serial.println("📂 Calibration unchanged — no write needed");
//       }

//       return true;
//     } else {
//       Serial.println("⚠️ Capacitive calibration fields missing.");
//       return false;
//     }

//   } else if (sensorType == "ultrasonic") {
//     Serial.println("📦 Ultrasonic calibration logic not implemented yet.");
//     // TODO: Implement ultrasonic calibration when available
//     return false;

//   } else {
//     Serial.println("❌ Unknown sensorType: " + sensorType);
//     return false;
//   }
// }


bool calibrateFromAttributes(JsonObject &shared) {
  if (!shared.containsKey("sensorType")) {
    //Serial.println("⚠️ Missing sensorType attribute");
    return false;
  }

  String sensorTypeStr = shared["sensorType"].as<String>();
  sensorTypeStr.toLowerCase();  // Optional normalization

  // ✅ Assign to global struct
  strncpy(deviceSettings.sensorType, sensorTypeStr.c_str(), sizeof(deviceSettings.sensorType) - 1);
  deviceSettings.sensorType[sizeof(deviceSettings.sensorType) - 1] = '\0';  // Ensure null termination

  if (sensorTypeStr == "capacitive") {
    if (shared.containsKey("emptyHz") && shared.containsKey("emptyDistance") && shared.containsKey("midHz") && shared.containsKey("midDistance") && shared.containsKey("fullHz") && shared.containsKey("fullDistance")) {

      Point p1 = { shared["emptyHz"].as<double>(), shared["emptyDistance"].as<double>() };
      Point p2 = { shared["midHz"].as<double>(), shared["midDistance"].as<double>() };
      Point p3 = { shared["fullHz"].as<double>(), shared["fullDistance"].as<double>() };

      calibrationData.sensorDisconnectedHz = shared["sensorDisconnectedHz"].as<double>();

      calibrateFromThreePoints(p1, p2, p3, a, b, c);

      // Round b and c to 6 decimal places
      double roundedB = round(b * 1000000.0) / 1000000.0;
      double roundedC = round(c * 1000000.0) / 1000000.0;

      Serial.println("🚀 Calibration Polynomial:");
      Serial.printf("y = %.10f * x^2 + %.6f * x + %.6f\n", a, roundedB, roundedC);

      // Test outputs
      Serial.printf("📐 H1 = %.2f mm | H2 = %.2f mm | H3 = %.2f mm\n",
                    a * p1.x * p1.x + b * p1.x + c,
                    a * p2.x * p2.x + b * p2.x + c,
                    a * p3.x * p3.x + b * p3.x + c);

      bool changed = false;

      // Raw calibration points
      long newEmptyHz = shared["emptyHz"];
      long newEmptyDistance = shared["emptyDistance"];
      long newMidHz = shared["midHz"];
      long newMidDistance = shared["midDistance"];
      long newFullHz = shared["fullHz"];
      long newFullDistance = shared["fullDistance"];
      long newSensorDisconnectedHz = shared["sensorDisconnectedHz"];

      if (newEmptyHz != calibrationData.emptyHz) {
        calibrationData.emptyHz = newEmptyHz;
        changed = true;
      }
      if (newEmptyDistance != calibrationData.emptyDistance) {
        calibrationData.emptyDistance = newEmptyDistance;
        changed = true;
      }
      if (newMidHz != calibrationData.midHz) {
        calibrationData.midHz = newMidHz;
        changed = true;
      }
      if (newMidDistance != calibrationData.midDistance) {
        calibrationData.midDistance = newMidDistance;
        changed = true;
      }
      if (newFullHz != calibrationData.fullHz) {
        calibrationData.fullHz = newFullHz;
        changed = true;
      }
      if (newFullDistance != calibrationData.fullDistance) {
        calibrationData.fullDistance = newFullDistance;
        changed = true;
      }

      if (newSensorDisconnectedHz != calibrationData.sensorDisconnectedHz) {
        calibrationData.sensorDisconnectedHz = newSensorDisconnectedHz;
        changed = true;
      }
      if (sensorTypeStr != deviceSettings.sensorType) {
        changed = true;
      }

      const double epsilon = 0.000001;
      bool coeffChanged = fabs(a - calibrationData.a) > epsilon || fabs(roundedB - calibrationData.b) > epsilon || fabs(roundedC - calibrationData.c) > epsilon;

      if (coeffChanged) {
        calibrationData.a = a;
        calibrationData.b = roundedB;
        calibrationData.c = roundedC;
        changed = true;
      }

      if (changed) {
        saveCalibrationData(calibration_filename);
        saveConfig(config_filename);
        Serial.println("📁 Calibration file updated");
      } else {
        Serial.println("📂 Calibration unchanged — no write needed");
      }

      return true;
    } else {
      Serial.println("⚠️ Capacitive calibration fields missing.");
      return false;
    }

  } else if (sensorTypeStr == "ultrasonic") {
    bool changed = false;

    strncpy(deviceSettings.sensorType, sensorTypeStr.c_str(), sizeof(deviceSettings.sensorType) - 1);
    deviceSettings.sensorType[sizeof(deviceSettings.sensorType) - 1] = '\0';
    changed = true;

    if (shared.containsKey("ultrasonicOffset")) {
      int newOffset = shared["ultrasonicOffset"].as<int>();
      if (newOffset != deviceSettings.ultrasonicOffset) {
        deviceSettings.ultrasonicOffset = newOffset;
        Serial.printf("📏 Ultrasonic offset set to %d mm\n", deviceSettings.ultrasonicOffset);
        changed = true;
      }
    }


    if (changed) {
      saveCalibrationData(calibration_filename);
      saveConfig(config_filename);
      Serial.println("📁 Calibration file updated");
    } else {
      Serial.println("📂 Calibration unchanged — no write needed");
    }

    return true;
  }
}


// bool saveCalibrationData(const char *filename) {
//   File file = LittleFS.open(filename, "w");
//   if (!file) {
//     Serial.println("❌ Failed to open calibration file for writing");
//     return false;
//   }

//   StaticJsonDocument<512> doc;
//   doc["emptyHz"] = calibrationData.emptyHz;
//   doc["emptyDistance"] = calibrationData.emptyDistance;
//   doc["midHz"] = calibrationData.midHz;
//   doc["midDistance"] = calibrationData.midDistance;
//   doc["fullHz"] = calibrationData.fullHz;
//   doc["fullDistance"] = calibrationData.fullDistance;
//   doc["a"] = calibrationData.a;
//   doc["sensorDisconnectedHz"] = calibrationData.sensorDisconnectedHz;

//   // Round b and c to 6 decimal places before saving
//   double roundedB = round(calibrationData.b * 1e6) / 1e6;
//   double roundedC = round(calibrationData.c * 1e6) / 1e6;

//   doc["b"] = roundedB;
//   doc["c"] = roundedC;

//   if (serializeJson(doc, file) == 0) {
//     Serial.println("❌ Failed to write calibration JSON");
//     return false;
//   }

//   Serial.println("💾 Calibration data saved");
//   return true;
// }

bool saveCalibrationData(const char *filename) {
  File file = LittleFS.open(filename, "w");
  if (!file) {
    Serial.println("❌ Failed to open calibration file for writing");
    return false;
  }

  StaticJsonDocument<512> doc;
  doc["emptyHz"] = calibrationData.emptyHz;
  doc["emptyDistance"] = calibrationData.emptyDistance;
  doc["midHz"] = calibrationData.midHz;
  doc["midDistance"] = calibrationData.midDistance;
  doc["fullHz"] = calibrationData.fullHz;
  doc["fullDistance"] = calibrationData.fullDistance;
  doc["a"] = calibrationData.a;
  doc["sensorDisconnectedHz"] = calibrationData.sensorDisconnectedHz;

  // Round b and c to 6 decimal places before saving
  double roundedB = round(calibrationData.b * 1e6) / 1e6;
  double roundedC = round(calibrationData.c * 1e6) / 1e6;

  doc["b"] = roundedB;
  doc["c"] = roundedC;

  // ✅ Save new fields
  doc["sensorType"] = deviceSettings.sensorType;
  doc["ultrasonicOffset"] = deviceSettings.ultrasonicOffset;

  if (serializeJson(doc, file) == 0) {
    Serial.println("❌ Failed to write calibration JSON");
    return false;
  }

  Serial.println("💾 Calibration data saved");
  return true;
}





bool loadCalibrationData(const char *filename) {
  if (!LittleFS.exists(filename)) {
    Serial.println("⚠️ Calibration file not found. Creating default...");
    return saveCalibrationData(filename);  // Create with defaults
  }

  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.println("❌ Failed to open calibration file for reading");
    return false;
  }

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, file);
  if (err) {
    Serial.println("❌ Failed to parse calibration JSON");
    return false;
  }

  calibrationData.emptyHz = doc["emptyHz"] | calibrationData.emptyHz;
  calibrationData.emptyDistance = doc["emptyDistance"] | calibrationData.emptyDistance;
  calibrationData.midHz = doc["midHz"] | calibrationData.midHz;
  calibrationData.midDistance = doc["midDistance"] | calibrationData.midDistance;
  calibrationData.fullHz = doc["fullHz"] | calibrationData.fullHz;
  calibrationData.fullDistance = doc["fullDistance"] | calibrationData.fullDistance;
  calibrationData.sensorDisconnectedHz = doc["sensorDisconnectedHz"] | calibrationData.sensorDisconnectedHz;
  calibrationData.a = doc["a"] | 0.0;
  calibrationData.b = doc["b"] | 0.0;
  calibrationData.c = doc["c"] | 0.0;

  // ✅ Load new fields into deviceSettings
  strlcpy(deviceSettings.sensorType, doc["sensorType"] | "", sizeof(deviceSettings.sensorType));
  deviceSettings.ultrasonicOffset = doc["ultrasonicOffset"] | deviceSettings.ultrasonicOffset;

  Serial.println("\n📥 Calibration data loaded:");
  Serial.printf("emptyHz: %ld, emptyDistance: %ld\n", calibrationData.emptyHz, calibrationData.emptyDistance);
  Serial.printf("midHz: %ld, midDistance: %ld\n", calibrationData.midHz, calibrationData.midDistance);
  Serial.printf("fullHz: %ld, fullDistance: %ld\n", calibrationData.fullHz, calibrationData.fullDistance);
  Serial.printf("Sensor Disconnected Hz: %ld\n", calibrationData.sensorDisconnectedHz);
  Serial.printf("a = %.10f, b = %.10f, c = %.10f\n", calibrationData.a, calibrationData.b, calibrationData.c);

  Serial.printf("Sensor Type: %s\n", deviceSettings.sensorType);
  Serial.printf("Ultrasonic Offset: %d\n\n", deviceSettings.ultrasonicOffset);

  return true;
}



void initCalibration() {
  if (!LittleFS.begin(true)) {
    Serial.println("❌ LittleFS mount failed");
    return;
  }

  Serial.println("✅ LittleFS mounted");
  if (!loadCalibrationData(calibration_filename)) {
    Serial.println("⚠️ Calibration load failed or missing, saved defaults");
  }
}
// bool loadCalibrationData(const char *filename) {
//   if (!LittleFS.exists(filename)) {
//     Serial.println("⚠️ Calibration file not found. Creating default...");
//     return saveCalibrationData(filename);  // Create with defaults
//   }

//   File file = LittleFS.open(filename, "r");
//   if (!file) {
//     Serial.println("❌ Failed to open calibration file for reading");
//     return false;
//   }

//   StaticJsonDocument<512> doc;
//   DeserializationError err = deserializeJson(doc, file);
//   if (err) {
//     Serial.println("❌ Failed to parse calibration JSON");
//     return false;
//   }

//   calibrationData.emptyHz = doc["emptyHz"] | calibrationData.emptyHz;
//   calibrationData.emptyDistance = doc["emptyDistance"] | calibrationData.emptyDistance;
//   calibrationData.midHz = doc["midHz"] | calibrationData.midHz;
//   calibrationData.midDistance = doc["midDistance"] | calibrationData.midDistance;
//   calibrationData.fullHz = doc["fullHz"] | calibrationData.fullHz;
//   calibrationData.fullDistance = doc["fullDistance"] | calibrationData.fullDistance;
//   calibrationData.sensorDisconnectedHz = doc["sensorDisconnectedHz"] | calibrationData.sensorDisconnectedHz;
//   calibrationData.a = doc["a"] | 0.0;
//   calibrationData.b = doc["b"] | 0.0;
//   calibrationData.c = doc["c"] | 0.0;

//   // Serial.println("📥 Calibration data loaded:");
//   // Serial.printf("emptyHz: %ld, emptyDistance: %ld\n", calibrationData.emptyHz, calibrationData.emptyDistance);
//   // Serial.printf("a = %.10f, b = %.10f, c = %.10f\n", calibrationData.a, calibrationData.b, calibrationData.c);

//   Serial.println("\n📥 Calibration data loaded:");
//   Serial.printf("emptyHz: %ld, emptyDistance: %ld\n", calibrationData.emptyHz, calibrationData.emptyDistance);
//   Serial.printf("midHz: %ld, midDistance: %ld\n", calibrationData.midHz, calibrationData.midDistance);
//   Serial.printf("fullHz: %ld, fullDistance: %ld\n", calibrationData.fullHz, calibrationData.fullDistance);
//   Serial.printf("Sensor Disconnected Hz: %ld\n", calibrationData.sensorDisconnectedHz);

//   Serial.printf("a = %.10f, b = %.10f, c = %.10f\n\n\n", calibrationData.a, calibrationData.b, calibrationData.c);
//   return true;
// }