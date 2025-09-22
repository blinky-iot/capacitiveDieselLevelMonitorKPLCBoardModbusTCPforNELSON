
 #define GPS_BAUD 9600

DateTime nowRtc;
const unsigned long GPS_SESSION_INTERVAL = 12UL * 60 * 60 * 1000; // 12 hours
const unsigned long GPS_FIX_TIMEOUT     = 5UL * 60 * 1000;        // 5 minutes
const unsigned long GPS_DEBUG_INTERVAL  = 5000;                   // 5 seconds
// ===================================================

// Control power
void powerOnGPS() {
  Serial.println("-----------------------powering GPS ON---------------");
  digitalWrite(PIN_GPS_PWR, HIGH);
}

void powerOffGPS() {
  Serial.println("-----------------------powering GPS OFF---------------");
  digitalWrite(PIN_GPS_PWR, LOW);
}

// Log GPS to SD
void logGpsToSD(uint64_t timestampMs, double lat, double lon) {
  File file = SD.open("/gps.csv", FILE_APPEND);
  if (!file) {
    Serial.println("❌ Failed to open gps.csv");
    return;
  }
  file.printf("%llu,%.6f,%.6f\n", timestampMs, lat, lon);
  file.close();
  Serial.println("💾 GPS logged to SD");
}
// 📤 Retrieve last logged GPS fix from SD

// Should be called in loop()
void handleGpsSession() {
  unsigned long now = millis();

  // Start a session every 12 hours
  if (now - lastGpsSessionTime >= GPS_SESSION_INTERVAL) {
    Serial.println("📡 Starting new GPS session...");
    powerOnGPS();
    gpsFixLogged = false;
    gpsStartTime = now;
    lastGpsSessionTime = now;
  }

  // Feed GPS parser
 while (gpsSerial.available()) {
  char c = gpsSerial.read();
  //Serial.write(c); // Print raw NMEA
  gps.encode(c);   // Parse
}


  // While GPS fix not yet logged and timeout hasn't passed
  if ((now - gpsStartTime < GPS_FIX_TIMEOUT) && !gpsFixLogged) {
    if (now - lastGpsDebugTime >= GPS_DEBUG_INTERVAL) {
      lastGpsDebugTime = now;
      if (gps.location.isValid()) {
        Serial.printf("📍 Lat: %.6f | Lon: %.6f | Age: %lu ms\n",
                      gps.location.lat(), gps.location.lng(), gps.location.age());
      } else {
        Serial.println("⏳ Waiting for GPS fix...");
      }
    }

    if (gps.location.isValid() && gps.location.age() < 2000) {
      Serial.println("🕒 GPS fix acquired. Waiting 30 seconds to stabilize...");

      unsigned long endWait = millis() + 30000;
      while (millis() < endWait) {
        while (gpsSerial.available()) {
          gps.encode(gpsSerial.read());
        }
        delay(10);  // Avoid starving CPU
      }

      DateTime nowRtc = rtc.now();
      uint64_t timestamp = (uint64_t)nowRtc.unixtime() * 1000ULL;
      logGpsToSD(timestamp, gps.location.lat(), gps.location.lng());

      Serial.println("✅ GPS fix logged after stabilization. Powering OFF.");
      powerOffGPS();
      gpsFixLogged = true;
    }
  }

  // Timeout fallback
  if (!gpsFixLogged && (now - gpsStartTime >= GPS_FIX_TIMEOUT)) {
    Serial.println("⌛ GPS fix timeout. Powering OFF.");
    powerOffGPS();
    gpsFixLogged = true;
  }
}

// Call once in main setup()
void initGpsHandler() {
  pinMode(PIN_GPS_PWR, OUTPUT);
  powerOnGPS();
  delay(1000);  // Allow GPS to power up

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  delay(1000);  // Allow GPS serial to settle

  Wire.begin(); // Only needed if using I2C (RTC, etc.)

  Serial.println("✅ GPS handler initialized");
}

bool getLastGpsFromSD(GpsFix& fix) {
  File file = SD.open("/gps.csv", FILE_READ);
  if (!file) {
    Serial.println("❌ Failed to open gps.csv for reading.");
    return false;
  }

  // ✅ Check if file is empty
  if (file.size() == 0) {
    Serial.println("⚠️ gps.csv is empty.");
    file.close();
    return false;
  }

  String lastLine;
  while (file.available()) {
    lastLine = file.readStringUntil('\n');
  }
  file.close();

  if (lastLine.length() == 0) {
    Serial.println("⚠️ gps.csv has no usable lines.");
    return false;
  }

  int firstComma = lastLine.indexOf(',');
  int secondComma = lastLine.indexOf(',', firstComma + 1);
  if (firstComma == -1 || secondComma == -1) {
    Serial.println("⚠️ gps.csv line is malformed.");
    return false;
  }

  fix.timestamp = lastLine.substring(0, firstComma).toInt();
  fix.lat = lastLine.substring(firstComma + 1, secondComma).toFloat();
  fix.lon = lastLine.substring(secondComma + 1).toFloat();

  // ✅ Check for invalid 0.0,0.0 fix
  if (fix.lat == 0.0 && fix.lon == 0.0) {
    Serial.println("⚠️ Last GPS fix is zero. Ignoring.");
    return false;
  }

  return true;
}

// bool getLastGpsFromSD(GpsFix& fix) {
//   File file = SD.open("/gps.csv", FILE_READ);
//   if (!file) {
//     Serial.println("❌ Failed to open gps.csv for reading.");
//     return false;
//   }

//   String lastLine;
//   while (file.available()) {
//     lastLine = file.readStringUntil('\n');
//   }
//   file.close();

//   if (lastLine.length() == 0) return false;

//   int firstComma = lastLine.indexOf(',');
//   int secondComma = lastLine.indexOf(',', firstComma + 1);
//   if (firstComma == -1 || secondComma == -1) return false;

//   fix.timestamp = lastLine.substring(0, firstComma).toInt();
//   fix.lat = lastLine.substring(firstComma + 1, secondComma).toFloat();
//   fix.lon = lastLine.substring(secondComma + 1).toFloat();

//   return true;
// }