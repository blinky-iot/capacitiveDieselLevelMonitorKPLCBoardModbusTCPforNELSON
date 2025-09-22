
bool flushing = false;

bool initSD() {
  sdReady = SD.begin(PIN_SD_CS);
  if (!sdReady) {
    Serial.println("❌ SD card initialization failed");
    return false;
  }

  Serial.println("✅ SD card initialized");

  // Create telemetry.csv if missing
  if (!SD.exists("/telemetry.csv")) {
    File telemetryFile = SD.open("/telemetry.csv", FILE_WRITE);
    if (telemetryFile) {
      //telemetryFile.println("timestamp,volume_liters,battery_voltage,fuelHz,fuelSensorStatus");
      //telemetryFile.println("timestamp,volume_liters,battery_voltage,fuelHz,fuelSensorStatus,Height,TempC");
      telemetryFile.println("timestamp,volume_liters,battery_voltage,fuelHz,fuelSensorStatus,Height_cap,TempC,ultrasonicDistance,ultrasonicSensorStatus");


      telemetryFile.close();
      Serial.println("📄 Created telemetry.csv with header");
    }
  }

  // Create gps.csv if missing
  if (!SD.exists("/gps.csv")) {
    File gpsFile = SD.open("/gps.csv", FILE_WRITE);
    if (gpsFile) {
      gpsFile.println("timestamp,latitude,longitude");
      gpsFile.close();
      Serial.println("📄 Created gps.csv with header");
    }
  }

  return true;
}

void logGpsToSD(uint64_t ts, float lat, float lon) {
  if (!sdReady) {
    Serial.println("❌ SD not available for logGpsToSD");
    return;
  }

  File f = SD.open(GPS_FILE, FILE_APPEND);
  if (f) {
    f.printf("%llu,%.6f,%.6f\n", ts, lat, lon);
    f.flush();
    f.close();
    Serial.println("✅ Logged to SD (GPS)");
  } else {
    Serial.println("❌ Failed to open gps.csv for appending");
  }
}
void logToSD(uint64_t ts, float volume_liters, float battery_voltage, float freqHz, FuelSensorStatus status, float height_mm, float rtcTemp, float Height_ultrsnc, int ultrasonicSensorStatus) {


  if (!sdReady) {
    Serial.println("❌ SD not available for logToSD");
     sdCardStatus = false;
    return;
  }

  if (!SD.exists(MAIN_FILE)) {
    File headerFile = SD.open(MAIN_FILE, FILE_WRITE);
    if (headerFile) {
      //headerFile.println("timestamp,volume_liters,battery_voltage,fuelHz,fuelSensorStatus,Height,TempC");
      headerFile.println("timestamp,volume_liters,battery_voltage,fuelHz,fuelSensorStatus,Height_cap,TempC,Height_ultrsnc,ultraSonicStatus");

      //telemetryFile.println("timestamp,volume_liters,battery_voltage,fuelHz,fuelSensorStatus,Height,TempC");

      headerFile.close();
    }
  }

  File f = SD.open(MAIN_FILE, FILE_APPEND);
  if (f) {
    flushing = true;
    const char* statusText[] = { "OK", "DISCONNECTED", "OUT_OF_RANGE" };
    //f.printf("%llu,%.2f,%.2f,%.2f,%s\n", ts, volume_liters, battery_voltage, freqHz, statusText[status],height_mm);
    //f.printf("%llu,%.2f,%.2f,%.2f,%s,%.1f\n", ts, volume_liters, battery_voltage, freqHz, statusText[status], height_mm);
    //f.printf("%llu,%.2f,%.2f,%.2f,%s,%.1f,%.1f\n", ts, volume_liters, battery_voltage, freqHz, statusText[status], height_mm, rtcTemp);
    f.printf("%llu,%.2f,%.2f,%.2f,%s,%.1f,%.1f,%.1f,%.1f\n", ts, volume_liters, battery_voltage, freqHz, statusText[status], height_mm, rtcTemp, Height_ultrsnc, ultrasonicSensorStatus);


    f.flush();
    f.close();
    flushing = false;
    Serial.println("✅ Logged to SD (volume/voltage/freq/status)");
     sdCardStatus = true;
  } else {
    Serial.println("❌ Failed to open telemetry.csv for appending");
    sdCardStatus = false;
  }
}
