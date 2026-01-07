

bool getValidNetworkTime(DateTime& localOut) {
  int yr, mo, da, hr, mi, se;
  float tzH;
  const uint8_t maxTries = 5;

  for (uint8_t attempt = 1; attempt <= maxTries; attempt++) {
    Serial.printf("⏱️ Trying to get network time (Attempt %d/%d)...\n", attempt, maxTries);
    if (modem.getNetworkTime(&yr, &mo, &da, &hr, &mi, &se, &tzH)) {
      if (yr >= 2023 && mo > 0 && da > 0) {
        localOut = DateTime(yr, mo, da, hr, mi, se);
        Serial.print("✅ Network time: ");
        Serial.println(localOut.timestamp());
        return true;
      } else {
        Serial.println("⚠️ Invalid time received. Retrying...");
      }
    } else {
      Serial.println("❌ Failed to get time from modem.");
    }
    delay(1000);
  }

  Serial.println("❌ Network time fetch failed after retries.");
  return false;
}


void provisionCLTS() {
  Serial.println("⚙️ Enabling CLTS (Cellular Local Time Sync)...");

  modem.sendAT("+CLTS=1");
  modem.waitResponse();
  modem.sendAT("&W");
  modem.waitResponse();
  // modem.sendAT("+CFUN=1,1");
  // modem.waitResponse();

  Serial.println("🔁 Waiting for modem reboot...");
  delay(10000);
}

void syncRtcOnce() {
  // 1. Show current RTC time before attempting sync
  DateTime now = rtc.now();
  Serial.printf("🕒 Current RTC time: %04d-%02d-%02d %02d:%02d:%02d\n",
                now.year(), now.month(), now.day(),
                now.hour(), now.minute(), now.second());

  // 2. Prompt user
  Serial.println("⌛ Press any key to sync RTC (expires in 20 seconds)...");

  // 3. Wait for input (max 10s)
  unsigned long startTime = millis();
  while (millis() - startTime < 20000) {
    if (Serial.available()) {
      Serial.read();  // clear buffer
      Serial.println("⏱️ Syncing RTC now...");
      break;
    }
    delay(10);
  }

  // 4. If timeout, skip sync
  if (!rtcTimeOk) {
  } else if (millis() - startTime >= 10000) {
    Serial.println("⏳ Timeout: No input received. Skipping RTC sync.");
    return;
  }



  // 5. Try network sync
  DateTime t;
  if (getValidNetworkTime(t)) {
    rtc.adjust(t);
    Serial.println("✅ RTC synced via network.");
    rtcTimeOk = true;
    saveRtcTimeOk(rtcTimeOk);  // persist it
    return;
  }

  // 6. Try CLTS fallback
  
  if (getValidNetworkTime(t)) {
    rtc.adjust(t);
    Serial.println("✅ RTC synced after CLTS.");
    rtcTimeOk = true;
    saveRtcTimeOk(rtcTimeOk);  // persist it

    return;
  }

  // 7. Final fallback to compile time
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println("⏳ RTC fallback to compile time.");
}


void saveRtcTimeOk(bool value) {
  preferences.begin("settings", false);  // namespace "settings", RW mode
  preferences.putBool("rtcTimeOk", value);
  preferences.end();
}

bool loadRtcTimeOk() {
  preferences.begin("settings", true);                   // RO mode
  bool value = preferences.getBool("rtcTimeOk", false);  // default to false
  preferences.end();
  return value;
}
