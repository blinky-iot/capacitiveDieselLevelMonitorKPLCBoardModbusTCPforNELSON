
void LCDsetup() {
  lcd.init();
  lcd.backlight();
}

//MsgStructure A= Conv("Hello My name is Alban Mwangi Wainaina");
//Display (A.str1, A.str2);

void Display(String Message1, String Message2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(String(Message1));
  lcd.setCursor(0, 1);
  lcd.print(String(Message2));
}

void StatusDisplay() {
  //lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Level: ");

  lcd.setCursor(0, 1);
  lcd.print("Volume: ");
  //DistanceDisplay(DisplayDistance, Volume);

  //clearLCDLine(2);
  lcd.setCursor(0, 2);
  lcd.print("Status:           ");  // Clear old text
  lcd.setCursor(8, 2);

  if (WiFi.status() == WL_CONNECTED) {
    lcd.print("Connected   ");
  } else {
    lcd.print("Disconnected");
  }
}



// void DistanceDisplay(float height_mm, int volume_liters) {
//   float height;
//   volume_liters = calculateVolumeLiters(height_mm);


//   if (height_mm >= 0) {
//     clearLCDLine(0, 7);
//     lcd.setCursor(7, 0);
//     lcd.print(height_mm / 1000.0, 2);  // Display in meters with 2 decimal places
//     lcd.print(" m");
//     Serial.print("lcd height_mm: ");
//     Serial.print(height_mm);

//     clearLCDLine(1, 8);
//     lcd.setCursor(8, 1);
//     lcd.print(volume_liters);
//     lcd.print(" L");
//     Serial.print("lcd volume_liters: ");
//     Serial.print(volume_liters);
//   } else {
//     clearLCDLine(0, 7);
//     lcd.setCursor(7, 0);
//     lcd.print("Invalid");

//     clearLCDLine(1, 8);
//     lcd.setCursor(8, 1);
//     lcd.print("Reading");
//   }
// }
// void DistanceDisplay(float height_mm, int volume_liters) {
//   if (height_mm >= 0) {
//     clearLCDLine(0, 7);
//     lcd.setCursor(7, 0);
//     lcd.print(height_mm / 1000.0, 2);  // meters with 2 decimal places
//     lcd.print(" m");
//     Serial.print("lcd height_mm: ");
//     Serial.println(height_mm);

//     clearLCDLine(1, 8);
//     lcd.setCursor(8, 1);
//     lcd.print(volume_liters);
//     lcd.print(" L");
//     Serial.print("lcd volume_liters: ");
//     Serial.println(volume_liters);
//   } else {
//     clearLCDLine(0, 7);
//     lcd.setCursor(7, 0);
//     lcd.print("Invalid");

//     clearLCDLine(1, 8);
//     lcd.setCursor(8, 1);
//     lcd.print("Reading");
//   }
// }

void DistanceDisplay(float height_mm, int volume_liters) {
  if (height_mm >= 0) {
    // Update Level value only (right-aligned from column 7)
    lcd.setCursor(7, 0);
    lcd.print("        ");  // Clear previous value (8 spaces)
    lcd.setCursor(7, 0);
    lcd.print(height_mm / 1000.0, 2);  // e.g., 1.23
    lcd.print(" m");

    // Update Volume value (right-aligned from column 8)
    lcd.setCursor(8, 1);
    lcd.print("       ");  // Clear previous value
    lcd.setCursor(8, 1);
    lcd.print(volume_liters);
    lcd.print(" L");

    //Serial.printf("lcd height_mm: %.2f mm, volume_liters: %d L\n", height_mm, volume_liters);
  } else {
    lcd.setCursor(7, 0);
    lcd.print("Invalid");

    lcd.setCursor(8, 1);
    lcd.print("Reading");
  }
}


MsgStructure Conv(String Message) {

  MsgStructure Conv1;

  char* msg = new char[Message.length() + 1];
  strcpy(msg, Message.c_str());
  int i = 0;

  for (i = 0; i <= 19; i++) {
    Conv1.str1 += msg[i];
  }

  if (Message.length() > 20) {
    for (int j = i; j <= Message.length() - 1; j++) {
      Conv1.str2 += msg[j];
    }
  }

  return Conv1;
}

void clearLCDLine(int line) {

  for (int n = 0; n < 20; n++) {  // 20 indicates symbols in line. For 2x16 LCD write - 16
    lcd.setCursor(n, line);
    lcd.print(" ");
  }
  lcd.setCursor(0, line);  // set cursor in the beginning of deleted line
}

void clearLCDLine(int line, int startCell) {

  for (int n = startCell; n < 20; n++) {  // 20 indicates symbols in line. For 2x16 LCD write - 16
    lcd.setCursor(n, line);
    lcd.print(" ");
  }
  lcd.setCursor(0, line);  // set cursor in the beginning of deleted line
}

void wifiConnection(void* parameters) {
  while (1) {
    mb.task();

    // Update status every 5s
    if (millis() - lastDisplay > 5000) {
      lastDisplay = millis();

      float volume_liters = getLevelLiters();  // updates global height_mm
      int volumeInt = (int)volume_liters;

      mb.Hreg(REG_BASE + 0, volumeInt);
      mb.Hreg(REG_BASE + 1, height_mm);
      StatusDisplay();
      if (WiFi.status() == WL_CONNECTED) {
        wifiReadyDisplay(local_IP);
        Serial.println("WiFi Connected");
      }
      DistanceDisplay(height_mm, volumeInt);  // ✅ no flicker now
    }
    // if (millis() - lastDisplay > 5000) {
    //   lastDisplay = millis();

    //   float volume_liters = getLevelLiters();  // this also updates global height_mm
    //   int volumeInt = (int)volume_liters;

    //   mb.Hreg(REG_BASE + 0, volumeInt);
    //   mb.Hreg(REG_BASE + 1, height_mm);

    //   StatusDisplay();  // Optional: shows "Level:" and "Volume:"
    //   DistanceDisplay(height_mm, volumeInt);  // ✅ update LCD values
    //   wifiReadyDisplay(local_IP);             // ✅ update IP on line 3

    //   Serial.println("WiFi Connected");
    // }

    // Delay or yield to avoid WDT resets
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// void wifiConnection(void* parameters) {
//   // InitializeWiFi();
//   while (1) {
//     mb.task();

//     // lastDisplay = millis();

//     if (millis() - lastDisplay > 5000) {
//       lastDisplay = millis();
//       StatusDisplay();
//       if (WiFi.status() == WL_CONNECTED) {
//         wifiReadyDisplay(local_IP);
//         Serial.println("WiFi Connected");
//       }
//     }

//     // Optionally update values (e.g., simulate changing data)
//     static uint32_t lastUpdate = 0;
//     if (millis() - lastUpdate > 1000) {
//       lastUpdate = millis();
//       float volume_liters = getLevelLiters();
//       mb.Hreg(REG_BASE + 0, volume_liters);
//       mb.Hreg(REG_BASE + 1, height_mm);
//       // for (int i = 0; i < REG_COUNT; i++) {
//       //   mb.Hreg(REG_BASE + i, random(0, 65000));  // Simulate sensor values
//       // }
//       //Serial.println("📤 Updated holding registers with new random values");

//       Serial.print("📤 Updated holding registers: ");
//       // Serial.print("REG[");
//       // Serial.print(REG_BASE + 0);
//       // Serial.print("] = ");
//       // Serial.print(volume_liters);
//       // Serial.print(", REG[");
//       // Serial.print(REG_BASE + 1);
//       // Serial.print("] = ");
//       // Serial.println(height_mm);
//     }
//   }
// }
void wifiReadyDisplay(IPAddress ip) {
  clearLCDLine(3);
  lcd.setCursor(0, 3);
  // Convert IPAddress to char array
  char ipChar[16];  // Enough for "255.255.255.255\0"
  snprintf(ipChar, sizeof(ipChar), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

  lcd.print(ipChar);  // ✅ print C-style string
}
// // Last known good timestamps
// unsigned long sensor1_last_good = 0;
// unsigned long ds_last_good = 0;

// // Debounced status
// bool sensor1_stable_ok = false;
// bool ds_stable_ok = false;
// int hysteresis = 1;
// // Constants
// const unsigned long debouncePeriod = 10000;  // 30 seconds

// void LCDTask(void *pvParameters) {
//   static float lastGoodTemp1 = -100.0;
//   static float lastGoodHumidity1 = -100.0;
//   static float lastGoodDSTemp = -100.0;

//   static unsigned long lastUpdate = 0;

//   const int updateInterval = 1000;

//   static String lastLine0 = "";
//   static String lastLine1 = "";

//   bool highAlert = false;
//   bool lowAlert = false;

//   static bool showNetworkStatus = true;
//   static unsigned long lastToggleTime = 0;
//   const unsigned long toggleInterval = 3000;  // 3 seconds

//   for (;;) {
//     float maxTemp = max(Temperature1, DS_Temperature);
//     // Serial.print("max temperature: ");
//     // Serial.println(maxTemp);

//     // Evaluate High Alert first (takes precedence)
//     if (maxTemp > high_critical_value) {
//       highAlert = true;
//       lowAlert = false;  // clear lowAlert when in highAlert
//       Serial.println("High Alert");
//     } else {
//       highAlert = false;
//     }

//     // Only check for lowAlert if highAlert is NOT active
//     if (!highAlert) {
//       if (maxTemp > low_critical_value) {
//         Serial.print("low_critical_value ");
//         Serial.println(low_critical_value);
//         Serial.println("LOW Alert");
//         lowAlert = true;
//       } else {
//         lowAlert = false;
//       }
//     }

//     if (highAlert || lowAlert) {
//       toggleBacklight();
//     } else lcd.backlight();


//     unsigned long now = millis();

//     // --- Update debounced sensor flags
//     if (sensor1_ok) sensor1_last_good = now;
//     if (ds_ok) ds_last_good = now;

//     sensor1_stable_ok = (now - sensor1_last_good <= debouncePeriod);
//     ds_stable_ok = (now - ds_last_good <= debouncePeriod);

//     // --- Cache valid readings
//     if (sensor1_ok) {
//       lastGoodTemp1 = Temperature1;
//       if (Humidity1 >= 0 && Humidity1 <= 100) {
//         lastGoodHumidity1 = Humidity1;
//       }
//     }
//     if (ds_ok) {
//       lastGoodDSTemp = DS_Temperature;
//     }

//     // --- Determine which temperature to evaluate
//     float activeTemp = ds_stable_ok ? lastGoodDSTemp : lastGoodTemp1;

//     // --- Apply hysteresis logic
//     if (activeTemp >= high_critical_value && !highAlert) {
//       highAlert = true;
//     } else if (activeTemp <= (high_critical_value - hysteresis) && highAlert) {
//       highAlert = false;
//     }

//     if (activeTemp <= low_critical_value && !lowAlert) {
//       lowAlert = true;
//     } else if (activeTemp >= (low_critical_value + hysteresis) && lowAlert) {
//       lowAlert = false;
//     }

//     // --- Every update interval
//     if (now - lastUpdate >= updateInterval) {
//       lastUpdate = now;




//      // Compose Line 0
//       String line0;
//       if (!backlightState) {
//         line0 = "H:" + String(low_critical_value, 1);
//         line0 += " HH:" + String(high_critical_value, 1);

//       } else {
//         line0 = "NetW:";
//         line0 += networkConnected ? "OK" : "ERR";
//         line0 += " DATA:";
//         line0 += mqttConnected ? "YES" : "ERR";
//       }

//       // Compose Line 1: Sensor data
//       String line1;
//       if (sensor1_stable_ok && ds_stable_ok) {
//         line1 = "T:" + String(lastGoodDSTemp, 1) + (char)223 + "C";
//         line1 += " H:" + String(lastGoodHumidity1, 1);
//       } else if (sensor1_stable_ok) {
//         line1 = "T:" + String(lastGoodTemp1, 1) + (char)223 + "C";
//         line1 += " H:" + String(lastGoodHumidity1, 1);
//       } else if (ds_stable_ok) {
//         line1 = "T:" + String(lastGoodDSTemp, 1) + (char)223 + "C";
//       } else {
//         line1 = "No sensor data   ";  // padded to clear leftovers
//       }

//       // Update LCD only if content has changed
//       if (line0 != lastLine0 || line1 != lastLine1) {
//         lcd.clear();
//         lcd.setCursor(0, 0);
//         lcd.print(line0);
//         lcd.setCursor(0, 1);
//         lcd.print(line1);
//         lastLine0 = line0;
//         lastLine1 = line1;
//       }
//     }
//     vTaskDelay(200 / portTICK_PERIOD_MS);
//   }
// }



// void displayTask() {
//   // Create LCD Task on Core 0
//   xTaskCreatePinnedToCore(
//     LCDTask,
//     "LCD Display Task",
//     4096,
//     NULL,
//     1,
//     &DisplayTaskHandle,
//     0  // Core 0
//   );
// }


// void lcdBegin() {
//   lcd.begin();      // Initialize LCD
//   lcd.backlight();  // Turn on backlight
//   lcd.clear();      // Optional: Clear display
//   lcd.setCursor(0, 0);
//   lcd.print("System Starting");
// }





// void toggleBacklight() {
//   unsigned long now = millis();
//   const unsigned long interval = 2000;  // 2 seconds

//   if (now - lastBlinkTime >= interval) {
//     lastBlinkTime = now;
//     backlightState = !backlightState;

//     if (backlightState) {
//       lcd.backlight();
//     } else {
//       lcd.noBacklight();
//     }
//   }
// }
