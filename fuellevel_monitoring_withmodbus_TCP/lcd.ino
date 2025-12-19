// Add these at the top of lcd.ino
bool displayOverriddenByError = false;
unsigned long lastNormalDisplayUpdate = 0;
const unsigned long NORMAL_DISPLAY_INTERVAL = 2000;  // Update normal display every 2s
unsigned long lastErrorBlink = 0;
bool errorBlinkState = false;

void LCDsetup() {
  lcd.init();
  lcd.backlight();
}

void Display(String Message1, String Message2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(String(Message1));
  lcd.setCursor(0, 1);
  lcd.print(String(Message2));
}

// Optimized StatusDisplay - doesn't clear entire screen
// void StatusDisplay() {
//   // Don't clear entire screen - just update specific areas

//   // --- Line 0: Level and Valve ---
//   // Level label
//   lcd.setCursor(0, 0);
//   lcd.print("Lev:");

//   // Valve status (right side)
//   lcd.setCursor(12, 0);
//   lcd.print("V:");
//   lcd.setCursor(15, 0);
//   lcd.print(valveOpen ? "ON" : "OFF");

//   // --- Line 1: Volume ---
//   lcd.setCursor(0, 1);
//   lcd.print("Vol:");

//   // --- Line 2: Network Status ---
//   lcd.setCursor(0, 2);
//   lcd.print("Net:");

//   // --- Line 3: IP Address ---
//   // IP label
//   lcd.setCursor(0, 3);
//   lcd.print("IP:");
// }
void StatusDisplay() {
  // First, clear entire line 0 completely
  //clearLCDLine(0);  // This clears all 20 characters on line 0

  // --- Line 0: Level and Valve ---
  // Level label
  lcd.setCursor(0, 0);
  lcd.print("Lev:");

  for (int i = 12; i < 20; i++) {
    lcd.setCursor(i, 0);
    lcd.print(" ");
  }

  // Valve status (right side)
  // Clear the valve area first (positions 12-19)
  //clearLCDArea(0, 12, 19);
  lcd.setCursor(12, 0);
  lcd.print("V:");
  lcd.setCursor(15, 0);
  lcd.print(valveOpen ? "ON " : "OFF");  // Space at end to clear leftover chars

  // Modbus status indicator (position 17)
  // lcd.setCursor(17, 0);
  // if (isModbusConnected()) {
  //   //lcd.print("✓");
  // } else {
  //   lcd.print(" ");
  // }

  // --- Line 1: Volume ---
  lcd.setCursor(0, 1);
  lcd.print("Vol:");

  // --- Line 2: Network Status ---
  lcd.setCursor(0, 2);
  lcd.print("Net:");

  // --- Line 3: IP Address ---
  lcd.setCursor(0, 3);
  lcd.print("IP:");
}

void DistanceDisplay(float height_mm, int volume_liters) {
  if (height_mm >= 0) {
    // Update Level value (position 4-10)
    lcd.setCursor(4, 0);
    // Clear only the value area (not entire line)
    for (int i = 4; i < 12; i++) {
      lcd.setCursor(i, 0);
      lcd.print(" ");
    }
    lcd.setCursor(4, 0);
    lcd.print(height_mm / 1000.0, 2);
    lcd.print(" M");

    // Update Volume value (position 4-10)
    lcd.setCursor(4, 1);
    // Clear only the value area
    for (int i = 4; i < 12; i++) {
      lcd.setCursor(i, 1);
      lcd.print(" ");
    }
    lcd.setCursor(4, 1);
    lcd.print(volume_liters);
    lcd.print(" L");
  } else {
    // Show error in value areas
    lcd.setCursor(4, 0);
    lcd.print("--- M");
    lcd.setCursor(4, 1);
    lcd.print("--- L");
  }
}

// Clear only part of a line
void clearLCDArea(int line, int startCol, int endCol) {
  for (int n = startCol; n <= endCol; n++) {
    lcd.setCursor(n, line);
    lcd.print(" ");
  }
}

void clearLCDLine(int line) {
  for (int n = 0; n < 20; n++) {
    lcd.setCursor(n, line);
    lcd.print(" ");
  }
}

void clearLCDLine(int line, int startCell) {
  for (int n = startCell; n < 20; n++) {
    lcd.setCursor(n, line);
    lcd.print(" ");
  }
}

// Function to display Modbus error (full screen)
void displayModbusError() {
  // Blink the error message for attention
  if (millis() - lastErrorBlink > 500) {
    lastErrorBlink = millis();
    errorBlinkState = !errorBlinkState;

    if (errorBlinkState) {
      // Clear entire screen for error display
      lcd.clear();

      // Line 0: Big error title
      lcd.setCursor(2, 0);
      lcd.print(" MODBUS ERROR");

      // Line 1: Error code/details
      lcd.setCursor(0, 1);
      String errorMsg = getLastModbusError();
      if (errorMsg.length() > 20) {
        errorMsg = errorMsg.substring(0, 20);
      }
      lcd.print(errorMsg);

      // Line 2: Instructions
      lcd.setCursor(0, 2);
      lcd.print("Check wiring & power");
      delay(1000);
      // Line 3: Time remaining
      // lcd.setCursor(0, 3);
      // unsigned long elapsed = millis() - modbusErrorStartTime;
      // if (elapsed < ERROR_DISPLAY_DURATION) {
      //   unsigned long remaining = (ERROR_DISPLAY_DURATION - elapsed) / 1000;
      //   lcd.print("Auto-clear: ");
      //   lcd.print(remaining);
      //   lcd.print("s");
      // } else {
      //   lcd.print("Clearing soon...");
      // }
    } else {
      // Blank screen during off phase of blink
      lcd.clear();
    }
  }
}

// Function to update network status (called during normal display)
void updateNetworkStatus() {
  // Update network status on line 2
  lcd.setCursor(4, 2);
  if (WiFi.status() == WL_CONNECTED) {
    lcd.print("Connected   ");
  } else {
    lcd.print("Disconnected");
  }

  // Update IP address on line 3
  lcd.setCursor(3, 3);
  if (WiFi.status() == WL_CONNECTED) {
    // Load and display IP
    preferences.begin("netcfg", true);
    IPAddress ip(
      preferences.getUInt("ip1", DEFAULT_LOCAL_IP[0]),
      preferences.getUInt("ip2", DEFAULT_LOCAL_IP[1]),
      preferences.getUInt("ip3", DEFAULT_LOCAL_IP[2]),
      preferences.getUInt("ip4", DEFAULT_LOCAL_IP[3]));
    preferences.end();

    // Clear IP area and display
    clearLCDArea(3, 3, 19);
    lcd.setCursor(3, 3);
    lcd.print(ip);
  } else {
    clearLCDArea(3, 3, 19);
    lcd.setCursor(3, 3);
    lcd.print("No Connection");
  }
}

// Function to update Modbus status indicator
void updateModbusStatusIndicator() {
  // Show Modbus status in a small indicator (top right corner, after valve)
  lcd.setCursor(17, 0);
  lcd.print(" ");  // Clear

  if (isModbusConnected()) {
    // lcd.setCursor(17, 0);
    // lcd.print("✓");
  } else {
    // Blink indicator when Modbus has issues but not critical error
    static unsigned long lastIndicatorBlink = 0;
    static bool indicatorState = false;

    if (millis() - lastIndicatorBlink > 1000) {
      lastIndicatorBlink = millis();
      indicatorState = !indicatorState;

      if (indicatorState) {
        lcd.setCursor(17, 0);
        lcd.print("!");
      } else {
        lcd.setCursor(17, 0);
        lcd.print(" ");
      }
    }
  }
}

// Function to display normal status (when no errors)
void displayNormalStatus() {
  // Only update display at regular intervals
  if (millis() - lastNormalDisplayUpdate >= NORMAL_DISPLAY_INTERVAL) {
    lastNormalDisplayUpdate = millis();

    // Update basic status display (labels)
    StatusDisplay();

    // Update network status and IP
    updateNetworkStatus();

    // Update Modbus status indicator
    updateModbusStatusIndicator();

    // Update tank values if Modbus data is valid
    if (isModbusConnected() && isModbusDataValid()) {
      float volume_liters = getLevelLiters();
      int volumeInt = (int)round(volume_liters);
      int heightInt = (int)round(height_mm);
      DistanceDisplay(heightInt, volumeInt);
    } else if (!isModbusConnected()) {
      // Show dashes if Modbus is disconnected
      lcd.setCursor(4, 0);
      lcd.print("--- M");
      lcd.setCursor(4, 1);
      lcd.print("--- L");
    }

    // Update valve status
    lcd.setCursor(15, 0);
    lcd.print(valveOpen ? "ON" : "OFF");
  }
}

// Main display update function
void updateDisplay() {
  // Check if Modbus error should take priority
  if (shouldDisplayModbusError()) {
    displayOverriddenByError = true;
    displayModbusError();
  } else if (displayOverriddenByError) {
    // Error was just cleared - transition back to normal display
    displayOverriddenByError = false;
    lcd.clear();                         // Clear error screen
    StatusDisplay();                     // Restore normal display structure
    lastNormalDisplayUpdate = millis();  // Force immediate update
    displayNormalStatus();               // Show current data
  } else {
    // Normal display mode - no errors
    displayOverriddenByError = false;
    displayNormalStatus();
  }
}

// Updated wifiConnection task
void wifiConnection(void* parameters) {
  // Initial display setup
  StatusDisplay();

  while (1) {
    // Always service Modbus TCP
    mb.task();

    // Update display (handles both normal and error modes)
    updateDisplay();

    // Update Modbus registers only if no error and Modbus is connected
    if (!shouldDisplayModbusError() && isModbusConnected() && isModbusDataValid()) {
      static unsigned long lastModbusUpdate = 0;
      if (millis() - lastModbusUpdate > 5000) {
        lastModbusUpdate = millis();

        // Get tank readings
        float volume_liters = getLevelLiters();
        int volumeInt = (int)round(volume_liters);
        int heightInt = (int)round(height_mm);

        // Calculate percentage fill
        int fillPercent = 0;
        if (deviceSettings.tankHeight > 0) {
          fillPercent = (int)round(((float)heightInt / deviceSettings.tankHeight) * 100.0);
          fillPercent = constrain(fillPercent, 0, 100);
        }

        // Update Modbus TCP registers
        mb.Hreg(REG_BASE + 0, (uint16_t)volumeInt);    // Volume
        mb.Hreg(REG_BASE + 1, (uint16_t)heightInt);    // Height
        mb.Hreg(REG_BASE + 2, (uint16_t)fillPercent);  // Percentage

        // Debug output (optional)
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 10000) {
          lastDebug = millis();
          Serial.printf("📊 Modbus TCP: V=%u L | H=%u mm | F=%u%%\n",
                        volumeInt, heightInt, fillPercent);
        }
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
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





































// void LCDsetup() {
//   lcd.init();
//   lcd.backlight();
// }

// //MsgStructure A= Conv("Hello My name is Alban Mwangi Wainaina");
// //Display (A.str1, A.str2);

// void Display(String Message1, String Message2) {
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print(String(Message1));
//   lcd.setCursor(0, 1);
//   lcd.print(String(Message2));
// }

// // void StatusDisplay() {
// //   //lcd.clear();
// //   lcd.init();
// //   lcd.setCursor(0, 0);
// //   lcd.print("Lev: ");

// //   lcd.setCursor(0, 1);
// //   lcd.print("Vol: ");
// //   //DistanceDisplay(DisplayDistance, Volume);

// //   //clearLCDLine(2);
// //   lcd.setCursor(0, 2);
// //   lcd.print("Sts:           ");  // Clear old text
// //   lcd.setCursor(8, 2);

// //   if (WiFi.status() == WL_CONNECTED) {
// //     lcd.print("Connected   ");
// //   } else {
// //     lcd.print("Disconnected");
// //   }
// // }

// void StatusDisplay() {
//   lcd.init();

//   // --- Line 1: Level ---
//   lcd.setCursor(0, 0);
//   lcd.print("Lev:");

//   // --- Line 2: Volume ---
//   lcd.setCursor(0, 1);
//   lcd.print("Vol:");

//   // --- Line 3: WiFi Status ---
//   lcd.setCursor(0, 2);
//   lcd.print("Net:         ");  // Clear old text
//   lcd.setCursor(5, 2);
//   if (WiFi.status() == WL_CONNECTED) {
//     lcd.print("Connected   ");
//   } else {
//     lcd.print("Disconnected");
//   }

//   lcd.setCursor(0, 3);
//   lcd.print("IP: ");


//   // --- Line 4: Valve Status ---
//   lcd.setCursor(12, 0);
//   lcd.print("VLV: ");  // Clear old text
//   lcd.setCursor(17, 0);
//   if (valveOpen) {
//     lcd.print("ON");
//   } else {
//     lcd.print("OFF");
//   }
// }


// void ModbusStatusDisplay(bool modbusConnected, const char* errorMsg = "") {
//   // Clear only the status area (line 3)
//   clearLCDLine(3);

//   lcd.setCursor(0, 3);

//   if (modbusConnected) {
//     lcd.print("Modbus: OK");
//   } else {
//     lcd.print("Modbus: ERROR");

//     // If there's a specific error message, display it on line 2
//     if (strlen(errorMsg) > 0) {
//       clearLCDLine(2);
//       lcd.setCursor(0, 2);
//       lcd.print(errorMsg);
//     }
//   }
// }

// void DistanceDisplay(float height_mm, int volume_liters) {
//   if (height_mm >= 0) {
//     // Update Level value only (right-aligned from column 7)
//     lcd.setCursor(5, 0);
//     lcd.print("       ");  // Clear previous value (8 spaces)
//     lcd.setCursor(5, 0);
//     lcd.print(height_mm / 1000.0, 2);  // e.g., 1.23
//     lcd.print(" M");

//     // Update Volume value (right-aligned from column 8)
//     lcd.setCursor(5, 1);
//     lcd.print("       ");  // Clear previous value
//     lcd.setCursor(5, 1);
//     lcd.print(volume_liters);
//     lcd.print(" L");

//     //Serial.printf("lcd height_mm: %.2f mm, volume_liters: %d L\n", height_mm, volume_liters);
//   } else {
//     lcd.setCursor(4, 0);
//     lcd.print("Invalid");

//     lcd.setCursor(4, 1);
//     lcd.print("Reading");
//   }
// }

// MsgStructure Conv(String Message) {

//   MsgStructure Conv1;

//   char* msg = new char[Message.length() + 1];
//   strcpy(msg, Message.c_str());
//   int i = 0;

//   for (i = 0; i <= 19; i++) {
//     Conv1.str1 += msg[i];
//   }

//   if (Message.length() > 20) {
//     for (int j = i; j <= Message.length() - 1; j++) {
//       Conv1.str2 += msg[j];
//     }
//   }

//   return Conv1;
// }

// void clearLCDLine(int line) {

//   for (int n = 0; n < 20; n++) {  // 20 indicates symbols in line. For 2x16 LCD write - 16
//     lcd.setCursor(n, line);
//     lcd.print(" ");
//   }
//   lcd.setCursor(0, line);  // set cursor in the beginning of deleted line
// }

// void clearLCDLine(int line, int startCell) {

//   for (int n = startCell; n < 20; n++) {  // 20 indicates symbols in line. For 2x16 LCD write - 16
//     lcd.setCursor(n, line);
//     lcd.print(" ");
//   }
//   lcd.setCursor(0, line);  // set cursor in the beginning of deleted line
// }

// /*
// this function stores
// REG_BASE + 0 → Volume (L, rounded integer)
// REG_BASE + 1 → Height (mm, rounded integer)
// */
// /*

// void wifiConnection(void* parameters) {
//   while (1) {
//     // Always service Modbus & MQTT
//     mb.task();
//     mqttClient.loop();

//     // ----- Update Modbus registers + display every 5s -----
//     if (millis() - lastDisplay > 5000) {
//       lastDisplay = millis();

//       // Get tank readings
//       float volume_liters = getLevelLiters();   // e.g., 4079.85
//       int volumeInt = (int)round(volume_liters); // store as whole liters
//       int heightInt = (int)round(height_mm);     // store as whole mm

//       // Update Modbus registers (16-bit safe)
//       mb.Hreg(REG_BASE + 0, (uint16_t)volumeInt);
//       mb.Hreg(REG_BASE + 1, (uint16_t)heightInt);

//       // ✅ Debug: verify register updates
//       uint16_t regVol = mb.Hreg(REG_BASE + 0);
//       uint16_t regHeight = mb.Hreg(REG_BASE + 1);

//       if (regVol == volumeInt && regHeight == heightInt) {
//         Serial.printf("✅ Holding registers updated: Volume=%u, Height=%u\n",
//                       regVol, regHeight);
//       } else {
//         Serial.printf("⚠️ Holding register mismatch! Expected (V=%d,H=%d), got (V=%u,H=%u)\n",
//                       volumeInt, heightInt, regVol, regHeight);
//       }

//       // Update display
//       StatusDisplay();
//       if (WiFi.status() == WL_CONNECTED) {
//         wifiReadyDisplay(local_IP);
//         Serial.println("WiFi Connected");
//       }
//       DistanceDisplay(heightInt, volumeInt);
//     }

//     vTaskDelay(10 / portTICK_PERIOD_MS);  // small delay so task yields
//   }
// }
// */

// /*
// this function stores
// REG_BASE + 0 → Volume (L, rounded integer)
// REG_BASE + 1 → Height (mm, rounded integer)
// REG_BASE + 2 → Fill percentage (0–100 %)
// */
// void wifiConnection(void* parameters) {

//   while (1) {
//     // Always service Modbus & MQTT
//     mb.task();
//     //mqttClient.loop();

//     // ----- Update Modbus registers + display every 5s -----
//     if (millis() - lastDisplay > 5000) {
//       lastDisplay = millis();

//       // Get tank readings
//       float volume_liters = getLevelLiters();     // e.g., 4079.85
//       int volumeInt = (int)round(volume_liters);  // whole liters
//       int heightInt = (int)round(height_mm);      // whole mm

//       // Calculate percentage fill
//       int fillPercent = 0;
//       if (deviceSettings.tankHeight > 0) {
//         fillPercent = (int)round(((float)heightInt / deviceSettings.tankHeight) * 100.0);
//         if (fillPercent > 100) fillPercent = 100;  // clamp max
//         if (fillPercent < 0) fillPercent = 0;      // clamp min
//       }

//       // Update Modbus registers (16-bit safe)
//       mb.Hreg(REG_BASE + 0, (uint16_t)volumeInt);    // Volume
//       mb.Hreg(REG_BASE + 1, (uint16_t)heightInt);    // Height
//       mb.Hreg(REG_BASE + 2, (uint16_t)fillPercent);  // Percentage

//       // ✅ Debug: verify register updates
//       uint16_t regVol = mb.Hreg(REG_BASE + 0);
//       uint16_t regHeight = mb.Hreg(REG_BASE + 1);
//       uint16_t regFill = mb.Hreg(REG_BASE + 2);

//       if (regVol == volumeInt && regHeight == heightInt && regFill == fillPercent) {
//         Serial.printf("✅ Modbus updated: Volume=%u L | Height=%u mm | Fill=%u%%\n",
//                       regVol, regHeight, regFill);
//       } else {
//         Serial.printf("⚠️ Modbus mismatch! Expected (V=%d,H=%d,F=%d), got (V=%u,H=%u,F=%u)\n",
//                       volumeInt, heightInt, fillPercent,
//                       regVol, regHeight, regFill);
//       }

//       // Update display
//       StatusDisplay();
//       if (WiFi.status() == WL_CONNECTED) {
//         //wifiReadyDisplay(local_IP);
//          wifiReadyDisplay();
//         Serial.println("WiFi Connected");
//       }
//       DistanceDisplay(heightInt, volumeInt);
//     }

//     vTaskDelay(10 / portTICK_PERIOD_MS);  // small delay so task yields
//   }

// }





// void wifiReadyDisplay() {
//     // Load stored static IP (or defaults)
//     preferences.begin("netcfg", true);  // read-only

//     IPAddress ip(
//         preferences.getUInt("ip1", DEFAULT_LOCAL_IP[0]),
//         preferences.getUInt("ip2", DEFAULT_LOCAL_IP[1]),
//         preferences.getUInt("ip3", DEFAULT_LOCAL_IP[2]),
//         preferences.getUInt("ip4", DEFAULT_LOCAL_IP[3])
//     );

//     preferences.end();

//     // Clear line 3 on LCD
//     clearLCDLine(3);

//     // Display
//     lcd.setCursor(0, 3);
//     lcd.print("IP: ");
//     lcd.print(ip);  // IPAddress prints in x.x.x.x format
// }



// // void wifiReadyDisplay(IPAddress ip) {
// //   clearLCDLine(3);
// //   lcd.setCursor(0, 3);
// //   lcd.print("IP : ");
// //   lcd.setCursor(5, 3);
// //   // Convert IPAddress to char array
// //   char ipChar[16];  // Enough for "255.255.255.255\0"
// //   snprintf(ipChar, sizeof(ipChar), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

// //   lcd.print(ipChar);  // ✅ print C-style string
// // }



// // void DistanceDisplay(float height_mm, int volume_liters) {
// //   float height;
// //   volume_liters = calculateVolumeLiters(height_mm);


// //   if (height_mm >= 0) {
// //     clearLCDLine(0, 7);
// //     lcd.setCursor(7, 0);
// //     lcd.print(height_mm / 1000.0, 2);  // Display in meters with 2 decimal places
// //     lcd.print(" m");
// //     Serial.print("lcd height_mm: ");
// //     Serial.print(height_mm);

// //     clearLCDLine(1, 8);
// //     lcd.setCursor(8, 1);
// //     lcd.print(volume_liters);
// //     lcd.print(" L");
// //     Serial.print("lcd volume_liters: ");
// //     Serial.print(volume_liters);
// //   } else {
// //     clearLCDLine(0, 7);
// //     lcd.setCursor(7, 0);
// //     lcd.print("Invalid");

// //     clearLCDLine(1, 8);
// //     lcd.setCursor(8, 1);
// //     lcd.print("Reading");
// //   }
// // }
// // void DistanceDisplay(float height_mm, int volume_liters) {
// //   if (height_mm >= 0) {
// //     clearLCDLine(0, 7);
// //     lcd.setCursor(7, 0);
// //     lcd.print(height_mm / 1000.0, 2);  // meters with 2 decimal places
// //     lcd.print(" m");
// //     Serial.print("lcd height_mm: ");
// //     Serial.println(height_mm);

// //     clearLCDLine(1, 8);
// //     lcd.setCursor(8, 1);
// //     lcd.print(volume_liters);
// //     lcd.print(" L");
// //     Serial.print("lcd volume_liters: ");
// //     Serial.println(volume_liters);
// //   } else {
// //     clearLCDLine(0, 7);
// //     lcd.setCursor(7, 0);
// //     lcd.print("Invalid");

// //     clearLCDLine(1, 8);
// //     lcd.setCursor(8, 1);
// //     lcd.print("Reading");
// //   }
// // }
// // void wifiConnection(void* parameters) {
// //   // InitializeWiFi();
// //   while (1) {
// //     mb.task();

// //     // lastDisplay = millis();

// //     if (millis() - lastDisplay > 5000) {
// //       lastDisplay = millis();
// //       StatusDisplay();
// //       if (WiFi.status() == WL_CONNECTED) {
// //         wifiReadyDisplay(local_IP);
// //         Serial.println("WiFi Connected");
// //       }
// //     }

// //     // Optionally update values (e.g., simulate changing data)
// //     static uint32_t lastUpdate = 0;
// //     if (millis() - lastUpdate > 1000) {
// //       lastUpdate = millis();
// //       float volume_liters = getLevelLiters();
// //       mb.Hreg(REG_BASE + 0, volume_liters);
// //       mb.Hreg(REG_BASE + 1, height_mm);
// //       // for (int i = 0; i < REG_COUNT; i++) {
// //       //   mb.Hreg(REG_BASE + i, random(0, 65000));  // Simulate sensor values
// //       // }
// //       //Serial.println("📤 Updated holding registers with new random values");

// //       Serial.print("📤 Updated holding registers: ");
// //       // Serial.print("REG[");
// //       // Serial.print(REG_BASE + 0);
// //       // Serial.print("] = ");
// //       // Serial.print(volume_liters);
// //       // Serial.print(", REG[");
// //       // Serial.print(REG_BASE + 1);
// //       // Serial.print("] = ");
// //       // Serial.println(height_mm);
// //     }
// //   }
// // }
// // // Last known good timestamps
// // unsigned long sensor1_last_good = 0;
// // unsigned long ds_last_good = 0;

// // // Debounced status
// // bool sensor1_stable_ok = false;
// // bool ds_stable_ok = false;
// // int hysteresis = 1;
// // // Constants
// // const unsigned long debouncePeriod = 10000;  // 30 seconds

// // void LCDTask(void *pvParameters) {
// //   static float lastGoodTemp1 = -100.0;
// //   static float lastGoodHumidity1 = -100.0;
// //   static float lastGoodDSTemp = -100.0;

// //   static unsigned long lastUpdate = 0;

// //   const int updateInterval = 1000;

// //   static String lastLine0 = "";
// //   static String lastLine1 = "";

// //   bool highAlert = false;
// //   bool lowAlert = false;

// //   static bool showNetworkStatus = true;
// //   static unsigned long lastToggleTime = 0;
// //   const unsigned long toggleInterval = 3000;  // 3 seconds

// //   for (;;) {
// //     float maxTemp = max(Temperature1, DS_Temperature);
// //     // Serial.print("max temperature: ");
// //     // Serial.println(maxTemp);

// //     // Evaluate High Alert first (takes precedence)
// //     if (maxTemp > high_critical_value) {
// //       highAlert = true;
// //       lowAlert = false;  // clear lowAlert when in highAlert
// //       Serial.println("High Alert");
// //     } else {
// //       highAlert = false;
// //     }

// //     // Only check for lowAlert if highAlert is NOT active
// //     if (!highAlert) {
// //       if (maxTemp > low_critical_value) {
// //         Serial.print("low_critical_value ");
// //         Serial.println(low_critical_value);
// //         Serial.println("LOW Alert");
// //         lowAlert = true;
// //       } else {
// //         lowAlert = false;
// //       }
// //     }

// //     if (highAlert || lowAlert) {
// //       toggleBacklight();
// //     } else lcd.backlight();


// //     unsigned long now = millis();

// //     // --- Update debounced sensor flags
// //     if (sensor1_ok) sensor1_last_good = now;
// //     if (ds_ok) ds_last_good = now;

// //     sensor1_stable_ok = (now - sensor1_last_good <= debouncePeriod);
// //     ds_stable_ok = (now - ds_last_good <= debouncePeriod);

// //     // --- Cache valid readings
// //     if (sensor1_ok) {
// //       lastGoodTemp1 = Temperature1;
// //       if (Humidity1 >= 0 && Humidity1 <= 100) {
// //         lastGoodHumidity1 = Humidity1;
// //       }
// //     }
// //     if (ds_ok) {
// //       lastGoodDSTemp = DS_Temperature;
// //     }

// //     // --- Determine which temperature to evaluate
// //     float activeTemp = ds_stable_ok ? lastGoodDSTemp : lastGoodTemp1;

// //     // --- Apply hysteresis logic
// //     if (activeTemp >= high_critical_value && !highAlert) {
// //       highAlert = true;
// //     } else if (activeTemp <= (high_critical_value - hysteresis) && highAlert) {
// //       highAlert = false;
// //     }

// //     if (activeTemp <= low_critical_value && !lowAlert) {
// //       lowAlert = true;
// //     } else if (activeTemp >= (low_critical_value + hysteresis) && lowAlert) {
// //       lowAlert = false;
// //     }

// //     // --- Every update interval
// //     if (now - lastUpdate >= updateInterval) {
// //       lastUpdate = now;




// //      // Compose Line 0
// //       String line0;
// //       if (!backlightState) {
// //         line0 = "H:" + String(low_critical_value, 1);
// //         line0 += " HH:" + String(high_critical_value, 1);

// //       } else {
// //         line0 = "NetW:";
// //         line0 += networkConnected ? "OK" : "ERR";
// //         line0 += " DATA:";
// //         line0 += mqttConnected ? "YES" : "ERR";
// //       }

// //       // Compose Line 1: Sensor data
// //       String line1;
// //       if (sensor1_stable_ok && ds_stable_ok) {
// //         line1 = "T:" + String(lastGoodDSTemp, 1) + (char)223 + "C";
// //         line1 += " H:" + String(lastGoodHumidity1, 1);
// //       } else if (sensor1_stable_ok) {
// //         line1 = "T:" + String(lastGoodTemp1, 1) + (char)223 + "C";
// //         line1 += " H:" + String(lastGoodHumidity1, 1);
// //       } else if (ds_stable_ok) {
// //         line1 = "T:" + String(lastGoodDSTemp, 1) + (char)223 + "C";
// //       } else {
// //         line1 = "No sensor data   ";  // padded to clear leftovers
// //       }

// //       // Update LCD only if content has changed
// //       if (line0 != lastLine0 || line1 != lastLine1) {
// //         lcd.clear();
// //         lcd.setCursor(0, 0);
// //         lcd.print(line0);
// //         lcd.setCursor(0, 1);
// //         lcd.print(line1);
// //         lastLine0 = line0;
// //         lastLine1 = line1;
// //       }
// //     }
// //     vTaskDelay(200 / portTICK_PERIOD_MS);
// //   }
// // }



// // void displayTask() {
// //   // Create LCD Task on Core 0
// //   xTaskCreatePinnedToCore(
// //     LCDTask,
// //     "LCD Display Task",
// //     4096,
// //     NULL,
// //     1,
// //     &DisplayTaskHandle,
// //     0  // Core 0
// //   );
// // }


// // void lcdBegin() {
// //   lcd.begin();      // Initialize LCD
// //   lcd.backlight();  // Turn on backlight
// //   lcd.clear();      // Optional: Clear display
// //   lcd.setCursor(0, 0);
// //   lcd.print("System Starting");
// // }





// // void toggleBacklight() {
// //   unsigned long now = millis();
// //   const unsigned long interval = 2000;  // 2 seconds

// //   if (now - lastBlinkTime >= interval) {
// //     lastBlinkTime = now;
// //     backlightState = !backlightState;

// //     if (backlightState) {
// //       lcd.backlight();
// //     } else {
// //       lcd.noBacklight();
// //     }
// //   }
// // }
