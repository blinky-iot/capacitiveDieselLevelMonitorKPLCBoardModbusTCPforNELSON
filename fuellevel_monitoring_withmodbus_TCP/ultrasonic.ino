// // UltrasonicSensor.cpp (main implementation)
// #include "UltrasonicSensor.h"

// // Global variables
// HardwareSerial ultrasonicLevel(1);


// // Filter variables
// #define FILTER_SIZE 5
// float distanceHistory[FILTER_SIZE] = { 0 };
// int historyIndex = 0;
// int validHistoryCount = 0;

// // Statistics
// unsigned long totalReadings = 0;
// unsigned long failedReadings = 0;
// unsigned long lastResetTime = 0;

// // External watchdog (if used)
// #ifdef USE_WATCHDOG
// extern WatchdogManager wdtManager;
// #endif

// // --- Setup function ---
// void setupUltrasonic() {
// #ifdef USE_WATCHDOG
//   wdtManager.criticalSectionStart();
// #endif

// // Initialize power pin if defined
// #ifdef PIN_555_ULTRASONIC
//   pinMode(PIN_555_ULTRASONIC, OUTPUT);
//   digitalWrite(PIN_555_ULTRASONIC, HIGH);
// #endif

//   // Initialize serial communication
//   ultrasonicLevel.begin(ULTRASONIC_BAUD_RATE, SERIAL_8N1, A02YYUW_RX, A02YYUW_TX);

//   // Give sensor time to initialize
//   delay(500);

//   // Clear any pending data
//   while (ultrasonicLevel.available()) {
//     ultrasonicLevel.read();
//   }

//   Serial.println("✅ Ultrasonic sensor initialized.");
//   Serial.printf("  Pins: RX=%d, TX=%d\n", A02YYUW_RX, A02YYUW_TX);
//   Serial.printf("  Range: %d-%d mm\n", ULTRASONIC_MIN_RANGE_MM, ULTRASONIC_MAX_RANGE_MM);

// #ifdef USE_WATCHDOG
//   wdtManager.criticalSectionEnd();
// #endif
// }

// // --- Robust single reading with timeout ---
// float readUltrasonicCM() {
// #ifdef USE_WATCHDOG
//   wdtManager.criticalSectionStart();
// #endif

//   unsigned long startTime = millis();
//   uint8_t data[4] = { 0 };
//   int bytesRead = 0;

//   // Clear buffer before reading
//   while (ultrasonicLevel.available()) {
//     ultrasonicLevel.read();
//   }

//   // Wait for complete frame with timeout
//   while ((millis() - startTime) < ULTRASONIC_READ_TIMEOUT_MS) {
// #ifdef USE_WATCHDOG
//     wdtManager.feedIfNeeded();
// #endif

//     // Check for available data
//     while (ultrasonicLevel.available() && bytesRead < 4) {
//       data[bytesRead] = ultrasonicLevel.read();
//       bytesRead++;
//     }

//     // If we have 4 bytes, process them
//     if (bytesRead == 4) {
//       // Check for valid header
//       if (data[0] != 0xFF) {
//         // Invalid header, shift buffer and continue
//         for (int i = 0; i < 3; i++) {
//           data[i] = data[i + 1];
//         }
//         bytesRead = 3;
//         continue;
//       }

//       // Calculate checksum
//       uint8_t checksum = (data[0] + data[1] + data[2]) & 0xFF;

//       if (checksum == data[3]) {
//         // Valid checksum
//         int distanceMM = (data[1] << 8) | data[2];
//         totalReadings++;

//         // Validate range
//         if (distanceMM >= ULTRASONIC_MIN_RANGE_MM && distanceMM <= ULTRASONIC_MAX_RANGE_MM) {
// #ifdef USE_WATCHDOG
//           wdtManager.criticalSectionEnd();
// #endif
//           return (float)distanceMM;
//         } else if (distanceMM < ULTRASONIC_MIN_RANGE_MM) {
// #ifdef USE_WATCHDOG
//           wdtManager.criticalSectionEnd();
// #endif
//           failedReadings++;
//           return (float)ULTRASONIC_TOO_CLOSE;
//         } else {
// #ifdef USE_WATCHDOG
//           wdtManager.criticalSectionEnd();
// #endif
//           failedReadings++;
//           return (float)ULTRASONIC_INVALID_DATA;
//         }
//       } else {
//         // Checksum failed
//         failedReadings++;
// #ifdef USE_WATCHDOG
//         wdtManager.criticalSectionEnd();
// #endif
//         return (float)ULTRASONIC_CHECKSUM_ERROR;
//       }
//     }

//     // Small delay to prevent CPU hogging
//     delay(1);
//   }

//   // Timeout occurred
//   failedReadings++;
// #ifdef USE_WATCHDOG
//   wdtManager.criticalSectionEnd();
// #endif
//   return (float)ULTRASONIC_TIMEOUT_ERROR;
// }

// // --- Update distance with retry logic ---
// void updateUltrasonicDistance() {
//   float x = 0;

//   for (int attempt = 0; attempt < ULTRASONIC_RETRY_COUNT; attempt++) {
//     x = readUltrasonicCM();

//     if (x > 0) {  // Valid distance reading
//       // Update history for filtering
//       distanceHistory[historyIndex] = x;
//       historyIndex = (historyIndex + 1) % FILTER_SIZE;
//       if (validHistoryCount < FILTER_SIZE) validHistoryCount++;

//       Height_ultrsnc = x;
//       ultrasonicSensorStatus = ULTRASONIC_OK;

//       // Reset failure counter on success
//       if (failedReadings > 0) {
//         failedReadings = max((unsigned long)0, failedReadings - 2);
//       }

//       return;  // Success, exit function
//     } else {
//       // Update status with error code
//       ultrasonicSensorStatus = (int)x;

//       // Optional: Log specific errors only occasionally
//       static int errorLogCounter = 0;
//       if (errorLogCounter++ % 20 == 0) {
//         switch (ultrasonicSensorStatus) {
//           case ULTRASONIC_TOO_CLOSE:
//             Serial.println("⚠️ Ultrasonic: Object too close (<30mm)");
//             break;
//           case ULTRASONIC_CHECKSUM_ERROR:
//             Serial.println("❌ Ultrasonic: Checksum error");
//             break;
//           case ULTRASONIC_TIMEOUT_ERROR:
//             Serial.println("⏱️ Ultrasonic: Read timeout");
//             break;
//           case ULTRASONIC_INVALID_DATA:
//             Serial.println("📊 Ultrasonic: Data out of valid range");
//             break;
//         }
//       }

//       // Small delay between retries
//       if (attempt < (ULTRASONIC_RETRY_COUNT - 1)) {
//         delay(ULTRASONIC_RETRY_DELAY_MS);
//       }
//     }
//   }

//   // All retries failed
//   Serial.println("🔴 Ultrasonic: All retry attempts failed");

//   // Auto-reset sensor after many failures
//   if (failedReadings > 50) {
//     Serial.println("🔄 Ultrasonic: Auto-resetting sensor due to repeated failures");
//     resetUltrasonicSensor();
//     failedReadings = 0;
//   }
// }

// // --- Get filtered distance (moving average) ---
// float getFilteredDistance() {
//   if (validHistoryCount == 0) {
//     return Height_ultrsnc;  // No history yet
//   }

//   // Remove outliers first (more than 20% deviation from median)
//   float values[FILTER_SIZE];
//   int count = 0;

//   for (int i = 0; i < FILTER_SIZE; i++) {
//     if (distanceHistory[i] > 0) {
//       values[count++] = distanceHistory[i];
//     }
//   }

//   if (count == 0) return Height_ultrsnc;

//   // Simple moving average
//   float sum = 0;
//   for (int i = 0; i < count; i++) {
//     sum += values[i];
//   }

//   return sum / count;
// }

// // --- Reset sensor (power cycle) ---
// void resetUltrasonicSensor() {
// #ifdef USE_WATCHDOG
//   wdtManager.criticalSectionStart();
// #endif

// #ifdef PIN_555_ULTRASONIC
//   Serial.println("🔌 Ultrasonic: Power cycling sensor...");

//   digitalWrite(PIN_555_ULTRASONIC, LOW);
//   delay(100);
//   digitalWrite(PIN_555_ULTRASONIC, HIGH);
//   delay(1000);  // Wait for sensor to reboot

//   // Reinitialize serial
//   ultrasonicLevel.end();
//   delay(100);
//   ultrasonicLevel.begin(ULTRASONIC_BAUD_RATE, SERIAL_8N1, A02YYUW_RX, A02YYUW_TX);
//   delay(500);

//   // Clear buffer
//   while (ultrasonicLevel.available()) {
//     ultrasonicLevel.read();
//   }

//   Serial.println("✅ Ultrasonic: Sensor reset complete");
// #endif

//   // Reset statistics
//   validHistoryCount = 0;
//   historyIndex = 0;
//   for (int i = 0; i < FILTER_SIZE; i++) {
//     distanceHistory[i] = 0;
//   }

//   lastResetTime = millis();

// #ifdef USE_WATCHDOG
//   wdtManager.criticalSectionEnd();
// #endif
// }

// // --- Debug function ---
// void debugUltrasonicSensor() {
//   Serial.println("\n🔍 Ultrasonic Sensor Debug:");
//   Serial.printf("  Last reading: %.1f mm\n", Height_ultrsnc);
//   Serial.printf("  Status: %d ", ultrasonicSensorStatus);

//   switch (ultrasonicSensorStatus) {
//     case ULTRASONIC_OK: Serial.println("(OK)"); break;
//     case ULTRASONIC_TOO_CLOSE: Serial.println("(Too close)"); break;
//     case ULTRASONIC_CHECKSUM_ERROR: Serial.println("(Checksum error)"); break;
//     case ULTRASONIC_TIMEOUT_ERROR: Serial.println("(Timeout)"); break;
//     case ULTRASONIC_INVALID_DATA: Serial.println("(Invalid data)"); break;
//     default: Serial.println("(Unknown)"); break;
//   }

//   Serial.printf("  Statistics: %lu total, %lu failed (%.1f%% success)\n",
//                 totalReadings, failedReadings,
//                 totalReadings > 0 ? 100.0 * (totalReadings - failedReadings) / totalReadings : 0.0);

//   Serial.printf("  Filtered distance: %.1f mm\n", getFilteredDistance());
//   Serial.printf("  History buffer: %d/%d valid readings\n", validHistoryCount, FILTER_SIZE);

//   // Test readings
//   Serial.println("  Test readings:");
//   for (int i = 0; i < 3; i++) {
//     float test = readUltrasonicCM();
//     if (test > 0) {
//       Serial.printf("    Attempt %d: %.1f mm\n", i + 1, test);
//     } else {
//       Serial.printf("    Attempt %d: Error %d\n", i + 1, (int)test);
//     }
//     if (i < 2) delay(200);
//   }

//   Serial.println("🔍 Debug complete\n");
// }

// // --- Helper functions ---
// bool isUltrasonicDataValid() {
//   return (ultrasonicSensorStatus == ULTRASONIC_OK);
// }

// int getUltrasonicStatus() {
//   return ultrasonicSensorStatus;
// }

// float getCurrentDistance() {
//   return Height_ultrsnc;
// }

// float getCurrentDistanceCM() {
//   return Height_ultrsnc / 10.0;
// }

// float getCurrentDistanceMeters() {
//   return Height_ultrsnc / 1000.0;
// }

// // --- Optional: Task-based update (if using FreeRTOS) ---
// #ifdef USE_FREERTOS
// void ultrasonicTask(void *parameter) {
//   setupUltrasonic();

//   // Initial delay for stabilization
//   vTaskDelay(pdMS_TO_TICKS(2000));

//   // Initial debug
//   debugUltrasonicSensor();

//   while (true) {
//     updateUltrasonicDistance();

//     // Optional: Log successful readings occasionally
//     static int logCounter = 0;
//     if (isUltrasonicDataValid() && (logCounter++ % 100 == 0)) {
//       Serial.printf("📏 Ultrasonic: %.1f mm (filtered: %.1f mm)\n",
//                     Height_ultrsnc, getFilteredDistance());
//     }

//     vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz update rate
//   }
// }
// #endif



// /*#define A02YYUW_TX 25
// #define A02YYUW_RX 26

// HardwareSerial ultrasonicLevel(1);
// float distance;
// void setupUltrasonic() {
//   pinMode(PIN_555_ultrasonic, OUTPUT);
//   digitalWrite(PIN_555_ultrasonic, HIGH);
//   ultrasonicLevel.begin(9600, SERIAL_8N1, A02YYUW_RX, A02YYUW_TX);
//   Serial.println("✅ Ultrasonic sensor initialized.");
// }

// // float readUltrasonicCM() {

// //   float finalDistance = 0;
// //   uint8_t data[4];

// //   do {
// //     for (int i = 0; i < 4; i++) {
// //       data[i] = ultrasonicLevel.read();
// //     }
// //   } while (ultrasonicLevel.read() == 0xff);

// //   ultrasonicLevel.flush();

// //   if (data[0] == 0xff) {
// //     int sum;
// //     sum = (data[0] + data[1] + data[2]) & 0x00FF;
// //     if (sum == data[3]) {
// //       distance = (data[1] << 8) + data[2];
// //       if (distance > 30) {
// //         Serial.print("distance=");
// //         finalDistance = distance;
// //         Serial.print(finalDistance);
// //         ultraSonicSensorOk = 1;
// //         Serial.println("mm");
// //       } else {
// //         Serial.println("Below the lower limit");
// //         ultraSonicSensorOk = 2;
// //       }
// //     } else {
// //       Serial.println("ERROR");
// //       ultraSonicSensorOk = 0;
// //     }
// //   }
// //   delay(150);
// //   //Serial.println(finalDistance);
// //   return finalDistance;
// // }

// float readUltrasonicCM() {
//   //delay(200);
//   uint8_t data[4];

//   // Read 4-byte response
//   if (ultrasonicLevel.available()) {
//     do {
//       for (int i = 0; i < 4; i++) {
//         data[i] = ultrasonicLevel.read();
//       }
//     } while (ultrasonicLevel.read() == 0XFF);
//   }
//   //Serial.printf("Raw: %02X %02X %02X %02X\n", data[0], data[1], data[2], data[3]);
//   // Validate packet
//   if (data[0] == 0xFF) {
//     int checksum = (data[0] + data[1] + data[2]) & 0xFF;
//     if (checksum == data[3]) {
//       int distanceMM = (data[1] << 8) + data[2];
//       //float distanceCM = distanceMM / 10.0;

//       if (distanceMM > 30) {
//         // Serial.print("📏 Distance: ");
//         // Serial.print(distanceMM);
//         // Serial.println(" mm");
//         return distanceMM;
//       } else {
//         // Serial.println("⚠️ Too close / Below valid range");
//         return -2;
//       }
//     } else {
//       //Serial.println("❌ Checksum error");
//       return -1;
//     }
//   }
// }

// void updateUltrasonicDistance(){
//     for (int i = 0; i < 10; i++) {
//     x = readUltrasonicCM();  // Must run frequently
//     if (x != -1 && x != -2) {
//       Height_ultrsnc = x;
//       ultrasonicSensorStatus = 1;
//     } else {
//       ultrasonicSensorStatus = x;
//      //Serial.println(x);
//     }
//   }
// }
// */