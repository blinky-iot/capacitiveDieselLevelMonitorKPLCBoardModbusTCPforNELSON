#define A02YYUW_TX 25
#define A02YYUW_RX 26

HardwareSerial ultrasonicLevel(1);
float distance;
void setupUltrasonic() {
  pinMode(PIN_555_ultrasonic, OUTPUT);
  digitalWrite(PIN_555_ultrasonic, HIGH);
  ultrasonicLevel.begin(9600, SERIAL_8N1, A02YYUW_RX, A02YYUW_TX);
  Serial.println("✅ Ultrasonic sensor initialized.");
}

// float readUltrasonicCM() {

//   float finalDistance = 0;
//   uint8_t data[4];

//   do {
//     for (int i = 0; i < 4; i++) {
//       data[i] = ultrasonicLevel.read();
//     }
//   } while (ultrasonicLevel.read() == 0xff);

//   ultrasonicLevel.flush();

//   if (data[0] == 0xff) {
//     int sum;
//     sum = (data[0] + data[1] + data[2]) & 0x00FF;
//     if (sum == data[3]) {
//       distance = (data[1] << 8) + data[2];
//       if (distance > 30) {
//         Serial.print("distance=");
//         finalDistance = distance;
//         Serial.print(finalDistance);
//         ultraSonicSensorOk = 1;
//         Serial.println("mm");
//       } else {
//         Serial.println("Below the lower limit");
//         ultraSonicSensorOk = 2;
//       }
//     } else {
//       Serial.println("ERROR");
//       ultraSonicSensorOk = 0;
//     }
//   }
//   delay(150);
//   //Serial.println(finalDistance);
//   return finalDistance;
// }

float readUltrasonicCM() {
  //delay(200);
  uint8_t data[4];

  // Read 4-byte response
  if (ultrasonicLevel.available()) {
    do {
      for (int i = 0; i < 4; i++) {
        data[i] = ultrasonicLevel.read();
      }
    } while (ultrasonicLevel.read() == 0XFF);
  }
  //Serial.printf("Raw: %02X %02X %02X %02X\n", data[0], data[1], data[2], data[3]);
  // Validate packet
  if (data[0] == 0xFF) {
    int checksum = (data[0] + data[1] + data[2]) & 0xFF;
    if (checksum == data[3]) {
      int distanceMM = (data[1] << 8) + data[2];
      //float distanceCM = distanceMM / 10.0;

      if (distanceMM > 30) {
        // Serial.print("📏 Distance: ");
        // Serial.print(distanceMM);
        // Serial.println(" mm");
        return distanceMM;
      } else {
        // Serial.println("⚠️ Too close / Below valid range");
        return -2;
      }
    } else {
      //Serial.println("❌ Checksum error");
      return -1;
    }
  }
}

void updateUltrasonicDistance(){
    for (int i = 0; i < 10; i++) {
    x = readUltrasonicCM();  // Must run frequently
    if (x != -1 && x != -2) {
      Height_ultrsnc = x;
      ultrasonicSensorStatus = 1;
    } else {
      ultrasonicSensorStatus = x;
     //Serial.println(x);
    }
  }
}
