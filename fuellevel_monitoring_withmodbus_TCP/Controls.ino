
// ---------------- Valve Control ----------------
void valveState(int percentageFill) {
  Serial.printf("🔎 Checking valve state... Fill = %d%%\n", percentageFill);

  // Validate input
  if (percentageFill < 0 || percentageFill > 100) {
    Serial.printf("⚠️ Invalid percentageFill (%d) → Skipping valve control\n", percentageFill);
    return;
  }

  // Control logic with stability check
  if (percentageFill <= 65) {
    if ((millis() - levelStability) >= levelStabilityTime) {
      Serial.printf("📉 Fill ≤ 70%% → Requesting valve OPEN (stable %lu ms)\n", millis() - levelStability);
      valveOpen = true;
    } else {
      Serial.printf("⏱️ Waiting stability... %lu/%lu ms elapsed\n", millis() - levelStability, levelStabilityTime);
    }
  } 
  else if (percentageFill >= 70) {
    if ((millis() - levelStability) >= levelStabilityTime) {
      Serial.printf("📈 Fill ≥ 80%% → Requesting valve CLOSE (stable %lu ms)\n", millis() - levelStability);
      valveOpen = false;
    } else {
      Serial.printf("⏱️ Waiting stability... %lu/%lu ms elapsed\n", millis() - levelStability, levelStabilityTime);
    }
  } 
  else {
    Serial.println("⏳ Fill between 71–79% → Holding stable, resetting stability timer");
    levelStability = millis();
  }

  // Apply control
  controlValve(valveOpen);
}

void controlValve(bool stateOfValve) {
  if (stateOfValve != previousstateOfValve) {
    if (stateOfValve) {
      Serial.println("✅ Action: Opening valve (pin HIGH)");
      digitalWrite(valvePin, HIGH);
    } else {
      Serial.println("✅ Action: Closing valve (pin LOW)");
      digitalWrite(valvePin, LOW);
    }
  } else {
    // Valve state unchanged
    Serial.printf("ℹ️ Valve already %s → No action\n", stateOfValve ? "OPEN" : "CLOSED");
  }

  previousstateOfValve = stateOfValve;
}

// ---------------- Setup Initialization ----------------
void initValveControl() {
  pinMode(valvePin, OUTPUT);
  digitalWrite(valvePin, HIGH);  // ensure valve starts CLOSED
  delay(2000);
    digitalWrite(valvePin, LOW);  // ensure valve starts CLOSED
  delay(2000);
  Serial.println("⚙️ Valve control initialized (default: CLOSED)");
}




// void valveState(int percentageFill) {
//   if (percentageFill <= 50 ) {
//     if ((millis() - levelStability) >= levelStabilityTime) valveOpen = true;
//   }
//   else if (percentageFill >= 80 ){
//     if ((millis() - levelStability) >= levelStabilityTime) valveOpen = false;
//   } else levelStability = millis();
  
//    controlValve(valveOpen);
// }

// void controlValve(bool stateOfValve) {
//   if (stateOfValve != previousstateOfValve) {
//     if (stateOfValve) {
//       Serial.println("Opening Valve");
//      digitalWrite(valvePin,HIGH);

//     } else {
//       Serial.println("Closing Valve");
//       digitalWrite(valvePin,LOW);
//     }
//   }
//   previousstateOfValve = stateOfValve;
// }

// void valveState(int percentageFill) {
//   Serial.printf("🔎 Checking valve state... Fill = %d%%\n", percentageFill);

//   if (percentageFill <= 70) {
//     if ((millis() - levelStability) >= levelStabilityTime) {
//       Serial.println("📉 Fill ≤ 50% → Requesting valve OPEN");
//       valveOpen = true;
//     }
//   } 
//   else if (percentageFill >= 80) {
//     if ((millis() - levelStability) >= levelStabilityTime) {
//       Serial.println("📈 Fill ≥ 80% → Requesting valve CLOSE");
//       valveOpen = false;
//     }
//   } 
//   else {
//     Serial.println("⏳ Fill between 51–79% → Holding stable, resetting stability timer");
//     levelStability = millis();
//   }

//   // Apply control
//   controlValve(valveOpen);
// }

// void controlValve(bool stateOfValve) {
//   if (stateOfValve != previousstateOfValve) {
//     if (stateOfValve) {
//       Serial.println("✅ Action: Opening valve (pin HIGH)");
//       digitalWrite(valvePin, HIGH);
//     } else {
//       Serial.println("✅ Action: Closing valve (pin LOW)");
//       digitalWrite(valvePin, LOW);
//     }
//   } else {
//     // Valve state unchanged
//     Serial.printf("ℹ️ Valve already %s → No action\n", stateOfValve ? "OPEN" : "CLOSED");
//   }

//   previousstateOfValve = stateOfValve;
// }