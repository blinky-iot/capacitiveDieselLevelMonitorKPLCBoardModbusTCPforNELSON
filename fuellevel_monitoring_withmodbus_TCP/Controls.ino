void valveState(int percentageFill) {
  if (percentageFill <= 50 ) {
    if ((millis() - levelStability) >= levelStabilityTime) valveOpen = true;
  }
  else if (percentageFill >= 80 ){
    if ((millis() - levelStability) >= levelStabilityTime) valveOpen = false;
  } else levelStability = millis();
  
   controlValve(valveOpen);
}

void controlValve(bool stateOfValve) {
  if (stateOfValve != previousstateOfValve) {
    if (stateOfValve) {
      Serial.println("Opening Valve");
     digitalWrite(valvePin,HIGH);

    } else {
      Serial.println("Closing Valve");
      digitalWrite(valvePin,LOW);
    }
  }
  previousstateOfValve = stateOfValve;
}

