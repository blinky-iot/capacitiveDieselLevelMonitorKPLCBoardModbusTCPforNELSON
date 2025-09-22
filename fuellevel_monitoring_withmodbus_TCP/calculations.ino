/*
  calculations.ino – Tank Volume Calculations for Rectangular and Cylindrical Tanks
  -------------------------------------------------------------------------------
  - Dynamically reads tank dimensions and type from `deviceSettings`
  - Calculates volume and fill percentage from measured fuel height
*/

extern DeviceSettings deviceSettings;

// Calculate cross-sectional area of partially filled cylinder
double XSectionalArea(float height, float radius) {
  double area = 0;

  if (height >= 2 * radius) {
    area = PI * pow(radius, 2);
  } else if (height > radius) {
    double theta = acos((radius - height) / radius);
    area = pow(radius, 2) * (theta - 0.5 * sin(2 * theta));
  } else if (height < radius) {
    double theta = acos((radius - height) / radius);
    area = pow(radius, 2) * (PI - theta + 0.5 * sin(2 * theta));
  } else {  // height == radius
    area = 0.5 * PI * pow(radius, 2);
  }

  return area;
}

// Cylindrical Tank Volume (Liters)
double calculateCylindricalVolume(float height) {
  double crossArea = XSectionalArea(height, deviceSettings.tankRadius);
  return (crossArea * deviceSettings.tankLength) / 1000000.0;  // mm³ to Liters
}

// Rectangular Tank Volume (Liters)
double calculateRectangularVolume(float height) {
  double volume_cm3 = deviceSettings.tankLength * deviceSettings.tankWidth * height;
  return volume_cm3 / 1000000.0;  // mm³ to Liters
}

// General Tank Volume from current level height (mm)
// float calculateVolumeLiters(float height) {
//   if (strcmp(deviceSettings.tankType, "cylindrical") == 0) {
//     return calculateCylindricalVolume(height);
//   } else {
//     return calculateRectangularVolume(height);
//   }
// }

float calculateVolumeLiters(float height) {
  if (strcmp(deviceSettings.tankType, "cylindrical") == 0) {
    Serial.println("📦 Calculating volume for cylindrical tank");
    return calculateCylindricalVolume(height);

  } else if (strcmp(deviceSettings.tankType, "rectangular") == 0) {
    Serial.println("📦 Calculating volume for rectangular tank");
    return calculateRectangularVolume(height);

  } else {
    Serial.println("❌ Unknown tank type: " + String(deviceSettings.tankType));
    return 0.0;
  }
}


// Fill Percentage based on full tank volume
int FillPercentage(float volume) {
  float fullVolume;

  if (strcmp(deviceSettings.tankType, "cylindrical") == 0) {
    fullVolume = (PI * pow(deviceSettings.tankRadius, 2) * deviceSettings.tankLength) / 1000000.0;
  } else {
    fullVolume = (deviceSettings.tankLength * deviceSettings.tankWidth * deviceSettings.tankHeight) / 1000000.0;
  }

  return round((volume / fullVolume) * 100.0);
}
