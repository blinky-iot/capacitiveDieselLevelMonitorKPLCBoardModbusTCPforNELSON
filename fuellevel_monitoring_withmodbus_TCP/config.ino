

// working

#define RESET_PASSWORD "blinkreset"

// Default AP credentials
#define CONFIG_AP_SSID "LIMURU 2"
#define CONFIG_AP_PASS "12345678"
// Timeout for config portal (seconds)
#define CONFIG_TIMEOUT 30


void setupConfig()
{
  preferences.begin("netcfg", false);

  // Load stored Wi-Fi credentials (or defaults)
  String ssid = preferences.getString("ssid", DEFAULT_SSID);
  String password = preferences.getString("pass", DEFAULT_PASSWORD);

  // Load stored mode (0 = DHCP, 1 = Static)
  int currentNetMode = preferences.getInt("netmode", 0); // default = DHCP

  // Load stored Static IP config (or defaults)
  IPAddress local_IP(
      preferences.getUInt("ip1", DEFAULT_LOCAL_IP[0]),
      preferences.getUInt("ip2", DEFAULT_LOCAL_IP[1]),
      preferences.getUInt("ip3", DEFAULT_LOCAL_IP[2]),
      preferences.getUInt("ip4", DEFAULT_LOCAL_IP[3]));
  IPAddress gateway(
      preferences.getUInt("gw1", DEFAULT_GATEWAY[0]),
      preferences.getUInt("gw2", DEFAULT_GATEWAY[1]),
      preferences.getUInt("gw3", DEFAULT_GATEWAY[2]),
      preferences.getUInt("gw4", DEFAULT_GATEWAY[3]));
  IPAddress subnet(
      preferences.getUInt("sn1", DEFAULT_SUBNET[0]),
      preferences.getUInt("sn2", DEFAULT_SUBNET[1]),
      preferences.getUInt("sn3", DEFAULT_SUBNET[2]),
      preferences.getUInt("sn4", DEFAULT_SUBNET[3]));

  // ---- WiFi Manager ----
  WiFiManager wm;
  wm.setConfigPortalTimeout(CONFIG_TIMEOUT);

  wm.setSaveConfigCallback([]()
                           { Serial.println("💾 Config portal saved settings"); });

  // Custom fields
  WiFiManagerParameter custom_mode("mode", "0=DHCP, 1=Static", String(currentNetMode).c_str(), 2);
  WiFiManagerParameter custom_ip("ip", "Static IP (x.x.x.x)", local_IP.toString().c_str(), 16);
  WiFiManagerParameter custom_gw("gw", "Gateway (x.x.x.x)", gateway.toString().c_str(), 16);
  WiFiManagerParameter custom_sn("sn", "Subnet (x.x.x.x)", subnet.toString().c_str(), 16);
  WiFiManagerParameter custom_reset("reset", "Enter reset password to factory reset", "", 32);

  wm.addParameter(&custom_mode);
  wm.addParameter(&custom_ip);
  wm.addParameter(&custom_gw);
  wm.addParameter(&custom_sn);
  wm.addParameter(&custom_reset);

  // LCD feedback - Show config portal info
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("   Connect WIFI    ");
  lcd.setCursor(0, 1);
  lcd.print(CONFIG_AP_SSID);
  lcd.setCursor(0, 2);
  lcd.print("Pass: 12345678");
  lcd.setCursor(3, 3);
  lcd.print("Timeout: ");
  lcd.print(CONFIG_TIMEOUT);

  // ---- Always start AP first ----
  if (!wm.startConfigPortal(CONFIG_AP_SSID, CONFIG_AP_PASS))
  {
    // Config portal timed out - use saved/defaults
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Using saved");
    lcd.setCursor(0, 1);
    lcd.print("credentials...");
    
    Serial.println("⏱️ No config in 20s → using saved/defaults...");

    // Apply stored mode
    if (currentNetMode == 1)
    { // static
      WiFi.config(local_IP, gateway, subnet);
    }
    
    // Start WiFi connection
    WiFi.begin(ssid.c_str(), password.c_str());

    preferences.end();
    
    // Brief display then return to normal
    delay(2000);
    return;
  }

  // ✅ Check for factory reset request
  if (String(custom_reset.getValue()) == RESET_PASSWORD)
  {
    Serial.println("🛑 Factory reset requested, clearing preferences...");
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Factory Reset");
    lcd.setCursor(0, 1);
    lcd.print("Clearing all data");
    lcd.setCursor(0, 2);
    lcd.print("Restarting...");
    
    preferences.clear();
    preferences.end();
    delay(2000);
    ESP.restart();
  }

  // Save WiFi credentials (from portal connection)
  String newSSID = WiFi.SSID();
  String newPass = WiFi.psk();
  preferences.putString("ssid", newSSID);
  preferences.putString("pass", newPass);

  // Get new mode from portal
  int newMode = atoi(custom_mode.getValue());
  preferences.putInt("netmode", newMode);

  // Save Static IP values entered (if any)
  IPAddress newIP, newGW, newSN;
  bool ipChanged = false;
  
  if (newIP.fromString(custom_ip.getValue()))
  {
    preferences.putUInt("ip1", newIP[0]);
    preferences.putUInt("ip2", newIP[1]);
    preferences.putUInt("ip3", newIP[2]);
    preferences.putUInt("ip4", newIP[3]);
    ipChanged = true;
  }
  
  if (newGW.fromString(custom_gw.getValue()))
  {
    preferences.putUInt("gw1", newGW[0]);
    preferences.putUInt("gw2", newGW[1]);
    preferences.putUInt("gw3", newGW[2]);
    preferences.putUInt("gw4", newGW[3]);
    ipChanged = true;
  }
  
  if (newSN.fromString(custom_sn.getValue()))
  {
    preferences.putUInt("sn1", newSN[0]);
    preferences.putUInt("sn2", newSN[1]);
    preferences.putUInt("sn3", newSN[2]);
    preferences.putUInt("sn4", newSN[3]);
    ipChanged = true;
  }

  preferences.end();

  // Apply network configuration immediately
  if (newMode == 1)
  { // Static mode
    if (ipChanged) {
      WiFi.config(newIP, newGW, newSN);
    } else {
      WiFi.config(local_IP, gateway, subnet);
    }
    Serial.println("🌐 Using STATIC IP mode");
    
    // Show confirmation for Static mode
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Static Mode Set");
    lcd.setCursor(0, 1);
    lcd.print("WiFi: " + newSSID.substring(0, 13));
    lcd.setCursor(0, 2);
    lcd.print("IP: ");
    lcd.print(WiFi.localIP());
    lcd.setCursor(0, 3);
    lcd.print("No restart needed");
    
    delay(3000); // Show for 3 seconds
  }
  else
  { // DHCP mode
    Serial.println("🌐 Using DHCP mode");
    
    // Show restart message for DHCP mode
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DHCP Mode Set");
    lcd.setCursor(0, 1);
    lcd.print("WiFi: " + newSSID.substring(0, 13));
    lcd.setCursor(0, 2);
    lcd.print("IP: ");
    lcd.print(WiFi.localIP());
    lcd.setCursor(0, 3);
    lcd.print("Restarting in 3s...");
    
    Serial.print("✅ WiFi Connected → IP: ");
    Serial.println(WiFi.localIP());
    
    // ⭐ CRITICAL: RESTART for DHCP mode
    Serial.println("🔄 DHCP mode selected - restarting in 3 seconds...");
    Serial.println("🔄 Reason: DHCP mode requires MQTT initialization");
    
    delay(3000); // Show message for 3 seconds
    
    // Perform restart
    ESP.restart();
  }
}