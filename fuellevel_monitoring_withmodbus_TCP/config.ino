

//working

#define RESET_PASSWORD "blinkreset"

// Default AP credentials
#define CONFIG_AP_SSID "COSMOS01"
#define CONFIG_AP_PASS "12345678" 
// Timeout for config portal (seconds)
#define CONFIG_TIMEOUT 60

void setupConfig() {
  preferences.begin("netcfg", false);

  // Load stored Wi-Fi credentials (or defaults)
  String ssid = preferences.getString("ssid", DEFAULT_SSID);
  String password = preferences.getString("pass", DEFAULT_PASSWORD);

  // Load stored mode (0 = DHCP, 1 = Static)
  int netMode = preferences.getInt("netmode", 0);  // default = DHCP

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

  wm.setSaveConfigCallback([]() {
    Serial.println("💾 Config portal saved settings");
  });

  // Custom fields
  WiFiManagerParameter custom_mode("mode", "0=DHCP, 1=Static", String(netMode).c_str(), 2);
  WiFiManagerParameter custom_ip("ip", "Static IP (x.x.x.x)", local_IP.toString().c_str(), 16);
  WiFiManagerParameter custom_gw("gw", "Gateway (x.x.x.x)", gateway.toString().c_str(), 16);
  WiFiManagerParameter custom_sn("sn", "Subnet (x.x.x.x)", subnet.toString().c_str(), 16);
  WiFiManagerParameter custom_reset("reset", "Enter reset password to factory reset", "", 32);

  wm.addParameter(&custom_mode);
  wm.addParameter(&custom_ip);
  wm.addParameter(&custom_gw);
  wm.addParameter(&custom_sn);
  wm.addParameter(&custom_reset);

  // LCD feedback
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("   Connect WIFI    ");
  lcd.setCursor(0, 1);
  lcd.print(CONFIG_AP_SSID);
  lcd.setCursor(0, 2);
  lcd.print("Pass: 12345678");
  lcd.setCursor(3, 3);
  lcd.print("Timeout: 20s");

  // ---- Always start AP first ----
  if (!wm.startConfigPortal(CONFIG_AP_SSID, CONFIG_AP_PASS)) {
    Serial.println("⏱️ No config in 20s → using saved/defaults...");

    if (netMode == 1) {  // static
      WiFi.config(local_IP, gateway, subnet);
    }
    WiFi.begin(ssid.c_str(), password.c_str());

    preferences.end();
    return;
  }

  // ✅ Check for factory reset request
  if (String(custom_reset.getValue()) == RESET_PASSWORD) {
    Serial.println("🛑 Factory reset requested, clearing preferences...");
    preferences.clear();
    preferences.end();
    delay(1000);
    ESP.restart();
  }

  // Save WiFi creds
  preferences.putString("ssid", WiFi.SSID());
  preferences.putString("pass", WiFi.psk());

  // Save mode
  preferences.putInt("netmode", atoi(custom_mode.getValue()));

  // Save Static IP values entered
  IPAddress newIP, newGW, newSN;
  if (newIP.fromString(custom_ip.getValue())) {
    preferences.putUInt("ip1", newIP[0]);
    preferences.putUInt("ip2", newIP[1]);
    preferences.putUInt("ip3", newIP[2]);
    preferences.putUInt("ip4", newIP[3]);
  }
  if (newGW.fromString(custom_gw.getValue())) {
    preferences.putUInt("gw1", newGW[0]);
    preferences.putUInt("gw2", newGW[1]);
    preferences.putUInt("gw3", newGW[2]);
    preferences.putUInt("gw4", newGW[3]);
  }
  if (newSN.fromString(custom_sn.getValue())) {
    preferences.putUInt("sn1", newSN[0]);
    preferences.putUInt("sn2", newSN[1]);
    preferences.putUInt("sn3", newSN[2]);
    preferences.putUInt("sn4", newSN[3]);
  }

  preferences.end();

  // Apply chosen mode
  netMode = atoi(custom_mode.getValue());
  if (netMode == 1) {
    WiFi.config(newIP, newGW, newSN);
    Serial.println("🌐 Using STATIC IP mode");
  } else {
    Serial.println("🌐 Using DHCP mode");
  }

  Serial.print("✅ WiFi Connected → IP: ");
  Serial.println(WiFi.localIP());
}
