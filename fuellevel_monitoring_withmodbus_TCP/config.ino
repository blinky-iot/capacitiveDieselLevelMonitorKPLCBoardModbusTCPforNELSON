// #include <WiFiManager.h>    // https://github.com/tzapu/WiFiManager
// #include <Preferences.h>
// #include "definations.h"

// Preferences preferences;

// Password for reset confirmation
/*
#define RESET_PASSWORD "blinkreset2025"

void setupConfig() {
  preferences.begin("netcfg", false);

  // Load stored or default Wi-Fi credentials
  String ssid     = preferences.getString("ssid", DEFAULT_SSID);
  String password = preferences.getString("pass", DEFAULT_PASSWORD);

  // Load stored or default Static IP config
  IPAddress local_IP(
    preferences.getUInt("ip1", DEFAULT_LOCAL_IP[0]),
    preferences.getUInt("ip2", DEFAULT_LOCAL_IP[1]),
    preferences.getUInt("ip3", DEFAULT_LOCAL_IP[2]),
    preferences.getUInt("ip4", DEFAULT_LOCAL_IP[3])
  );
  IPAddress gateway(
    preferences.getUInt("gw1", DEFAULT_GATEWAY[0]),
    preferences.getUInt("gw2", DEFAULT_GATEWAY[1]),
    preferences.getUInt("gw3", DEFAULT_GATEWAY[2]),
    preferences.getUInt("gw4", DEFAULT_GATEWAY[3])
  );
  IPAddress subnet(
    preferences.getUInt("sn1", DEFAULT_SUBNET[0]),
    preferences.getUInt("sn2", DEFAULT_SUBNET[1]),
    preferences.getUInt("sn3", DEFAULT_SUBNET[2]),
    preferences.getUInt("sn4", DEFAULT_SUBNET[3])
  );

  // ----- WiFi Manager -----
  WiFiManager wm;
  wm.setSaveConfigCallback([]() {
    Serial.println("💾 Config portal saved settings");
  });

  // Custom fields for Static IP
  WiFiManagerParameter custom_ip("ip", "Static IP (x.x.x.x)", local_IP.toString().c_str(), 16);
  WiFiManagerParameter custom_gw("gw", "Gateway (x.x.x.x)", gateway.toString().c_str(), 16);
  WiFiManagerParameter custom_sn("sn", "Subnet (x.x.x.x)", subnet.toString().c_str(), 16);

  // Custom reset password field
  WiFiManagerParameter custom_reset("reset", "Enter reset password to factory reset", "", 32);

  wm.addParameter(&custom_ip);
  wm.addParameter(&custom_gw);
  wm.addParameter(&custom_sn);
  wm.addParameter(&custom_reset);

  // Try auto-connect, fallback to AP mode if fails
  if (!wm.autoConnect("ESP32_ConfigAP")) {
    Serial.println("❌ Failed to connect, rebooting...");
    ESP.restart();
  }

  // ✅ Check for factory reset request
  if (String(custom_reset.getValue()) == RESET_PASSWORD) {
    Serial.println("🛑 Factory reset requested, clearing preferences...");
    preferences.clear();   // wipe all saved settings
    preferences.end();
    delay(1000);
    ESP.restart();
  }

  // Save WiFi creds
  preferences.putString("ssid", WiFi.SSID());
  preferences.putString("pass", WiFi.psk());

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

  Serial.print("✅ WiFi Connected → IP: ");
  Serial.println(WiFi.localIP());
}
*/

//working
/*
#define RESET_PASSWORD "blinkreset2025"

// Default AP credentials
#define CONFIG_AP_SSID "ESP32_ConfigAP"

// Timeout for config portal (seconds)
#define CONFIG_TIMEOUT 60

void setupConfig() {
  preferences.begin("netcfg", false);

  // Load stored Wi-Fi credentials (or defaults)
  String ssid     = preferences.getString("ssid", DEFAULT_SSID);
  String password = preferences.getString("pass", DEFAULT_PASSWORD);

  // Load stored mode (0 = DHCP, 1 = Static)
  int netMode = preferences.getInt("netmode", 0); // default = DHCP

  // Load stored Static IP config (or defaults)
  IPAddress local_IP(
    preferences.getUInt("ip1", DEFAULT_LOCAL_IP[0]),
    preferences.getUInt("ip2", DEFAULT_LOCAL_IP[1]),
    preferences.getUInt("ip3", DEFAULT_LOCAL_IP[2]),
    preferences.getUInt("ip4", DEFAULT_LOCAL_IP[3])
  );
  IPAddress gateway(
    preferences.getUInt("gw1", DEFAULT_GATEWAY[0]),
    preferences.getUInt("gw2", DEFAULT_GATEWAY[1]),
    preferences.getUInt("gw3", DEFAULT_GATEWAY[2]),
    preferences.getUInt("gw4", DEFAULT_GATEWAY[3])
  );
  IPAddress subnet(
    preferences.getUInt("sn1", DEFAULT_SUBNET[0]),
    preferences.getUInt("sn2", DEFAULT_SUBNET[1]),
    preferences.getUInt("sn3", DEFAULT_SUBNET[2]),
    preferences.getUInt("sn4", DEFAULT_SUBNET[3])
  );

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
  lcd.print("Connect AP:");
  lcd.setCursor(0, 1);
  lcd.print(CONFIG_AP_SSID);
  lcd.setCursor(0, 2);
  lcd.print("Pass: none");
  lcd.setCursor(0, 3);
  lcd.print("Timeout: 20s");

  // ---- Always start AP first ----
  if (!wm.startConfigPortal(CONFIG_AP_SSID)) {
    Serial.println("⏱️ No config in 20s → using saved/defaults...");

    if (netMode == 1) { // static
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
}*/

#define RESET_PASSWORD "blinkreset2025"
#define CONFIG_AP_SSID "ESP32_ConfigAP"
#define CONFIG_TIMEOUT 60

const char* customCSS = R"rawliteral(
<style>
    body { 
        font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
        background: linear-gradient(135deg, #2a0a0a 0%, #1a1a1a 100%);
        color: #ffffff; 
        margin: 0;
        padding: 20px;
        min-height: 100vh;
    }
    
    h2 { 
        color: #ff5c5c; 
        text-align: center; 
        font-size: 2rem;
        margin-bottom: 20px;
        text-shadow: 0 2px 10px rgba(255, 59, 48, 0.3);
    }
    
    .formfield {
        background: #2a2a2a;
        padding: 20px;
        border-radius: 12px;
        margin: 15px 0;
        border: 1px solid rgba(255, 59, 48, 0.2);
        box-shadow: 0 8px 16px rgba(255, 59, 48, 0.2);
    }
    
    label { 
        display: block; 
        margin-bottom: 8px; 
        font-weight: bold;
        color: #ff6961;
        font-size: 1.1rem;
    }
    
    input[type=text], input[type=password] {
        width: 95%; 
        padding: 12px 15px; 
        margin: 8px 0; 
        border-radius: 8px; 
        border: 2px solid rgba(255, 59, 48, 0.3);
        background: rgba(0, 0, 0, 0.3);
        color: #ffffff;
        font-size: 16px;
        transition: all 0.3s ease;
    }
    
    input[type=text]:focus, input[type=password]:focus {
        border-color: #ff3b30;
        box-shadow: 0 0 0 3px rgba(255, 59, 48, 0.2);
        outline: none;
    }
    
    .wm_button {
        background: linear-gradient(to right, #ff3b30, #d70015); 
        color: white; 
        padding: 14px 30px;
        border: none; 
        border-radius: 8px; 
        font-size: 1.1rem; 
        font-weight: bold;
        cursor: pointer; 
        transition: all 0.3s ease;
        display: block;
        width: 100%;
        margin: 20px 0;
        box-shadow: 0 4px 6px rgba(255, 59, 48, 0.3);
    }
    
    .wm_button:hover { 
        background: linear-gradient(to right, #d70015, #ff3b30);
        transform: translateY(-2px);
        box-shadow: 0 6px 12px rgba(255, 59, 48, 0.4);
    }
    
    p { 
        color: #cccccc; 
        text-align: center;
        margin: 10px 0;
    }
    
    .info {
        background: rgba(255, 59, 48, 0.1);
        border-left: 4px solid #ff3b30;
        padding: 15px;
        margin: 15px 0;
        border-radius: 0 8px 8px 0;
        font-size: 0.9rem;
    }
    
    .status {
        background: rgba(255, 59, 48, 0.2);
        padding: 10px;
        border-radius: 8px;
        text-align: center;
        margin: 15px 0;
        border: 1px solid rgba(255, 59, 48, 0.3);
    }
    
    @media (max-width: 600px) {
        body {
            padding: 10px;
        }
        
        input[type=text], input[type=password] {
            width: 92%;
        }
    }
</style>
)rawliteral";
void setupConfig() {
    preferences.begin("netcfg", false);

    // Load stored Wi-Fi credentials or defaults
    String ssid     = preferences.getString("ssid", DEFAULT_SSID);
    String password = preferences.getString("pass", DEFAULT_PASSWORD);
    int netMode     = preferences.getInt("netmode", 0); // 0=DHCP, 1=Static

    IPAddress local_IP(
        preferences.getUInt("ip1", DEFAULT_LOCAL_IP[0]),
        preferences.getUInt("ip2", DEFAULT_LOCAL_IP[1]),
        preferences.getUInt("ip3", DEFAULT_LOCAL_IP[2]),
        preferences.getUInt("ip4", DEFAULT_LOCAL_IP[3])
    );
    IPAddress gateway(
        preferences.getUInt("gw1", DEFAULT_GATEWAY[0]),
        preferences.getUInt("gw2", DEFAULT_GATEWAY[1]),
        preferences.getUInt("gw3", DEFAULT_GATEWAY[2]),
        preferences.getUInt("gw4", DEFAULT_GATEWAY[3])
    );
    IPAddress subnet(
        preferences.getUInt("sn1", DEFAULT_SUBNET[0]),
        preferences.getUInt("sn2", DEFAULT_SUBNET[1]),
        preferences.getUInt("sn3", DEFAULT_SUBNET[2]),
        preferences.getUInt("sn4", DEFAULT_SUBNET[3])
    );

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
    WiFiManagerParameter custom_reset("reset", "Reset password", "", 32);

    wm.addParameter(&custom_mode);
    wm.addParameter(&custom_ip);
    wm.addParameter(&custom_gw);
    wm.addParameter(&custom_sn);
    wm.addParameter(&custom_reset);

    // Inject the new red-themed CSS
    wm.setCustomHeadElement(customCSS);

    // LCD feedback (keep your existing LCD code)
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Connect AP:");
    lcd.setCursor(0,1); lcd.print(CONFIG_AP_SSID);
    lcd.setCursor(0,2); lcd.print("Pass: none");
    lcd.setCursor(0,3); lcd.print("Timeout: 20s");

    // Start portal
    if (!wm.startConfigPortal(CONFIG_AP_SSID)) {
        Serial.println("⏱️ Timeout, using saved/defaults...");
        if (netMode == 1) WiFi.config(local_IP, gateway, subnet);
        WiFi.begin(ssid.c_str(), password.c_str());
        preferences.end();
        return;
    }

    // Factory reset
    if (String(custom_reset.getValue()) == RESET_PASSWORD) {
        Serial.println("🛑 Factory reset requested");
        preferences.clear();
        preferences.end();
        delay(1000);
        ESP.restart();
    }

    // Save values (keep your existing save logic)
    preferences.putString("ssid", WiFi.SSID());
    preferences.putString("pass", WiFi.psk());
    preferences.putInt("netmode", atoi(custom_mode.getValue()));

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

    netMode = atoi(custom_mode.getValue());
    if (netMode == 1) WiFi.config(newIP, newGW, newSN);

    Serial.print("✅ WiFi Connected → IP: ");
    Serial.println(WiFi.localIP());
}

// Replace your current customCSS with this red-themed version:
