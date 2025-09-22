#include <ModbusMaster.h>
#include <esp_task_wdt.h>
//#include <ESPSoftwareSerial.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <PubSubClient.h>
#include <RTClib.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include "driver/pcnt.h"
// #include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define TINY_GSM_MODEM_SIM800  // ✅ Define your modem model here
#include <TinyGsmClient.h>
#include <StreamDebugger.h>

#include <Preferences.h>
Preferences preferences;


//#define USE_GSM
#define USE_WIFI

//#define blinkBoard
#define powWaterBoard

/* GPS pins & power */
const gpio_num_t PIN_GPS_RX = GPIO_NUM_37;
const gpio_num_t PIN_GPS_TX = GPIO_NUM_39;
const gpio_num_t PIN_GPS_PWR = GPIO_NUM_4;

// Internal state variables

unsigned long lastSent = 0;
static bool modemInitialized = false;

// State tracking
unsigned long lastGpsSessionTime = 0;
unsigned long gpsStartTime = 0;
unsigned long lastGpsDebugTime = 0;
bool gpsFixLogged = false;

HardwareSerial gpsSerial(3);
TinyGPSPlus gps;
struct GpsFix {
  uint64_t timestamp;
  double lat;
  double lon;
};

const char MAIN_FILE[] = "/telemetry.csv";
const char TMP_FILE[] = "/telemetry.tmp";

/* Peripherals */
//LiquidCrystal_I2C lcd(0x27, 16, 2);



RTC_DS3231 rtc;

/* SD-card */
const uint8_t PIN_SD_CS = 5;

/* SIM800 pins */

const uint8_t PIN_SIM_PWR = 27;
const uint8_t PIN_555_ultrasonic = 33;

/* Level sensor/ADC */
const gpio_num_t PIN_FREQ = GPIO_NUM_14;
const pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;
const pcnt_channel_t PCNT_CH = PCNT_CHANNEL_0;
const uint32_t PCNT_GATE_MS = 100;
const uint8_t NUM_SAMPLES = 40;

// Calibration for fuel tank
const float FREQ_FULL_HZ = 154450.0;
const float FREQ_EMPTY_HZ = 212000.0;
//const float TUBE_HEIGHT_CM = 100.0;  // Tube height in cm
float freqHz;

// Tank dimensions (change as per your tank)
const float TANK_LENGTH_CM = 40.0;
const float TANK_WIDTH_CM = 50.0;

const uint8_t PIN_ADC_BAT = 35;

/* Timing */
// int SAMPLE_INT_MS = 30000;  // default value
const uint32_t GSM_INT_MS = 60000;
const uint32_t RETRY_DELAY_MS = 1000;


// #define ONE_WIRE_BUS 27  // 1 for blinkBoard
// OneWire oneWire(ONE_WIRE_BUS);
// DallasTemperature ds18b20(&oneWire);
// float DS_Temperature = 0;
TaskHandle_t Task1;

#define WDT_TIMEOUT 3600

/* Function prototypes */
bool parseLine(const String& ln, uint64_t& ts, float& level, float& vbat);


float dataArray[20];
unsigned long dataSensor;


String telemetryPayload = "{}";  // Default JSON string
String filteredPayload = "{}";


// --- WiFi Settings
const char* ssid = "Blink Electrics";
const char* password = "blink2023?";

// --- MQTT & Config Settings
WiFiClient espClient;
#ifdef USE_WIFI
PubSubClient mqttClient(espClient);
#endif
double height_mm;
//float volume_liters;
// --- Global Struct for Config
struct DeviceSettings {
  char TOKEN[64] = "nelsonYb5LaYMNChr6hf";//or 2nelsonYb5LaYMNChr6hf
  char SERVER[64] = "telemetry.blinkelectrics.co.ke";
  int port = 1883;
  int telemetryInterval = 60;
  int measurementInterval = 5000;

  char tankType[20]="rectangular";  // "cylindrical" or "rectangular"
  float tankLength;
  float tankWidth;
  float tankHeight;
  float tankRadius;
  char sensorType[20]="ultrasonic";
  int ultrasonicOffset =100;
};


const uint8_t MAX_RETRY = 3;
const size_t MAX_PAYLOAD = 512;

DeviceSettings deviceSettings;
bool attributesUpdated = false;
const char* config_filename = "/config.json";

// --- Forward Declarations
bool saveConfig(const char* filename);
bool readConfig(const char* filename);
String readFile(const char* filename);
void writeFile(const char* filename, String data);
bool connectWiFi();
bool mqttConnect();
void requestSharedAttributes();
void callback(char* topic, byte* payload, unsigned int length);

// --- Global attribute keys ---
String mqttAttributes = "accessToken,server,port,telemetryInterval,measurementInterval";
String tankAttributes = "radius,height,length,width,tankType";
String sensorAttributes = "emptyHz,emptyDistance,midHz,midDistance,fullHz,fullDistance,sensorDisconnectedHz,sensorType,ultrasonicOffset";


#define MQTT_MAX_PACKET_SIZE 512  // or 256


#define MODEM_RX 16
#define MODEM_TX 17

#define MODEM_RST 13

#define MODEM_BAUD 115200

#define GSM_AUTOBAUD_MIN 4800
#define GSM_AUTOBAUD_MAX 115200

#define SerialMon Serial  // For debug output to PC
#define SerialAT Serial2  // Hardware serial for SIM800

//#define USE_DEBUGGER

#ifdef USE_DEBUGGER
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);  // 👈 Use debugger
#else
TinyGsm modem(SerialAT);  // 👈 No debugger
#endif

TinyGsmClient gsmClient(modem);

#ifdef USE_GSM
PubSubClient mqttClient(gsmClient);
#endif

TaskHandle_t telemetryHandle = nullptr;

static bool gsmTelemetrySent = false;

const char apn[] = "iot.safaricom.com";  // Safaricom IoT APN
const char gprsUser[] = "";
const char gprsPass[] = "";


void initSettings() {
  if (!LittleFS.begin(true)) {
    Serial.println("❌ LittleFS mount failed");
  } else {
    Serial.println("✅ LittleFS mounted");
    if (!readConfig(config_filename)) {
      Serial.println("⚠️ No config found, saving defaults");
      saveConfig(config_filename);
    } else readConfig(config_filename);  // 🔁 Reload from file into deviceSettings
  }
}


// #include <LCD_I2C.h>

// LCD_I2C lcd(0x27, 16, 2);  // Default address of most PCF8574 modules, change according

// Global Flags and Data
bool networkConnected = false;
bool mqttConnected = false;

TaskHandle_t DisplayTaskHandle;


// Declare these globally or as static inside a function/task
static unsigned long lastBlinkTime = 0;
static bool backlightState = true;

// SD Logging additions
extern bool flushing;  // Used to indicate if SD write is in progress
bool sdReady = false;
// Add GPS file path
const char GPS_FILE[] = "/gps.csv";

// Logging function prototypes
uint64_t localMsToUtcMs(uint64_t ms);
void logToSD(float volume_liters, float battery_voltage);
void logGpsToSD(double lat, double lon);



// Calibration functions and variables prototypes

struct Point {
  double x;  // Frequency (Hz)
  double y;  // Height (mm)
};

// Polynomial coefficients
double a, b, c;

// === Function Prototypes ===
void calibrateFromThreePoints(Point p1, Point p2, Point p3, double& a, double& b, double& c);
float calculateDieselHeight(unsigned long freq);
void gaussJordan(double m[3][4]);

struct CalibrationData {
  long emptyHz = 99978;
  long emptyDistance = 0;
  long midHz = 87366;
  long midDistance = 1000;
  long fullHz = 76521;
  long fullDistance = 1910;
  long sensorDisconnectedHz = 30000;
  double a = 0, b = 0, c = 0;
};

CalibrationData calibrationData;
const char* calibration_filename = "/calibration.json";


enum FuelSensorStatus {
  SENSOR_OK,
  SENSOR_DISCONNECTED,
  SENSOR_OUT_OF_RANGE
};



float Height_ultrsnc;
int ultraSonicSensorOk = 0;
float readUltrasonicCM();
int x;
int ultrasonicSensorStatus = 0;



#define NUM_SAMPLES 10
const float VREF = 3.3;
const float ADC_MAX = 4095.0;
const float VOLTAGE_MULTIPLIER = 7.0;
int samples[NUM_SAMPLES];
int sampleIndex = 0;
int Ratio = 1.034;
float batteryVoltage;
const int numPoints = 20;

float lipoBatteryVoltage;
int adcTable[numPoints] = {
  2030, 2000, 1967, 1931, 1898, 1846, 1806, 1745, 1682, 1601,
  1530, 1414, 1292, 1164, 990, 842, 667, 493, 327, 173
};

float voltageTable[numPoints] = {
  21.2, 20.2, 19.2, 18.2, 17.2, 16.2, 15.2, 14.2, 13.2, 12.2,
  11.2, 10.2, 9.2, 8.2, 7.2, 6.2, 5.2, 4.2, 3.2, 2.2
};

bool rtcTimeOk = false;
bool sdCardStatus = false;

const int valvePin = 26;
#define MAX_BUFFER_SIZE 64
#define MODBUSIP_DEBUG
#include <ModbusIP_ESP8266.h>
// 📌 Modbus register base address
const int REG_BASE = 0;  // Holding registers from address 0 to 9

// 📌 Number of holding registers
const int REG_COUNT = 10;

// 📌 Create ModbusIP server instance
ModbusIP mb;
//IPAddress local_IP(192, 168, 1, 68);
IPAddress local_IP(192, 168, 1, 69);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255, 255, 255, 0);

int lcdColumns = 20;
int lcdRows = 4;

LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  //use for the first two lcd displays
//LiquidCrystal_I2C lcd(0x26, lcdColumns, lcdRows); //use for the third lcd which is next to the wall

struct MsgStructure {
  String str1;
  String str2;
};

void Display(String Message1, String Message2);
MsgStructure Conv(String Message);
bool ClientStatus = false;
bool clientConnected = false;

float DisplayDistance = 0.000F;
void clearLCDLine(int line, int startCell);
void clearLCDLine(int line);

/*=======================================================================*/
//Controls
bool valveOpen = false;
bool previousstateOfValve = false;
long levelStability;
long levelStabilityTime = 5000;
unsigned long int lastDisplay = 0;

unsigned long lastLCDUpdate = 0;
const unsigned long lcdUpdateInterval = 5000;  // 5 seconds


float getAvgFrequency_EMA(uint8_t samples, float alpha);
