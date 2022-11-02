// NOTE: This project requires 2 dependencies that are not available in the Arduino package manager:
// https://github.com/me-no-dev/ESPAsyncWebServer
// https://github.com/me-no-dev/AsyncTCP





// Fast ESP32->ESP32 communication
#include <esp_now.h>

// LED
#include "lib\UMS3.h"

UMS3 ums3;

// General
#include <chrono>
#include <list>
#include <map>

using namespace std;


// WIFI AP, WEB SERVER and WIFI DIRECT
/*
  WiFiAccessPoint.ino creates a WiFi access point and provides a web server on it.

  Steps:
  1. Connect to the access point "yourAp"
  2. Point your web browser to http://192.168.4.1/H to turn the LED on or http://192.168.4.1/L to turn it off
     OR
     Run raw TCP "GET /H" and "GET /L" on PuTTY terminal with 192.168.4.1 as IP address and 80 as port

  Created for arduino-esp32 on 04 July, 2018
  by Elochukwu Ifediora (fedy0)
*/
// WIFI AP, WEB SERVER, WIFI DIRECT and ESP-NOW
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <esp_now.h>


// SCD-41
#include <SensirionI2CScd4x.h>
#include <Wire.h>
// PMSA003I
#include "Adafruit_PM25AQI.h"
// BME688
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
// LSM9DS1
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!


// --------------- START USER SETTINGS ---------------
bool enableLed = true;
int loopTimeLed = 5; // Default: 10
int brightnessLed = 255 / 3; // 0-255
// Flickers the LED, for loop performance testing
bool flickerLed = false;
int flickerTime = 250;

// WIFI Modes (only enable one of these)
bool enableWifiAp = false;
bool enableWifiDirectServer = false;
bool enableWifiDirectClient = false;
bool enableEspNowServer = true;
bool enableEspNowClient = false;

// WEB SERVER
bool enableWebServer = false;

// Sensors
bool enableScd41 = false;
bool enablePmsa003i = false;
bool enableBme688 = false;
bool enableLsm9ds1 = false;
bool enableGps = true;

int loopTimeGlobal = 0; // Default: 1000/0 for off

int loopTimeScd41 = 5000; // Default: 5000
int loopTimePmsa003i = 1000; // Default: 1000
int loopTimeBme688 = 2000; // Default: 2000, Tested_Low: 500
int loopTimeLsm9ds1 = 50; // Default: 200, Tested_Low: 20

// --------------- END USER SETTINGS ---------------


// WIFI AP and WIFI DIRECT
// Set these to your desired credentials.
// const char *ssid = "wat.jpg";
// const char *password = "delta1234";
// Setting network credentials
const char *ssid = "S-SA";
const char *password = "delta1234";


// WIFI Direct Server
#define SERVER_NUM_CLIENTS 10
String data;
String CLIENT;
String ACTION;

unsigned long previousMillis = 0;
int interval = 500;

String CLIENT_NAME = "SENSOR1";

String ServerMessage;

IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 9);
IPAddress subnet(255, 255, 255, 0);
WiFiServer wifi_server(100);
// This matrix is defined for further development, it has not been used in the code
WiFiClient *clients[SERVER_NUM_CLIENTS] = {NULL};


// WIFI Direct Client
WiFiClient client;

// IP Address of the server
IPAddress client_server(192, 168, 4, 1);



// WEB SERVER
// Creating a AsyncWebServer object
AsyncWebServer web_server(80);

const char index_html[] PROGMEM = R"rawliteral(
<a href=/info.json>Click here</a> to access the SensorArray's output.<br>
)rawliteral";



// SCD-41
SensirionI2CScd4x scd4x;

// PMSA003I
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

// BME688
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

// LSM9DS1
#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

// GPS
#define GPSSerial Serial0

#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#define PMTK_Q_RELEASE "$PMTK605*31"


// WIFI
// Create map to keep track of all of the settings, for providing to wifi clients
std::map<string, string> settings = {};
String settings_str = "";
String last_settings_str = "";

// ESP-NOW
// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress1[] = {0xF4, 0x12, 0xFA, 0x42, 0x0C, 0x88};
// uint8_t broadcastAddress2[] = {0xFF, , , , , };
// uint8_t broadcastAddress3[] = {0xFF, , , , , };

// typedef struct test_struct {
//   int x;
//   int y;
// } test_struct;

// test_struct test;

esp_now_peer_info_t peerInfo;

#define MSG_LEN 250
struct __attribute__((packed)) MSG {                                          
  char text[MSG_LEN];
} msg;
int msgSize = sizeof(msg);



void setup() {
  Serial.begin(115200);
  while (!Serial) {
      delay(100);
  }

  // --------------- START INIT PRO-S3/FEATHER-S3 ---------------
  // Initialize all board peripherals, call this first
  ums3.begin();

  if (enableLed == true) {
    // Brightness is 0-255. We set it to 1/3 brightness here
    ums3.setPixelBrightness(brightnessLed);
  }

  // --------------- END INIT PRO-S3/FEATHER-S3 ---------------
  if (enableWifiAp == true) {
    // --------------- START INIT WIFI AP ---------------
    // Connect to Wi-Fi
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("WIFI AP Server IP address: ");
    Serial.println(myIP);
    wifi_server.begin();

    Serial.println("WIFI AP Server started");
    // --------------- END INIT WIFI AP ---------------
  }

  if (enableWebServer == true) {
    // --------------- START INIT WIFI AP ---------------
    // Route for root / web page
    web_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/html", index_html);
    });

    // Send a GET request to <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
    web_server.on("/info.json", HTTP_GET, [] (AsyncWebServerRequest *request) {
      request->send(200, "application/json", settings_str);
    });

    // Start server
    web_server.begin();
    // --------------- END INIT WEB SERVER ---------------
  }

  if (enableWifiDirectServer == true) {
    // --------------- START WIFI DIRECT SERVER ---------------
    Serial.println();
    Serial.print("Setting soft AP (Access Point)…");
    // Remove the password parameter, if you want the AP (Access Point) to be open
    WiFi.mode(WIFI_AP);
    // WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    wifi_server.begin();
    // --------------- END WIFI DIRECT SERVER ---------------
  }

  if (enableWifiDirectClient == true) {
    // --------------- START WIFI DIRECT CLIENT ---------------
    WiFi.begin(ssid, password);
    Serial.println("Connecting");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(50);
      Serial.print(".");
      Serial.println("");
      Serial.print("Connected to WiFi network with IP Address: ");
      Serial.println(WiFi.localIP());
      break;
    }
    // --------------- END WIFI DIRECT CLIENT ---------------
  }

  if (enableEspNowServer == true) {
    // --------------- START ESP-NOW SERVER ---------------
    WiFi.mode(WIFI_STA);
  
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    
    esp_now_register_send_cb(OnDataSent);
    
    // register peer
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    // register first peer  
    memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    // // register second peer  
    // memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
    // if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //   Serial.println("Failed to add peer");
    //   return;
    // }
    // /// register third peer
    // memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
    // if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //   Serial.println("Failed to add peer");
    //   return;
    // }
    // --------------- END ESP-NOW SERVER ---------------
  }

  if (enableEspNowClient == true) {
    // --------------- START ESP-NOW CLIENT ---------------
    //Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    //Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    
    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(OnDataRecv);
    // --------------- END ESP-NOW CLIENT ---------------
  }
  

  if (enableScd41 == true) {
    // --------------- START SCD-41 CODE ---------------
    Wire.begin();

    uint16_t error;
    char errorMessage[256];

    scd4x.begin(Wire);

    // stop potentially previously started measurement
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    uint16_t serial0;
    uint16_t serial1;
    uint16_t serial2;
    error = scd4x.getSerialNumber(serial0, serial1, serial2);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        printSerialNumber(serial0, serial1, serial2);
    }

    // Start Measurement
    error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    Serial.println("Waiting for first measurement... (5 sec)");
    // --------------- END SCD-41 CODE ---------------
  }
  
  if (enablePmsa003i == true) {
    // --------------- START PMSA003I CODE ---------------
    // Wait for serial monitor to open

    Serial.println("Adafruit PMSA003I Air Quality Sensor");

    // Wait one second for sensor to boot up!
    delay(1000);

    // If using serial, initialize it and set baudrate before starting!
    // Uncomment one of the following
    //Serial1.begin(9600);
    //pmSerial.begin(9600);

    // There are 3 options for connectivity!
    if (! aqi.begin_I2C()) {      // connect to the sensor over I2C
    //if (! aqi.begin_UART(&Serial1)) { // connect to the sensor over hardware serial
    //if (! aqi.begin_UART(&pmSerial)) { // connect to the sensor over software serial 
      Serial.println("Could not find PM 2.5 sensor!");
      while (1) delay(10);
    }

    Serial.println("PM25 found!");
    // --------------- END PMSA003I CODE ---------------
  }

  if (enableBme688 == true) {
    // --------------- START BME688 CODE ---------------
    Serial.println(F("BME680 test"));

    if (!bme.begin()) {
      Serial.println("Could not find a valid BME680 sensor, check wiring!");
      while (1);
    }

    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
    // --------------- END BME688 CODE ---------------
  }

  if (enableLsm9ds1 == true) {
    Serial.println("LSM9DS1 data read demo");
    
    // Try to initialise and warn if we couldn't detect the chip
    if (!lsm.begin()) {
      Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
      while (1); }
    Serial.println("Found LSM9DS1 9DOF");

    // helper to just set the default scaling we want, see above!
    setupSensor();
  }

  if (enableGps == true) {
    GPSSerial.begin(9600);
    delay(2000);

    Serial.println("Software Serial GPS Test Echo Test");
    // you can send various commands to get it started
    //GPSSerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPSSerial.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);

    GPSSerial.println(PMTK_SET_NMEA_UPDATE_10HZ);

    Serial.println("Get version!");
    GPSSerial.println(PMTK_Q_RELEASE);
  }
}


// Keeps track of time since readng for each sensor type
auto timeGlobal = std::chrono::high_resolution_clock::now();

auto timeLed = std::chrono::high_resolution_clock::now();
auto timeFlickerLed = std::chrono::high_resolution_clock::now();

auto timeScd41 = std::chrono::high_resolution_clock::now();
auto timePmsa003i = std::chrono::high_resolution_clock::now();
auto timeBme688 = std::chrono::high_resolution_clock::now();
auto timeLsm9ds1 = std::chrono::high_resolution_clock::now();


// Keeps track of position for color rotation
int color = 0;
// Tracks if the light is on or off, for performance testing the loop 
bool flickerOn = true;

bool connected = false;

void loop() {

  if (client.connected() == false) {
    connected = false;
  }

  // WIFI Direct Server
  if (enableWifiDirectServer == true) {
    if (settings_str != last_settings_str) {
      clientCommand(settings_str);
      Serial.println("Settings Sent: " + settings_str);
      last_settings_str = settings_str;
    }
  }
  // WIFI Direct Client
  else if (enableWifiDirectClient == true) {
    unsigned long currentMillis = millis();
    // if (currentMillis - previousMillis >= interval) {
      // Serial.println("Attemting to connect");
      // Check WiFi connection status
      if (WiFi.status() == WL_CONNECTED) {
        // Serial.println("wifi status connected");
        if (connected == false) {
          client.connect(client_server, 100);
          connected = true;
        }
        while (client.connected()) {
          // Serial.println("client connected");
          String data = client.readStringUntil('\r');
          // Serial.println(data);
          if (data != "\0") {
            // Serial.print("Received data from Server:");
            Serial.println(data);
            continue;

            int Index = data.indexOf(':');
            String CLIENT = data.substring(0, Index);
            String ACTION = data.substring(Index + 1);
            if (CLIENT == CLIENT_NAME) {
              if (ACTION == "TEMPERATURE?") {
                client.println("ACK:10");
                Serial.println("This is Client: ACK10 was sent to server");
              }
            }
            else if (CLIENT == "OUTPUT1") {
              if (ACTION == "ON" || ACTION == "OFF") {
                Serial.println("This is Client: server is sending output1 On or Off command");
              }
            }

            client.stop();
            data = "\0";
          }
        }
      }
      previousMillis = millis();
    // }
  }
  // ESP-NOW SERVER
  if (enableEspNowServer == true) {
    // --------------- START ESP-NOW CODE ---------------
    if (settings_str != last_settings_str) {
      strcpy(msg.text, settings_str.c_str());
      unsigned char store[msgSize];
      memcpy(&store, &msg, msgSize);
      esp_err_t result = esp_now_send(NULL, store, msgSize);

      // esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
      
      if (result == ESP_OK) {
        Serial.println("Settings Sent: " + settings_str);
        last_settings_str = settings_str;
      }
      else {
        Serial.println("Error sending the data");
      }
    }
    // --------------- END ESP-NOW CODE ---------------
  }

  if (enableLed == true) {
    // Flicker LED, if enabled
    if (flickerLed == true) {
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - timeFlickerLed).count();
      if (duration > flickerTime) {
        timeFlickerLed = std::chrono::high_resolution_clock::now();
        if (flickerOn == true) {
          ums3.setPixelBrightness(0);
          flickerOn = false;
        } else {
          ums3.setPixelBrightness(brightnessLed);
          flickerOn = true;
        }
      }
    }

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - timeLed).count();
    if (duration > loopTimeLed) {
      timeLed = std::chrono::high_resolution_clock::now();
      // Rotate the LED's color
      ums3.setPixelColor(UMS3::colorWheel(color));
        color++;
    } else {

    }
  }

  // Skip the loop if the global == 0
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - timeGlobal).count();
  if (loopTimeGlobal == 0 || duration > loopTimeGlobal) {
    timeGlobal = std::chrono::high_resolution_clock::now();
  
    if (enableScd41 == true) {
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - timeScd41).count();
      if (duration > loopTimeScd41) {
        timeScd41 = std::chrono::high_resolution_clock::now();

        // --------------- START SCD-41 CODE ---------------
        uint16_t error;
        char errorMessage[256];

        // Read Measurement
        uint16_t co2;
        float temperature;
        float humidity;
        error = scd4x.readMeasurement(co2, temperature, humidity);
        if (error) {
            Serial.print("Error trying to execute readMeasurement(): ");
            errorToString(error, errorMessage, 256);
            Serial.println(errorMessage);
        } else if (co2 == 0) {
            Serial.println("Invalid sample detected, skipping.");
        } else {
            Serial.print("SCD-41 - ");
            Serial.print("Co2:");
            Serial.print(co2);
            Serial.print(", ");
            Serial.print("Temperature:");
            Serial.print(temperature);
            Serial.print(", ");
            Serial.print("Humidity:");
            Serial.print(humidity);

            Serial.println();
            Serial.println();

            // Save the info to the settings object
            settings["scd41_co2"] = to_string(co2);
            settings["scd41_temperature"] = to_string(temperature);
            settings["scd41_humidity"] = to_string(humidity);
            settings_str = String(map_to_json(settings).c_str());
        }
        // --------------- END SCD-41 CODE ---------------
      }
    }
    if (enablePmsa003i == true) {
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - timePmsa003i).count();
      if (duration > loopTimePmsa003i) {
        timePmsa003i = std::chrono::high_resolution_clock::now();

        // --------------- START PMSA003I CODE ---------------
        PM25_AQI_Data data;
          
        if (! aqi.read(&data)) {
          Serial.println("Could not read from AQI");
          return;
        }
        Serial.println("AQI reading success");

        Serial.println();
        Serial.println(F("---------------------------------------"));
        Serial.println(F("Concentration Units (standard)"));
        Serial.println(F("---------------------------------------"));
        Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_standard);
        Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_standard);
        Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_standard);
        Serial.println(F("Concentration Units (environmental)"));
        Serial.println(F("---------------------------------------"));
        Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_env);
        Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_env);
        Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_env);
        Serial.println(F("---------------------------------------"));
        Serial.print(F("Particles > 0.3um / 0.1L air:")); Serial.println(data.particles_03um);
        Serial.print(F("Particles > 0.5um / 0.1L air:")); Serial.println(data.particles_05um);
        Serial.print(F("Particles > 1.0um / 0.1L air:")); Serial.println(data.particles_10um);
        Serial.print(F("Particles > 2.5um / 0.1L air:")); Serial.println(data.particles_25um);
        Serial.print(F("Particles > 5.0um / 0.1L air:")); Serial.println(data.particles_50um);
        Serial.print(F("Particles > 10 um / 0.1L air:")); Serial.println(data.particles_100um);
        Serial.println(F("---------------------------------------"));
        
        // --------------- END PMSA003I CODE ---------------
      }
    }

    if (enableBme688 == true) {
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - timeBme688).count();
      if (duration > loopTimeBme688) {
        timeBme688 = std::chrono::high_resolution_clock::now();

        // --------------- START BME688 CODE ---------------
        bool failed = false;

        if (! bme.performReading()) {
          Serial.println("Failed to perform reading :(");
          failed = true;
        }

        if (failed == false) {
          Serial.print("BME688 - ");

          Serial.print("Temperature:");
          Serial.print(bme.temperature);
          // Serial.print(" *C");
          Serial.print(", ");

          Serial.print("Pressure: ");
          Serial.print(bme.pressure / 100.0);
          // Serial.print(" hPa");
          Serial.print(", ");

          Serial.print("Humidity: ");
          Serial.print(bme.humidity);
          // Serial.print(" %");
          Serial.print(", ");

          Serial.print("Gas: ");
          Serial.print(bme.gas_resistance / 1000.0);
          // Serial.print(" KOhms");
          Serial.print(", ");

          // Disabled, given that this function is slow and isn't needed currently
          // Serial.print("Approx. Altitude: ");
          // Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
          // // Serial.print(" m");

          Serial.println();
          Serial.println();
        }

        // --------------- END BME688 CODE ---------------
      }
    }

    if (enableLsm9ds1 == true) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - timeLsm9ds1).count();
        if (duration > loopTimeLsm9ds1) {
          // --------------- START LSM9DS1 CODE ---------------
          timeLsm9ds1 = std::chrono::high_resolution_clock::now();

          lsm.read();  /* ask it to read in the data */ 

          /* Get a new sensor event */ 
          sensors_event_t a, m, g, temp;

          lsm.getEvent(&a, &m, &g, &temp); 

          Serial.print("LSM9DS1 - ");
          Serial.print("Accel_X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2"); Serial.print(", ");
          Serial.print("Accel_Y: "); Serial.print(a.acceleration.y); Serial.print(" m/s^2"); Serial.print(", ");
          Serial.print("Accel_Z: "); Serial.print(a.acceleration.z); Serial.print(" m/s^2"); Serial.print(", ");

          Serial.print("Mag_X: "); Serial.print(m.magnetic.x); Serial.print(" uT"); Serial.print(", ");
          Serial.print("Mag_Y: "); Serial.print(m.magnetic.y); Serial.print(" uT"); Serial.print(", ");
          Serial.print("Mag_Z: "); Serial.print(m.magnetic.z); Serial.print(" uT"); Serial.print(", ");
  
          Serial.print("Gyro_X: "); Serial.print(g.gyro.x); Serial.print(" rad/s"); Serial.print(", ");
          Serial.print("Gyro_Y: "); Serial.print(g.gyro.y); Serial.print(" rad/s"); Serial.print(", ");
          Serial.print("Gyro_Z: "); Serial.print(g.gyro.z); Serial.println(" rad/s");

          Serial.println();
          // --------------- END LSM9DS1 CODE ---------------
        }
    }

    if (enableGps == true) {
      // --------------- START GPS CODE ---------------
      if (Serial.available()) {
        char c = ' ';
        while (c != '\n') {
          c = Serial.read();
          GPSSerial.write(c);
        }
      } 
      if (GPSSerial.available()) {
        char c = ' ';
        string gps_str = "";
        while (c != '\n') {
          c = GPSSerial.read();
          gps_str = gps_str + c;
          Serial.write(c);
        }
        // Save the info to the settings object
        settings["gps_data"] = gps_str;
        settings_str = String(map_to_json(settings).c_str());
      }
      // --------------- END GPS CODE ---------------
    }
  }
}


// Global functions
std::list<int> rgb_color_wheel(int wheel_pos) {
    // Color wheel to allow for cycling through the rainbow of RGB colors.
    wheel_pos = wheel_pos % 255;

    if (wheel_pos < 85) {
        return {255 - wheel_pos * 3, 0, wheel_pos * 3};
    } else if (wheel_pos < 170) {
        wheel_pos -= 85;
        return {0, wheel_pos * 3, 255 - wheel_pos * 3};
    } else {
        wheel_pos -= 170;
        return {wheel_pos * 3, 255 - wheel_pos * 3, 0};
    }
}

string map_to_json(std::map<string, string> mapToConvert) {

    if (mapToConvert.size() == 0) {
      return "{}";
    }

    string jsonString = "{";

    for (auto const& pair : mapToConvert) {
        string key = pair.first;
        string value = pair.second;

        jsonString.append("\"" + key + "\":\"" + value + "\",");
    }

    jsonString.pop_back();
    jsonString.append("}");

    return jsonString;
}

bool ends_with(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}



// SCD-41
void printUint16Hex(uint16_t value) {
    Serial.print(value < 4096 ? "0" : "");
    Serial.print(value < 256 ? "0" : "");
    Serial.print(value < 16 ? "0" : "");
    Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
    Serial.print("Serial: 0x");
    printUint16Hex(serial0);
    printUint16Hex(serial1);
    printUint16Hex(serial2);
    Serial.println();
}

// LSM9DS1
void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

// WIFI
void clientCommand(String input)
{
  for (int i = 0; i < SERVER_NUM_CLIENTS; i++)
  {
    WiFiClient client = wifi_server.available();
    if (client)
    {
      if (client.connected())
      {
        client.println(input);
        delay(10);
      }
    }
    // client.stop();
  }
}

String clientRequest(String input)
{
  String response = "\0";
  for (int i = 0; i < SERVER_NUM_CLIENTS; i++)
  {
    WiFiClient client = wifi_server.available();
    if (client)
    {
      client.setTimeout(50);
      if (client.connected())
      {
        client.println(input);

        data = client.readStringUntil('\r'); // received the server's answer
        if (data != "\0")
        {
          int Index = data.indexOf(':');

          CLIENT = data.substring(0, Index);
          ACTION = data.substring(Index + 1);

          if (CLIENT == "ACK")
          {
            response = ACTION;
          }
          data = "\0";
        }
      }
    }
  }
  return response;
}

// WIFI
// This void is for further developmets and has not ben used
void connect_wifi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    //delay(100);
  }
}

// ESP-NOW
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&msg, incomingData, sizeof(msg));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Settings Received: ");
  Serial.println(msg.text);
  Serial.println();
}

