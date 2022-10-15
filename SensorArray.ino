// LED
#include "lib\UMS3.h"

UMS3 ums3;

// General
#include <chrono>
#include <list>

using namespace std::chrono;


// SCD-41
#include <Arduino.h>
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
int loopTimeLed = 20; // Default: 10
int brightnessLed = 255 / 3; // 0-255
// Flickers the LED, for loop performance testing
bool flickerLed = false;
int flickerTime = 250;

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

void loop() {
  if (enableLed == true) {
    // Flicker LED, if enabled
    if (flickerLed == true) {
      auto duration = duration_cast<milliseconds>(std::chrono::high_resolution_clock::now() - timeFlickerLed).count();
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

    auto duration = duration_cast<milliseconds>(std::chrono::high_resolution_clock::now() - timeLed).count();
    if (duration > loopTimeLed) {
      timeLed = std::chrono::high_resolution_clock::now();
      // Rotate the LED's color
      ums3.setPixelColor(UMS3::colorWheel(color));
        color++;
    } else {

    }
  }

  // Skip the loop if the global is not NULL
  auto duration = duration_cast<milliseconds>(std::chrono::high_resolution_clock::now() - timeGlobal).count();
  if (duration > loopTimeGlobal) {
    timeGlobal = std::chrono::high_resolution_clock::now();
  
    if (enableScd41 == true) {
      auto duration = duration_cast<milliseconds>(std::chrono::high_resolution_clock::now() - timeScd41).count();
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
        }
        // --------------- END SCD-41 CODE ---------------
      }
    }
    if (enablePmsa003i == true) {
      auto duration = duration_cast<milliseconds>(std::chrono::high_resolution_clock::now() - timePmsa003i).count();
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
      auto duration = duration_cast<milliseconds>(std::chrono::high_resolution_clock::now() - timeBme688).count();
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
        auto duration = duration_cast<milliseconds>(std::chrono::high_resolution_clock::now() - timeLsm9ds1).count();
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
        while (c != '\n') {
          c = GPSSerial.read();
          Serial.write(c);
        }
      }
      // --------------- END LSM9DS1 CODE ---------------
    }
  }
}
