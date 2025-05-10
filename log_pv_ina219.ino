#include <Wire.h>
// #include <Adafruit_INA219.h> // Commented out INA219 library
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>

// Define pins for ESP32
#define SDA_PIN 16
#define SCL_PIN 17

// Define temperature sensor pins
#define SENSOR_PIN_1 27
#define SENSOR_PIN_2 26
#define SENSOR_PIN_3 25
#define SENSOR_PIN_4 33
#define SENSOR_PIN_5 32
#define SENSOR_PIN_6 4

// Define RTC pins
#define RTC_SDA_PIN 21 // I2C SDA untuk RTC
#define RTC_SCL_PIN 22 // I2C SCL untuk RTC

// Define SD card pins
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 15
#define SD_CS_PIN 5 // Chip select microSD (HW-125)

// Setup DS18B20 sensors
OneWire oneWire1(SENSOR_PIN_1);
OneWire oneWire2(SENSOR_PIN_2);
OneWire oneWire3(SENSOR_PIN_3);
OneWire oneWire4(SENSOR_PIN_4);
OneWire oneWire5(SENSOR_PIN_5);
OneWire oneWire6(SENSOR_PIN_6);

DallasTemperature tempSensor1(&oneWire1);
DallasTemperature tempSensor2(&oneWire2);
DallasTemperature tempSensor3(&oneWire3);
DallasTemperature tempSensor4(&oneWire4);
DallasTemperature tempSensor5(&oneWire5);
DallasTemperature tempSensor6(&oneWire6);

// Array to store temperature readings
float temperatures[6];

// Create I2C instances for ESP32
TwoWire I2Cone = TwoWire(0); // For future INA219 use
TwoWire I2Ctwo = TwoWire(1); // For RTC

// Create INA219 with default address - don't pass Wire in constructor
// Adafruit_INA219 ina219; // Commented out INA219 object
RTC_DS3231 rtc;

// Variables for SD card logging
File logFile;
String logFileName;
unsigned long lastLogTime = 0;
const unsigned long LOG_INTERVAL = 60000; // Log every 1 minutes

// Function to generate a timestamp-formatted filename
String getLogFileName()
{
  DateTime now = rtc.now();
  char filename[32];
  sprintf(filename, "/LOG_%04d%02d%02d.CSV",
          now.year(), now.month(), now.day());
  return String(filename);
}

// Function to format date and time for log entries
String getTimestamp()
{
  DateTime now = rtc.now();
  char timestamp[20];
  sprintf(timestamp, "%04d/%02d/%02d %02d:%02d:%02d",
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
  return String(timestamp);
}

void setup(void)
{
  Serial.begin(115200);
  delay(1000); // Allow serial to initialize

  Serial.println("Mengukur dan logging suhu dengan DS18B20 dan RTC pada ESP32...");

  // Initialize I2C buses with specified pins for ESP32
  I2Cone.begin(SDA_PIN, SCL_PIN);         // Start I2C bus for future use
  I2Ctwo.begin(RTC_SDA_PIN, RTC_SCL_PIN); // Start I2C bus for RTC

  // Initialize temperature sensors
  tempSensor1.begin();
  tempSensor2.begin();
  tempSensor3.begin();
  tempSensor4.begin();
  tempSensor5.begin();
  tempSensor6.begin();
  Serial.println("All DS18B20 temperature sensors initialized");

  /* Commented out INA219 initialization
  // Initialize INA219 with specific I2C bus
  if (!ina219.begin(&I2Cone))
  {
    Serial.println("Gagal menemukan INA219. Periksa koneksi!");
    while (1)
      ;
  }
  Serial.println("INA219 initialized");
  */

  // Initialize RTC with specific I2C bus
  if (!rtc.begin(&I2Ctwo))
  {
    Serial.println("Gagal menemukan RTC. Periksa koneksi!");
    while (1)
      ;
  }

  // Set RTC time to compilation time if needed or lost power
  if (rtc.lostPower())
  {
    Serial.println("RTC lost power, setting time to compilation time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.println("RTC initialized and time set");

  /* Commented out INA219 calibration
  // Kalibrasi INA219
  ina219.setCalibration_32V_2A(); // Untuk rentang 32V, 2A (default)
  Serial.println("INA219 calibrated for 32V, 2A range");
  */

  // Initialize SD card
  Serial.println("Initializing SD card...");
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (!SD.begin(SD_CS_PIN))
  {
    Serial.println("SD card initialization failed!");
    while (1)
      ;
  }
  Serial.println("SD card initialized successfully.");

  // Create and open log file
  logFileName = getLogFileName();
  Serial.print("Log file: ");
  Serial.println(logFileName);

  // Check if file exists, if not create with header
  if (!SD.exists(logFileName))
  {
    logFile = SD.open(logFileName, FILE_WRITE);
    if (logFile)
    {
      // Updated header to include only temperature readings
      logFile.println("Timestamp,Temp1(C),Temp2(C),Temp3(C),Temp4(C),Temp5(C),Temp6(C)");
      logFile.close();
      Serial.println("Created new log file with headers");
    }
    else
    {
      Serial.println("Error creating log file!");
      while (1)
        ;
    }
  }

  Serial.println("All sensors and logging ready!");
  Serial.println("");
}

void loop(void)
{
  unsigned long currentMillis = millis();

  /* Commented out INA219 reading code
  // Read sensor values
  // Read INA219 sensor values
  float shuntVoltage_mV = abs(ina219.getShuntVoltage_mV());
  float busVoltage_V = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  float loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000);

  // Konversi dari mA ke A dan pastikan nilai positif
  float current_A = abs(current_mA) / 1000.0;

  // Konversi dari mW ke W dengan mengikuti arah arus
  float power_W = abs(power_mW) / 1000.0;
  */

  // Read temperature from all DS18B20 sensors
  tempSensor1.requestTemperatures();
  tempSensor2.requestTemperatures();
  tempSensor3.requestTemperatures();
  tempSensor4.requestTemperatures();
  tempSensor5.requestTemperatures();
  tempSensor6.requestTemperatures();

  temperatures[0] = tempSensor1.getTempCByIndex(0);
  temperatures[1] = tempSensor2.getTempCByIndex(0);
  temperatures[2] = tempSensor3.getTempCByIndex(0);
  temperatures[3] = tempSensor4.getTempCByIndex(0);
  temperatures[4] = tempSensor5.getTempCByIndex(0);
  temperatures[5] = tempSensor6.getTempCByIndex(0);

  // Check if it's time to log data
  if (currentMillis - lastLogTime >= LOG_INTERVAL)
  {
    lastLogTime = currentMillis;

    // Create log entry
    String timestamp = getTimestamp();

    // Start the data string with timestamp
    String dataString = timestamp;

    // Add all temperature readings, even if some are invalid
    for (int i = 0; i < 6; i++)
    {
      dataString += ",";
      if (temperatures[i] != DEVICE_DISCONNECTED_C && temperatures[i] != -127.00)
      {
        dataString += String(temperatures[i], 2);
      }
      else
      {
        dataString += "ERROR";
      }
    }

    // Removed INA219 readings from logging data

    // Write to SD card
    logFile = SD.open(logFileName, FILE_APPEND);
    if (logFile)
    {
      logFile.println(dataString);
      logFile.close();
      Serial.println("Data logged: " + dataString);
    }
    else
    {
      Serial.println("Error opening log file!");
    }
  }

  // Display sensor readings (non-blocking)
  // Print all sensor readings
  Serial.println("---------------- SENSOR READINGS ----------------");

  // Print RTC time
  DateTime now = rtc.now();
  Serial.print("Tanggal & Waktu: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);

  // Print all temperature readings
  for (int i = 0; i < 6; i++)
  {
    Serial.print("Suhu Sensor ");
    Serial.print(i + 1);
    Serial.print(":      ");
    if (temperatures[i] != DEVICE_DISCONNECTED_C && temperatures[i] != -127.00)
    {
      Serial.print(temperatures[i]);
      Serial.println(" °C");
    }
    else
    {
      Serial.println("ERROR");
    }
  }

  /* Commented out INA219 display code
  // Print INA219 readings
  Serial.print("Tegangan Bus:   ");
  Serial.print(busVoltage_V, 3);
  Serial.println(" V");
  Serial.print("Tegangan Beban: ");
  Serial.print(loadVoltage_V, 3);
  Serial.println(" V");
  Serial.print("Tegangan Shunt: ");
  Serial.print(shuntVoltage_mV, 3);
  Serial.println(" mV");
  Serial.print("Arus:           ");
  Serial.print(current_A, 3);
  Serial.println(" A");
  Serial.print("Daya:           ");
  Serial.print(power_W, 3);
  Serial.println(" W");
  */

  Serial.println("--------------------------------------------------");

  delay(1000); // Display readings every second, but log every minute
}