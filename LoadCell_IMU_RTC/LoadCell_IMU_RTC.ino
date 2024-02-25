#include <Wire.h>
#include "RTClib.h"
#include "HX711.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#define USE_SPI       // Uncomment this to use SPI
#define SERIAL_PORT Serial
#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 27     // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

// RTC and HX711 setup
RTC_DS3231 rtc;
const int LOADCELL_DOUT_PIN = 15;
const int LOADCELL_SCK_PIN = 32;
HX711 scale;

// BLE setup
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define WEIGHT_DATA_CHAR_UUID "beefcafe-36e1-4688-b7f5-eeddccaa00bb"
BLECharacteristic *pWeightDataCharacteristic;


// Structure to hold weight and date-time information
struct WeightReading {
  float weight;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

WeightReading reading;

// EEPROM setup
const int EEPROM_SIZE = EEPROM.length(); // Use the EEPROM length function for size
const int EEPROM_READING_START = sizeof(int); // Start after the stored index
const int MAX_READINGS = (EEPROM_SIZE - EEPROM_READING_START) / sizeof(WeightReading); // Calculate the max number of readings

// Current index for EEPROM storage
int currentReadingIndex = 0;

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

void setup() {
  Serial.begin(38400);
  Wire.begin();

#ifndef QUAT_ANIMATION
  SERIAL_PORT.println(F("ICM-20948 Example"));
#endif

  delay(100);

  #ifndef QUAT_ANIMATION
    while (SERIAL_PORT.available()) // Make sure the serial RX buffer is empty
      SERIAL_PORT.read();

  SERIAL_PORT.println(F("Press any key to continue..."));

  while (!SERIAL_PORT.available()) // Wait for the user to press a key (send any serial character)
    ;
#endif

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

#ifndef QUAT_ANIMATION
  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
#endif

  bool initialized = false;
  while (!initialized)
  {

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

#ifndef QUAT_ANIMATION
    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
#endif
    if (myICM.status != ICM_20948_Stat_Ok)
    {
#ifndef QUAT_ANIMATION
      SERIAL_PORT.println(F("Trying again..."));
#endif
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

#ifndef QUAT_ANIMATION
  SERIAL_PORT.println(F("Device connected!"));
#endif

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
  // Check success
  if (success)
  {
#ifndef QUAT_ANIMATION
    SERIAL_PORT.println(F("DMP enabled!"));
#endif
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting the time to the current compilation time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize scale
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(12898.815483871); // Adjust to your scale
  scale.tare();

  // Initialize BLE
  BLEDevice::init("Smart-Scale");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pWeightDataCharacteristic = pService->createCharacteristic(
    WEIGHT_DATA_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pWeightDataCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("BLE device is ready to be connected");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, currentReadingIndex); // Read the current index from EEPROM
  if (currentReadingIndex < 0 || currentReadingIndex >= MAX_READINGS) {
    currentReadingIndex = 0; // If the index is invalid, reset it
  }
}

void readLoadCellData() {
  DateTime now = rtc.now(); // Get the current date and time
  reading.weight = scale.get_units(10); // Get the current weight
  
  // Store the current date and time in the reading
  reading.year = now.year();
  reading.month = now.month();
  reading.day = now.day();
  reading.hour = now.hour();
  reading.minute = now.minute();
  reading.second = now.second();

  // Convert WeightReading struct to bytes
  byte readingData[sizeof(WeightReading)];
  memcpy(readingData, &reading, sizeof(WeightReading));

  // Update BLE characteristic with the byte array
  pWeightDataCharacteristic->setValue(readingData, sizeof(WeightReading));
  pWeightDataCharacteristic->notify();

  Serial.println("Weight and date-time data sent over BLE as raw bytes");
}


void storeReadingToEEPROM() {
  // Calculate the address for the new reading based on the current index
  int addr = EEPROM_READING_START + (currentReadingIndex * sizeof(WeightReading));
  
  // Store the new reading at the calculated address
  EEPROM.put(addr, reading);
  
  // Increment the index for the next reading
  currentReadingIndex++;
  
  // Wrap the index if it exceeds the maximum number of readings
  if (currentReadingIndex >= MAX_READINGS) {
    currentReadingIndex = 0;
  }
  
  // Store the updated index back at the beginning of EEPROM
  EEPROM.put(0, currentReadingIndex);
  
  // Commit the changes to EEPROM memory
  EEPROM.commit();
}


void calibrate()
{

#ifndef QUAT_ANIMATION
  while (SERIAL_PORT.available()) // Make sure the serial RX buffer is empty
    SERIAL_PORT.read();
#endif

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

#ifndef QUAT_ANIMATION
  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
#endif

  bool initialized = false;
  while (!initialized)
  {

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    if (myICM.status != ICM_20948_Stat_Ok)
    {
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
  // Check success
  if (success)
  {
#ifndef QUAT_ANIMATION
    SERIAL_PORT.println(F("DMP enabled!"));
#endif
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }
}

void IMU_FUNCTIONALITY() {
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30


      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      yaw = atan2(t3, t4) * 180.0 / PI;

#ifndef QUAT_ANIMATION
      SERIAL_PORT.print(F("Roll:"));
      SERIAL_PORT.print(roll, 1);
      SERIAL_PORT.print(F(" Pitch:"));
      SERIAL_PORT.print(pitch, 1);
      SERIAL_PORT.print(F(" Yaw:"));
      SERIAL_PORT.println(yaw, 1);
#else
      // Output the Quaternion data in the format expected by ZaneL's Node.js Quaternion animation tool
      SERIAL_PORT.print(F("{\"quat_w\":"));
      SERIAL_PORT.print(q0, 3);
      SERIAL_PORT.print(F(", \"quat_x\":"));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(", \"quat_y\":"));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(", \"quat_z\":"));
      SERIAL_PORT.print(q3, 3);
      SERIAL_PORT.println(F("}"));
#endif
      // Condition to trigger the reset
      /*if (abs(yaw) > 50) {
        myICM.swReset(); // Reset the sensor
        delay(100); // Wait a bit for the sensor to reset

        // Re-initialize the sensor
        if (myICM.status == ICM_20948_Stat_Ok) {
          Serial.println("Sensor re-initialized successfully.");
          calibrate();
          delay(1000);
        } else {
          Serial.println("Sensor re-initialization failed.");
        }
      }*/

    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
}

void loop() {
  IMU_FUNCTIONALITY(); // This function updates the global roll, pitch, and yaw values

  // Check if roll and pitch are within +/- 5 degrees
  if (abs(roll) <= 5.0 && abs(pitch) <= 5.0) {
    readLoadCellData(); // Read weight data if the condition is met
    storeReadingToEEPROM(); // Store the reading to EEPROM
  } else {
    Serial.println("Roll or pitch out of range. Skipping weight reading.");
  }

  delay(10000); // Adjust based on how frequently you want to read and store data
}



