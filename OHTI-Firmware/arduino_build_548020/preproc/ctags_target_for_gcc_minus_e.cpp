# 1 "C:\\Users\\Bosse\\Documents\\Arduino\\OHTI-Release-002a\\OHTI-Release-002a.ino"
# 1 "C:\\Users\\Bosse\\Documents\\Arduino\\OHTI-Release-002a\\OHTI-Release-002a.ino"
// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// This works Print on  BLE only
// to be modified to support calibration if booted with a pin high.
// Also save the calibration values to program memory and load the old calibration values if pin is low.

/*
Serial Port over BLE
Create UART service compatible with Nordic's *nRF Toolbox* and Adafruit's *Bluefruit LE* iOS/Android apps.

Serial class implements same protocols as Arduino's built-in Serial class and can be used as it's wireless
replacement. Data transfers are routed through a BLE service with TX and RX characteristics. To make ble
service discoverable all UUIDs are NUS (Nordic UART Service) compatible.

Please note that TX and RX characteristics use Notify and WriteWithoutResponse, so there's no guarantee
that the data will make it to the other end. However, under normal circumstances and reasonable signal
strengths everything works well.

03R  use quatqount to put in to doubble DD
*/

// Import libraries (BLEPeripheral depends on SPI)
# 25 "C:\\Users\\Bosse\\Documents\\Arduino\\OHTI-Release-002a\\OHTI-Release-002a.ino" 2
# 26 "C:\\Users\\Bosse\\Documents\\Arduino\\OHTI-Release-002a\\OHTI-Release-002a.ino" 2
# 27 "C:\\Users\\Bosse\\Documents\\Arduino\\OHTI-Release-002a\\OHTI-Release-002a.ino" 2
// #include "string.h"
# 29 "C:\\Users\\Bosse\\Documents\\Arduino\\OHTI-Release-002a\\OHTI-Release-002a.ino" 2
# 30 "C:\\Users\\Bosse\\Documents\\Arduino\\OHTI-Release-002a\\OHTI-Release-002a.ino" 2
// #include "base64.hpp"
//#include <CmdMessenger.h>  // CmdMessenger

using namespace std;

// BLE Service
BLEDis bledis;
BLEUart bleuart;
// BLEBas  blebas; // removed battery service



BNO055 mySensor(0X28 /* I2C address selection pin LOW*/);



uint32_t blinkyms;

// -------------------------
// BNO055 Register Map
// http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_10_Release.pdf
//
// BNO055 Page 0
# 141 "C:\\Users\\Bosse\\Documents\\Arduino\\OHTI-Release-002a\\OHTI-Release-002a.ino"
//
// BNO055 Page 1
# 173 "C:\\Users\\Bosse\\Documents\\Arduino\\OHTI-Release-002a\\OHTI-Release-002a.ino"
// Set initial input parameters
enum Ascale { // ACC Full Scale
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_18G
};

enum Abw { // ACC Bandwidth
  ABW_7_81Hz = 0,
  ABW_15_63Hz,
  ABW_31_25Hz,
  ABW_62_5Hz,
  ABW_125Hz,
  ABW_250Hz,
  ABW_500Hz,
  ABW_1000Hz, //0x07
};

enum APwrMode { // ACC Pwr Mode
  NormalA = 0,
  SuspendA,
  LowPower1A,
  StandbyA,
  LowPower2A,
  DeepSuspendA
};

enum Gscale { // gyro full scale
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS,
  GFS_125DPS // 0x04
};

enum GPwrMode { // GYR Pwr Mode
  NormalG = 0,
  FastPowerUpG,
  DeepSuspendedG,
  SuspendG,
  AdvancedPowerSaveG
};

enum Gbw { // gyro bandwidth
  GBW_523Hz = 0,
  GBW_230Hz,
  GBW_116Hz,
  GBW_47Hz,
  GBW_23Hz,
  GBW_12Hz,
  GBW_64Hz,
  GBW_32Hz
};

enum OPRMode { // BNO-55 operation modes
  CONFIGMODE = 0x00,
// Sensor Mode
  ACCONLY,
  MAGONLY,
  GYROONLY,
  ACCMAG,
  ACCGYRO,
  MAGGYRO,
  AMG, // 0x07
// Fusion Mode
  IMU,
  COMPASS,
  M4G,
  NDOF_FMC_OFF,
  NDOF // 0x0C
};

enum PWRMode {
  Normalpwr = 0,
  Lowpower,
  Suspendpwr
};

enum Modr { // magnetometer output data rate  
  MODR_2Hz = 0,
  MODR_6Hz,
  MODR_8Hz,
  MODR_10Hz,
  MODR_15Hz,
  MODR_20Hz,
  MODR_25Hz,
  MODR_30Hz
};

enum MOpMode { // MAG Op Mode
  LowPower = 0,
  Regular,
  EnhancedRegular,
  HighAccuracy
};

enum MPwrMode { // MAG power mode
  Normal = 0,
  Sleep,
  Suspend,
  ForceMode
};
# 286 "C:\\Users\\Bosse\\Documents\\Arduino\\OHTI-Release-002a\\OHTI-Release-002a.ino"
// Specify sensor configuration
uint8_t OSR = 0x0A; // set pressure amd temperature oversample rate

uint8_t GPwrMode = Normal; // Gyro power mode
uint8_t Gscale = GFS_250DPS; // Gyro full scale
//uint8_t Godr = GODR_250Hz;    // Gyro sample rate
uint8_t Gbw = GBW_23Hz; // Gyro bandwidth

uint8_t Ascale = AFS_2G; // Accel full scale
//uint8_t Aodr = AODR_250Hz;    // Accel sample rate
uint8_t APwrMode = Normal; // Accel power mode
uint8_t Abw = ABW_31_25Hz; // Accel bandwidth, accel sample rate divided by ABW_divx

//uint8_t Mscale = MFS_4Gauss;  // Select magnetometer full-scale resolution
uint8_t MOpMode = HighAccuracy; // Select magnetometer perfomance mode
uint8_t MPwrMode = Normal; // Select magnetometer power mode
uint8_t Modr = MODR_10Hz; // Select magnetometer ODR when in BNO055 bypass mode

uint8_t PWRMode = Normal ; // Select BNO055 power mode
uint8_t OPRMode = NDOF; // specify operation mode for sensors
uint8_t status; // BNO055 data status register
float aRes, gRes, mRes; // scale resolutions per LSB for the sensors

// Pin definitions
int intPin = 8; // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13;

uint16_t Pcal[8]; // calibration constants from MS5637 PROM registers
unsigned char nCRC; // calculated check sum to ensure PROM integrity
uint32_t D1 = 0, D2 = 0; // raw MS5637 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2; // First order and second order corrections for raw S5637 temperature and pressure data
int16_t accelCount[3]; // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3]; // Stores the 16-bit signed gyro sensor output
int16_t magCount[3]; // Stores the 16-bit signed magnetometer sensor output
int16_t quatCount[4]; // Stores the 16-bit signed quaternion output
int16_t EulCount[3]; // Stores the 16-bit signed Euler angle output
int16_t LIACount[3]; // Stores the 16-bit signed linear acceleration output
int16_t GRVCount[3]; // Stores the 16-bit signed gravity vector output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}; // Bias corrections for gyro, accelerometer, and magnetometer
int16_t tempGCount, tempMCount; // temperature raw count output of mag and gyro
float Gtemperature, Mtemperature; // Stores the BNO055 gyro and LIS3MDL mag internal chip temperatures in degrees Celsius
double Temperature, Pressure; // stores MS5637 pressures sensor pressure and temperature

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = 3.1415926535897932384626433832795 * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = 3.1415926535897932384626433832795 * (0.0f / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value



uint32_t delt_t = 0, count = 0, sumCount = 0; // used to control display output rate
float pitch, yaw, roll;
float Pitch, Yaw, Roll;
float LIAx, LIAy, LIAz, GRVx, GRVy, GRVz;
float deltat = 0.0f, sum = 0.0f; // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0; // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
//float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f}; // vector to hold integral error for Mahony method

// Bosses added 10-26
//char  selva[8];
//double DD64;
int16_t quatCountNy[2];

union combo_quat
{
  // elements of a union share the memory, i.e.
  // reside at the same address, not consecutive ones
  // 4 x 16bit = 8 bytes = 64 bit
  int16_t quat[4]; //4x16bit = 64 bit = 8 bytes
  char selva[8]; //8x8  = 64 bit 8 bytes -
} hej;



/// -------------------------------

void setupAdv(void)
{
  Bluefruit.Advertising.addFlags(((0x02) /**< LE General Discoverable Mode. */ | (0x04) /**< BR/EDR not supported. */) /**< LE General Discoverable Mode, BR/EDR not supported. */);
  Bluefruit.Advertising.addTxPower();

  // Include // bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.ScanResponse.addName();

    /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30); // number of seconds in fast mode
  Bluefruit.Advertising.start(0); // 0 = Don't stop advertising after n seconds  
}

// ________________________--


void setup() {
//  Wire.begin();
//  TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1

  Wire.begin();
  //By default .begin() will set I2C SCL to Standard Speed mode of 100kHz
  Wire.setClock(400000); //Optional - set I2C SCL to High Speed Mode of 400kHz
  delay(2000);

  // custom services and characteristics can be added as well
  // bleuart.setLocalName("OHTI");

  Serial.begin(115200);
  bleuart.begin();

 //custom boards may override default pin definitions with bleuart(PIN_REQ, PIN_RDY, PIN_RST)
  // BLEbleuart blebleuart;
 // bleuart.println("Bluefruit52 BLEUART Example");

  // Setup LED pins and reset blinky counter
  pinMode((31), (0x1));
  blinkyms = millis();
  pinMode((17), (0x1));

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this manually via PIN 19
  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  Bluefruit.setName("OHTI");
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit OHTI");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Set up Advertising Packet
  setupAdv();

  // Start Advertising
  Bluefruit.Advertising.start();

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, (0x0));
  pinMode(myLed, (0x1));
  digitalWrite(myLed, (0x1));


 delay(1000);
// Calibration and comm test started here !! - Bosse

 // bleuart.println("Please use Adafruit Bluefruit LE app to connect in UART mode");
  // bleuart.println("Then Enter characters to send");
 // Read the WHO_AM_I register, this is a good test of communication
  bleuart.println("BNO055");
  byte c = readByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x00 /* should be 0xA0              */); // Read WHO_AM_I register for BNO055
  //bleuart.print("BNO055 Address = 0x"); bleuart.println(BNO055_ADDRESS, HEX);
  //bleuart.print("BNO055 WHO_AM_I = 0x"); bleuart.println(BNO055_CHIP_ID, HEX);
  //bleuart.print("BNO055 "); bleuart.print("I AM "); bleuart.print(c, HEX); bleuart.println(" I should be 0xA0");  

  delay(1000);

    // Read the WHO_AM_I register of the accelerometer, this is a good test of communication
  byte d = readByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x01 /* should be 0xFB              */); // Read WHO_AM_I register for accelerometer
  //bleuart.print("BNO055 ACC "); bleuart.print("I AM "); bleuart.print(d, HEX); bleuart.println(" I should be 0xFB");  

  delay(1000);

  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  byte e = readByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x02 /* should be 0x32              */); // Read WHO_AM_I register for magnetometer
  //bleuart.print("BNO055 MAG "); bleuart.print("I AM "); bleuart.print(e, HEX); bleuart.println(" I should be 0x32");

  delay(1000);

  // Read the WHO_AM_I register of the gyroscope, this is a good test of communication
  byte f = readByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x03 /* should be 0x0F              */); // Read WHO_AM_I register for LIS3MDL
  //bleuart.print("BNO055 GYRO "); bleuart.print("I AM "); bleuart.print(f, HEX); bleuart.println(" I should be 0x0F");

  delay(1000);

  if (c == 0xA0) // BNO055 WHO_AM_I should always be 0xA0
  {
    bleuart.println("BNO055 is online...");

    // Check software revision ID
    byte swlsb = readByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x04);
    byte swmsb = readByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x05);
    //bleuart.print("BNO055 SW Revision ID: "); bleuart.print(swmsb, HEX); bleuart.print("."); bleuart.println(swlsb, HEX); 
    //bleuart.println("Should be 03.04");

    // Check bootloader version
    byte blid = readByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x06);
    //bleuart.print("BNO055 bootloader Version: "); bleuart.println(blid); 

    // Check self-test results
    byte selftest = readByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x36);

    if(selftest & 0x01) {
      bleuart.println("accelerometer passed selftest");
    } else {
      bleuart.println("accelerometer failed selftest");
    }
    if(selftest & 0x02) {
      bleuart.println("magnetometer passed selftest");
    } else {
      bleuart.println("magnetometer failed selftest");
    }
    if(selftest & 0x04) {
      bleuart.println("gyroscope passed selftest");
    } else {
      bleuart.println("gyroscope failed selftest");
    }
    if(selftest & 0x08) {
      bleuart.println("MCU passed selftest");
    } else {
      bleuart.println("MCU failed selftest");
    }

    delay(1000);

  delay(1000);

  accelgyroCalBNO055(accelBias, gyroBias);

 // bleuart.println("Average accelerometer bias (mg) = "); bleuart.println(accelBias[0]); bleuart.println(accelBias[1]); bleuart.println(accelBias[2]);
 // bleuart.println("Average gyro bias (dps) = "); bleuart.println(gyroBias[0]); bleuart.println(gyroBias[1]); bleuart.println(gyroBias[2]);

  delay(1000);

  magCalBNO055(magBias);

  //bleuart.println("Average magnetometer bias (mG) = "); bleuart.println(magBias[0]); bleuart.println(magBias[1]); bleuart.println(magBias[2]);

  delay(1000);

  // Check calibration status of the sensors
  uint8_t calstat = readByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x35);
  bleuart.println("Not calibrated = 0, fully calibrated = 3");
  bleuart.print("System calibration status "); bleuart.println( (0xC0 & calstat) >> 6);
  //bleuart.print("Gyro   calibration status "); bleuart.println( (0x30 & calstat) >> 4);
  //bleuart.print("Accel  calibration status "); bleuart.println( (0x0C & calstat) >> 2);
  //bleuart.print("Mag    calibration status "); bleuart.println( (0x03 & calstat) >> 0);

  initBNO055(); // Initialize the BNO055
  bleuart.println("BNO055 initialized for sensor mode...."); // Initialize BNO055 for sensor read 

  }
  else
  {
    bleuart.print("Could not connect to BNO055: 0x");
    bleuart.println(c, 16);
    while(1) ; // Loop forever if communication doesn't happen
  }
}


// --------------------------------- Main -----------------------------------


void loop() {
  delay(10);
  mySensor.readQuat(); // Read sensor
  readQuatData(quatCountNy); // Read the w/x/y/z adc values, quatCount = int16_t (signed)
  // Calculate the quaternion values - divide when receiving with 16384

  hej.quat[0] = (int16_t)(quatCountNy[0]);
  hej.quat[1] = (int16_t)(quatCountNy[1]);
  hej.quat[2] = (int16_t)(quatCountNy[2]);
  hej.quat[3] = (int16_t)(quatCountNy[3]);

  hej.quat[0] = (int16_t)(quatCountNy[0]);
  hej.quat[1] = (int16_t)(quatCountNy[1]);
  hej.quat[2] = (int16_t)(quatCountNy[2]);
  hej.quat[3] = (int16_t)(quatCountNy[3]);

  //dtostrf(float, characters to store result, number of digit, array tp stpre result in)
  //dtostrf((float)(quatCount[0])/16384., 15, 8, hej.selva);

  double DD;
  /*
  double* pToDouble = &DD;
  char* bytes = reinterpret_cast<char*>(pToDouble);
  // bleuart.print(bytes); 
  */
  memcpy(&DD, &mySensor.quat, 8);

// Encode binary to printable string
   char encBuf[10+1] = {}; // +1 for null terminating char
                                     //size_t bytesEncoded = Z85_encode(helloData, encBuf, 8);  //original
   size_t bytesEncoded = Z85_encode(hej.selva, encBuf, 8);

   bleuart.write( encBuf );
   bleuart.write("\n");

}


// -------------------- Still in main scope --------------------------


void connect_callback(void)
{
  bleuart.println("Connected");
}

void disconnect_callback(uint8_t reason)
{
  (void) reason;

  bleuart.println();
  bleuart.println("Disconnected");
  bleuart.println("Bluefruit will auto start advertising (default)");
}

//===================================================================================================================
//====== Calibration procedure moved here -Bosse
//===================================================================================================================

void calibrateBNO055()
{

}


//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6]; // x/y/z accel register data stored here
  readBytes(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x08, 6, &rawData[0]); // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ; // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6]; // x/y/z gyro register data stored here
  readBytes(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x14, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ; // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

int8_t readGyroTempData()
{
  return readByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x34); // Read the two raw data registers sequentially into data array 
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[6]; // x/y/z gyro register data stored here
  readBytes(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x0E, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ; // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readQuatData(int16_t * destination)
{
  uint8_t rawData[8]; // w/x/y/z gyro register data stored here
  readBytes(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x20, 8, &rawData[0]); // Read the 8 (boss4) raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ; // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
  destination[3] = ((int16_t)rawData[7] << 8) | rawData[6] ;
}

void readEulData(int16_t * destination)
{
  uint8_t rawData[6]; // x/y/z gyro register data stored here
  readBytes(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x1A, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ; // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readLIAData(int16_t * destination)
{
  uint8_t rawData[6]; // x/y/z gyro register data stored here
  readBytes(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x28, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ; // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readGRVData(int16_t * destination)
{
  uint8_t rawData[6]; // x/y/z gyro register data stored here
  readBytes(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x2E, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ; // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void initBNO055() {
   // Select page 1 to configure sensors
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x07, 0x01);
   // Configure ACC
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x08, APwrMode << 5 | Abw << 3 | Ascale );
   // Configure GYR
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x0A, Gbw << 3 | Gscale );
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x0B, GPwrMode);
   // Configure MAG
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x09, MPwrMode << 4 | MOpMode << 2 | Modr );

   // Select page 0 to read sensors
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x07, 0x00);

   // Select BNO055 gyro temperature source 
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x40, 0x01 );

   // Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x3B, 0x01 );

   // Select BNO055 system power mode
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x3E, PWRMode );

   // Select BNO055 system operation mode
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x3D, OPRMode );
}

void accelgyroCalBNO055(float * dest1, float * dest2)
{
  uint8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  bleuart.println("AccGyro"); //ibration: Put device on a level surface and keep motionless! Wait......");
  // bleuart.println("Accel/Gyro Calibration: Put device on a level surface and keep motionless! Wait......");
  // delay(4000);
 digitalWrite((17), (0x1)); // turn the LED on (HIGH is the voltage level)
 delay(4000);

  // Select page 0 to read sensors
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x07, 0x00);
   // Select BNO055 system operation mode as NDOF for calibration
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x3D, CONFIGMODE );
   delay(25);
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x3D, NDOF );

 // In NDF fusion mode, accel full scale is at +/- 4g, ODR is 62.5 Hz
   sample_count = 256;
   for(ii = 0; ii < sample_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x08, 6, &data[0]); // Read the six raw data registers into data array
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ; // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    delay(20); // at 62.5 Hz ODR, new accel data is available every 16 ms
   }
    accel_bias[0] /= (int32_t) sample_count; // get average accel bias in mg
    accel_bias[1] /= (int32_t) sample_count;
    accel_bias[2] /= (int32_t) sample_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) 1000;} // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) 1000;}

    dest1[0] = (float) accel_bias[0]; // save accel biases in mg for use in main program
    dest1[1] = (float) accel_bias[1]; // accel data is 1 LSB/mg
    dest1[2] = (float) accel_bias[2];

 // In NDF fusion mode, gyro full scale is at +/- 2000 dps, ODR is 32 Hz
   for(ii = 0; ii < sample_count; ii++) {
    int16_t gyro_temp[3] = {0, 0, 0};
    readBytes(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x14, 6, &data[0]); // Read the six raw data registers into data array
    gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ; // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    gyro_bias[0] += (int32_t) gyro_temp[0];
    gyro_bias[1] += (int32_t) gyro_temp[1];
    gyro_bias[2] += (int32_t) gyro_temp[2];
    delay(35); // at 32 Hz ODR, new gyro data available every 31 ms
   }
    gyro_bias[0] /= (int32_t) sample_count; // get average gyro bias in counts
    gyro_bias[1] /= (int32_t) sample_count;
    gyro_bias[2] /= (int32_t) sample_count;

    dest2[0] = (float) gyro_bias[0]/16.; // save gyro biases in dps for use in main program
    dest2[1] = (float) gyro_bias[1]/16.; // gyro data is 16 LSB/dps
    dest2[2] = (float) gyro_bias[2]/16.;

  // Return to config mode to write accelerometer biases in offset register
  // This offset register is only used while in fusion mode when accelerometer full-scale is +/- 4g
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x3D, CONFIGMODE );
  delay(25);

  //write biases to accelerometer offset registers ad 16 LSB/dps
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x55, (int16_t)accel_bias[0] & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x56, ((int16_t)accel_bias[0] >> 8) & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x57, (int16_t)accel_bias[1] & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x58, ((int16_t)accel_bias[1] >> 8) & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x59, (int16_t)accel_bias[2] & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x5A, ((int16_t)accel_bias[2] >> 8) & 0xFF);

  // Check that offsets were properly written to offset registers
//  bleuart.println("Average accelerometer bias = "); 
//  bleuart.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_LSB))); 
//  bleuart.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_LSB))); 
//  bleuart.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_LSB)));

   //write biases to gyro offset registers
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x61, (int16_t)gyro_bias[0] & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x62, ((int16_t)gyro_bias[0] >> 8) & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x63, (int16_t)gyro_bias[1] & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x64, ((int16_t)gyro_bias[1] >> 8) & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x65, (int16_t)gyro_bias[2] & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x66, ((int16_t)gyro_bias[2] >> 8) & 0xFF);

  // Select BNO055 system operation mode
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x3D, OPRMode );

 // Check that offsets were properly written to offset registers
//  bleuart.println("Average gyro bias = "); 
//  bleuart.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_LSB))); 
//  bleuart.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_LSB))); 
//  bleuart.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_LSB)));

  // bleuart.println("Accel/Gyro Calibration done!");
   // bleuart.println("Accel/Gyro Calibration done!");
    digitalWrite((17), (0x0)); // turn the LED off by making the voltage LOW
}

void magCalBNO055(float * dest1)
{
  uint8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

  bleuart.println("MagCal8"); //ibration: Wave device slowly in a figure eight until done!");
    // bleuart.println("Mag Calibration: Wave device in a figure eight until done!");
    // delay(4000); // replace with blinks
 for (int j=1; j<=10; j=j+1) { // Start our for loop 10 sek
  // Toggle both LEDs every second
  digitalToggle((17));
  delay(1000);
}

  // Select page 0 to read sensors
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x07, 0x00);
   // Select BNO055 system operation mode as NDOF for calibration
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x3D, CONFIGMODE );
   delay(25);
   writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x3D, NDOF );

 // In NDF fusion mode, mag data is in 16 LSB/microTesla, ODR is 20 Hz in forced mode
   sample_count = 256;
   for(ii = 0; ii < sample_count; ii++) {
    int16_t mag_temp[3] = {0, 0, 0};
    readBytes(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x0E, 6, &data[0]); // Read the six raw data registers into data array
    mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ; // Form signed 16-bit integer for each sample in FIFO
    mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(55); // at 20 Hz ODR, new mag data is available every 50 ms
   }

 //   bleuart.println("mag x min/max:"); bleuart.println(mag_max[0]); bleuart.println(mag_min[0]);
 //   bleuart.println("mag y min/max:"); bleuart.println(mag_max[1]); bleuart.println(mag_min[1]);
 //   bleuart.println("mag z min/max:"); bleuart.println(mag_max[2]); bleuart.println(mag_min[2]);

    mag_bias[0] = (mag_max[0] + mag_min[0])/2; // get average x mag bias in counts
    mag_bias[1] = (mag_max[1] + mag_min[1])/2; // get average y mag bias in counts
    mag_bias[2] = (mag_max[2] + mag_min[2])/2; // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0] / 1.6; // save mag biases in mG for use in main program
    dest1[1] = (float) mag_bias[1] / 1.6; // mag data is 1.6 LSB/mg
    dest1[2] = (float) mag_bias[2] / 1.6;

  // Return to config mode to write mag biases in offset register
  // This offset register is only used while in fusion mode when magnetometer sensitivity is 16 LSB/microTesla
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x3D, CONFIGMODE );
  delay(25);

  //write biases to accelerometer offset registers as 16 LSB/microTesla
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x5B, (int16_t)mag_bias[0] & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x5C, ((int16_t)mag_bias[0] >> 8) & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x5D, (int16_t)mag_bias[1] & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x5E, ((int16_t)mag_bias[1] >> 8) & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x5F, (int16_t)mag_bias[2] & 0xFF);
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x60, ((int16_t)mag_bias[2] >> 8) & 0xFF);

  // Check that offsets were properly written to offset registers
//  bleuart.println("Average magnetometer bias = "); 
//  bleuart.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB))); 
//  bleuart.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_LSB))); 
//  bleuart.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_LSB)));
  // Select BNO055 system operation mode
  writeByte(0x28 /*  Device address of BNO055 when ADO = 1 Bosses changed*/, 0x3D, OPRMode );

   bleuart.println("Mag Calibration done!");
   // bleuart.println("Mag Calibration done!");
}

// I2C read/write functions for the BNO055 sensor

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address); // Initialize the Tx buffer
    Wire.write(subAddress); // Put slave register address in Tx buffer
    Wire.write(data); // Put data in Tx buffer
    Wire.endTransmission(); // Send the Tx buffer
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data     
    Wire.beginTransmission(address); // Initialize the Tx buffer
    Wire.write(subAddress); // Put slave register address in Tx buffer
//    Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
    Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
//    Wire.requestFrom(address, 1);  // Read one byte from slave register address 
    Wire.requestFrom(address, (size_t) 1); // Read one byte from slave register address 
    data = Wire.read(); // Fill Rx buffer with result
    return data; // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    Wire.beginTransmission(address); // Initialize the Tx buffer
    Wire.write(subAddress); // Put slave register address in Tx buffer
//    Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
    Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count); // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read(); } // Put read results in the Rx buffer
}

// Added for new adafruit ble

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("Disconnected");
}
