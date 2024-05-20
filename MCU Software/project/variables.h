/*
 * EFIS Project Variables Header File
 * IBOSOFT
 * Created: 14.04.2024 00:28:50
 * Author: Caglar
 */ 

/* == Includes == */
#include <avr/io.h>
#include <stdio.h>
#include "arduino.h"
#include "Wire.h"
#include "SoftwareSerial.h"
#include "virtuabotixRTC.h"
#include "Adafruit_BMP085.h"
#include "ms4525do.h"

/* == Axis Bits for Inverting == */
#define X_BIT 0b100
#define Y_BIT 0b010
#define Z_BIT 0b001

/* == Com == */
#define I2C_CLOCK 400000				// I2C clock in Hz
#define SERIAL_BAUDRATE 115200			// Serial baudrate
#define MYSERIAL_BAUDRATE 9600			// GNSS software serial baudrate
SoftwareSerial mySerial(5, 4, 1);		// RX, TX; last argument should 1 for enabling inverse logic for high speed

/* == I2C Switch == */
#define I2C_CHANNEL_1 0b00000010
#define I2C_CHANNEL_2 0b00000100
#define I2C_CHANNEL_3 0b00001000
#define I2C_CHANNEL_4 0b00010000
#define I2C_CHANNEL_5 0b00100000
#define I2C_CHANNEL_6 0b01000000
static const int8_t I2CSwAdress = 0x70;

/* == RTC == */
static const uint8_t RtcCePin = 10;		// DS1302 Chip Enable
static const uint8_t RtcIoPin = 9;		// DS1302 Serial Data
static const uint8_t RtcSclkPin = 8;	// DS1302 Clock
uint16_t RtcSetSec=0, RtcSetMin=0, RtcSedHr=0, RtcSetDay=1, RtcSetMonth=1, RtcSetYear=2000;

/* == GNSS == */
static const uint16_t GNSS_BUFFER_SIZE = 400;
char gnssBuffer[GNSS_BUFFER_SIZE];
uint8_t gnssBufferIndex = 0;
boolean gnssNewMessage = false;
char GNSS_GGA[64], GNSS_GSA[64], GNSS_RMC[64], GNSS_VTG[64];

/* == General Variables == */
size_t i;
unsigned long timePrev;
unsigned long timeNext;
static const uint8_t loopInterval = 100; // Loop interval in ms
static const uint8_t loopOverflow = 2; // Loop overflow limit when exceeding loop interval
uint16_t loopPrevElapsedTime = 0;

/* == Constants == */
static const double constStdP = 101325.0;		// pascal, ref: ICAO Doc 7488/3
static const float constStdAirD = 1.225;		// kg/m^3, ref: ICAO Doc 7488/3
static const float constR = 287.05287;			// J/(kg*K), specific gas constant, ref: ICAO Doc 7488/3
static const float constRun = 8.31432;			// J/(K*kmol), universal gas constant, derived from ICAO Doc 7488/3
static const float constKMinusC =  273.15;		// ref: ICAO Doc 7488/3
//static const float constt0 = 15.0;			// celsius, ref: ICAO Doc 7488/3
static const float constT0 = 288.15;			// K, ref: ICAO Doc 7488/3
static const float constg0 = 9.80665;			// m/s^2, ref: ICAO Doc 7488/3
static const float constM0 = 0.02896442;		// kg/mol, derived from ICAO Doc 7488/3
static const float constLb = -0.0065;			// K/m, std lapse rate, derived from ICAO Doc 7488/3, (0-11000m)
static const float constYAir = 1.401;			// specific heat capacity ratio for air
static const float constmtoft = 3.2808399;
static const float constPiDiv180 = 57.2957795131; // PI/180=57.2957795131

/* == Settings == */
static const uint8_t SETTINGS_BUFFER_SIZE = 64;
char settingsBuffer[SETTINGS_BUFFER_SIZE];
int settingsBufferIndex = 0;
boolean newSettingsData = false;
// RTC
boolean set_RtcMessageStatus = 0;
char set_RtcTimeMessage[] = "2000-01-01T00:00:00Z";
// Altimeter Setting
boolean set_altStd = 0;			// Altimeter STD setting
double set_altStg = 101300.0;	// Altimeter setting value, standart at first initialization



/* == Sensors == */
// A/G sensing
boolean ag_onGnd1;
boolean ag_onGnd2;
boolean ag_onGnd3;

// AOA
static const uint16_t aoaAdcMin = 0; // 0 - 1023
static const uint16_t aoaAdcMax = 1023; // 0 - 1023
static const float aoa_minAngle = -135.0;
static const float aoa_maxAngle = 135.0;
float aoa_angle;

// Temp
static const float tempFactor = 0.48828125; // 5000/1024/10
uint16_t tempRefPin;
uint16_t tempOutPin;
float temp_TATC;

// IMU
static const uint8_t IMUChannel = I2C_CHANNEL_1;
static const uint8_t imuSwitchYZ = 1;
static const uint8_t imuInvertAxises = 0b100; // x, y, z
boolean imuStatusPrev;
boolean imuStatus = 0;
static const int8_t imuAdress = 0x68; // Default
static const uint8_t imuConfigReg = 0x1A;
static const uint8_t imuConfigValue = 0 << 3 | 6 << 0; // 7 -, 6 -, 543 EXT_SYNC_SET[2:0], 210 DLPF_CFG[2:0]
static const uint8_t imuSampleRateReg = 0x19; // SMPLRT_DIV
static const uint8_t imuSampleRateValue = 3; // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
static const uint8_t imuGyroDataStart = 0x43;
static const uint8_t imuGyroConfigReg = 0x1B;
static const uint8_t imuGyroConfigValue = 1 << 3; // 7 XG_ST, 6 YG_ST, 5 ZG_ST, 43 FS_SEL[1:0], 2 -, 1 -, 0 -;  1 = +-500dps
static const float imuGyroFactor = 65.5; // 65.5 LSB/°/s
static const uint8_t imuAccelDataStart = 0x3B;
static const uint8_t imuAccelConfigReg = 0x1C;
static const uint8_t imuAccelConfigValue = 2 << 3; // 7 XA_ST, 6 YA_ST, 5 ZA_ST, 43 AFS_SEL[1:0], 2 -, 1 -, 0 -;  2 = +-8g
static const float imuAccelFactor = 4096.0; // 4096 LSB/g
static const uint8_t imuPowerReg = 0x6B;
static const uint8_t imuPowerValue = 0b00001000; // 7 Device Reset, 6 Sleep, 5 Cycle, 4 -, 3 Temp Disable, 210 CLKSEL[2:0]; 0 = Internal 8MHz oscillator


float imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz;

static const float imuAccelxErr = 0.0;
static const float imuAccelyErr = 0.0;
static const float imuAccelzErr = 0.0;
static const float imuGyroxErr = 0.0;
static const float imuGyroyErr = 0.0;
static const float imuGyrozErr = 0.0;

// Mag
static const uint8_t MagChannel = I2C_CHANNEL_2;
static const uint8_t magSwitchYZ = 1;
static const uint8_t magInvertAxises = 0b100; // x, y, z
boolean magStatusPrev;
boolean magStatus = 0;
static const int8_t magAdress = 0x0D;		// QMC5883
static const uint8_t magControlReg = 0x09;	// Mode Control Register
static const uint8_t magControlValue =
0b00000001 | // Mode_Standby 0b00000000, Mode_Continuous 0b00000001
0b00001000 | // 10 Hz 0b00000000, 50 Hz 0b00000100, 100 Hz 0b00001000, 200 Hz 0b00001100
0b00010000 | // 2G 0b00000000, 8G 0b00010000
0b00000000 ; // 64 0b11000000, 128 0b10000000, 256 0b01000000, 512 0b00000000
static const uint8_t magSRReg = 0x0B;		// Define Set/Reset period
static const uint8_t magSRValue = 0x01;		// Define Set/Reset period
uint8_t magDataReady = 0;
static const uint8_t magDataReadyReg = 0x06;
static const uint8_t magDataStart = 0x00;	// QMC5883
static const uint8_t magRetryInterval = 2;	// ms
float mag_x, mag_y, mag_z;
float mag_hdg;
int magx, magy, magz;

static const float magxErr = 0.0;
static const float magyErr = 0.0;
static const float magzErr = 0.0;

// Press
static const uint8_t PressChannel = I2C_CHANNEL_5;
boolean pressStatusPrev;
boolean pressStatus = 0;
Adafruit_BMP085 bmp;
static const uint8_t bmpOversampling = BMP085_ULTRAHIGHRES; // BMP085_ULTRAHIGHRES, BMP085_HIGHRES, BMP085_STANDARD
double press_pressPa;

// Diff
static const uint8_t DiffChannel = I2C_CHANNEL_6;
boolean diffStatusPrev;
boolean diffStatus = 0;
static const int8_t diffAdress = 0x28; // Default
bfs::Ms4525do pres;
//float diffDieTempC;
float diff_pressPa;

static const float diffPressPaErr = -10.0;

/* == Derived Values == */
// Pitch
float drv_pitch;
// Roll
float drv_roll;
// Turn Rate
float drv_turnRate;
// SAT
float drv_SATC;
// Pressure Alt ft
float drvPrevPressAlt;
float drv_pressAltFt;
// Vertical Speed
uint8_t drvBaroVspdLoopCount = 0;
static const uint8_t drvBaroVspdLoopMeasureCount = 5;
float drv_baroVspdFpm;
float drvBaroVspdSumOfAltDiff;
unsigned long drvBaroVspdTime = 0; // For millis
// Indicated Alt ft
float drv_indAltFt;
// KIAS
float drv_kias;
// KTAS
float drv_ktas;
// Mach
float drv_mach;
// KCAS
float drv_kcas;