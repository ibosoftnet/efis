/*
 * EFIS Project
 * IBOSOFT alpha
 * Created: 28.03.2024 01:12:01
 * Author : Caglar
 * 
 */ 

//#pragma GCC optimize ("O0")

/* == Com ==*/
#define SERIAL_BAUDRATE 230400 // Serial port baud rate
#define I2C_CLOCK 1000000 // I2C clock in Hz

/* == Includes == */
#include <avr/io.h>
#include <util/delay.h>
#include <arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <ms4525do.h>

/* == General Variables == */
int i;
unsigned long timePrev;
unsigned long timeNext;
static const uint16_t loopInterval = 1000; // Loop interval in ms
uint16_t loopPrevElapsedTime = 0;

/* == Constants == */
static const double constStdP = 101325.000;		// pascal, ref: ICAO Doc 7488/3
static const float constStdAirD = 1.22500;		// kg/m^3, ref: ICAO Doc 7488/3
static const float constR = 287.05;				// J/(kg*K)
static const float constKMinusC =  273.15;
static const float constStdTC = 15.000;			// celsius, ref: ICAO Doc 7488/3
static const float constStdTK = 288.150;		// K, ref: ICAO Doc 7488/3
static const float constYAir = 1.401;			// specific heat capacity ratio for air

/*   Settings   */
bool setMessageInProgress = false;
char setReceivedChar;
// Altimeter Setting
double set_altStg = constStdP;

/* == Sensors == */
// A/G sensing
bool ag_onGnd1;
bool ag_onGnd2;
bool ag_onGnd3;

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
bool imuStatusPrev;
bool imuStatus = 0;
static const int8_t imuAdress = 0x68; // Default
static const int8_t imuConfigReg = 0x1A;
static const int8_t imuConfigValue = 7 << 3 | 2 << 0; // 7 -, 6 -, 543 EXT_SYNC_SET[2:0], 210 DLPF_CFG[2:0]
static const int8_t imuSampleRateReg = 0x19;
static const int8_t imuSampleRateValue = 2; // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
static const int8_t imuGyroDataStart = 0x43;
static const int8_t imuGyroConfigReg = 0x1B;
static const int8_t imuGyroConfigValue = 1 << 3; // 7 XG_ST, 6 YG_ST, 5 ZG_ST, 43 FS_SEL[1:0], 2 -, 1 -, 0 -;  1 = +-500dps = 1000
static const float imuGyroFactor = 65.536; // 65536/1000
static const int8_t imuAccelDataStart = 0x3B;
static const int8_t imuAccelConfigReg = 0x1C;
static const int8_t imuAccelConfigValue = 2 << 3; // 7 XA_ST, 6 YA_ST, 5 ZA_ST, 43 AFS_SEL[1:0], 2 -, 1 -, 0 -;  2 = +-8g = 16
static const float imuAccelFactor = 4096.0; // 65536/16 

float imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz;

static const float imuAccelxErr = 0.0;
static const float imuAccelyErr = 0.0;
static const float imuAccelzErr = 0.0;
static const float imuGyroxErr = 0.0;
static const float imuGyroyErr = 0.0;
static const float imuGyrozErr = 0.0;

// Mag
bool magStatusPrev;
bool magStatus = 0;
static const int8_t magAdress = 0x0D; // QMC5883
static const uint8_t magControlReg = 0x09; // Mode Control Register
static const uint8_t magControlValue = 
0b00000001 | // Mode_Standby 0b00000000, Mode_Continuous 0b00000001
0b00000100 | // 10 Hz 0b00000000, 50 Hz 0b00000100, 100 Hz 0b00001000, 200 Hz 0b00001100
0b00000000 | // 2G 0b00000000, 8G 0b00010000
0b10000000 ; // 64 0b11000000, 128 0b10000000, 256 0b01000000, 512 0b00000000
static const uint8_t magDatastart = 0x00; // QMC5883
float mag_x, mag_y, mag_z; 
float mag_hdg;
int magx, magy, magz;

static const float magxErr = 0.0;
static const float magyErr = 0.0;
static const float magzErr = 0.0;

// Press
bool pressStatusPrev;
bool pressStatus = 0;
Adafruit_BMP085 bmp;
static const uint8_t bmpOversampling = BMP085_HIGHRES; // BMP085_ULTRAHIGHRES, BMP085_HIGHRES, BMP085_STANDARD
double press_press;

// Diff
bool diffStatusPrev;
bool diffStatus = 0;
static const int8_t diffAdress = 0x28; // Default
bfs::Ms4525do pres;
//float diffDieTempC;
float diff_pressPa;

static const float diffPressPaErr = -4.0;

/* == Derived Values == */
// Pitch
float der_pitch;
// Roll
float der_roll;
// SAT
float der_SATC;
// Pressure Alt ft
float der_pressAltFt;
// Indicated Alt ft
float der_indAltFt;
// KIAS
bool der_kiasStatus;
float der_kias;
// KTAS
bool der_ktasStatus;
float der_ktas;
// Mach
bool der_machStatus;
float der_mach;
// KCAS
float der_kcas;

/* == Functions == */
void readSettings();
void imuInit();
void imuCheck();
void imuRead();
void magInit();
void magCheck();
void magRead();
void diffCheck();
void diffRead();

// ############################################################################################# //

void setup() {

	// Blink at startup
	DDRB |= 1 << PINB5;
	for(i=0; i<6; i++) {
		// turn LED on
		PORTB ^= 1 << PINB5;
		_delay_ms(150);
	}
	
	// Com
	Serial.begin(SERIAL_BAUDRATE);
	Wire.begin();
	Wire.setClock(I2C_CLOCK);
	
	// ADC
	ADMUX = 0b01000000;			// 7,6 > REFS[1:0], 00 = AREF, 01 = AVcc + cap at AREF, 10 = reserved, 11 = int 1.1V ref + cap at AREF; 5 ADLAR, 1 = Align Left (ADC H), 0 = Align Right (ADC L) ; 4 none; 3,2,1,0 > MUX
	ADCSRA = 0b10000111;
	
	/* Initialization */
	// A/G sense
	DDRB &= ~(1 << DDB0);		// A/G sense pin 1
	PORTB |= (1 << PORTB0);		// A/G sense pin 1 pull-up
	DDRB &= ~(1 << DDB1);		// A/G sense pin 2
	PORTB |= (1 << PORTB1);		// A/G sense pin 2 pull-up
	DDRB &= ~(1 << DDB2);		// A/G sense pin 3
	PORTB |= (1 << PORTB2);		// A/G sense pin 3 pull-up
	// IMU
	imuInit();
	_delay_ms(2);
	// Mag
	magInit();
	_delay_ms(2);
	// Press
	bmp.begin(bmpOversampling);
	_delay_ms(2);
	// Diff
	pres.Config(&Wire, diffAdress, 1.0f, -1.0f);
	pres.Begin();
	//_delay_ms(2);
	
	// Delay after initialization
	Serial.print("Initialization Ok!");
	_delay_ms(1000);
}

void loop() {
	timePrev = millis();
	/* Readings */
	
	// A/G sense
	ag_onGnd1 = !(PINB & (1 << PINB0));
	ag_onGnd2 = !(PINB & (1 << PINB1));
	ag_onGnd3 = !(PINB & (1 << PINB2));
	
	// AOA
	ADMUX &= 0b11110000;			// Reset first 4 bits from LSB of ADMUX
	//ADMUX |= 0b00000000;			// Select ADC0
	ADCSRA |= (1 << ADSC);			// Read ADC0 value
	while (ADCSRA & (1 << ADSC));	// Wait for reading
	aoa_angle = ((float)(ADCW - aoaAdcMin)) * ((aoa_maxAngle - aoa_minAngle) / (aoaAdcMax - aoaAdcMin)) + aoa_minAngle;
	
	// Temp
	ADMUX &= 0b11110000;			// Reset first 4 bits from LSB of ADMUX
	ADMUX |= 0b00000001;			// Select ADC1
	ADCSRA |= (1 << ADSC);			// Start reading
	while (ADCSRA & (1 << ADSC));	// Wait for reading
	tempRefPin = ADCW;
	ADMUX &= 0b11110000;			// Reset first 4 bits from LSB of ADMUX
	ADMUX |= 0b00000010;			// Select ADC2
	ADCSRA |= (1 << ADSC);			// Start reading
	while (ADCSRA & (1 << ADSC));	// Wait for reading
	tempOutPin = ADCW;
	temp_TATC = ((float)(tempOutPin - tempRefPin)) * tempFactor;
	
	// IMU
	imuCheck();
	_delay_ms(2);
	imuRead();
		
	// Mag
	magCheck();
	_delay_ms(2);
	magRead();
	
	// Press
	pressStatusPrev = pressStatus;
	if (bmp.begin()) {pressStatus =  true;} else {pressStatus =  false;}
	if (!pressStatusPrev & pressStatus) {bmp.begin(bmpOversampling);}
	_delay_ms(2);
	press_press = bmp.readPressure();
	
	// Diff
	diffStatusPrev = diffStatus;
	if (pres.Begin()) {diffStatus =  true;} else {diffStatus =  false;}
	if (!pressStatusPrev & pressStatus) {pres.Begin();}
	_delay_ms(1);
	pres.Read();
	_delay_ms(2);
	diff_pressPa = pres.pres_pa() + diffPressPaErr;
	//diffDieTempC = pres.die_temp_c();
	
	/* Derived Values */
	// Pitch
	der_pitch = atan2(imu_az, imu_ay) * 180.0 / M_PI;
	// Roll
	der_roll = atan2(imu_ax, imu_ay) * 180.0 / M_PI;
	// SAT
	der_SATC = temp_TATC; // su anlik donusum faktoru yok.
	// Pressure ALT ft
	der_pressAltFt = 44330 * (1.0 - pow(press_press / constStdP, 0.1903));
	// Indicated ALT ft
	der_indAltFt = 44330 * (1.0 - pow(press_press / set_altStg, 0.1903));
	// KIAS
	der_kiasStatus = pressStatus & diffStatus;
	der_kias = 1.943845249221964 * sqrt( ((2*constYAir*press_press)/((constYAir-1)*constStdAirD)) * ( pow( (press_press+diff_pressPa)/press_press, (constYAir-1.0)/constYAir ) - 1.0) );
	if (isnan(der_kias)) {der_kias=0;} // If result of sqrt is nan, set 0 to result
	// KTAS
	der_ktasStatus = pressStatus & diffStatus;
	der_ktas = 1.943845249221964 * sqrt( (2*diff_pressPa*constStdAirD) / sqrt( pow( (press_press/(constR*(der_SATC+constKMinusC))), 2 ) ) );
	if (isnan(der_ktas)) {der_ktas=0;} // If result of sqrt is nan, set 0 to result
	// Mach
	der_machStatus = pressStatus & diffStatus;
	der_mach = sqrt( (2/(constYAir-1)) * ( pow( (diff_pressPa/press_press)+1.0, (constYAir-1.0)/constYAir ) - 1.0) );
	if (isnan(der_mach)) {der_mach=0;} // If result of sqrt is nan, set 0 to result
	// KCAS
	der_kcas = der_kias; // su anlik donusum faktoru yok.
	
	
	
	/* Print */
	Serial.print('#'); Serial.println(loopPrevElapsedTime);
	
	// Settings
	Serial.print("!set_altStg="); Serial.println(set_altStg);
	
	// A/G sense
	Serial.print("$ag_onGnd1="); Serial.println(ag_onGnd1);
	Serial.print("$ag_onGnd2="); Serial.println(ag_onGnd2);
	Serial.print("$ag_onGnd3="); Serial.println(ag_onGnd3);
	
	// AOA
	Serial.print("$aoa_minAngle="); Serial.println(aoa_minAngle);
	Serial.print("$aoa_maxAngle="); Serial.println(aoa_maxAngle);
	Serial.print("$aoa_angle="); Serial.println(aoa_angle);
	
	// Temp
	Serial.print("$temp_TATC="); Serial.println(temp_TATC);
	
	// IMU
	Serial.print("*imuStatus="); Serial.println(imuStatus);
	Serial.print("$imu_ax="); Serial.println(imu_ax);
	Serial.print("$imu_ay="); Serial.println(imu_ay);
	Serial.print("$imu_az="); Serial.println(imu_az);
	Serial.print("$imu_gx="); Serial.println(imu_gx);
	Serial.print("$imu_gy="); Serial.println(imu_gy);
	Serial.print("$imu_gz="); Serial.println(imu_gz);
	
	// Mag
	Serial.print("*magStatus="); Serial.println(magStatus);
	Serial.print("$mag_hdg="); Serial.println(mag_hdg);
	
	// Press
	Serial.print("*pressStatus="); Serial.println(pressStatus);
	Serial.print("$press_press="); Serial.println(press_press, 1);
	
	// Diff
	Serial.print("*diffStatus="); Serial.println(diffStatus);
	Serial.print("$diff_pressPa="); Serial.println(diff_pressPa);
	
	/* Derived Values */
	// Pitch
	Serial.print("*der_pitch="); Serial.println(der_pitch);
	// Roll
	Serial.print("*der_roll="); Serial.println(der_roll);
	// SAT
	Serial.print("*der_SATC="); Serial.println(der_SATC);
	// Preessure Alt ft
	Serial.print("*der_pressAltFt="); Serial.println(der_pressAltFt);
	// Indicated Alt ft
	Serial.print("*der_indAltFt="); Serial.println(der_indAltFt);
	// KIAS
	Serial.print("*der_kiasStatus="); Serial.println(der_indAltFt);
	Serial.print("$der_kias="); Serial.println(der_kias);
	// KTAS
	Serial.print("*der_ktasStatus="); Serial.println(der_ktasStatus);
	Serial.print("$der_ktas="); Serial.println(der_ktas);
	// Mach
	Serial.print("*der_machStatus="); Serial.println(der_machStatus);
	Serial.print("$der_mach="); Serial.println(der_mach, 4);
	
	// End of message
	Serial.println('+');
	
	/* Loop timing */ 
	readSettings();
	timeNext = millis();
	loopPrevElapsedTime = timeNext - timePrev;
	while ((timeNext - timePrev) <= loopInterval) {
		readSettings();
		timeNext = millis();
	}
}


// ############################################################################################# //

void imuInit() {
	Wire.beginTransmission(imuAdress);
	Wire.write(imuConfigReg);
	Wire.write(imuConfigValue);
	Wire.endTransmission(true);
	
	Wire.beginTransmission(imuAdress);
	Wire.write(imuSampleRateReg);
	Wire.write(imuSampleRateValue);
	Wire.endTransmission(true);

	Wire.beginTransmission(imuAdress);
	Wire.write(imuAccelConfigReg);
	Wire.write(imuAccelConfigValue);
	Wire.endTransmission(true);
	
	Wire.beginTransmission(imuAdress);
	Wire.write(imuGyroConfigReg);
	Wire.write(imuGyroConfigValue);
	Wire.endTransmission(true);
}

void imuCheck() {
	imuStatusPrev = imuStatus;	
	Wire.beginTransmission(imuAdress);
	if (Wire.endTransmission() == 0) {
		imuStatus = true;} else {imuStatus = false;}	
	if (!imuStatusPrev & imuStatus) {imuInit();}
}

void imuRead() {
	Wire.beginTransmission(imuAdress);
	Wire.write(imuAccelDataStart);
	Wire.endTransmission(false);
	Wire.requestFrom(imuAdress,6,true);
	
	imu_ax = (Wire.read() << 8 | Wire.read()) / imuAccelFactor + imuAccelxErr;
	imu_ay = (Wire.read() << 8 | Wire.read()) / imuAccelFactor + imuAccelyErr;
	imu_az = (Wire.read() << 8 | Wire.read()) / imuAccelFactor + imuAccelzErr;
	
	Wire.beginTransmission(imuAdress);
	Wire.write(imuAccelDataStart);
	Wire.endTransmission(false);
	Wire.requestFrom(imuAdress,6,true);
	
	imu_gx = ((Wire.read() << 8 | Wire.read()) / imuGyroFactor) + imuGyroxErr;
	imu_gy = ((Wire.read() << 8 | Wire.read()) / imuGyroFactor) + imuGyroyErr;
	imu_gz = ((Wire.read() << 8 | Wire.read()) / imuGyroFactor) + imuGyrozErr;

}

void magInit() {
	
	Wire.beginTransmission(magAdress);
	Wire.write(magControlReg);
	Wire.write(magControlValue);
	Wire.endTransmission();
}

void magCheck() {
	magStatusPrev = magStatus;
	Wire.beginTransmission(magAdress);
	if (Wire.endTransmission() == 0) {magStatus = true;} else {magStatus = false;}
	if (!magStatusPrev & magStatus) {magInit();}
}

void magRead() {
	Wire.beginTransmission(magAdress);
	Wire.write(magDatastart);
	Wire.endTransmission();
	  
	Wire.requestFrom(magAdress, 6);
	if (6 <= Wire.available()) {
		magx = (Wire.read() << 8 | Wire.read()) + magxErr;
		magz = (Wire.read() << 8 | Wire.read()) + magzErr;
		magy = (Wire.read() << 8 | Wire.read()) + magyErr;
	}

	mag_hdg = atan2(magz, magx) * 180 / PI;
	if (mag_hdg < 0) {
		mag_hdg += 360;
	} 
}

void readSettings() {
  if (Serial.available()) {													// Seri portta veri var mi kontrol et
	  setReceivedChar = Serial.read();										// Bir karakter al
	  if (setReceivedChar == '#') {											// Eger alinan karakter "#" ise
		  setMessageInProgress = true;										// Mesaj alimi basladi
		  /*} else if (setReceivedChar == '+') {							// Eger alinan karakter "+" ise
		  setMessageInProgress = false;*/									// Mesaj alimi tamamlandi
		  } else if (setMessageInProgress && setReceivedChar != '\n') {		// Eger mesaj alimi devam ediyorsa ve alinan karakter yeni satir karakteri degilse
																			// Mesaj alimi sirasinda ise, karakterleri isle
		  if (setReceivedChar == '!') {
			  String setCommand = Serial.readStringUntil('=');
			if (setCommand == "!set_altStg") {
				  String setValueString = Serial.readStringUntil('\n');
				  setValueString.trim();
				  set_altStg = setValueString.toFloat();
			} /* else if (command == "!set_another_variable") {
				  String valueString = Serial.readStringUntil('\n');
				  valueString.trim();
				  set_another_variable = valueString.toFloat();
			} */
			  // Diger komutlar icin benzer kontrolleri buraya ekleyebilirsiniz
		  }
	  }
  }

}
