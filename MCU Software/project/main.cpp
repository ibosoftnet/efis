/*
 * EFIS Project
 * IBOSOFT
 * Created: 28.03.2024 01:12:01
 * Author: Caglar
 */ 

/* == Variables Header Files == */
#include "variables.h"

/* == Includes == */
#include <avr/io.h>
#include <stdio.h>
#include "arduino.h"
#include "Wire.h"
#include "SoftwareSerial.h"
#include "virtuabotixRTC.h"
#include "Adafruit_BMP085.h"
#include "ms4525do.h"

/* == Functions == */
void readSettings();
void processSettingsMessage();
void dataOut();
void selectI2CChannel(uint8_t I2CSwChannel);
void imuInit();
void imuCheck();
void imuRead();
void magInit();
void magCheck();
void magRead();
void diffCheck();
void diffRead();
void readGnss();
void processGnssMessage();
void parseGnssMessage();

/* == Others == */
virtuabotixRTC RTC(RtcSclkPin, RtcIoPin, RtcCePin); // RTC

// ############################################################################################# //

void setup() {
	/* Pin Initializations */
	// Flow Control
	// Flow control pins not used so they pulled to high for preventing their LEDs to be bright.
	DDRD |= (1 << PD2) | (1 << PD3);	// Output
	PORTD |= (1 << PD2) | (1 << PD3);	// High
	// Status LED
	DDRD |= 1 << PIND7;			// Output
	// I2C Switch
	DDRD |= (1 << PD6);			// Reset pin of the I2C switch IC must be high
	PORTD |= (1 << PD6);		// Reset pin of the I2C switch IC must be high
	// A/G sense
	DDRC &= ~(1 << DDC1);		// A/G sense pin 1
	PORTC |= (1 << PORTC1);		// A/G sense pin 1 pull-up
	DDRC &= ~(1 << DDC2);		// A/G sense pin 2
	PORTC |= (1 << PORTC2);		// A/G sense pin 2 pull-up
	DDRC &= ~(1 << DDC3);		// A/G sense pin 3
	PORTC |= (1 << PORTC3);		// A/G sense pin 3 pull-up
	/* -------- */

	// Com
	Serial.begin(SERIAL_BAUDRATE);
	mySerial.begin(MYSERIAL_BAUDRATE);
	Wire.begin();
	Wire.setClock(I2C_CLOCK);
	
	// ADC
	ADMUX = 0b01000000;		// 7,6 > REFS[1:0], 00 = AREF, 01 = AVcc + cap at AREF, 10 = reserved, 11 = int 1.1V ref + cap at AREF; 5 ADLAR, 1 = Align Left (ADC H), 0 = Align Right (ADC L) ; 4 none; 3,2,1,0 > MUX
	ADCSRA = 0b10000111;
	
	// IMU
	selectI2CChannel(IMUChannel);
	imuInit();
	delayMicroseconds(10);
	// Mag
	selectI2CChannel(MagChannel);
	magInit();
	delayMicroseconds(10);
	// Press
	selectI2CChannel(PressChannel);
	bmp.begin(bmpOversampling);
	delayMicroseconds(10);
	// Diff
	selectI2CChannel(DiffChannel);
	pres.Config(&Wire, diffAdress, 1.0f, -1.0f);
	pres.Begin();
	//delayMicroseconds(10);
	
	// Delay after initialization
	Serial.println("Initialization Ok!");
	
	// Blink at startup
	for(i=0; i<6; i++) {
		// turn LED on
		PORTD ^= 1 << PIND7;
		delay(150);
	}
	
	delay(1000);
}

void loop() {
	timePrev = millis();
	/* Readings */
	
	// Diff
	selectI2CChannel(DiffChannel);
	diffStatusPrev = diffStatus;
	if (pres.Begin()) {diffStatus =  true;} else {diffStatus =  false;}
	if (!pressStatusPrev & pressStatus) {pres.Begin();}
	delayMicroseconds(10);
	pres.Read();
	diff_pressPa = pres.pres_pa() + diffPressPaErr;
	//diffDieTempC = pres.die_temp_c();
	delayMicroseconds(10);
	
	// Press
	selectI2CChannel(PressChannel);
	pressStatusPrev = pressStatus;
	if (bmp.begin()) {pressStatus =  true;} else {pressStatus =  false;}
	if (!pressStatusPrev & pressStatus) {bmp.begin(bmpOversampling);}
	delayMicroseconds(10);
	press_pressPa = bmp.readPressure();
	delayMicroseconds(10);
	
	// A/G sense
	ag_onGnd1 = !(PINC & (1 << PINC1));
	ag_onGnd2 = !(PINC & (1 << PINC2));
	ag_onGnd3 = !(PINC & (1 << PINC3));
	
	// AOA
	ADMUX &= 0b11110000;			// Reset first 4 bits from LSB of ADMUX
	//ADMUX |= 0b00000000;			// Select ADC0
	ADCSRA |= (1 << ADSC);			// Read ADC0 value
	while (ADCSRA & (1 << ADSC));	// Wait for reading
	aoa_angle = ((float)(ADCW - aoaAdcMin)) * ((aoa_maxAngle - aoa_minAngle) / (aoaAdcMax - aoaAdcMin)) + aoa_minAngle;
	
	// Temp
	ADMUX &= 0b11110000;			// Reset first 4 bits from LSB of ADMUX
	ADMUX |= 0b00000110;			// Select ADC6
	ADCSRA |= (1 << ADSC);			// Start reading
	while (ADCSRA & (1 << ADSC));	// Wait for reading
	tempRefPin = ADCW;
	ADMUX &= 0b11110000;			// Reset first 4 bits from LSB of ADMUX
	ADMUX |= 0b00000111;			// Select ADC7
	ADCSRA |= (1 << ADSC);			// Start reading
	while (ADCSRA & (1 << ADSC));	// Wait for reading
	tempOutPin = ADCW;
	temp_TATC = ((float)(tempOutPin - tempRefPin)) * tempFactor;
	
	// Mag
	selectI2CChannel(MagChannel);
	magCheck();
	delayMicroseconds(10);
	magRead();
	delayMicroseconds(10);
	
	// IMU
	selectI2CChannel(IMUChannel);
	imuCheck();
	delayMicroseconds(10);
	imuRead();
	//delayMicroseconds(10);
	
	
	/* Derived Values */
	// Pitch
	drv_pitch = atan2(imu_az, imu_ay) * 57.2957795131; // PI/180=57.2957795131
	// Roll
	drv_roll = atan2(imu_ax, imu_ay) * 57.2957795131; // PI/180=57.2957795131
	// Turn Rate
	drv_turnRate = -imu_gy / cos(drv_roll);
	// SAT
	drv_SATC = temp_TATC; // su anlik donusum faktoru yok.
	// Pressure ALT ft
	drvPrevPressAlt = drv_pressAltFt;
	drv_pressAltFt = (constT0/constLb)*( pow(press_pressPa/constStdP, (-constRun*constLb)/(constg0*constM0)) - 1.0 ) * constmtoft;
	// Vertical Speed
	drvBaroVspdLoopCount++;
	drvBaroVspdSumOfAltDiff =+ (drv_pressAltFt - drvPrevPressAlt);
	if (drvBaroVspdLoopCount == drvBaroVspdLoopMeasureCount) {
		drv_baroVspdFpm = 60000.0 * (drvBaroVspdSumOfAltDiff/(float)drvBaroVspdLoopMeasureCount) / (float)(millis() - drvBaroVspdTime);
		drvBaroVspdTime = millis();
		drvBaroVspdLoopCount = 0;
		drvBaroVspdSumOfAltDiff = 0.0;
	}
	// Indicated ALT ft
	if (set_altStd == 1) {
		drv_indAltFt = drv_pressAltFt;
	} else {
		drv_indAltFt = (constT0/constLb)*( pow(press_pressPa/set_altStg, (-constRun*constLb)/(constg0*constM0)) - 1.0 ) * constmtoft;
	}
	// KIAS
	drv_kias = 1.943845249221964 * sqrt( ((2*constYAir*press_pressPa)/((constYAir-1)*constStdAirD)) * ( pow( (press_pressPa+diff_pressPa)/press_pressPa, (constYAir-1.0)/constYAir ) - 1.0) );
	if (isnan(drv_kias)) {drv_kias=0;} // If result of sqrt is nan, set 0 to result
	// KCAS
	drv_kcas = drv_kias; // su anlik donusum faktoru yok.
	// KTAS
	drv_ktas = 1.943845249221964 * sqrt( (2*diff_pressPa*constStdAirD) / sqrt( pow( (press_pressPa/(constR*(drv_SATC+constKMinusC))), 2 ) ) );
	if (isnan(drv_ktas)) {drv_ktas=0;} // If result of sqrt is nan, set 0 to result
	// Mach
	drv_mach = sqrt( (2/(constYAir-1)) * ( pow( (diff_pressPa/press_pressPa)+1.0, (constYAir-1.0)/constYAir ) - 1.0) );
	if (isnan(drv_mach)) {drv_mach=0;} // If result of sqrt is nan, set 0 to result
		
	// RTC
	if (set_RtcMessageStatus == 1) {
		sscanf(set_RtcTimeMessage, "%u-%u-%uT%u:%u:%uZ", &RtcSetYear, &RtcSetMonth, &RtcSetDay, &RtcSedHr, &RtcSetMin, &RtcSetSec);
		RTC.setDS1302Time(RtcSetSec, RtcSetMin, RtcSedHr, 0, RtcSetDay, RtcSetMonth, RtcSetYear);
		set_RtcMessageStatus =0;
	}	
	RTC.updateTime(); 
	
	
	/* Print datas */
	dataOut();
	
	/* Loop timing */ 
	readGnss();
	readSettings();
	timeNext = millis();
	loopPrevElapsedTime = timeNext - timePrev;
	while ((timeNext - timePrev) <= loopInterval) {
		readGnss();
		readSettings();
		timeNext = millis();
	}
}


// ############################################################################################# //

void selectI2CChannel(uint8_t I2CSwChannel) {
	Wire.beginTransmission(I2CSwAdress);
	Wire.write(I2CSwChannel);
	Wire.endTransmission();
	delayMicroseconds(5);
}

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
	
	Wire.beginTransmission(imuAdress);
	Wire.write(imuPowerReg);
	Wire.write(imuPowerValue);
	Wire.endTransmission(true);
}

void imuCheck() {
	imuStatusPrev = imuStatus;	
	Wire.beginTransmission(imuAdress);
	if (Wire.endTransmission() == 0) {imuStatus = true;} else {imuStatus = false;}	
	if (!imuStatusPrev & imuStatus) {imuInit();}
}

void imuRead() {
	Wire.beginTransmission(imuAdress);
	Wire.write(imuAccelDataStart);
	Wire.endTransmission(false);
	Wire.requestFrom(imuAdress,6,true);
	
	// Data sequence is H-L-H-L-H-L so (Wire.read() << 8 | Wire.read())
	imu_ax = (Wire.read() << 8 | Wire.read()) / imuAccelFactor + imuAccelxErr;
	switch (imuSwitchYZ) {
		case 1:	
			imu_az = (Wire.read() << 8 | Wire.read()) / imuAccelFactor + imuAccelzErr;
			imu_ay = (Wire.read() << 8 | Wire.read()) / imuAccelFactor + imuAccelyErr;
			break;
		default:
			imu_ay = (Wire.read() << 8 | Wire.read()) / imuAccelFactor + imuAccelyErr;
			imu_az = (Wire.read() << 8 | Wire.read()) / imuAccelFactor + imuAccelzErr;
			break;
	}
	
	Wire.beginTransmission(imuAdress);
	Wire.write(imuGyroDataStart);
	Wire.endTransmission(false);
	Wire.requestFrom(imuAdress,6,true);
	
	// Data sequence is H-L-H-L-H-L so (Wire.read() << 8 | Wire.read())
	imu_gx = ((Wire.read() << 8 | Wire.read()) / imuGyroFactor) + imuGyroxErr;
	switch (imuSwitchYZ) {
		case 1:
			imu_gz = ((Wire.read() << 8 | Wire.read()) / imuGyroFactor) + imuGyrozErr;
			imu_gy = ((Wire.read() << 8 | Wire.read()) / imuGyroFactor) + imuGyroyErr;
			break;
		default:
			imu_gy = ((Wire.read() << 8 | Wire.read()) / imuGyroFactor) + imuGyroyErr;
			imu_gz = ((Wire.read() << 8 | Wire.read()) / imuGyroFactor) + imuGyrozErr;
			break;
	}
	if (imuInvertAxises & X_BIT) {imu_ax = -(imu_ax); imu_gx = -(imu_gx);}
	if (imuInvertAxises & Y_BIT) {imu_ay = -(imu_ay); imu_gy = -(imu_gy);}
	if (imuInvertAxises & Z_BIT) {imu_az = -(imu_az); imu_gz = -(imu_gz);}
}

void magInit() {
	
	Wire.beginTransmission(magAdress);
	Wire.write(magSRReg);
	Wire.write(magSRValue);
	Wire.endTransmission();
	
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
	// Control if data is ready
	Wire.beginTransmission(magAdress);
	Wire.write(magDataReadyReg);
	Wire.endTransmission();
	
	Wire.requestFrom(magAdress, 1);
	while (Wire.available()) {magDataReady = Wire.read();}
	
	if (magDataReady != 0) {

		Wire.beginTransmission(magAdress);
		Wire.write(magDataStart);
		Wire.endTransmission();
		
		Wire.requestFrom(magAdress, 6);
		if (6 <= Wire.available()) {
			// Data sequence is L-H-L-H-L-H so (Wire.read() | (Wire.read() << 8))
			magx = (Wire.read() | (Wire.read() << 8)) + magxErr;
			switch (magSwitchYZ) {
				case 1:
					magz = (Wire.read() | (Wire.read() << 8)) + magyErr;
					magy = (Wire.read() | (Wire.read() << 8)) + magzErr;
					break;
				default:
					magy = (Wire.read() | (Wire.read() << 8)) + magyErr;
					magz = (Wire.read() | (Wire.read() << 8)) + magzErr;
					break;
			}
		}
		
		if (magInvertAxises & X_BIT) {magx = -(magx);}
		if (magInvertAxises & Y_BIT) {magy = -(magy);}
		if (magInvertAxises & Z_BIT) {magz = -(magz);}

		mag_hdg = 180.0 - atan2(magz, magx) * 57.2957795131; // PI/180=57.2957795131
		if (mag_hdg == 360.0) {mag_hdg = 0.0;}
		
	} else {
		delay(magRetryInterval);
		Wire.requestFrom(magAdress, 1);
		while (Wire.available()) {magDataReady = Wire.read();}
		if (magDataReady == 0) {
		magStatus = 0;
		} else {
			magRead();
		}	
	}
}

// ############################################################################################# //

void readSettings() {
	if (Serial.available()) {								// Seri portta veri var mi kontrol ediliyor
		char c = Serial.read();								// Bir karakter okunuyor

		if (c == '#') {										// Yeni bir mesajin baþlangici kontrol ediliyor
			settingsBufferIndex = 0;						// Buffer sifirlaniyor
			newSettingsData = false;						// Yeni veri geldigi belirtiliyor
			} else if (c == '+') {							// Mesajin sonu kontrol ediliyor
			settingsBuffer[settingsBufferIndex] = '\0';		// String sonlandiriliyor
			processSettingsMessage();						// Mesaj isleniyor
			newSettingsData = true;							// Yeni veri geldiði belirtiliyor
			} else {										// Mesajin icerigi okunuyor
			settingsBuffer[settingsBufferIndex] = c;		// Karakter buffer'a ekleniyor
			settingsBufferIndex = (settingsBufferIndex + 1) % SETTINGS_BUFFER_SIZE; // Buffer indeksi güncelleniyor
		}
	}

	// Yeni veri geldiyse, isleme alindiktan sonra buffer temizleniyor
	if (newSettingsData) {
		settingsBufferIndex = 0;
		newSettingsData = false;
	}
}

void processSettingsMessage() {
	// Gelen veriyi isleme
	char *token = strtok(settingsBuffer, "!");			// "!" karakterine göre veriyi parçalýyoruz

	while (token != NULL) {
		if (strstr(token, "asd=") != NULL) {			// "set_altStd=" içeren kýsýmlarý kontrol ediyoruz
			char *value = strchr(token, '=') + 1;		// "=" karakterinin hemen sonrasýndaki deðeri alýyoruz
			set_altStd = atoi(value);					// Ayar deðerini booleanean olarak almak için atoi fonksiyonunu kullanýyoruz
		} else if (strstr(token, "atg=") != NULL) {		// "set_altStg=" içeren kýsýmlarý kontrol ediyoruz
			char *value = strchr(token, '=') + 1;		// "=" karakterinin hemen sonrasýndaki deðeri alýyoruz
			set_altStg = atof(value);					// Ayar deðerini double olarak almak için atof fonksiyonunu kullanýyoruz
		} else if (strstr(token, "clk") != NULL) {		// "!clk" ile başlayan kısmı kontrol ediyoruz
			set_RtcMessageStatus = 1;					// Yeni mesaj geldiğini belirt
			char *value = strchr(token, '=') + 1;		// "=" karakterinin hemen sonrasındaki değeri alıyoruz
			strcpy(set_RtcTimeMessage, value);			// Zamanı RtcTimeMessage değişkenine kopyalıyoruz
		}

		token = strtok(NULL, "!");						// Sonraki tokena geçiyoruz
	}
}

// ############################################################################################# //

void readGnss() {
	if (mySerial.available()) {
		gnssNewMessage = true;
		while (mySerial.available()) {				// Seri portta veri varsa
			char c = mySerial.read();				// Bir karakter okunuyor

			if (c != '\n' && c != '\r') {			// Satır sonu karakterleri hariç
				gnssBuffer[gnssBufferIndex] = c;	// Karakter buffer'a ekleniyor
				gnssBufferIndex = (gnssBufferIndex + 1) % GNSS_BUFFER_SIZE;	// Buffer indeksi güncelleniyor
			
				if ( gnssBufferIndex == 0 || mySerial.available() == 0 ) {	// Buffer dolduğunda veya iletişim kesildiğinde
					gnssBuffer[GNSS_BUFFER_SIZE - 1] = '\0';				// String sonlandırılıyor
					processGnssMessage();									// Mesaj işleniyor
				}
			}
		}
		parseGnssMessage();
	} else {
		gnssNewMessage = false;
	}
}

void processGnssMessage() {
	
	// Stringleri '\0' ile sıfırla
	for (i = 0; i < sizeof(GNSS_GGA); ++i) {GNSS_GGA[i] = '\0';}
	for (i = 0; i < sizeof(GNSS_GSA); ++i) {GNSS_GSA[i] = '\0';}
	for (i = 0; i < sizeof(GNSS_RMC); ++i) {GNSS_RMC[i] = '\0';}
	for (i = 0; i < sizeof(GNSS_VTG); ++i) {GNSS_VTG[i] = '\0';}
	
	// Gelen veriyi işleme
	char *token = strtok(gnssBuffer, "$"); // "$" karakterine göre veriyi parçalıyoruz

	while (token != NULL) {
		if (strstr(token, "GNGSA") != NULL) { // "GNGSA" cümlesini kontrol ediyoruz
			char *value = strchr(token, ',') + 1; // İlk virgülün hemen sonrasındaki değeri alıyoruz
			strcpy(GNSS_GSA, value); // Değeri GNSS_GSA değişkenine kopyalıyoruz
			} else if (strstr(token, "GNGGA") != NULL) { // "GNGGA" cümlesini kontrol ediyoruz
			char *value = strchr(token, ',') + 1; // İlk virgülün hemen sonrasındaki değeri alıyoruz
			strcpy(GNSS_GGA, value); // Değeri GNSS_GGA değişkenine kopyalıyoruz
			} else if (strstr(token, "GNRMC") != NULL) { // "GNRMC" cümlesini kontrol ediyoruz
			char *value = strchr(token, ',') + 1; // İlk virgülün hemen sonrasındaki değeri alıyoruz
			strcpy(GNSS_RMC, value); // Değeri GNSS_RMC değişkenine kopyalıyoruz
			} else if (strstr(token, "GNVTG") != NULL) { // "GNVTG" cümlesini kontrol ediyoruz
			char *value = strchr(token, ',') + 1; // İlk virgülün hemen sonrasındaki değeri alıyoruz
			strcpy(GNSS_VTG, value); // Değeri GNSS_VTG değişkenine kopyalıyoruz
		}

		token = strtok(NULL, "$"); // Sonraki tokena geçiyoruz
	}
	
	// Stringleri '\0' ile sıfırla
	for (i = 0; i < sizeof(gnssBuffer); ++i) {gnssBuffer[i] = '\0';}
	
}

void parseGnssMessage() {
	/*sscanf(GNSS_GGA,);
	sscanf(GNSS_GSA,);
	sscanf(GNSS_RMC,);
	sscanf(GNSS_VTG,);*/
}


// ############################################################################################# //


void dataOut() {
	/* Print */
	Serial.println('#'); 
	Serial.print("/i="); Serial.println(loopPrevElapsedTime);

	// RTC (ISO 8601 Date and time in UTC)
	Serial.print('@'); // Indicates head of the data (not realated to ISO standart)
	Serial.print(RTC.year);
	Serial.print('-');
	if (RTC.month < 10) {Serial.print('0');} Serial.print(RTC.month);
	Serial.print('-');
	if (RTC.dayofmonth < 10) {Serial.print('0');} Serial.print(RTC.dayofmonth);
	Serial.print('T');
	if (RTC.hours < 10) {Serial.print('0');} Serial.print(RTC.hours);
	Serial.print(':');
	if (RTC.minutes < 10) {Serial.print('0');} Serial.print(RTC.minutes);
	Serial.print(':');
	if (RTC.seconds < 10) {Serial.print('0');} Serial.print(RTC.seconds);
	Serial.println('Z');

	
	// Settings
	Serial.print("!asd="); Serial.println(set_altStd);
	Serial.print("!atg="); Serial.println(set_altStg);
	
	// A/G sense
	Serial.print("$gn1="); Serial.println(ag_onGnd1);
	Serial.print("$gn2="); Serial.println(ag_onGnd2);
	Serial.print("$gn3="); Serial.println(ag_onGnd3);
	
	// AOA (deg)
	//Serial.print("$amn="); Serial.println(aoa_minAngle);
	//Serial.print("$amx="); Serial.println(aoa_maxAngle);
	Serial.print("$aoa="); Serial.println(aoa_angle);
	
	// Temp (celsius)
	Serial.print("$tat="); Serial.println(temp_TATC);
	
	// IMU (g & dps)
	Serial.print("%imu="); Serial.println(imuStatus);
	Serial.print("$ax="); Serial.println(imu_ax, 3);
	Serial.print("$ay="); Serial.println(imu_ay, 3);
	Serial.print("$az="); Serial.println(imu_az, 3);
	Serial.print("$gx="); Serial.println(imu_gx, 3);
	Serial.print("$gy="); Serial.println(imu_gy, 3);
	Serial.print("$gz="); Serial.println(imu_gz, 3);
	
	// Mag (deg)
	Serial.print("%mag="); Serial.println(magStatus);
	Serial.print("$mhd="); Serial.println(mag_hdg);
	
	// Press (Pa)
	Serial.print("%prs="); Serial.println(pressStatus);
	Serial.print("$prs="); Serial.println(press_pressPa, 1);
	
	// Diff (Pa)
	Serial.print("%dif="); Serial.println(diffStatus);
	Serial.print("$dif="); Serial.println(diff_pressPa);
	
	/* Derived Values */
	// Pitch (deg)
	Serial.print("&pit="); Serial.println(drv_pitch);
	// Roll (deg)
	Serial.print("&rol="); Serial.println(drv_roll);
	// Turn Rate (dps)
	Serial.print("&trn="); Serial.println(drv_turnRate);
	// SAT (celsius)
	Serial.print("&sat="); Serial.println(drv_SATC);
	// Preessure Alt (ft)
	Serial.print("&plt="); Serial.println(drv_pressAltFt);
	// Indicated Alt (ft)
	Serial.print("&ilt="); Serial.println(drv_indAltFt);
	// Vertical Speed (fpm)
	Serial.print("&vsp="); Serial.println(drv_baroVspdFpm);
	// KIAS (kts)
	Serial.print("&ias="); Serial.println(drv_kias);
	// KCAS (kts)
	Serial.print("&cas="); Serial.println(drv_kcas);
	// KTAS (kts)
	Serial.print("&tas="); Serial.println(drv_ktas);
	// Mach
	Serial.print("&mac="); Serial.println(drv_mach, 4);
	
	// GNSS
	Serial.print("%gnm="); Serial.println(gnssNewMessage);
	Serial.print("?gsa="); Serial.println(GNSS_GSA);
	Serial.print("?gga="); Serial.println(GNSS_GGA);
	Serial.print("?rmc="); Serial.println(GNSS_RMC);
	Serial.print("?vtg="); Serial.println(GNSS_VTG);
	
	
	// End of message
	Serial.println('+');
}