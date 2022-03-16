/*  TETRIX (T2) PRIZM robotics controller cpp File for PRIZM Arduino Library
   	Written by: Paul W. Uttley
		10/21/2021
		Version 1.0
		For: ESP32 Module
		==================== PROTOTYPE version in Process ==================
		Some functions are not yet fully implemented
*/

#include <Arduino.h>
#include <Wire.h>
#include "PRIZM_ESP32.h"

PRIZM::PRIZM(u16 n /*= 8*/, u8 pin_gpio /*= 2*/, u8 chn /*= 0*/, LED_TYPE t /*= TYPE_GRB*/)
{
	ledCounts = 3;
	pin = 23;
	rmt_chn = 0;
	rmt_mem = RMT_MEM_64;
	br = 255;
	setLedType(t);
}

bool PRIZM::beginLed()
{
//	PRIZM(3, 23, 0, TYPE_GRB);
	switch(rmt_chn){
		case 0:
			rmt_mem=RMT_MEM_64;break;
		case 1:
			rmt_mem=RMT_MEM_128;break;
		case 2:
			rmt_mem=RMT_MEM_192;break;
		case 3:
			rmt_mem=RMT_MEM_256;break;
		case 4:
			rmt_mem=RMT_MEM_320;break;
		case 5:
			rmt_mem=RMT_MEM_384;break;
		case 6:
			rmt_mem=RMT_MEM_448;break;
		case 7:
			rmt_mem=RMT_MEM_512;break;
		default:
			rmt_mem=RMT_MEM_64;break;
	}
	if ((rmt_send = rmtInit(pin, true, rmt_mem)) == NULL){
		return false;
	}
	for(int i=0;i<ledCounts;i++)
	{
		for (int bit = 0; bit < 24; bit++) {
			led_data[i*24+bit].level0 = 1;
			led_data[i*24+bit].duration0 = 4;
			led_data[i*24+bit].level1 = 0;
			led_data[i*24+bit].duration1 = 8;
		}
	}
	realTick = rmtSetTick(rmt_send, 100);
	return true;
}

void PRIZM::setLedCount(u16 n)
{
	ledCounts = n;
	beginLed();
}

void PRIZM::setLedType(LED_TYPE t)
{
	rOffset = (t >> 4) & 0x03;
	gOffset = (t >> 2) & 0x03;
	bOffset = t & 0x03;
}

void PRIZM::setLedBrightness(u8 brightness)
{
	br = constrain(brightness, 0, 255);
}

esp_err_t PRIZM::setLedColorData(int index, u32 rgb)
{
	return setLedColorData(index, rgb >> 16, rgb >> 8, rgb);
}

esp_err_t PRIZM::setLedColorData(int index, u8 r, u8 g, u8 b)
{
	u8 p[3];
	p[rOffset] = r * br / 255;
	p[gOffset] = g * br / 255;
	p[bOffset] = b * br / 255;
	return set_pixel(index, p[0], p[1], p[2]);
}

esp_err_t PRIZM::set_pixel(int index, u8 r, u8 g, u8 b)
{
	u32 color = r << 16 | g << 8 | b ;
	for (int bit = 0; bit < 24; bit++) {
		if (color & (1 << (23-bit))) {
			led_data[index*24+bit].level0 = 1;
			led_data[index*24+bit].duration0 = 8;
			led_data[index*24+bit].level1 = 0;
			led_data[index*24+bit].duration1 = 4;
		} else {
			led_data[index*24+bit].level0 = 1;
			led_data[index*24+bit].duration0 = 4;
			led_data[index*24+bit].level1 = 0;
			led_data[index*24+bit].duration1 = 8;
		}
	}
	return ESP_OK;
}

esp_err_t PRIZM::setLedColor(int index, u32 rgb)
{
	return setLedColor(index, rgb >> 16, rgb >> 8, rgb);
}

esp_err_t PRIZM::setLedColor(int index, u8 r, u8 g, u8 b)
{
	setLedColorData(index, r, g, b);
	return showLedData();
}

esp_err_t PRIZM::setLedColorsData(u32 rgb)
{
	for (int i = 0; i < ledCounts; i++)
	{
		setLedColorData(i, rgb);
	}
	return ESP_OK;
}

esp_err_t PRIZM::setLedColorsData(u8 r, u8 g, u8 b)
{
	for (int i = 0; i < ledCounts; i++)
	{
		setLedColorData(i, r, g, b);
	}
	return ESP_OK;
}

esp_err_t PRIZM::setLedColors(u32 rgb)
{
	setLedColorsData(rgb);
	showLedData();
	return ESP_OK;
}

esp_err_t PRIZM::setLedColors(u8 r, u8 g, u8 b)
{
	setLedColorsData(r, g, b);
	showLedData();
	return ESP_OK;
}

esp_err_t PRIZM::showLedData()
{
  delay(10);
	return rmtWrite(rmt_send, led_data, ledCounts*24);
}

uint32_t PRIZM::colorWheel(byte pos)
{
	u32 WheelPos = pos % 0xff;
	if (WheelPos < 85) {
		return ((255 - WheelPos * 3) << 16) | ((WheelPos * 3) << 8);
	}
	if (WheelPos < 170) {
		WheelPos -= 85;
		return (((255 - WheelPos * 3) << 8) | (WheelPos * 3));
	}
	WheelPos -= 170;
	return ((WheelPos * 3) << 16 | (255 - WheelPos * 3));
}

uint32_t PRIZM::hsv2rgb(uint32_t h, uint32_t s, uint32_t v)
{
	u8 r, g, b;
	h %= 360; // h -> [0,360]
	uint32_t rgb_max = v * 2.55f;
	uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

	uint32_t i = h / 60;
	uint32_t diff = h % 60;

	// RGB adjustment amount by hue
	uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

	switch (i) {
	case 0:
		r = rgb_max;
		g = rgb_min + rgb_adj;
		b = rgb_min;
		break;
	case 1:
		r = rgb_max - rgb_adj;
		g = rgb_max;
		b = rgb_min;
		break;
	case 2:
		r = rgb_min;
		g = rgb_max;
		b = rgb_min + rgb_adj;
		break;
	case 3:
		r = rgb_min;
		g = rgb_max - rgb_adj;
		b = rgb_max;
		break;
	case 4:
		r = rgb_min + rgb_adj;
		g = rgb_min;
		b = rgb_max;
		break;
	default:
		r = rgb_max;
		g = rgb_min;
		b = rgb_max - rgb_adj;
		break;
	}
	return (uint32_t)(r << 16 | g << 8 | b);
}









void PRIZM::setMotorSpeedPID(int P, int I, int D){	//=== Change the speed PID parameters for DC Chip

  int lobyteP;
  int hibyteP;
  int lobyteI;
  int hibyteI;
  int lobyteD;
  int hibyteD;

  lobyteP  = lowByte(P);
  hibyteP  = highByte(P);
  lobyteI  = lowByte(I);
  hibyteI  = highByte(I);
  lobyteD  = lowByte(D);
  hibyteD  = highByte(D);

  Wire.beginTransmission(5);
  Wire.write(0X56);
  Wire.write(hibyteP);
  Wire.write(lobyteP);
  Wire.write(hibyteI);
  Wire.write(lobyteI);
  Wire.write(hibyteD);
  Wire.write(lobyteD);
  Wire.endTransmission();
  delay(10);

}

void PRIZM::setMotorTargetPID(int P, int I, int D){	//=== Change the target PID parameters for DC chip

  int lobyteP;
  int hibyteP;
  int lobyteI;
  int hibyteI;
  int lobyteD;
  int hibyteD;

  lobyteP  = lowByte(P);
  hibyteP  = highByte(P);
  lobyteI  = lowByte(I);
  hibyteI  = highByte(I);
  lobyteD  = lowByte(D);
  hibyteD  = highByte(D);

  Wire.beginTransmission(5);    	   //transmit to DC address
  Wire.write(0X57);
  Wire.write(hibyteP);
  Wire.write(lobyteP);
  Wire.write(hibyteI);
  Wire.write(lobyteI);
  Wire.write(hibyteD);
  Wire.write(lobyteD);
  Wire.endTransmission();
  delay(10);

}

int PRIZM::readDCFirmware(int chip){	  //==== Request firmware version of DC motor chip

  int byte1;
  int DCversion;
	int chip_address;

	if(chip==1){chip_address = 5;}	// DC motor chip 1,2
	if(chip==2){chip_address = 9;}	// DC motor chip 3,4

  Wire.beginTransmission(chip_address);
  Wire.write(0x1F);
  Wire.endTransmission();
  delay(10);
  Wire.requestFrom(chip_address, 1);
  byte1 = Wire.read();
  DCversion=byte1;
  delay(10);
  return DCversion;
}

int PRIZM::readSVOFirmware(){		//==== Request firmware version of Servo motor chip

  int byte1;
  int SVOversion;

  Wire.beginTransmission(6);
  Wire.write(0x1F);
  Wire.endTransmission();
  delay(10);
  Wire.requestFrom(6, 1);
  byte1 = Wire.read();
  SVOversion=byte1;
  delay(10);
  return SVOversion;
}

void PRIZM::PrizmBegin(){  //======= Send a SW reset to all motor control co-processors and EXPANSION port I2C, 0x20 is co-processors, 0x27 is expansion
	Wire.begin(19, 18, 100000);   	// create i2c bus on pins 18 and 19 (5 volt Bus)
	Wire1.begin(21, 22, 100000);		// create i2c bus on pins 21 and 22 (3.3 volt bus)
																	// DC motor channels 1 and 2 = address 5; DC channels 3 and 4 = address 9; servo channels = address 6
  delay(500);               			// Give EXPANSION controllers time to reset
  	     													// SW reset on Expansion and DC + Servo chips at addresses 5, 6 and 9 (7 is not used)
  Wire.beginTransmission(5);    	// Supported I2C addresses for EXPANSIONansion controllers is 1 - 4 (4 boxes total)
  Wire.write(0x20);              	// If additional boxes above that are added at different addresses, writes for each need to be added as well.
  Wire.endTransmission();         // No guarantee that more than 4 boxes will work on single I2C bus because of cable capacitance
  delay(10);
  Wire.beginTransmission(6);
  Wire.write(0x20);
  Wire.endTransmission();
  delay(10);
	Wire.beginTransmission(9);
	Wire.write(0x20);
	Wire.endTransmission();
	delay(10);
  Wire.beginTransmission(1);
  Wire.write(0x27);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(2);
  Wire.write(0x27);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(3);
  Wire.write(0x27);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(4);
  Wire.write(0x27);
  Wire.endTransmission();
  delay(10);

  beginLed();                         // initialize RGB 2812 LED's after a reset
  setLedBrightness(50);               // default brightness
  setLedColors(0,0,0);             	  // turn off the LED's
  delay(50);


    setLedColors(255,0,0);           // all led's red
    delay(1000);

		setLedColor(0,0,0,0);
		delay(120);
		setLedColor(1,0,0,0);
		delay(120);
		setLedColor(2,0,0,0);
		delay(120);
		setLedColor(2,255,255,0);
		delay(120);
		setLedColor(2,0,0,0);
		setLedColor(1,255,255,0);
		delay(120);
		setLedColor(1,0,0,0);
		setLedColor(0,255,255,0);
		delay(120);
		setLedColor(0,0,0,0);

/*
		setLedColor(255,0,0);
		delay(1000);
		setLedColor(2,0,255,0);
		delay(120);
		setLedColor(1,0,255,0);
		delay(120);
		setLedColor(0,0,255,0);
		delay(1000);
*/

  setGreenLED(HIGH);  								// Turn on when we're reset
  while(readGreenButton()==0){};   		// wait for the program start (green) button pressed
  setGreenLED(LOW);   								// turn green off

  Wire.beginTransmission(5);     			// Supported I2C addresses for EXPANSIONansion controllers is 1 - 4 (4 boxes total)
  Wire.write(0x20);                  	// Do a reset on Expansions and PRIZM Motor chips after green button start
  Wire.endTransmission();             // If additional boxes above that are added at different addresses, writes for each need to be added as well.
  delay(10);
  Wire.beginTransmission(6);
  Wire.write(0x20);
  Wire.endTransmission();
  delay(10);
	Wire.beginTransmission(9);
	Wire.write(0x20);
	Wire.endTransmission();
	delay(10);
  Wire.beginTransmission(1);
  Wire.write(0x27);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(2);
  Wire.write(0x27);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(3);
  Wire.write(0x27);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(4);
  Wire.write(0x27);
  Wire.endTransmission();
  delay(10);

  delay(1000);     									// 1 second delay between time GO button is pushed and program starts gives time for resets

  Wire.beginTransmission(5);    		// Send an "Enable" Byte to DC and Servo controller chips and EXPANSIONansion controllers
  Wire.write(0x1E);                 // enable command so that the robots won't move without a PrizmBegin statement
  Wire.endTransmission();						// DC and servo chips enable = 0x1E; expansion boxes = 0x25
  delay(10);
  Wire.beginTransmission(6);
  Wire.write(0x1E);
  Wire.endTransmission();
  delay(10);
	Wire.beginTransmission(9);
	Wire.write(0x1E);
	Wire.endTransmission();
	delay(10);
  Wire.beginTransmission(1);
  Wire.write(0x25);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(2);
  Wire.write(0x25);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(3);
  Wire.write(0x25);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(4);
  Wire.write(0x25);
  Wire.endTransmission();
  delay(10);

	soundEnable();				// call sound enable setup

}

void PRIZM::PrizmEnd(){  //======= Send a SW reset to all I2C devices(resets everything) This is done mainly to stop all motors

  Wire.beginTransmission(5);     			// Supported I2C addresses for EXPANSIONansion controllers is 1 - 4
  Wire.write(0x20);                  	// 5, 6 and 9 is PRIZM DC and Servo chips = 0x20
  Wire.endTransmission();							// expansion boxes are 0x27
  delay(10);
  Wire.beginTransmission(6);
  Wire.write(0x20);
  Wire.endTransmission();
  delay(10);
	Wire.beginTransmission(9);
	Wire.write(0x20);
	Wire.endTransmission();
	delay(10);
  Wire.beginTransmission(1);
  Wire.write(0x27);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(2);
  Wire.write(0x27);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(3);
  Wire.write(0x27);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(4);
  Wire.write(0x27);
  Wire.endTransmission();
  delay(10);

//  wdt_enable(WDTO_15MS);  // set the wdt to timeout after 15ms automatically resets
//  for(;;)
//    {
//    }

}

// ============================= SENSORS =============================

int PRIZM::readLineSensor(int pin){     // Can sense black or white (follow the edge of a black line on a white surface)
  int BWstate; // black or white
  pinMode(pin, INPUT);
  if(HIGH == digitalRead(pin)){BWstate = 1;} else {BWstate = 0;}
  return BWstate;
}

int PRIZM::readSonicSensorCM(int port){   // Returns distance of object from sensor in Centimeters
	int pin = 0;

	// assign data pin for each port - not allowed, portA and portF
	if (port == 2) pin = 32;
	if (port == 3) pin = 25;
	if (port == 4) pin = 27;
	if (port == 5) pin = 16;

  delayMicroseconds(1000);  	// added in version 2 to help with reading accuracy, can't read sonic sensors very fast
  int duration;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin,LOW);
  pinMode(pin,INPUT);
  duration = pulseIn(pin,HIGH);
  return duration/29/2; // convert time of echo to centimeters distance

}

int PRIZM::readSonicSensorIN(int port){   // Returns distance of object from sensor in Inches
	int pin = 0;

	// assign data pin for each port - not allowed, portA and portF
	if (port == 2) pin = 32;
	if (port == 3) pin = 25;
	if (port == 4) pin = 27;
	if (port == 5) pin = 16;

  delayMicroseconds(1000);  	// added in version 2 to help with reading accuracy, can't read sonic sensors very fast
  int duration;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin,LOW);
  pinMode(pin,INPUT);
  duration = pulseIn(pin,HIGH);
  return duration/74/2; // convert time of echo to centimeters distance

}

// ================================ LED'S ==========================================

void PRIZM::setGreenLED (int state){
  pinMode(14, OUTPUT); 									//===== GREEN LED is on IO 14
  if (state==1){digitalWrite(14, HIGH);}
  if (state==0){digitalWrite(14, LOW);}
}

// ============================== PUSHBUTTONS =======================================

int PRIZM::readGreenButton(){       //============== function returns; unpressed button == 0; pressed button == 1
  pinMode(39, INPUT); 							// PB-2 Button is on pin 39; unpressed = high, pressed = low
  int gButton = digitalRead(39);
  gButton = !gButton; 						// invert variable to make sense;
  return gButton;
}

int PRIZM::readBlackButton(){       //============== function returns; unpressed button == 0; pressed button == 1
  pinMode(36, INPUT); 							// PB-2 Button is on pin 36; unpressed = high, pressed = low
  int bButton = digitalRead(36);
  bButton = !bButton; 						  // invert variable to make sense;
  return bButton;
}

// ================================ SERVO MOTORS ===================================

void PRIZM::setServoSpeed (int channel, int servospeed){   //=========== function for setting PRIZM servo speeds individually

  if(channel==1){channel= 0x28;}
  if(channel==2){channel= 0x29;}
  if(channel==3){channel= 0x2A;}
  if(channel==4){channel= 0x2B;}
  if(channel==5){channel= 0x2C;}
  if(channel==6){channel= 0x2D;}
//	if(channel==7){channel= 0x2E;}
//  if(channel==8){channel= 0x2F;}

  Wire.beginTransmission(6);
  Wire.write(channel);
  Wire.write(servospeed);
  Wire.endTransmission();
  delay(10);

}

void PRIZM::setServoSpeeds (int servospeed1, int servospeed2, int servospeed3, int servospeed4, int servospeed5, int servospeed6){   // function to set all PRIZM servo speeds at once

  Wire.beginTransmission(6);
  Wire.write(0x30);
  Wire.write(servospeed1);
  Wire.write(servospeed2);
  Wire.write(servospeed3);
  Wire.write(servospeed4);
  Wire.write(servospeed5);
  Wire.write(servospeed6);
//	Wire.write(servospeed7);
//	Wire.write(servospeed8);
  Wire.endTransmission();
  delay(10);

}

void PRIZM::setServoPosition (int channel, int servoposition){   //function to set PRIZM servo positions individually

  int xmit = 0;
  if(channel==1 && servoposition != lastPosition_1) {channel= 0x31; xmit = 1; lastPosition_1 = servoposition;}	// gotta see if new position is different than last
  if(channel==2 && servoposition != lastPosition_2) {channel= 0x32; xmit = 1; lastPosition_2 = servoposition;}
  if(channel==3 && servoposition != lastPosition_3) {channel= 0x33; xmit = 1; lastPosition_3 = servoposition;}
  if(channel==4 && servoposition != lastPosition_4) {channel= 0x34; xmit = 1; lastPosition_4 = servoposition;}
  if(channel==5 && servoposition != lastPosition_5) {channel= 0x35; xmit = 1; lastPosition_5 = servoposition;}
  if(channel==6 && servoposition != lastPosition_6) {channel= 0x36; xmit = 1; lastPosition_6 = servoposition;}
//  if(channel==7 && servoposition != lastPosition_7) {channel= 0x37; xmit = 1; lastPosition_7 = servoposition;}
//  if(channel==8 && servoposition != lastPosition_8) {channel= 0x38; xmit = 1; lastPosition_8 = servoposition;}

  if(xmit == 1){									// no need to send if positions have not changed - result frees up I2C bus
  Wire.beginTransmission(6);     	// Even though this is a single position command, sending all channels at once works around minor servo glitching in version 1 PRIZM servo chip firmware.
  Wire.write(0x39);
  Wire.write(lastPosition_1);
  Wire.write(lastPosition_2);
  Wire.write(lastPosition_3);
  Wire.write(lastPosition_4);
  Wire.write(lastPosition_5);
  Wire.write(lastPosition_6);
//	Wire.write(lastPosition_7);
//	Wire.write(lastPosition_8);
  Wire.endTransmission();
  delay(10);
  xmit = 0;
  }

}

void PRIZM::setServoPositions (int servoposition1,int servoposition2,int servoposition3,int servoposition4,int servoposition5,int servoposition6){  // Sets all PRIZM servo positions at once

// if arguement is not valid or missing, use lastposition as position
	if (servoposition1 < 0 || servoposition1 > 180) {servoposition1 = lastPosition_1;}
	if (servoposition2 < 0 || servoposition2 > 180) {servoposition2 = lastPosition_2;}
	if (servoposition3 < 0 || servoposition3 > 180) {servoposition3 = lastPosition_3;}
	if (servoposition4 < 0 || servoposition4 > 180) {servoposition4 = lastPosition_4;}
	if (servoposition5 < 0 || servoposition5 > 180) {servoposition5 = lastPosition_5;}
	if (servoposition6 < 0 || servoposition6 > 180) {servoposition6 = lastPosition_6;}

	lastPosition_1 = servoposition1;
	lastPosition_2 = servoposition2;
	lastPosition_3 = servoposition3;
	lastPosition_4 = servoposition4;
	lastPosition_5 = servoposition5;
	lastPosition_6 = servoposition6;

  Wire.beginTransmission(6);
  Wire.write(0x39);
  Wire.write(servoposition1);
  Wire.write(servoposition2);
  Wire.write(servoposition3);
  Wire.write(servoposition4);
  Wire.write(servoposition5);
  Wire.write(servoposition6);
  Wire.endTransmission();
  delay(10);

}

int PRIZM::readServoPosition (int channel){		// read the servo PRIZM servo positions

  int readServoPosition;    // return value variable

  if(channel==1){channel= 0x3A;}
  if(channel==2){channel= 0x3B;}
  if(channel==3){channel= 0x3C;}
  if(channel==4){channel= 0x3D;}
  if(channel==5){channel= 0x3E;}
  if(channel==6){channel= 0x4F;}
//	if(channel==7){channel= 0x40;}
//	if(channel==8){channel= 0x41;}

  Wire.beginTransmission(6);
  Wire.write(channel);
  Wire.endTransmission();
  delay(10);
  Wire.requestFrom(6, 1);
  readServoPosition = Wire.read();
  delay(10);
  return readServoPosition;

}

// ====================================== DC MOTOR FUNCTIONS ================================

void PRIZM::setMotorPower(int channel, int power)	// set Motor Channel power on PRIZM
{

	int chip_address;
	if(channel==1){channel = 0x40; chip_address = 5;}   // DC channel 1, address = 5; motors 1 and 2
	if(channel==2){channel = 0x41; chip_address = 5;}   // DC channel 2, address = 5
	if(channel==3){channel = 0x40; chip_address = 9;}   // DC channel 1, address = 9; motors 3 and 4
	if(channel==4){channel = 0x41; chip_address = 9;}   // DC channel 2, address = 9

  	Wire.beginTransmission(chip_address);
  	Wire.write(channel);
  	Wire.write(power);
  	Wire.endTransmission();
	delay(10);
}

void PRIZM::setMotorPowers (int power1, int power2, int power3, int power4){     //power only Block Command for all four motor channels

  Wire.beginTransmission(5);		// motors 1 and 2
  Wire.write(0x42);
  Wire.write(power1);
  Wire.write(power2);
  Wire.endTransmission();
  delay(10);
	Wire.beginTransmission(9);		// motors 3 and 4
  Wire.write(0x42);
  Wire.write(power3);
  Wire.write(power4);
  Wire.endTransmission();
  delay(10);

}

void PRIZM::setMotorSpeed (int channel, long Mspeed){      // === set speed of each PRIZM DC motor == requires a 1440 CPR installed encoder to do the PID

  int lobyte;
  int hibyte;
	int chip_address;

  lobyte  = lowByte(Mspeed);
  hibyte  = highByte(Mspeed);

	if(channel==1){channel = 0x43; chip_address = 5;}   // DC channel 1, address = 5; motors 1 and 2
	if(channel==2){channel = 0x44; chip_address = 5;}   // DC channel 2, address = 5
	if(channel==3){channel = 0x43; chip_address = 9;}   // DC channel 1, address = 9; motors 3 and 4
	if(channel==4){channel = 0x44; chip_address = 9;}   // DC channel 2, address = 9

  Wire.beginTransmission(chip_address);
  Wire.write(channel);
  Wire.write(hibyte);
  Wire.write(lobyte);
  Wire.endTransmission();
  delay(10);

}

void PRIZM::setMotorSpeeds (long Mspeed1, long Mspeed2, long Mspeed3, long Mspeed4){      // === BLOCK write to set speeds of PRIZM motors at once == 1440 CPR encoder must be installed to do PID

  int lobyte1;
  int hibyte1;
  int lobyte2;
  int hibyte2;
	int lobyte3;
  int hibyte3;
	int lobyte4;
  int hibyte4;

  lobyte1  = lowByte(Mspeed1);
  hibyte1  = highByte(Mspeed1);
  lobyte2  = lowByte(Mspeed2);
  hibyte2  = highByte(Mspeed2);
	lobyte3  = lowByte(Mspeed3);
  hibyte3  = highByte(Mspeed3);
	lobyte4  = lowByte(Mspeed4);
  hibyte4  = highByte(Mspeed4);

  Wire.beginTransmission(5);			// motor chip channels 1 and 2
  Wire.write(0x45);
  Wire.write(hibyte1);
  Wire.write(lobyte1);
  Wire.write(hibyte2);
  Wire.write(lobyte2);
  Wire.endTransmission();
  delay(10);
	Wire.beginTransmission(9);			// motor chip channels 3 and 4
  Wire.write(0x45);
  Wire.write(hibyte3);
  Wire.write(lobyte3);
  Wire.write(hibyte4);
  Wire.write(lobyte4);
  Wire.endTransmission();
  delay(10);
}

void PRIZM::setMotorTarget (int channel, long Mspeed, long Mtarget){      // === set speed and encoder target of each PRIZM DC motor == requires a 1440 CPR encoder to do the PID

  int lobyte;
  int hibyte;

  lobyte  = lowByte(Mspeed);
  hibyte  = highByte(Mspeed);

  byte four  = (Mtarget);
  byte three = (Mtarget>>8);
  byte two   = (Mtarget>>16);
  byte one   = (Mtarget>>24);

	int chip_address;

  if(channel==1){channel= 0x46; chip_address = 5;}   // DC channel 1
  if(channel==2){channel= 0x47; chip_address = 5;}   // DC channel 2
	if(channel==1){channel= 0x46; chip_address = 9;}   // DC channel 3
  if(channel==2){channel= 0x47; chip_address = 9;}   // DC channel 4

  Wire.beginTransmission(chip_address);
  Wire.write(channel);
  Wire.write(hibyte);
  Wire.write(lobyte);
  Wire.write(one);
  Wire.write(two);
  Wire.write(three);
  Wire.write(four);
  Wire.endTransmission();
  delay(10);

}

void PRIZM::setMotorTargets (long Mspeed1, long Mtarget1, long Mspeed2, long Mtarget2, long Mspeed3, long Mtarget3, long Mspeed4, long Mtarget4){      // === BLOCK WRITE set speed and encoder target of both PRIZM DC motors == requires a 1440 CPR encoder to do the PID

  int lobyte1;
  int hibyte1;
  int lobyte2;
  int hibyte2;
	int lobyte3;
  int hibyte3;
	int lobyte4;
  int hibyte4;

  lobyte1  = lowByte(Mspeed1);
  hibyte1  = highByte(Mspeed1);
  lobyte2  = lowByte(Mspeed2);
  hibyte2  = highByte(Mspeed2);
	lobyte3  = lowByte(Mspeed3);
  hibyte3  = highByte(Mspeed3);
	lobyte4  = lowByte(Mspeed4);
  hibyte4  = highByte(Mspeed4);

  byte four1  = (Mtarget1);
  byte three1 = (Mtarget1>>8);
  byte two1   = (Mtarget1>>16);
  byte one1   = (Mtarget1>>24);

  byte four2  = (Mtarget2);
  byte three2 = (Mtarget2>>8);
  byte two2   = (Mtarget2>>16);
  byte one2   = (Mtarget2>>24);

	byte four3  = (Mtarget3);
  byte three3 = (Mtarget3>>8);
  byte two3   = (Mtarget3>>16);
  byte one3   = (Mtarget3>>24);

	byte four4  = (Mtarget4);
  byte three4 = (Mtarget4>>8);
  byte two4   = (Mtarget4>>16);
  byte one4   = (Mtarget4>>24);

  Wire.beginTransmission(5);
  Wire.write(0x48);
  Wire.write(hibyte1);
  Wire.write(lobyte1);
  Wire.write(one1);
  Wire.write(two1);
  Wire.write(three1);
  Wire.write(four1);
  Wire.write(hibyte2);
  Wire.write(lobyte2);
  Wire.write(one2);
  Wire.write(two2);
  Wire.write(three2);
  Wire.write(four2);
  Wire.endTransmission();
  delay(10);

	Wire.beginTransmission(9);
  Wire.write(0x48);
  Wire.write(hibyte3);
  Wire.write(lobyte3);
  Wire.write(one3);
  Wire.write(two3);
  Wire.write(three3);
  Wire.write(four3);
  Wire.write(hibyte4);
  Wire.write(lobyte4);
  Wire.write(one4);
  Wire.write(two4);
  Wire.write(three4);
  Wire.write(four4);
  Wire.endTransmission();
  delay(10);

}

void PRIZM::setMotorDegree (int channel, long Mspeed, long Mdegrees){      // === set speed and encoder target of each PRIZM DC motor in DEGREES  == requires a 1440 CPR encoder to do the PID

  int lobyte;
  int hibyte;

  lobyte  = lowByte(Mspeed);
  hibyte  = highByte(Mspeed);

  byte four  = (Mdegrees);
  byte three = (Mdegrees>>8);
  byte two   = (Mdegrees>>16);
  byte one   = (Mdegrees>>24);

	int chip_address;

  if(channel==1){channel= 0x58; chip_address = 5;}
  if(channel==2){channel= 0x59; chip_address = 5;}
	if(channel==3){channel= 0x58; chip_address = 9;}
  if(channel==4){channel= 0x59; chip_address = 9;}

  Wire.beginTransmission(chip_address);
  Wire.write(channel);
  Wire.write(hibyte);
  Wire.write(lobyte);
  Wire.write(one);
  Wire.write(two);
  Wire.write(three);
  Wire.write(four);
  Wire.endTransmission();
  delay(10);

}

void PRIZM::setMotorDegrees (long Mspeed1, long Mdegrees1, long Mspeed2, long Mdegrees2, long Mspeed3, long Mdegrees3, long Mspeed4, long Mdegrees4){      // === BLOCK WRITE set speed and encoder target in DEGREES for PRIZM DC motors == requires a 1440 CPR encoder to do the PID

  int lobyte1;
  int hibyte1;
  int lobyte2;
  int hibyte2;
	int lobyte3;
  int hibyte3;
	int lobyte4;
  int hibyte4;

  lobyte1  = lowByte(Mspeed1);
  hibyte1  = highByte(Mspeed1);
  lobyte2  = lowByte(Mspeed2);
  hibyte2  = highByte(Mspeed2);
	lobyte3  = lowByte(Mspeed3);
  hibyte3  = highByte(Mspeed3);
	lobyte4  = lowByte(Mspeed4);
  hibyte4  = highByte(Mspeed4);

  byte four1  = (Mdegrees1);
  byte three1 = (Mdegrees1>>8);
  byte two1   = (Mdegrees1>>16);
  byte one1   = (Mdegrees1>>24);

  byte four2  = (Mdegrees2);
  byte three2 = (Mdegrees2>>8);
  byte two2   = (Mdegrees2>>16);
  byte one2   = (Mdegrees2>>24);

	byte four3  = (Mdegrees3);
  byte three3 = (Mdegrees3>>8);
  byte two3   = (Mdegrees3>>16);
  byte one3   = (Mdegrees3>>24);

	byte four4  = (Mdegrees4);
  byte three4 = (Mdegrees4>>8);
  byte two4   = (Mdegrees4>>16);
  byte one4   = (Mdegrees4>>24);

  Wire.beginTransmission(5);
  Wire.write(0x5A);
  Wire.write(hibyte1);
  Wire.write(lobyte1);
  Wire.write(one1);
  Wire.write(two1);
  Wire.write(three1);
  Wire.write(four1);
  Wire.write(hibyte2);
  Wire.write(lobyte2);
  Wire.write(one2);
  Wire.write(two2);
  Wire.write(three2);
  Wire.write(four2);
  Wire.endTransmission();
  delay(10);

	Wire.beginTransmission(9);
  Wire.write(0x5A);
  Wire.write(hibyte3);
  Wire.write(lobyte3);
  Wire.write(one3);
  Wire.write(two3);
  Wire.write(three3);
  Wire.write(four3);
  Wire.write(hibyte4);
  Wire.write(lobyte4);
  Wire.write(one4);
  Wire.write(two4);
  Wire.write(three4);
  Wire.write(four4);
  Wire.endTransmission();
  delay(10);

}

int PRIZM::readMotorBusy (int channel){			// ================== Read Busy Status of PRIZM DC motor channel =======================

  int byte1;
  int MotorStatus;

	int chip_address;

  if(channel==1){channel= 0x4F; chip_address = 5;}       // channel 1 busy flag
  if(channel==2){channel= 0x50; chip_address = 5;}       // channel 2 busy flag
	if(channel==3){channel= 0x4F; chip_address = 9;}       // channel 3 busy flag
  if(channel==4){channel= 0x50; chip_address = 9;}       // channel 4 busy flag

  Wire.beginTransmission(chip_address);
  Wire.write(channel);
  Wire.endTransmission();
  delay(10);

  Wire.requestFrom(chip_address, 1);
  byte1 = Wire.read();
  MotorStatus=byte1;
  delay(10);

  return MotorStatus;

}

void PRIZM::setMotorInvert (int channel, int invert){						// ======================== Set the PRIZM DC Motor Direction invert status ====================

	int chip_address;

	if(channel==1){channel= 0x51; chip_address = 5;}       // channel 1
  if(channel==2){channel= 0x52; chip_address = 5;}       // channel 2
	if(channel==3){channel= 0x51; chip_address = 9;}       // channel 3
  if(channel==4){channel= 0x52; chip_address = 9;}       // channel 4

  Wire.beginTransmission(chip_address);
  Wire.write(channel);
  Wire.write(invert);
  Wire.endTransmission();
  delay(10);

}

// ================================================== ENCODERS ==========================================

long PRIZM::readEncoderCount (int channel){   // ========= READ PRIZM ENCODER DATA COUNTS =====

  unsigned long eCount;    // return value variable. We have to pass this an unsigned into Arduino.

  byte byte1;
  byte byte2;
  byte byte3;
  byte byte4;

	int chip_address;

  if(channel==1){channel= 0x49; chip_address = 5;}       // channel 1 encoder FOR count value
  if(channel==2){channel= 0x4A; chip_address = 5;}       // channel 2 encoder FOR count value
	if(channel==3){channel= 0x49; chip_address = 9;}       // channel 3 encoder FOR count value
  if(channel==4){channel= 0x4A; chip_address = 9;}       // channel 4 encoder FOR count value

  Wire.beginTransmission(chip_address);
  Wire.write(channel);
  Wire.endTransmission();
  delay(10);

  Wire.requestFrom(chip_address, 4);
  byte1 = Wire.read();
  byte2 = Wire.read();
  byte3 = Wire.read();
  byte4 = Wire.read();

  eCount = byte1;
  eCount = (eCount*256)+byte2;
  eCount = (eCount*256)+byte3;
  eCount = (eCount*256)+byte4;
  delay(10);
  return eCount;

}

long PRIZM::readEncoderDegrees (int channel){   // ==== READ PRIZM ENCODER DATA DEGREES =======

  unsigned long eCount;    // return value variable. We have to pass this an unsigned into Arduino.

  byte byte1;
  byte byte2;
  byte byte3;
  byte byte4;

	int chip_address;

  if(channel==1){channel= 0x5B; chip_address = 5;}       // channel 1 encoder FOR degrees
  if(channel==2){channel= 0x5C; chip_address = 5;}       // channel 2 encoder FOR degrees
	if(channel==3){channel= 0x5B; chip_address = 9;}       // channel 3 encoder FOR degrees
  if(channel==4){channel= 0x5C; chip_address = 9;}       // channel 4 encoder FOR degrees

  Wire.beginTransmission(chip_address);
  Wire.write(channel);
  Wire.endTransmission();
  delay(10);

  Wire.requestFrom(chip_address, 4);
  byte1 = Wire.read();
  byte2 = Wire.read();
  byte3 = Wire.read();
  byte4 = Wire.read();

  eCount = byte1;
  eCount = (eCount*256)+byte2;
  eCount = (eCount*256)+byte3;
  eCount = (eCount*256)+byte4;
  delay(10);
  return eCount;

}

void PRIZM::resetEncoder (int channel){    // =========== RESET PRIZM ENCODERS 1 or 2 ==========

	int chip_address;
	if(channel==1){channel= 0x4C; chip_address = 5;}       // channel 1 encoder reset command
  if(channel==2){channel= 0x4D; chip_address = 5;}       // channel 2 encoder reset command
	if(channel==3){channel= 0x4C; chip_address = 9;}       // channel 3 encoder reset command
  if(channel==4){channel= 0x4D; chip_address = 9;}       // channel 4 encoder reset command

  Wire.beginTransmission(chip_address);
  Wire.write(channel);
  Wire.endTransmission();
  delay(10);

}

void PRIZM::resetEncoders(){					// =========== Reset all encoders at once ========

  Wire.beginTransmission(5);					// encoders 1 and 2
  Wire.write(0x4E);
  Wire.endTransmission();
  delay(10);

	Wire.beginTransmission(9);					// encoders 3 and 4
  Wire.write(0x4E);
  Wire.endTransmission();
  delay(10);

}

// ========================================== SOUND FUNCTIONS ===========================

void PRIZM::soundEnable(){
	ledcSetup(1, 0, 8);					// setup channel, freq, resolution
  ledcAttachPin(2, 1);				// attach speaker pin, channel
}

void PRIZM::playTone(double freq, double duration){		// play a frequency
	if(freq == lastToneFreq && freq != 0)
		return;

	ledcWriteTone(1, freq);					// esp32 RMT channel, frequency
	lastToneFreq = freq;
	delay(duration);
}

void PRIZM::playNote(mnote_t note, int octave, double duration){		// play a note on an octave

	const uint16_t noteFrequencyBase[12] = {
	    //   C        C#       D        Eb       E        F       F#        G       G#        A       Bb        B
	        4186,    4435,    4699,    4978,    5274,    5588,    5920,    6272,    6645,    7040,    7459,    7902
	    };

	    if(octave > 8 || note >= note_MAX){
	       return;
	    }
	    double noteFreq =  (double)noteFrequencyBase[note] / (double)(1 << (8-octave));

			if(noteFreq == lastNoteFreq)
				return;

	    ledcWriteTone(1, noteFreq);
			lastNoteFreq = noteFreq;
			delay(duration);

}

void PRIZM::playOff(double duration){
			ledcWriteTone(1, 0);		// sound output bOffset
			lastToneFreq	 = -1;
			lastNoteFreq	 = -1;
			delay(duration);
}



//=========THE END =============================================================================================================================================
