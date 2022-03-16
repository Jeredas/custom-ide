/* 	Header file for TETRIX (T2) PRIZM robotics controller Arduino Library
   	Written by: Paul W. Uttley
		10/21/2021
		Version 1.0
		For: ESP32 WROOM - 32D Module
    ============ This library is a PROTOTYPE in Progress ======================
    Some functions are not yet fully implemented
*/

#ifndef PRIZM_ESP32_H
#define PRIZM_ESP32_H

//#include "notes.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp32-hal.h"

#define	portA	1			// GPIO ports
#define portB	2
#define portC	3
#define	portD 4
#define portE 5
#define portF	6

#define	D1	1				// GPIO port data pins
#define D2	2

typedef enum {
    note_C, note_Cs, note_D, note_Eb, note_E, note_F, note_Fs, note_G, note_Gs, note_A, note_Bb, note_B, note_MAX
} mnote_t;


typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;

//Modify the definition to expand the number of leds
//Supports a maximum of 1100 leds
#define NR_OF_LEDS   256
#define NR_OF_ALL_BITS 24*NR_OF_LEDS

enum LED_TYPE
{					  //R  G  B
	TYPE_RGB = 0x06,  //00 01 10
	TYPE_RBG = 0x09,  //00 10 01
	TYPE_GRB = 0x12,  //01 00 10
	TYPE_GBR = 0x21,  //10 00 01
	TYPE_BRG = 0x18,  //01 10 00
	TYPE_BGR = 0x24	  //10 01 00
};

class PRIZM
{
protected:

		u16 ledCounts =3;
		u8 pin =23;
		u8 br =255;
		u8 rmt_chn =0;

		u8 rOffset;
		u8 gOffset;
		u8 bOffset;

		float realTick;
		rmt_reserve_memsize_t rmt_mem;
		rmt_data_t led_data[NR_OF_ALL_BITS];
		rmt_obj_t* rmt_send = NULL;

		int lastPosition_1 = 90;		// these hold the current 'last' positions of each servo channel
		int lastPosition_2 = 90;
		int lastPosition_3 = 90;
		int lastPosition_4 = 90;
		int lastPosition_5 = 90;
		int lastPosition_6 = 90;

		double 	 lastToneFreq	 = -1;
		double	 lastNoteFreq	 = -1;

	public:

		PRIZM(u16 n = 3, u8 pin_gpio = 23, u8 chn = 0, LED_TYPE t = TYPE_GRB);

		bool beginLed();
		void setLedCount(u16 n);
		void setLedType(LED_TYPE t);
		void setLedBrightness(u8 brightness =50);

		esp_err_t set_pixel(int index, u8 r, u8 g, u8 b);

		esp_err_t setLedColorData(int index, u32 rgb);
		esp_err_t setLedColorData(int index, u8 r, u8 g, u8 b);

		esp_err_t setLedColorsData(u32 rgb);
		esp_err_t setLedColorsData(u8 r, u8 g, u8 b);

		esp_err_t setLedColor(int index, u32 rgb);
		esp_err_t setLedColor(int index, u8 r, u8 g, u8 b);

		esp_err_t setLedColors(u32 rgb);
		esp_err_t setLedColors(u8 r, u8 g, u8 b);

		esp_err_t showLedData();

		uint32_t colorWheel(byte pos);
		uint32_t hsv2rgb(uint32_t h, uint32_t s, uint32_t v);

		void setMotorPower(int channel, int power);
		void setMotorPowers(int power1, int power2, int power3, int power4);
		void setMotorSpeed (int channel, long Mspeed);
		void setMotorSpeeds (long Mspeed1, long Mspeed2, long Mspeed3, long Mspeed4);
		void setMotorTarget (int channel, long Mspeed, long Mtarget);
		void setMotorTargets (long Mspeed1, long Mtarget1, long Mspeed2, long Mtarget2, long Mspeed3, long Mtarget3, long Mspeed4, long Mtarget4);
		void setMotorDegree (int channel, long Mspeed, long Mdegrees);
		void setMotorDegrees (long Mspeed1, long Mdegrees1, long Mspeed2, long Mdegrees2, long Mspeed3, long Mdegrees3, long Mspeed4, long Mdegrees4);
		void setMotorInvert (int channel, int invert);
		int readMotorBusy (int channel);

		long readEncoderCount (int channel);
		long readEncoderDegrees (int channel);
		void resetEncoder (int channel);
		void resetEncoders (void);

		void setMotorSpeedPID  (int P, int I, int D);
		void setMotorTargetPID (int P, int I, int D);

		int readLineSensor(int pin);
		int readSonicSensorCM(int port);
		int readSonicSensorIN(int port);

		int readBatteryVoltage(void);

		void PrizmBegin(void);
		void PrizmEnd(void);
		int readDCFirmware(int chip);
		int readSVOFirmware(void);

		void setGreenLED(int state);

		int  readGreenButton(void);
		int  readBlackButton(void);

		void setServoSpeed(int channel, int servospeed);
		void setServoSpeeds(int servospeed1, int servospeed2, int servospeed3, int servospeed4, int servospeed5, int servospeed6);
		void setServoPosition (int channel, int servoposition);
		void setServoPositions (int servoposition1 =-1, int servoposition2 =-1, int servoposition3 =-1, int servoposition4 =-1, int servoposition5 =-1, int servoposition6 =-1);
		int readServoPosition (int channel);

		void soundEnable(void);
		void playTone(double freq = 0, double duration = 0);
		void playNote(mnote_t note, int octave, double duration = 0);
		void playOff(double duration = 0);

	private:
};

#endif
