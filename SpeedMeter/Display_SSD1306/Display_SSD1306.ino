/*
 Name:		Display_SSD1306.ino
 Created:	6/15/2021 9:05:12 PM
 Author:	gebruiker
*/
#include <Wire.h>
#include "ACROBOTIC_SSD1306.h"

#define display_Address               0x3C
#define display_Command_Mode          0x80
#define display_Data_Mode             0x40
#define Display_Off_Cmd				 0xAE
#define Display_On_Cmd					0xAF
#define Display_Max_Y                 63

//variabelen uit private van ACROBOTIC_SSD1306.h
const uint8_t* m_font;      // Current font.
uint8_t m_font_offset = 2;  // Font bytes for meta data.
uint8_t m_font_width;       // Font witdth.
uint8_t m_col;              // Cursor column.
uint8_t m_row;              // Cursor row (RAM). 
bool m_inverse = false;       // Inverse text.


unsigned long slowtime;
byte switchstatus=15;

void setup(){

	Serial.begin(9600);


	Wire.begin();
	oled.init();                      // Initialze SSD1306 OLED display
	oled.clearDisplay();              // Clear screen
	oled.setTextXY(0, 0);              // Set cursor position, start of line 0
	oled.putString("Wisselmotor");
	oled.setTextXY(1, 0);              // Set cursor position, start of line 1
	oled.putString("Designs");
	oled.setTextXY(2, 0);              // Set cursor position, start of line 2
	oled.putString("Hoofddorp,");
	oled.setTextXY(2, 10);             // Set cursor position, line 2 10th character
	oled.putString("NH");


	//pin 8~11 4 xschakelaar input
	DDRB &= ~(15 << 0);
	PORTB |= (15 << 0); //pull-up to pins 8~11
}

void loop(){
	//check voor inschakelen interupts

	if (micros() - slowtime > 1000) {
		slowtime = micros();
		SW_exe();
		//SD_exe(); //sends to  simpledyno
	}
}
void SW_exe() {
	byte ss; byte changed;
	ss = PINB;
	ss = ss << 4;
	ss = ss >> 4; //clear bits 4,5,6,7
	changed = switchstatus ^ ss;
	if (changed > 0) {
		for (byte i = 0; i < 4; i++) {
			if (changed & (1 << i)) {
				if (ss & (1 << i)) {
					SW_off(i);

				}
				else {
					SW_on(i);

				}
			}
		}
	}
	switchstatus = ss;
}
void SW_on(byte sw) {
	Serial.print(F("Switch-on: ")); Serial.println(sw);

	switch (sw) {
	case 0:
		//I2C_send(Display_Off_Cmd);
		break;
	case 1:
		//I2C_send(Display_On_Cmd);
		break;
	case 2:
		//iets in het display?
		//DP_putString("Wisselmotor");
		break;
	case 3:
		//oled.init();
		//oled.clearDisplay();
		//DP_clear();

		break;
	}
}
void SW_off(byte sw) {

}

void I2C_send(byte command) {
	Wire.beginTransmission(display_Address);    // begin I2C communication adress0x3C
	Wire.write(display_Command_Mode);           // Set OLED Command mode 0x80
	Wire.write(command);
	Wire.endTransmission();                       // End I2C communication
}

void I2C_sendData(unsigned char Data) {
	Wire.beginTransmission(display_Address); // begin I2C transmission
	Wire.write(display_Data_Mode);            // data mode
	Wire.write(m_inverse ? ~Data : Data);
	Wire.endTransmission();                    // stop I2C transmission
}