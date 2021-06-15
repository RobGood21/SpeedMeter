/*
 Name:		Display_Adafruit.ino
 Created:	6/15/2021 9:39:04 PM
 Author:	gebruiker
*/


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

byte switchstatus;
unsigned long slowtime;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);

	//pin 8~11 4 xschakelaar input
	DDRB &= ~(15 << 0);
	PORTB |= (15 << 0); //pull-up to pins 8~11

	// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
		Serial.println(F("SSD1306 allocation failed"));
		for (;;); // Don't proceed, loop forever
	}

	display.clearDisplay();
	// display.drawPixel(x, y, color)

}

// the loop function runs over and over again until power down or reset
void loop() {

	if (micros() - slowtime > 1000) {
		slowtime = micros();
		SW_exe();

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

		//Display_Off_Cmd				  0xAE
		//Display_On_Cmd				0xAF
		I2C_send(0xAE);
		break;
	case 1:
		I2C_send(0xAF);


		display.setTextColor(WHITE);
		display.setTextSize(1);
		display.setCursor(10, 10);
		display.print("Hallo");
		display.display();

		break;
	case 2:
		//display.clearDisplay();
		display.drawRect(2, 2, 125, 30, WHITE);
		display.display();


		break;
	case 3:
		display.clearDisplay();
		display.display();
		break;
	}
}
void SW_off(byte sw) {

}

void I2C_send(byte command) {
	Wire.beginTransmission(SCREEN_ADDRESS);    // begin I2C communication adress0x3C
	Wire.write(0x80);           // Set OLED Command mode 0x80
	Wire.write(command);
	Wire.endTransmission();                       // End I2C communication
}