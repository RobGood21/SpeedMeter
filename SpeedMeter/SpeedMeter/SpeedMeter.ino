/*
 Name:		SpeedMeter.ino
 Created:	6/10/2021 11:49:37 AM
 Author:	RobAntonisse

 Use of library: https://github.com/acrobotic/Ai_Ardulib_SSD1306



*/

#include <Wire.h>
//#include <ACROBOTIC_SSD1306.h>

#define display_Address               0x3C
#define display_Command_Mode          0x80
#define display_Data_Mode             0x40
#define Display_Off_Cmd       0xAE
#define Display_On_Cmd        0xAF


//voor SimpleDyno


//message string" micros(),temptime1,time1,temptime2.time2,0,0,0,0,0,0"
//De 6 nullen zijn de 6 optionele Analog reads voor: voltage, stroom, temeratuur1 en temperatuur2. 5 en 6 zijn not in use
//Message 1000Hz (1x per 1 ms)

String toSend;
unsigned long SD_times[4];
unsigned int SD_reads[6];
//RPM1 time =sd_times[0]
//RPM1 interval =sd_times[1]
//RPM2 time =sd_times[2]
//RPM2 interval =sd_times[3]

unsigned long oldtime1; //voor berekening interval tussen twee pulsen
unsigned long oldtime2;
unsigned long slowtime;
byte ledcount[2];

//variabelen
byte holes1 = 20; //20; 5=kwart rotatie rpm x4 aantal pulsen per rotatie, gaatjes in de IR disc
byte holes2 = 1;

byte holecount1; byte holecount2;

unsigned long antidender1;
unsigned long antidender2;
int switchstatus=15;

//variabelen uit private van ACROBOTIC_SSD1306.h
const uint8_t* m_font;      // Current font.
uint8_t m_font_offset = 2;  // Font bytes for meta data.
uint8_t m_font_width;       // Font witdth.
uint8_t m_col;              // Cursor column.
uint8_t m_row;              // Cursor row (RAM). 
bool m_inverse = false;       // Inverse text.




void setup() {
	Serial.begin(9600);
	GPIOR0 = 0;
	//Poorten en pins definieren
	DDRD |= (1 << 7); //Pin 7 Groene led, output
	DDRD |= (1 << 6); //Pin 6 Rode led
	//pin 8~11 4 xschakelaar input
	DDRB &= ~(15 << 0);
	PORTB |= (15 << 0); //pull-up to pins 8~11


//define interrupt
	//Interupt on rising edge PIN2, INT0
	//01 = any change 10=falling edge 11=rising edge
	EICRA |= (1 << 0); //set bit0 of register ISC00 (rising edge both)
	EICRA |= (1 << 1); //set bit1 of register ISC01

	EICRA |= (1 << 2); //set bit2 of register ISC10 (rising edge, both)
	EICRA |= (1 << 3); //set bit3 of register ISC11

	EIMSK |= (1 << 0); //Interupt INT0 enabled
	EIMSK |= (1 << 1); //Interupt INT1 enabled

//display
	Wire.begin();
	//oled.init();                      // Initialze SSD1306 OLED display
	//oled.clearDisplay();
}

ISR(INT0_vect) {
	cli();
	if (micros() - antidender1 > 1000) {
		antidender1 = micros();
		holecount1++;
		if (holecount1 >= holes1) {
			holecount1 = 0;
			PIND |= (1 << 6);

			SD_times[0] = micros();
			SD_times[1] = SD_times[0] - oldtime1; //bereken interval
			oldtime1 = SD_times[0];
		}
	}
	sei();
}

ISR(INT1_vect) {
	cli();

	if (micros() - antidender2 > 1000) {
		antidender2 = micros();
		holecount2++;

		if (holecount2 >= holes2) {
			holecount2 = 0;
			PIND |= (1 << 7); //groene led aan
			SD_times[2] = micros();
			SD_times[3] = SD_times[2] - oldtime2; //bereken interval
			oldtime2 = SD_times[2];
		}
	}
	sei();
}


void loop() {
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
		I2C_send(Display_Off_Cmd);
		break;
	case 1:
		I2C_send(Display_On_Cmd);
		break;
	case 2:
		break;
	case 3:
		//oled.init();
		//oled.clearDisplay();
		DP_clear();

		break;
	}
}
void SW_off(byte sw) {

}
void SD_exe() {
	//berekend en sends message over serial port to SympleDyno

	//SD_times[2] = SD_times[0];
	//SD_times[3] = SD_times[1];


	toSend = ""; //init
	toSend += micros();
	toSend += ",";
	for (byte i = 0; i < 4; i++) {
		toSend += SD_times[i]; toSend += ",";
	}
	//SimpleDyno += " >>> "; //leesbaarheid in debugging
	for (byte i = 0; i < 6; i++) {
		toSend += SD_reads[i];
		if (i < 5)toSend += ",";
	}
	Serial.println(toSend);
	Serial.flush();
}

void DP_clear() {
//#define Display_Off_Cmd       0xAE
//#define Display_On_Cmd        0xAF

		unsigned char i, j;
		I2C_send(Display_Off_Cmd);     //display off
		for (j = 0; j < 8; j++)
		{
			DP_setTextXY(j, 0);
			{
				for (i = 0; i < 16; i++)  //clear all columns
				{
					DP_putChar(' ');
				}
			}
		}
		I2C_send(Display_On_Cmd);     //display on
		DP_setTextXY(0, 0);	
}

void DP_setTextXY(unsigned char row, unsigned char col){
	I2C_send(0xB0 + row);                          //set page address
	I2C_send(0x00 + (m_font_width*col & 0x0F));    //set column lower addr
	I2C_send(0x10 + ((m_font_width*col >> 4) & 0x0F)); //set column higher addr
}

void DP_setFont(const uint8_t* font, bool inverse)
{
	m_font = font;
	m_inverse = inverse;
	m_font_width = pgm_read_byte(&m_font[0]);
}

bool DP_putChar(unsigned char ch){
	if (!m_font) return 0;
	//Ignore non-printable ASCII characters. This can be modified for
	//multilingual font. 
	if (ch < 32 || ch > 127)
	{
		ch = ' ';
	}
	for (unsigned char i = 0; i < m_font_width; i++)
	{
		// Font array starts at 0, ASCII starts at 32
		I2C_sendData(pgm_read_byte(&m_font[(ch - 32)*m_font_width + m_font_offset + i]));
	}
	return 1;
}


void I2C_send(byte command) {
	Wire.beginTransmission(display_Address);    // begin I2C communication adress0x3C
	Wire.write(display_Command_Mode);           // Set OLED Command mode 0x80
	Wire.write(command);
	Wire.endTransmission();                       // End I2C communication
}

void I2C_sendData(unsigned char Data){
	Wire.beginTransmission(display_Address); // begin I2C transmission
	Wire.write(display_Data_Mode);            // data mode
	Wire.write(m_inverse ? ~Data : Data);
	Wire.endTransmission();                    // stop I2C transmission
}


