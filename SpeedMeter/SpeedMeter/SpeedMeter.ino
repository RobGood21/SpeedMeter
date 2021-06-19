/*
 Name:		SpeedMeter.ino
 Created:	6/10/2021 11:49:37 AM
 Author:	RobAntonisse

 Use of library: https://github.com/acrobotic/Ai_Ardulib_SSD1306



*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Display
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display(128, 64, &Wire, 4);



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

int SW_time; //counter, timer voor zeer langzame processen (1xper 20ms, 50hz)

//variabelen
byte holes1 = 20; //20; 5=kwart rotatie rpm x4 aantal pulsen per rotatie, gaatjes in de IR disc
byte holes2 = 1;

byte holecount1; byte holecount2;

unsigned long antidender1;
unsigned long antidender2;
int switchstatus=15;

int RPM1;
int RPM2;
byte DP_type=0; //welk scherm wordt getoond, wisseld met knop 4



//temps
int count=9000;

void setup() {
	Serial.begin(9600);

	//Voor Display

	// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
		Serial.println(F("SSD1306 allocation failed"));
		for (;;); // Don't proceed, loop forever
	}
	display.clearDisplay();
	display.display();
	// display.drawPixel(x, y, color)


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



}

ISR(INT0_vect) {
	cli();	
	if (micros() - antidender1 > 100) {
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

	if (millis() - slowtime > 1) {
		slowtime = millis();
		// SD_exe(); //sends to  simpledyno
		calc();

		SW_time++;
		if (SW_time > 50) { //in combi met slowtime zorgen dit ongeveer een 20hz (50ms)
			SW_time = 0;
			SW_exe();
			DP_exe();
		}
	}
}





void DP_exe() {
	//Ververst het display op zichtbare snelheid dus 50x per seconde (20ms)

	//Serial.print(F("RPM1= ")); Serial.print(RPM1); Serial.print(F("   RPM2= ")); Serial.println(RPM2);
display.clearDisplay();
	display.setTextColor(WHITE);
	display.setTextSize(3);
	display.setCursor(3, 3);
	

	switch (DP_type) {
	case 0:
display.print(RPM2);

		break;
	case 1:
		display.print("Scherm 1 ");
		break;
	case 2:
		display.print("display 2" );
		break;
	case 3:
		display.print("Toon 3");
		break;
	case 4:
		display.print("program?");
		break;
	}

	display.display();


	//I2C_send(0xAF); //display on

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
		//wisseld scherm
		DP_type++;
		if (DP_type > 4)DP_type = 0;
		break;
	case 1:

		break;
	case 2:

		break;
	case 3:

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

void calc() {
	//maakt alle berekeningen
RPM1 = 60000000 / SD_times[1];
RPM2 = 60000000 / SD_times[3];

}

void I2C_send(byte command) {
	Wire.beginTransmission(0x3C);    // begin I2C communication adress0x3C
	Wire.write(0x80);           // Set OLED Command mode 0x80
	Wire.write(command);
	Wire.endTransmission();                       // End I2C communication
}

