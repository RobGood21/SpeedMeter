/*
 Name:		SpeedMeter.ino
 Created:	6/10/2021 11:49:37 AM
 Author:	RobAntonisse

 Use of library: https://github.com/acrobotic/Ai_Ardulib_SSD1306



*/

#include <Wire.h>
#include <ACROBOTIC_SSD1306.h>

//voor SimpleDyno


//message string" micros(),temptime1,time1,temptime2.time2,0,0,0,0,0,0"
//De 6 nullen zijn de 6 optionele Analog reads voor: voltage, stroom, temeratuur1 en temperatuur2. 5 en 6 zijn not in use
//Message 1000Hz (1x per 1 ms)

String SimpleDyno;
unsigned long SD_times[4];
unsigned int SD_reads[6];
//RPM1 time =sd_times[0]
//RPM1 interval =sd_times[1]
//RPM2 time =sd_times[2]
//RPM2 interval =sd_times[3]

unsigned long oldtime[2]; //voor berekening interval tussen twee pulsen
unsigned long slowtime;


//variabelen
byte holes = 20; //aantal pulsen per rotatie, gaatjes in de IR disc

byte holecount;

unsigned long antidender;


void setup() {
	Serial.begin(9600);

	//Poorten en pins definieren
	DDRB |= (1 << 0); //Pin 8 as output	

//define interrupt
	//Interupt on rising edge PIN2, INT0
	EICRA |= (1 << 0); //set bit0 of register ISC00
	//EICRA |= (1 << 1); //set bit1 of register ISC01
	EIMSK |= (1 << 0); //Interupt INT0 enabled

}

ISR(INT0_vect) {
	cli();
	if (micros() - antidender > 100) {
		antidender = micros();
		holecount++;
		if (holecount >= holes) {
			holecount = 0;
			PINB |= (1 << 0);

			//voor Simpledyno sensor 1 voor RPM1
			GPIOR0 ^=(1 << 0);

			if (GPIOR0 & (1 << 0)) {
				SD_times[0] = micros();
				SD_times[1] = SD_times[0] - oldtime[0]; //bereken interval
				oldtime[0] = SD_times[0];
			}
		}
	}
	sei();
}

void loop()
{

	if (millis() - slowtime > 2000) {

		slowtime = millis();
		SD_exe();

	}

}

void SD_exe() {
	//berekend en sends message over serial port to SympleDyno
	SimpleDyno = "";
	SimpleDyno += micros();
	SimpleDyno += ",";
	for (byte i=0; i < 4; i++) {
		SimpleDyno += SD_times[i]; SimpleDyno += ",";
	}
	//SimpleDyno += " >>> "; //leesbaarheid in debugging
	for (byte i=0; i < 6; i++) {
		SimpleDyno += SD_reads[i]; SimpleDyno += ",";
	}
	Serial.println(SimpleDyno);
	Serial.flush();
}