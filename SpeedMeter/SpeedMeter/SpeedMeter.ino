/*
 Name:		SpeedMeter.ino
 Created:	6/10/2021 11:49:37 AM
 Author:	RobAntonisse

 Use of library: https://github.com/acrobotic/Ai_Ardulib_SSD1306

*/


#include <EEPROM.h>
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


//constanten
#define DPtypes 1 //aantal verschillende DisPlay_type (begint bij 0 0=1 1=2 enz.)


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
int switchstatus = 15;

unsigned int RPM1 = 0;
unsigned int RPM2 = 0;
unsigned int rpm1old = 0;
unsigned int rpm2old = 0;
unsigned int countstop = 0;

byte dr1; //diameter rol 1 in mm
byte dr2; //diameter rol2
byte dw1;//diameter wiel voertuig op RM1 
byte dw2; //diameter wiel voertuig op RM2




byte DP_type = 0; //EEPROM #101 welk scherm wordt getoond, wisseld met knop 4
byte MEM_reg; //EEPROM #100


//temps
int countsign=0;
unsigned long counttekens; //gebruikt om tekens te kunnen opzoeken, in loop opnemen

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
	//factory reset
	if (~PINB & (1 << 0) && ~PINB & (1 << 3))factory();

	//initialisaties
	MEM_read();
}



ISR(INT0_vect) {
	cli();
	countstop = 0;
	if (micros() - antidender1 > 1000 / holes1) {
		antidender1 = micros();
		holecount1++;
		if (holecount1 >= holes1) {
			holecount1 = 0;
			PIND |= (1 << 6);

			SD_times[0] = micros();
			SD_times[1] = SD_times[0] - oldtime1; //bereken interval
			oldtime1 = SD_times[0];
			calc(false);
		}
	}
	sei();
}



ISR(INT1_vect) {
	cli();
	countstop = 0;
	if (micros() - antidender2 > 5000 / holes2) {
		antidender2 = micros();
		holecount2++;

		if (holecount2 >= holes2) {
			holecount2 = 0;
			PIND |= (1 << 7); //groene led aan
			SD_times[2] = micros();
			SD_times[3] = SD_times[2] - oldtime2; //bereken interval
			oldtime2 = SD_times[2];
			calc(true);
		}
	}
	sei();
}


void loop() {

	if (millis() - slowtime > 20) {
		slowtime = millis();
		countstop++;
		if (countstop > 5) {
			RPM1 = 0;
			RPM2 = 0;
			PORTD &= ~(B11000000 << 0); //leds uit			
		}
		SD_exe(); //sends to  simpledyno	
		SW_exe();
		//tekens(); //gebruiken om speciaal teken op te zoeken
		DP_exe();
	}
}
void factory() {
	for (int i = 0; i < EEPROM.length(); i++) {
		EEPROM.update(i, 0xFF);
	}
	display.clearDisplay();
	display.setTextColor(WHITE);
	display.setCursor(10, 10);
	display.setTextSize(2);
	display.print(F("Factory"));
	display.display();
	delay(1000);
}

void MEM_read() {
	//Leest EEPROM na power up
	MEM_reg = EEPROM.read(100);
	DP_type = EEPROM.read(101);
	if (DP_type > DPtypes)DP_type = 0;
	//roller en wieldiameters
	dr1 = EEPROM.read(110);
	dr2 = EEPROM.read(111);
	dw1 = EEPROM.read(112);
	dw2 = EEPROM.read(113);
	if (dr1 == 0xFF)dr1 = 20;
	if (dr2 = 0xFF)dr2 = 20;
	if (dw1 = 0xFF)dw1 = dr1;
	if (dw2 = 0xFF)dw2 = dr2;

}
void MEM_write() {
	//updates EEPROM, vaak.... dus als rare fouten na enkele jaren denkbaar EEPROM defect van de arduino
	EEPROM.update(100, MEM_reg);
	EEPROM.update(101, DP_type);
}

void DP_exe() { //called from loop()	
	//Ververst het display op zichtbare snelheid dus 50x per seconde (20ms) called from loop()
	//Serial.print(F("RPM1= ")); Serial.print(RPM1); Serial.print(F("   RPM2= ")); Serial.println(RPM2);

	display.clearDisplay();
	switch (DP_type) {
	case 0:
		scherm1();
		break;
	case 1:
		scherm2();
		break;
	case 2:
		display.print("display 2");
		break;
	case 3:
		display.print("Toon 3");
		break;
	case 4:
		display.print("program?");
		break;
	}
	display.display();
}

void scherm1() {
	String txt_data = "";
	String txt_type; String txt_rm;
	byte spatie = 0;
	int speed;

	//Main venster
	if (MEM_reg & (1 << 0)) { //Mainvenster RPM 			
		txt_type += F("RPM");
	}
	else { //Mainvenster kmh
		txt_type += F("KMh");
	}

	if (MEM_reg & (1 << 1)) { //Mainvenster RM1		
		speed = RPM1;
		txt_rm = F("RM1");
	}
	else { //MainVenster RM2
		speed = RPM2;
		txt_rm = F("RM2");
	}

	spatie = spaties(speed, 5); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += speed;

	display.setTextColor(WHITE);
	display.setTextSize(3);
	display.setCursor(1, 5);
	display.print(txt_data);

	display.setTextSize(1);
	display.setCursor(95, 7);
	display.print(txt_rm);
	display.setCursor(95, 19);
	display.print(txt_type);

	//Ondervenster
	display.drawLine(2, 30, 120, 30, WHITE);

	//RPM1 klein
	display.setCursor(14, 38);
	display.print(F("RM1"));
	txt_data = "";
	speed = RPM1;
	spatie = spaties(speed, 5); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += speed;
	display.setCursor(1, 48);
	display.print(txt_data);

	//RPM2 klein
	display.setCursor(45, 38);
	display.print(F("RM2"));
	txt_data = "";
	speed = RPM2;
	spatie = spaties(speed, 5); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += speed;
	display.setCursor(33, 48);
	display.print(txt_data);


}
void scherm2() {
	//programmeer venster	
	byte spatie;
	String txt_data;
	display.setTextColor(WHITE);
	display.setTextSize(1);

	//level0********************Diameter roller 1
	display.setCursor(1, 1);
	display.write(236);
	display.print(F("RM1 "));
	
	txt_data = "";
	spatie = spaties(dr1, 3); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += dr1;
	display.print(txt_data);
	//Level 1 *******************Diameter wiel op RM1 
	display.setCursor(63, 1);
	display.write(236);
	display.print(F("Wiel "));
	
	txt_data = "";
	spatie = spaties(dw1, 3); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += dw1;
	display.print(txt_data);
	//level2 ********************Diameter roller 2
	display.setCursor(1, 13);
	display.write(236);
	display.print(F("RM2 "));

	txt_data = "";
	spatie = spaties(dr2, 3); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += dr2;
	display.print(txt_data);
	//level3 *******************Diameter wiel op RM2 
	display.setCursor(63, 13);
	display.write(236);
	display.print(F("Wiel "));

	txt_data = "";
	spatie = spaties(dw2, 3); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += dw2;
	display.print(txt_data);
	//level 4 ************************Aantal pulsen per rotatie RM1
	display.setCursor(1,25);
	display.write(24);
	display.print(F("RM1 "));

	txt_data = "";
	spatie = spaties(holes1, 3); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += holes1;
	display.print(txt_data);

	//level5 *******************Aantal pulsen per rotatie RM2
	display.setCursor(63, 25);
	display.write(24);
	display.print(F("RM2  "));

	txt_data = "";
	spatie = spaties(holes2, 3); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += holes2;
	display.print(txt_data);
	//Level6 ******************************



}
void tekens() { //opnemen in loop overige display acties uitschakelen

	if (millis() - counttekens > 500) {
		display.clearDisplay();
		counttekens = millis();


		display.setTextColor(WHITE);
		display.setTextSize(3);
		display.setCursor(1, 5);
		display.println(countsign);
		display.write(countsign);
		display.display();
		countsign++;
		if (countsign > 255)countsign = 200;
	}
}



byte spaties(int nummer, byte digits) {
	byte aantal = 0;
	byte spatie = 0;

	while (nummer > 9) {
		aantal++;
		nummer = nummer / 10;
	}
	//aantal is nu aantal cyfers
	spatie = digits - 1 - aantal;

	return spatie;
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
		if (DP_type > DPtypes)DP_type = 0;

		break;
	case 1:
		switch (DP_type) {
		case 0: //wisseld tonen kmh of RPM in hoofd venster
			MEM_reg ^= (1 << 0);
			break;
		}
		break;
	case 2:
		MEM_reg ^= (1 << 1);
		break;
	case 3:

		break;
	}

	//if (DP_type==0) 
		MEM_write(); //? klopt dit? 

}
void tekens(byte teken) {

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

void calc(bool rm) {
	//maakt alle berekeningen, gemiddelde van twee laatste metingen
	int speed;
	if (rm) { //RM1
		speed = 60000000 / SD_times[3] + 1;
		RPM2 = (speed + rpm2old) / 2;
		rpm2old = speed;
	}
	else { //RM0
		speed = 60000000 / SD_times[1] + 1;
		RPM1 = (speed + rpm1old) / 2;
		rpm1old = speed;
	}
}

void I2C_send(byte command) {
	Wire.beginTransmission(0x3C);    // begin I2C communication adress0x3C
	Wire.write(0x80);           // Set OLED Command mode 0x80
	Wire.write(command);
	Wire.endTransmission();                       // End I2C communication
}

