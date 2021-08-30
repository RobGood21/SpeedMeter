/*
 Name:		SpeedMeter.ino
 Created:	6/10/2021 11:49:37 AM
 Author:	RobAntonisse

 Use of library: https://github.com/acrobotic/Ai_Ardulib_SSD1306


 V1.01 26 juni 2021
 Eerste versie, verder werken als hardware SpeedMeter klaar is.
 Ervaringen en evaluatie gebruikers met verschillende type sensoren en rollerbanken is nodig.
 Werkt alleen met simpleDyno en gemonteerde display
 Display in twee typen op de markt let op de maten gaten 27mm en 28mm uit elkaar (allGoods)

 V2.01
 Versie zichtbaar maken in display
 0 waardes  onmogelijk maken
 Toepassing van 'KPF Zeller Speed-Cat' als sensor mogelijk maken.
 Itrain met gebruik van 'KPF Zeller Speed-Cat' instelling koppelen (iTrain 5.0 - Handleiding blz.72)

*/



#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define Version "V2.01"


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



struct PRESETS {

	byte dr1; //diameter rol 1 in mm
	byte dr2; //diameter rol2
	byte dw1;//diameter wiel voertuig op RM1 
	byte dw2; //diameter wiel voertuig op RM2
	byte puls1; //20; 5=kwart rotatie rpm x4 aantal pulsen per rotatie, gaatjes in de IR disc
	byte puls2;
	byte schaal;
	byte precisie; //1~10  1 voor langzame metingen 10 voor extreem snelle metingen 5 = default gemiddeld
};

PRESETS preset[6];

byte p = 0;
//variabelen
byte holecount1; byte holecount2;

unsigned int dender[2];
unsigned long antidender[2];

int switchstatus = 15;
byte switchcount[4];

unsigned int RPM1 = 0;
unsigned int RPM2 = 0;
unsigned int rpm1old = 0;
unsigned int rpm2old = 0;
unsigned int countstop = 0;

unsigned long SCtime; // tbv SpeedCat/Itrain
int countSC = 0; //puls counter tbv SpeedCat
char txtSC[80];

byte DP_type = 0; //EEPROM #101 welk scherm wordt getoond, wisseld met knop 4
byte DP_level = 0;
byte MEM_reg; //EEPROM #100

//temps
int countsign = 0;
unsigned long counttekens; //gebruikt om tekens te kunnen opzoeken, in loop opnemen
unsigned long oldtime;



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

	//Poorten en pins definieren
	DDRD |= (1 << 7); //Pin 7 Groene led, output
	DDRD |= (1 << 6); //Pin 6 Rode led
	DDRD |= (1 << 5); //PIN5 INT1 enabled
	DDRD |= (1 << 4); //PIN4 INT0 enabled
	//pin 8~11 4 xschakelaar input
	DDRB &= ~(15 << 0);
	PORTB |= (15 << 0); //pull-up to pins 8~11
	PORTD &= ~(1 << 5);
	PORTD &= ~(1 << 4);



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
	R_dender();
}



ISR(INT0_vect) {
	//cli();
	//Deze ook gebruiken voor Speed-Cat
	countstop = 0;
	countSC++;
	if (micros() - antidender[0] > dender[0]) {
		antidender[0] = micros(); //reset timer

		//speedCat counter, puls tellen


		holecount1++;
		if (holecount1 >= preset[p].puls1) {
			holecount1 = 0;
			
			PIND |= (1 << 6);

			SD_times[0] = micros();
			SD_times[1] = SD_times[0] - oldtime1; //bereken interval
			oldtime1 = SD_times[0];
			calc(false);
		}

	}

	//sei();
}

ISR(INT1_vect) {

	//denk dat die struct volatile moet worden anders tussen variabel voor maken INT0 doettu welll
	//cli();
	countstop = 0;
	if (micros() - antidender[1] > dender[1]) {
		antidender[1] = micros();
		holecount2++;

		if (holecount2 >= preset[p].puls2) {
			holecount2 = 0;
			PIND |= (1 << 7); //groene led aan
			SD_times[2] = micros();
			SD_times[3] = SD_times[2] - oldtime2; //bereken interval
			oldtime2 = SD_times[2];
			calc(true);
		}
	}
	//sei();
}


void loop() {
	//unsigned long time;
	//time = millis();


	if (MEM_reg & (1 << 4)) { //send data to SpeedCat app or Itrain
		if (millis() - SCtime > 996) { //1sec

			SC_exe();
			SCtime = millis();
		}
	}

	if (millis() - slowtime > 20) { //20
		slowtime = millis();

		countstop++;
		if (countstop > 10) {
			RPM1 = 0;
			RPM2 = 0;
			PORTD &= ~(B11000000 << 0); //leds uit			
		}

		if (~MEM_reg & (1 << 4)) SD_exe(); //sends msg to  simpledyno via serial connection

		SW_exe();
		//tekens(); //gebruiken om speciaal teken op te zoeken
		DP_exe(); //dit proces duurt te lang! vertraagd de time based processen te veel 
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
	MEM_reg = EEPROM.read(10);
	DP_type = EEPROM.read(11);
	if (DP_type > DPtypes)DP_type = 0;
	//roller en wieldiameters


	p = EEPROM.read(12);
	if (p == 0xFF)p = 0;
	///P =current preset moet nu al geladen zijn uit EEPROM

	//6 presets mogelijk

	for (byte i = 0; i < 6; i++) {

		preset[i].dr1 = EEPROM.read(100 + (i * 10));
		preset[i].dr2 = EEPROM.read(101 + (i * 10));
		preset[i].dw1 = EEPROM.read(102 + (i * 10));
		preset[i].dw2 = EEPROM.read(103 + (i * 10));
		preset[i].puls1 = EEPROM.read(104 + (i * 10));
		preset[i].puls2 = EEPROM.read(105 + (i * 10));
		preset[i].schaal = EEPROM.read(106 + (i * 10));
		preset[i].precisie = EEPROM.read(107 + (i * 10));

		if (preset[i].dr1 == 0xFF)preset[i].dr1 = 20;
		if (preset[i].dr2 == 0xFF)preset[i].dr2 = 20;
		if (preset[i].dw1 == 0xFF)preset[i].dw1 = preset[p].dr1;
		if (preset[i].dw2 == 0xFF)preset[i].dw2 = preset[p].dr2;
		if (preset[i].puls1 == 0xFF)preset[i].puls1 = 1;
		if (preset[i].puls2 == 0xFF)preset[i].puls2 = 1;
		if (preset[i].schaal == 0xFF)preset[i].schaal = 1;
		if (preset[i].precisie == 0xFF)preset[i].precisie = 6;
	}


}
void MEM_write() {
	//updates EEPROM, vaak.... dus als rare fouten na enkele jaren denkbaar EEPROM defect van de arduino
	EEPROM.update(10, MEM_reg);
	EEPROM.update(11, DP_type);
	EEPROM.update(12, p);
	//alleen de current preset p hoeft te worden geupdated.
	EEPROM.update(100, preset[p].dr1);
	EEPROM.update(101, preset[p].dr2);
	EEPROM.update(102, preset[p].dw1);
	EEPROM.update(103, preset[p].dw2);
	EEPROM.update(104, preset[p].puls1);
	EEPROM.update(105, preset[p].puls2);
	EEPROM.update(106, preset[p].schaal);
	EEPROM.update(107, preset[p].precisie);

	R_dender(); //bereken denderwaardes, van precisie

}

void R_dender() {
	//berekend de dender waarde in us voor de twee rotatie meters. dender 50~50000 
	//interupts niet actief tijdens deze waarde
	//voor hoge snelheid precisie laag cyfer, hoe langzamer cyfer verhogen. filtert dender(bouncing) van sensoren.

	int temp = 10;
	for (byte n = 0; n < preset[p].precisie; n++) {
		temp = temp * 2;
	}
	dender[0] = temp / preset[p].puls1;
	dender[1] = temp / preset[p].puls2;

	//Serial.print("Dender 0 = "); Serial.print(dender[0]); Serial.print("  Dender 1 = "); Serial.println(dender[1]);

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
	unsigned long speed = 0; //Voor RPM
	unsigned long rest;


	//Main venster
	if (MEM_reg & (1 << 0)) { //Mainvenster RPM 	
		txt_type += F("RPM");
		//logo welk wiel wordt getoond
		if (MEM_reg & (1 << 2)) { //rpm roller
			display.drawCircle(105, 40, 5, WHITE);
			display.fillCircle(95, 50, 8, WHITE);
			txt_rm = (F("RM"));

		}
		else { //rpm wiel voertuig
			display.fillCircle(105, 40, 5, WHITE);
			display.drawCircle(95, 50, 8, WHITE);
			txt_rm = (F("AS"));
		}

		if (MEM_reg & (1 << 1)) { //Mainvenster RM1	
			speed = RPM1;
			txt_rm += F("1");
			if (~MEM_reg & (1 << 2))speed = speed * preset[p].dr1 / preset[p].dw1;
		}
		else { //MainVenster RM2
			speed = RPM2;
			if (~MEM_reg & (1 << 2))speed = speed * preset[p].dr2 / preset[p].dw2;
			txt_rm += F("2");
		}

	}

	else { //Mainvenster kmh		

		txt_data = "";
		txt_rm = F("RM");
		txt_type += F("KMh");

		if (MEM_reg & (1 << 1)) { //RM1

			txt_rm += F("1");
			//rotatie per minuut x omtrek x 60 minuten = afgelegde afstand in mm
			speed = RPM1 * preset[p].dr1*3.14 * 60;
		}
		else {
			txt_rm += F("2"); //RM2
			speed = RPM2 * preset[p].dr2*3.14 * 60;
		}

		display.drawLine(80, 41, 110, 41, WHITE);
		display.drawLine(80, 53, 110, 53, WHITE);
		display.setCursor(85, 44);
		display.setTextSize(1);
		display.setTextColor(WHITE);

		display.print(F("1:"));
		if (MEM_reg & (1 << 3)) { //snelheid tonen 1:1
			display.print("1");
		}
		else { //snelheid tonen op schaal
			speed = speed * preset[p].schaal;
			display.print(preset[p].schaal);
			display.setCursor(120, 1);
			display.write(231);
		}


		speed = speed / 100000;
		//laatste cyfer berekenen
		rest = speed;
		while (rest > 9) {
			rest = rest - 10;
		}
		speed = speed / 10;
		if (rest > 4)speed++;
	}

	spatie = spaties(speed, 5); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";

	}

	txt_data += speed;

	//
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

//bool rest() {


	//return rest;
//}

void scherm2() {
	//programmeer venster	
	byte spatie;
	String txt_data;
	display.setTextColor(WHITE);
	display.setTextSize(1);

	//level0*************Preset keuze
	display.setCursor(1, 1);
	display.print(F("Preset "));
	display.print(p + 1);
	display.setCursor(95, 1);
	display.print(Version);


	//level1********************Diameter roller 1
	display.setCursor(1, 15);
	display.write(236);
	display.print(F("RM1 "));

	txt_data = "";
	spatie = spaties(preset[p].dr1, 3); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += preset[p].dr1;
	display.print(txt_data);
	//Level 2 *******************Diameter wiel op RM1 
	display.setCursor(63, 15);
	display.write(236);
	display.print(F("Wiel "));

	txt_data = "";
	spatie = spaties(preset[p].dw1, 3); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += preset[p].dw1;
	display.print(txt_data);
	//level3 ********************Diameter roller 2
	display.setCursor(1, 27);
	display.write(236);
	display.print(F("RM2 "));

	txt_data = "";
	spatie = spaties(preset[p].dr2, 3); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += preset[p].dr2;
	display.print(txt_data);
	//level4 *******************Diameter wiel op RM2 
	display.setCursor(63, 27);
	display.write(236);
	display.print(F("Wiel "));

	txt_data = "";
	spatie = spaties(preset[p].dw2, 3); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += preset[p].dw2;
	display.print(txt_data);
	//level 5 ************************Aantal pulsen per rotatie RM1
	display.setCursor(1, 39);
	display.write(24);
	display.print(F("RM1 "));

	txt_data = "";
	spatie = spaties(preset[p].puls1, 3); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += preset[p].puls1;
	display.print(txt_data);

	//level6 *******************Aantal pulsen per rotatie RM2
	display.setCursor(63, 39);
	display.write(24);
	display.print(F("RM2  "));

	txt_data = "";
	spatie = spaties(preset[p].puls2, 3); //getal, aantal cyfers in het getal
	for (byte i = 0; i < spatie; i++) {
		txt_data += " ";
	}
	txt_data += preset[p].puls2;
	display.print(txt_data);
	//Level7 *********************Schaal 1:(data)
	display.setCursor(1, 51);
	display.write(231);
	display.print(F(" 1:"));
	display.print(preset[p].schaal);
	//Level8 *******************Precisie
	display.setCursor(63, 51);
	display.write(245);
	display.print(F(" "));
	display.print(preset[p].precisie);

	//onderstreping
	switch (DP_level) {
	case 0:
		display.drawLine(1, 9, 48, 9, WHITE);
		break;
	case 1: //øRM1
		display.drawLine(1, 23, 48, 23, WHITE);
		break;
	case 2: //øWiel
		display.drawLine(61, 23, 115, 23, WHITE);
		break;
	case 3: //øRM2
		display.drawLine(1, 35, 48, 35, WHITE);
		break;
	case 4: //øWiel2
		display.drawLine(61, 35, 115, 35, WHITE);
		break;
	case 5://↑RM1
		display.drawLine(1, 47, 48, 47, WHITE);
		break;
	case 6: //↑RM2
		display.drawLine(61, 47, 115, 47, WHITE);
		break;
	case 7: //Schaal 
		display.drawLine(1, 59, 34, 59, WHITE);
		break;
	case 8: //precisie
		display.drawLine(61, 59, 84, 59, WHITE);
		break;
	}
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

void SW_exe() { //called from loop() every 20ms
	byte ss; byte changed;
	ss = PINB;
	ss = ss << 4;
	ss = ss >> 4; //clear bits 4,5,6,7
	changed = switchstatus ^ ss;

	if (changed > 0) {
		SW_clear();
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
	else { //dus geen verandering, scrollen.
		if (ss != 15) { //dus switch ingedrukt
			if (switchcount[0] < 200) switchcount[0]++;
			if (switchcount[0] > 10) { //na 1 seconde				
				//PIND |= (1 << 6); //rode led ff als test
				if (~ss & (1 << 2)) { //sw 3 ingedrukt
					SW_on(2);
				}
				else if (~ss & (1 << 3)) { //sw 4 ingedrukt
					SW_on(3);
				}
			}
		}
	}
	switchstatus = ss;
}

void SW_clear() {
	//set counters naar 0
	for (byte i = 0; i < 5; i++) {
		switchcount[i] = 0;
	}
}

void SW_on(byte sw) {
	//Serial.print(F("Switch-on: ")); Serial.println(sw);
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
		case 1:
			//next parameter
			DP_level++;
			if (DP_level > 8)DP_level = 0;
			break;
		}
		break;
	case 2:
		switch (DP_type) {
		case 0:
			MEM_reg ^= (1 << 1);
			break;
		case 1://value down

			switch (DP_level) {
			case 0:
				if (p > 0) p--;
				break;
			case 1: //rm1 roller
				if (preset[p].dr1 > 1) preset[p].dr1--;
				break;
			case 2://rm1 Wiel
				if (preset[p].dw1 > 1) preset[p].dw1--;
				break;
			case 3://rm2 roller
				if (preset[p].dr2 > 1) preset[p].dr2--;
				break;
			case 4://rm2 wiel
				if (preset[p].dw2 > 1) preset[p].dw2--;
				break;
			case 5://rm1 pulsen per rotatie
				if (preset[p].puls1 > 1) preset[p].puls1--;
				break;
			case 6://rm2 pulsen per rotatie
				if (preset[p].puls2 > 1) preset[p].puls2--;
				break;
			case 7:// schaal 1: value
				if (preset[p].schaal > 1) preset[p].schaal--;
				break;
			case 8:
				if (preset[p].precisie > 0) preset[p].precisie--;
				break;
			}
			break;
		}

		break;
	case 3:

		switch (DP_type) {
		case 0:
			if (~MEM_reg & (1 << 0)) { //KMh mode
				MEM_reg ^= (1 << 3); //snelheid op schaal of 1:1
			}
			else { //RPM mode
				MEM_reg ^= (1 << 2); //toon welk RPM roller/wiel getoond wordt
			}


			break;

		case 1:	//value up
			switch (DP_level) {
			case 0:
				if (p < 5)p++;
				break;
			case 1: //rm1 roller
				if (preset[p].dr1 < 250) preset[p].dr1++;
				break;
			case 2://rm1 Wiel
				if (preset[p].dw1 < 250) preset[p].dw1++;
				break;
			case 3://rm2 roller
				if (preset[p].dr2 < 250) preset[p].dr2++;
				break;
			case 4://rm2 wiel
				if (preset[p].dw2 < 250) preset[p].dw2++;
				break;
			case 5://rm1 pulsen per rotatie
				if (preset[p].puls1 < 250) preset[p].puls1++;
				break;
			case 6://rm2 pulsen per rotatie
				if (preset[p].puls2 < 250) preset[p].puls2++;
				break;
			case 7:// schaal 1: value
				if (preset[p].schaal < 250) preset[p].schaal++;
				break;
			case 8: //precisie
				if (preset[p].precisie < 12)preset[p].precisie++;
				break;
			}

			break;
		}
		break;
	}

	if (DP_type == 0)MEM_write();
}
void tekens(byte teken) {

}

void SW_off(byte sw) {

}
void SC_exe() {
	//stuurt boodschap over USB poort ten behoeve van SpeedCat (Itrain compatibel)
	unsigned int tijd;
	unsigned int pls = 0;
	tijd = millis() - oldtime;
	//aantal pulsen per rotatie doorgeven
	//instelbaar voor verschillende rollerbanken te gebruiken.
	//vermoed default 4 instellen 1~25

	pls = (countSC * 4) / preset[p].puls1; //4 nog instelbaar maken

	sprintf(txtSC, "*%04d;V3.0%s", pls, "%");
	Serial.println(txtSC);

	//Serial.print(tijd);Serial.print("   Puls: "); Serial.println(countSC);
	oldtime = millis();
	countSC = 0;
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

