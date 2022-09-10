/*
 Name:		SpeedMeter.ino
 Created:	6/10/2021 11:49:37 AM
 Author:	RobAntonisse

 Use of library: https://github.com/acrobotic/Ai_Ardulib_SSD1306


 V1.01 26 juni 2021
 Eerste versie, verder werken als hardware SpeedMeter klaar is.
 Ervaringen en evaluatie gebruikers met verschillende type sensoren en rollerbanken is nodig.
 Werkt alleen met symple Dyno
 Display in twee typen op de markt let op de maten gaten 27mm en 28mm uit elkaar (allGoods)

 V2.01
 Versie zichtbaar maken in display
 Toepassing van 'KPF Zeller Speed-Cat' als sensor mogelijk maken.
 Itrain met gebruik van 'KPF Zeller Speed-Cat' instelling koppelen (iTrain 5.0 - Handleiding blz.72)

 bugsfixes:
-0 waardes  onmogelijk maken fixed
-Presets 2~6 werden niet goed in EEPROM opgeslagen. Fixed
-Bij 'in bedrijf' scherm, scroll van knop 3 en 4 niet. Fixed
-Debug en ontwikkel code weggehaald

V3.01
Toevoegen afstand snelheids meting


*/

#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define Version "V3.01"
#define resettime 500 
//na 1 interrupt worden alle interrupts geblocked gedurende de resettime, belangrijk dus dat het traject minstens de 
//reset time moet duren. en dat de hele trein gepasseerd moet zijn plus de resettijd voordat de andere sensor
//kan worden geactiveerd


//Display constructor
Adafruit_SSD1306 display(128, 64, &Wire, 4);

//voor SimpleDyno
//message string" micros(),temptime1,time1,temptime2.time2,0,0,0,0,0,0"
//De 6 nullen zijn de 6 optionele Analog reads voor: voltage, stroom, temeratuur1 en temperatuur2. 5 en 6 zijn not in use
//Message 1000Hz (1x per 1 ms max, in Speedmeter 1xper 20ms)


//Variables
String toSend;
unsigned long SD_times[4];
unsigned int SD_reads[6];
unsigned long oldtime1; //voor berekening interval tussen twee pulsen
unsigned long oldtime2;
unsigned long slowtime;

int SW_time; //counter, timer voor zeer langzame processen (1xper 20ms, 50hz)

struct PRESETS {
	byte mode; //0=rotatiemeting 1=point to point meting
	byte traject; //lengte van het traject in cm (max dus 2M55?)
	byte usb; //0=geen output 1=Itrain/SpeedCat 2=Simpledyn
	byte dr1; //diameter rol 1 in mm
	byte dr2; //diameter rol2
	byte dw1;//diameter wiel voertuig op RM1 
	byte dw2; //diameter wiel voertuig op RM2
	byte puls1; //20; 5=kwart rotatie rpm x4 aantal pulsen per rotatie, gaatjes in de IR disc
	byte puls2;
	byte pulsIt; //aantal pulsen per rotatie naar Itrain Speed-Cat waarschijnlijk vast 4 (x2 meet alle changes up en down)
	byte Dsc; //diameter sc speedcat, met welke diameter mm/10 rekend Itrain, te gebruiken voor Yken
	byte schaal;
	byte precisie; //1~10  1 voor langzame metingen 10 voor extreem snelle metingen 5 = default gemiddeld
};

PRESETS preset[6];

//preset 6 is default voor Itrain
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
byte DP_level = 0;
byte MEM_reg; //EEPROM #100
byte prglvl = 10;

byte lastsensor = 0;
unsigned long sensortime = 0;
byte trajectfase = 0;
unsigned long trajecttime = 0;
unsigned long gemetentijd = 0;
byte countleds = 0;
int laatstesnelheid[2] = { 0,0 };

//temps
//int countsign = 0;
//unsigned long counttekens; //gebruikt om tekens te kunnen opzoeken, in loop opnemen
//unsigned long oldtime;
int symbol = 0;

void setup() {
	Serial.begin(9600);
	//Display.
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Ook als display er niet is starten.	   
	display.clearDisplay();
	display.display();
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

	//setup interrupts, Interupt on rising edge PIN2, INT0, 01 = any change 10=falling edge 11=rising edge
	EICRA |= (1 << 0); //set bit0 of register ISC00 (rising edge both)
	EICRA |= (1 << 1); //set bit1 of register ISC01
	EICRA |= (1 << 2); //set bit2 of register ISC10 (rising edge, both)
	EICRA |= (1 << 3); //set bit3 of register ISC11
	EIMSK |= (1 << 0); //Interupt INT0 enabled
	EIMSK |= (1 << 1); //Interupt INT1 enabled

	//factory reset, knop1 en knop 4 ingedrukt
	if (~PINB & (1 << 0) && ~PINB & (1 << 3))factory();

	//initialisaties
	MEM_read();
	R_dender();
}
ISR(INT0_vect) {
	cli();
	if (preset[p].mode & (1 << 0)) { //test bit 0 true traject mode
		traject(1); //sensor 1
	}
	else { //roller mode
		countstop = 0;//teller om stilstaan te detecteren
		if (micros() - antidender[0] > dender[0]) {
			antidender[0] = micros(); //reset timer
			countSC++;//Counter voor Itrain/SpeedCat
			holecount1++;
			if (holecount1 >= preset[p].puls1) {
				holecount1 = 0;
				PIND |= (1 << 6); //Flip rode led
				SD_times[0] = micros();
				SD_times[1] = SD_times[0] - oldtime1; //bereken interval
				oldtime1 = SD_times[0];
				calc(false);
			}
		}
	}

	sei();
}
ISR(INT1_vect) {
	cli();
	if (preset[p].mode & (1 << 0)) { //test bit 0 true traject mode
		traject(2); //sensor 2
	}
	else { //roller mode

		countstop = 0; //teller om stilstaan te detecteren
		if (micros() - antidender[1] > dender[1]) {
			antidender[1] = micros();
			holecount2++;

			if (holecount2 >= preset[p].puls2) {
				holecount2 = 0;
				PIND |= (1 << 7); //Flip groene led
				SD_times[2] = micros();
				SD_times[3] = SD_times[2] - oldtime2; //bereken interval
				oldtime2 = SD_times[2];
				calc(true);
			}
		}
	}

	sei();
}
void loop() {
	if (preset[p].usb == 1) { //send data to SpeedCat app or Itrain

		if (millis() - SCtime > 999) { //1sec
			SC_exe();
			SCtime = millis();
		}
		else if (millis() - SCtime > 950) {
			//slow events niet vlak voor een telling voor Itrain uitvoeren
			//Slow events duurt te lang ongeveer 30ms, maakt de seconde telling voor Itrain onnauwkeurig
			//millis() gaat door en de ISR gaat door... 
			//lelijke oplossing, werkt wel nog eens naar kijken
			return;
		}
	}

	if (millis() - slowtime > 20) { //20 
		slowtime = millis();

		if (preset[p].mode & (1 << 0)) {

			if (millis() - sensortime > resettime) {
				GPIOR0 &= ~(1 << 5); //reset flag one shot interrupts
				PORTD &= ~(1 << 6); //kill red led
				if (GPIOR0 & (1 << 6)) trajectend();
			}
			if (GPIOR0 & (1 << 4))trajectleds();
		}
		else { //v3.01 deze if else
			if (countstop > 10) {
				RPM1 = 0;
				RPM2 = 0;
				PORTD &= ~(B11000000 << 0); //leds uit			
			}
			countstop++;
		}

		if (preset[p].usb == 2) SD_exe(); //sends msg to  simpledyno via serial connection
		SW_exe();
		DP_exe(); //dit proces duurt lang! vertraagd de time based processen 
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
	p = EEPROM.read(12);
	if (p == 0xFF)p = 0;///P =current preset 
	//6 presets mogelijk V2.01
	byte xtr;
	for (byte i = 0; i < 6; i++) {
		xtr = i * 20;
		preset[i].dr1 = EEPROM.read(100 + xtr);
		preset[i].dr2 = EEPROM.read(101 + xtr);
		preset[i].dw1 = EEPROM.read(102 + xtr);
		preset[i].dw2 = EEPROM.read(103 + xtr);
		preset[i].puls1 = EEPROM.read(104 + xtr);
		preset[i].puls2 = EEPROM.read(105 + xtr);
		preset[i].schaal = EEPROM.read(106 + xtr);
		preset[i].precisie = EEPROM.read(107 + xtr);
		preset[i].pulsIt = EEPROM.read(108 + xtr);
		preset[i].Dsc = EEPROM.read(109 + xtr);//ijken
		preset[i].usb = EEPROM.read(110 + xtr);
		preset[i].mode = EEPROM.read(111 + xtr); //mode 0=rotatie 1=<->
		preset[i].traject = EEPROM.read(112 + xtr); //lengte van het traject

		switch (i) {
		case 0: //preset 1 traject meting 1M
			if (preset[i].dr1 == 0xFF)preset[i].dr1 = 20;
			if (preset[i].dr2 == 0xFF)preset[i].dr2 = 20;
			if (preset[i].dw1 == 0xFF)preset[i].dw1 = preset[p].dr1;
			if (preset[i].dw2 == 0xFF)preset[i].dw2 = preset[p].dr2;
			if (preset[i].puls1 == 0xFF)preset[i].puls1 = 1;
			if (preset[i].puls2 == 0xFF)preset[i].puls2 = 1;
			if (preset[i].schaal == 0xFF)preset[i].schaal = 87;
			if (preset[i].precisie == 0xFF)preset[i].precisie = 6;
			if (preset[i].pulsIt == 0xFF)preset[i].pulsIt = 4; //waarschijnlijk standaard... 
			if (preset[i].Dsc == 0xFF)preset[i].Dsc = 63; ////diameter mm/10 roller waar Itrain/speedcat mee rekenen, ijken
			if (preset[i].usb == 0xFF)preset[i].usb = 0; //0=geen usb uit 1=Itrain/SpeedCat 2=SimpleDyno
			if (preset[i].mode == 0xFF)preset[i].mode = 1; //rotatie of <-> V3.01
			if (preset[i].traject == 0xFF) preset[i].traject = 100;
			break;
		case 5: //default preset for Itrain/speedmeter
			if (preset[i].dr1 == 0xFF)preset[i].dr1 = 6;
			if (preset[i].dr2 == 0xFF)preset[i].dr2 = 1;
			if (preset[i].dw1 == 0xFF)preset[i].dw1 = 1;
			if (preset[i].dw2 == 0xFF)preset[i].dw2 = 1;
			if (preset[i].puls1 == 0xFF)preset[i].puls1 = 4;
			if (preset[i].puls2 == 0xFF)preset[i].puls2 = 1;
			if (preset[i].schaal == 0xFF)preset[i].schaal = 87;
			if (preset[i].precisie == 0xFF)preset[i].precisie = 6;
			if (preset[i].pulsIt == 0xFF)preset[i].pulsIt = 4; //waarschijnlijk standaard... 
			if (preset[i].Dsc == 0xFF)preset[i].Dsc = 63; ////diameter mm/10 roller waar Itrain/speedcat mee rekenen, ijken
			if (preset[i].usb == 0xFF)preset[i].usb = 1; //0=geen usb uit 1=Itrain/SpeedCat 2=SimpleDyno
			if (preset[i].mode == 0xFF)preset[i].mode = 0;
			if (preset[i].traject == 0xFF) preset[i].traject = 100;
			break;

		default:
			if (preset[i].dr1 == 0xFF)preset[i].dr1 = 20;
			if (preset[i].dr2 == 0xFF)preset[i].dr2 = 20;
			if (preset[i].dw1 == 0xFF)preset[i].dw1 = preset[p].dr1;
			if (preset[i].dw2 == 0xFF)preset[i].dw2 = preset[p].dr2;
			if (preset[i].puls1 == 0xFF)preset[i].puls1 = 1;
			if (preset[i].puls2 == 0xFF)preset[i].puls2 = 1;
			if (preset[i].schaal == 0xFF)preset[i].schaal = 87;
			if (preset[i].precisie == 0xFF)preset[i].precisie = 6;
			if (preset[i].pulsIt == 0xFF)preset[i].pulsIt = 4; //waarschijnlijk standaard... 
			if (preset[i].Dsc == 0xFF)preset[i].Dsc = 63; ////diameter mm/10 roller waar Itrain/speedcat mee rekenen, ijken
			if (preset[i].usb == 0xFF)preset[i].usb = 0; //0=geen usb uit 1=Itrain/SpeedCat 2=SimpleDyno
			if (preset[i].mode == 0xFF)preset[i].mode = 0; //rotatie of <-> V3.01
			if (preset[i].traject == 0xFF) preset[i].traject = 100;
			break;
		}
	}
}
void MEM_write() {
	byte xtr = 20 * p;
	EEPROM.update(10, MEM_reg);
	EEPROM.update(12, p);
	//alleen de current preset p hoeft te worden geupdated.
	EEPROM.update(100 + xtr, preset[p].dr1);
	EEPROM.update(101 + xtr, preset[p].dr2);
	EEPROM.update(102 + xtr, preset[p].dw1);
	EEPROM.update(103 + xtr, preset[p].dw2);
	EEPROM.update(104 + xtr, preset[p].puls1);
	EEPROM.update(105 + xtr, preset[p].puls2);
	EEPROM.update(106 + xtr, preset[p].schaal);
	EEPROM.update(107 + xtr, preset[p].precisie);
	EEPROM.update(108 + xtr, preset[p].pulsIt);
	EEPROM.update(109 + xtr, preset[p].Dsc);
	EEPROM.update(110 + xtr, preset[p].usb);
	EEPROM.update(111 + xtr, preset[p].mode); //v3.01
	EEPROM.update(112 + xtr, preset[p].traject); //v3.01

	//inits
	R_dender(); //bereken denderwaardes, van precisie   
}
void R_dender() {
	//pauzeert de interupts om denderen van de sensors te ondervangen
	int temp = 10;
	for (byte n = 0; n < preset[p].precisie; n++) {
		temp = temp * 2;
	}
	dender[0] = temp / preset[p].puls1;
	dender[1] = temp / preset[p].puls2;
}
void traject(byte s) { //V3.01 s=sensor nummer
	//Called from ISR's sensors direct, traject meting
	if (~GPIOR0 & (1 << 5)) { //alleen als register false is, one shot

		GPIOR0 |= (1 << 5); //set register flag for one shot

		if (lastsensor == s) { //zelfde sensor

			//if (millis() - sensortime > resettime) {
				//hier proces reset opnieuw starten traject fase 1
				//misschien beter een drukknop gebruiken voor reset???
				//trajectfase = 1; trajecttime = millis();
			//}
		}
		else { //andere sensor
			//sensortime = millis(); //create one shot on the interrupts
			PORTD |= (1 << 6); //set red led
			switch (trajectfase) {
			case 0:	//start meting in millisecondes
				trajectfase = 1;
				trajecttime = millis();
				lastsensor = s;
				GPIOR0 |= (1 << 4); //set bit 4 start led knipperen
				laatstesnelheid[1] = laatstesnelheid[0];
				break;

			case 1: //meting in proces tweede sensor bereikt.
				GPIOR0 |= (1 << 6); //set flag end of cyclus
				gemetentijd = (millis() - trajecttime);
				GPIOR0 &= ~(1 << 4); //reset flag knipperend groen	
				PORTD &= ~(1 << 7); //leds uit
				break;
			}
		}
	}
	sensortime = millis();
}

void trajectend() {
	trajectfase = 0;
	lastsensor = 0;
	GPIOR0 &= ~(1 << 6); //reset flag trajectend
	PORTD &= ~(1 << 6);
}

void trajectleds() {

	countleds++;
	if (countleds > 4) {
		//Serial.print("*");
		countleds = 0;
		PIND |= (1 << 7);
	}
}

void DP_exe() { //called from loop()	
	//Ververst het display op zichtbare snelheid dus 50x per seconde (20ms) called from loop()
	//Langzaam proces duurt ongeveer een 30ms
	display.clearDisplay();

	if (GPIOR0 & (1 << 2)) { //program scherm
		scherm2();
	}
	else { //false default in bedrijf scherm
		switch (preset[p].mode) {
		case 0:
			scherm1();
			break;
		case 1:
			scherm3();
			break;
		}
	}
	display.display();
}
void scherm1() { //in bedrijf, rotatie mode
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
			speed = RPM1 * preset[p].dr1 * 3.14 * 60;
		}
		else {
			txt_rm += F("2"); //RM2
			speed = RPM2 * preset[p].dr2 * 3.14 * 60;
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

	//level 1 mode selectie roller of traject
	display.setCursor(62, 1);

	//displays de ascii codes in het display
	//display.write(symbol);
	//display.setCursor(70, 1);
	//display.print(symbol);


	if (preset[p].mode == 0) { //rollers
		display.write(231);

	}
	else { //rails 
		display.write(174); display.write(175);

	}

	if (preset[p].mode & (1 << 0)) { //traject mode
		//**** trajectlengte
		display.setCursor(1, 15);
		display.print(F("Traject: "));
		display.setCursor(61, 15);
		display.print(preset[p].traject); display.print("cm");


	}
	else { //roller mode


		//Alleen in mode 0*****************************************************************************MODE 0

		//level2********************Diameter roller 1
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

		//Level 3 *******************Diameter wiel op RM1 
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
		//level4 ********************Diameter roller 2
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
		//level5 *******************Diameter wiel op RM2 
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
		//level 6 ************************Aantal pulsen per rotatie RM1
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

		//level 7 *******************Aantal pulsen per rotatie RM2
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

		//verplaatst V3.01
		display.setCursor(35, 51);
		display.write(245);
		//display.print(F(" "));
		display.print(preset[p].precisie);

		//level 10 *****USB 
		display.setCursor(63, 51);
		prglvl = 10;
		switch (preset[p].usb) {
		case 0:
			display.print(F("-"));
			break;
		case 1:
			display.print(F("It"));
			prglvl = 12;
			display.setCursor(80, 51);
			display.print(F("Y")); display.print(preset[p].Dsc);
			display.setCursor(104, 51);
			display.print(F("P")); display.print(preset[p].pulsIt);
			break;
		case 2:
			display.print(F("Sd"));
			break;
		}
		//********MODE 0 alleen einde***************************************************

	}

	//Level 8 *********************Schaal 1:(data)
	display.setCursor(1, 51);
	//display.write(231); //symbool
	display.print(F("1:"));
	display.print(preset[p].schaal);
	//Level 9 *******************Precisie









	//cursor, streep
	byte x; byte y; byte w;
	//x-begin vanaf linker kant, y=begin vanaf boven w=eind vanaf linkerkant
	switch (DP_level) {
	case 0:
		x = 1; y = 9; w = 48;
		break;
	case 1: //mode
		x = 61; y = 9; w = 71;
		break;
	case 2: //øRM1
		x = 1; y = 23; w = 48;
		break;
	case 3: //øWiel
		x = 61; y = 23; w = 115;
		break;
	case 4: //øRM2
		x = 1; y = 35; w = 48;
		break;
	case 5: //øWiel2
		x = 61; y = 35; w = 115;
		break;
	case 6://↑RM1
		x = 1; y = 47; w = 48;
		break;
	case 7: //↑RM2
		x = 61; y = 47; w = 115;
		break;
	case 8: //Schaal 
		x = 1; y = 59; w = 25;
		break;
	case 9: //precisie
		x = 33; y = 59; w = 50;
		break;
	case 10: //USB
		x = 61; y = 59; w = 75;
		break;
	case 11:
		x = 79; y = 59; w = 97;
		break;
	case 12:
		x = 103; y = 59; w = 116;
		break;
	case 20:
		x = 61; y = 23; w = 90;
		break;
	}
	display.drawLine(x, y, w, y, 1);
}
void scherm3() {// in bedrijf sensor naar sensor <->
	unsigned long speed = 0;
	unsigned long dist = 0;
	float time = 0;
	display.drawLine(1, 55, 128, 55, 1);
	display.setTextColor(WHITE);
	display.setTextSize(1);
	display.setCursor(1, 57);
	display.write(174); display.write(175); display.print(preset[p].traject); display.print("cm");
	//schaal weergeven in onderbalk
	display.setCursor(52, 57); display.print("1:"); display.print(preset[p].schaal);

	display.setCursor(85, 57);
	display.print(laatstesnelheid[1]); display.print("km/u");

	//snelheid berekenen traject in cm >KM duur in ms > uur = traject/gemeten*36 
	if (gemetentijd > 10) {
		dist = preset[p].traject * 100000;
		speed = dist / gemetentijd * 36;
		speed = speed / 100;
		time = gemetentijd; // / 1000;
		time = time / 1000;
		laatstesnelheid[0] = speed * preset[p].schaal / 1000;
	}

	display.setCursor(1, 1); display.print(speed); display.print("m/u");
	display.setCursor(54, 1); display.print(time); display.print("s");
	display.drawLine(1, 10, 128, 10, 1);

	display.setCursor(25, 25); display.setTextSize(2);
	for (byte i = 0; i < spaties(laatstesnelheid[0], 3); i++) {
		display.print(" ");
	}
	display.print(laatstesnelheid[0]);  display.setTextSize(1);
	display.setCursor(61, 32);
	display.print("km/u");
}
byte spaties(int nummer, byte digits) {
	/*
	Berekend aantal spaties in een txt om deze rechts uit te kunnen lijnen
	*/
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
	ss = PINB; ss = ss << 4;	ss = ss >> 4; //clear bits 4,5,6,7
	changed = switchstatus ^ ss;
	if (changed > 0) {
		for (byte i = 0; i < 4; i++) {
			if (changed & (1 << i)) {
				if (ss & (1 << i)) {
					SW_off(i); //clear scroll counters
				}
				else {
					SW_on(i);
				}
			}
		}
	}
	else if (GPIOR0 & (1 << 2)) { //dus geen verandering, scrollen. Alleen in program mode
		//alleen sw 2 en 3 kunnen scrollen	
		for (byte s = 2; s < 4; s++) {
			if (~ss & (1 << s)) {
				if (switchcount[s] < 10) {
					switchcount[s]++;
				}
				else {
					SW_on(s);
				}
			}
		}
	}
	switchstatus = ss;
}
void SW_off(byte sw) {
	switchcount[sw] = 0;
}
void SW_on(byte sw) {
	switch (sw) {
	case 0: //Knop1
		GPIOR0 ^= (1 << 2);//wisselt het scherm
		break;
	case 1: //Knop 2
		if (~GPIOR0 & (1 << 2)) { //in bedrijf
			MEM_reg ^= (1 << 0);
		}
		else { //program scherm V3.01 twee soorten
			DP_level++;
			switch (preset[p].mode) {
			case 0:
				if (DP_level > prglvl)DP_level = 0;
				break;
			case 1:
				//level gaat nu op en neer dus ff handmatig telkens aanpassen
				switch (DP_level) {
				case 2:
					DP_level = 20;//instellen trajectlengte 
					break;
				case 21:
					DP_level = 8; //schaal instellen
					break;
				case 9:
					DP_level = 0; //instellen preset nummer
					break;
				}
			}
		}
		break;

	case 2: //Knop 3
		if (~GPIOR0 & (1 << 2)) { //in berijf
			if (preset[p].mode & (1 << 0)) { //traject mode
				traject(1);
			}
			else { //rotatie mode
				MEM_reg ^= (1 << 1);
			}

		}
		else {
			switch (DP_level) {
			case 0:
				if (p > 0) p--;
				break;
			case 1: //mode
				preset[p].mode ^= (1 << 0); //flip alleen bit 0
				//symbol++; //show volgende symbool
				break;
			case 2: //rm1 roller
				if (preset[p].dr1 > 1) preset[p].dr1--;
				break;
			case 3://rm1 Wiel
				if (preset[p].dw1 > 1) preset[p].dw1--;
				break;
			case 4://rm2 roller
				if (preset[p].dr2 > 1) preset[p].dr2--;
				break;
			case 5://rm2 wiel
				if (preset[p].dw2 > 1) preset[p].dw2--;
				break;
			case 6://rm1 pulsen per rotatie
				if (preset[p].puls1 > 1) preset[p].puls1--;
				break;
			case 7://rm2 pulsen per rotatie
				if (preset[p].puls2 > 1) preset[p].puls2--;
				break;
			case 8:// schaal 1: value
				if (preset[p].schaal > 1) preset[p].schaal--;
				break;
			case 9:
				if (preset[p].precisie > 1) preset[p].precisie--;
				break;
			case 10: //USB type
				if (preset[p].usb > 0) preset[p].usb--;
				break;
			case 11: //ijken, doorsnede rollerwiel zoals Itrain/Speedcat dit verwerkt
				if (preset[p].Dsc > 1)preset[p].Dsc--;
				break;
			case 12:
				if (preset[p].pulsIt > 1)preset[p].pulsIt--;
				break;

			}
		}
		break;
	case 3: //Knop 4
		if (~GPIOR0 & (1 << 2)) {
			if (preset[p].mode & (1 << 0)) { //traject mode
				traject(2);
			}
			else { //rotatie mode
				if (~MEM_reg & (1 << 0)) { //KMh mode
					MEM_reg ^= (1 << 3); //snelheid op schaal of 1:1
				}
				else { //RPM mode
					MEM_reg ^= (1 << 2); //toon welk RPM roller/wiel getoond wordt
				}

			}
		}
		else {
			switch (DP_level) {
			case 0:
				if (p < 5)p++;
				break;
			case 1: //mode
				//symbol--; //show voorgaand symbool
				preset[p].mode ^= (1 << 0); //flip bit 0 0-1 
				break;
			case 2: //rm1 roller
				if (preset[p].dr1 < 250) preset[p].dr1++;
				break;
			case 3://rm1 Wiel
				if (preset[p].dw1 < 250) preset[p].dw1++;
				break;
			case 4://rm2 roller
				if (preset[p].dr2 < 250) preset[p].dr2++;
				break;
			case 5://rm2 wiel
				if (preset[p].dw2 < 250) preset[p].dw2++;
				break;
			case 6://rm1 pulsen per rotatie
				if (preset[p].puls1 < 250) preset[p].puls1++;
				break;
			case 7://rm2 pulsen per rotatie
				if (preset[p].puls2 < 250) preset[p].puls2++;
				break;
			case 8:// schaal 1: value
				if (preset[p].schaal < 250) preset[p].schaal++;
				break;
			case 9: //precisie
				if (preset[p].precisie < 12)preset[p].precisie++;
				break;
			case 10:
				if (preset[p].usb < 2)preset[p].usb++;
				break;
			case 11:
				if (preset[p].Dsc < 250) preset[p].Dsc++;
				break;
			case 12:
				if (preset[p].pulsIt < 24)preset[p].pulsIt++;
				break;
			}
		}
		break;
	}
	if (~GPIOR0 & (1 << 2))MEM_write();
}
void SC_exe() {
	char txt[15]; //11 minimaal nodig
	//stuurt boodschap over USB poort ten behoeve van SpeedCat (Itrain compatibel)
	/*
	Berekening.
	Itrain/SpeedCat app wil 8 pulsen in 1 rotatie.
	Afgelegde afstand voor 1 puls is een constante, altijd gebruik van Zeller rolletje ongeveer 6mm diameter
	Delen van de ingestelde diameter voor RM1 door deze constante omtrek van de Zeller rol.
	Deze maat, zeller rol,  in tienden van mm, en instelbaar om de beide meetmethoden op elkaar te kunnen ijken.
	Afstand delen door het aantal pulsen in een rotatie van RM1
	Afstand vermenigvuldigen met het aantal te leveren pulsen aan Itrain/Speedmeter
	Deze waarde maal twee omdat Itrain/Speedmeter beide flanken van de puls tellen, Speedmeter
	teld alleen de positieve flank.
	*/
	unsigned int pls = 0;
	float dia; //aantal pulsen uitrekenen
	dia = preset[p].dr1; //diameter roller 1
	dia = dia / preset[p].Dsc * 10; //roller Dsc = in tienden mm
	dia = dia / preset[p].puls1;
	pls = (countSC * preset[p].pulsIt * 2) * dia;
	sprintf(txt, "*%04d;V3.0%s", pls, "%");
	//sprintf(txtSC, "*%04d;Speedmeter%s", pls, "%"); //voorbeeld andere txt, niet getest op Itrain
	//format: * start?  Vereist
	//puls in 4 decimalen als CHAR 
	//txt: V3.0  willekeurige tekst, testen of Itrain deze txt toont
	//txtvariabel: % einde, Vereist. misschien zijn er meerder parameters hier? of alleen nodig voor de tekst? 
	Serial.println(txt); //uitvoer naar Itrain/SpeedCat
	countSC = 0;
}
void SD_exe() {
	//berekend en sends message over serial port to SympleDyno
	/*

	SimpleDyno verwacht een boodschap die de tijdsduur aangeeft tussen twee pulsen
	Het verwacht, de tijd in micros
	De starttijd van de puls in micros, de eindtijd van de puls in micros
	Verder een rits meetwaardes die niet in project Speedmeter zijn meegenomen.
	Aan de hand van de tijden kan SimpleDyno rotaties en snelheden uitrekenen.
	De op SpeedMeter getoonde waardes worden ook op deze manier verkregen.
	*/
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
	//Serial.flush(); //pauzeerd programma totdat de send buffer leeg is. Gebruiken hier? Waarom?
}
void calc(bool rm) { //called from ISR's
	//maakt berekeningen, gemiddelde van twee laatste metingen
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
