// "Light Controller"
//biblioteki systemowe
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>

//biblioteki peryferiów
#include "LCD_ATM8/lcd44780.h"
#include "I2C_TWI/i2c_twi.h"
//#################################################################################################################
///                                           	/* MAKRODEFINICJE */
//#################################################################################################################
#define	Lprz	(1<<PC3) //deklaracja przycisku lewo
#define	Rprz	(1<<PC2) //deklaracja przycisku prawo
#define	EXprz	(1<<PC0) //deklaracja przycisku exit
#define	ENprz	(1<<PC1) //deklaracja przycisku enter

//konieczne zmiany w przypadku wyboru innego portu poni¿ej i na pocz¹tku funkcji main
#define	LKEY	!(PINC&Lprz)
#define	RKEY	!(PINC&Rprz)
#define	EXKEY	!(PINC&EXprz)
#define	ENKEY	!(PINC&ENprz)
#define NOKEY	((PINC&(0x0F))==(0x0F))// szesnastkowo/ jedynkami oznacza siê bity do których pod³ s¹ przyciski

#define E_IDDLE		0
#define E_UP 		1
#define E_DOWN 		2
#define E_OK 		3
#define E_EXIT 		4

#define PCF8583_ADDR 0xA0



//#################################################################################################################
///                                           	/* DEKLARACJE GLOBALNE */
//#################################################################################################################

///F U N K C J E
//<menu>
void change_menu();
void keyread (void);	 //funkcja odczytujaca stan przyciskow
void obsluga_menu (void);//scalenie keyread i change_menu z timerem programowym
//<rtc>
uint8_t dec2bcd(uint8_t dec);// konwersja liczby dziesiêtnej na BCD
uint8_t bcd2dec(uint8_t bcd);// konwersja liczby BCD na dziesiêtn¹
void set_time (uint8_t ust_godz, uint8_t ust_min, uint8_t ust_sec);//ustawianie czasu
void wysw_czas (void);
//<sterowanie œwiat³em>
void oswietlenie (void);
void oswietlenie_1 (void);
void oswietlenie_2 (void);
void oswietlenie_3 (void);
void priorytety (void);

//<funkcje wykonawcze w menu>
void IDLE 	   (unsigned char event);
void ust_jas_1 (unsigned char event);
void ust_jas_2 (unsigned char event);
void ust_jas_3 (unsigned char event);
void ust_zak_1(unsigned char event);
void ust_zak_2(unsigned char event);
void ust_zak_3(unsigned char event);
void ust_czas (unsigned char event);
///Z M I E N N E
volatile unsigned char	current_menu = 0;
volatile unsigned char	menu_event = E_IDDLE;
uint8_t keylock;
volatile uint16_t Timer1, Timer2, Timer3;	// timery programowe
enum {ss=1, mm, hh};						// zmienna wyliczeniowa dla bufora odczytu RTC
uint8_t bufor[4];							// rezerwacja bufora 4 bajty dla RTC
volatile uint8_t int0_flag=1;			    // flaga zmieniana w przerwaniu i sprawdzana w pêtli g³ównej
volatile uint8_t sekundy, minuty, godziny;  // zmienna przechowuj¹ca aktualn¹ wartoœæ godziny
enum {zwolniony,zakres1,zakres2,zakres3};   // typ wyliczeniowy u³atwiaj¹cy rozpoznanie priorytetu zakresów
uint8_t PRIORYTET;							// zmienna pomocnicza informuj¹ca o rezerwacji przedzia³u czasu
uint8_t jasnosc1,jasnosc2,jasnosc3;			// zmienne przechowuj¹ce jasnoœæ dla poszczególnych zakresów
// zmienne przechowuj¹ce granice zakresów--------------
uint8_t godziny1,minuty1,godziny2,minuty2,godziny3,minuty3,godziny1k,minuty1k,godziny2k,minuty2k,godziny3k,minuty3k;
//-----------------------------------------------------

///STA£E, STRUKTURY
const uint8_t fastar=255, slowar=254;		// sta³e dla autorepeat'a
// sta³e tekstowe dla LCD------------------------------
const unsigned char M_0_0a[]  PROGMEM="LIGHT CONTROLLER";
const unsigned char M_0_0b[]  PROGMEM="                ";

const unsigned char M_0_1a[]  PROGMEM="USTAWIENIA      ";
const unsigned char M_0_1b[]  PROGMEM="STEROWANIA      ";

const unsigned char M_0_2a[]  PROGMEM="USTAWIENIA      ";
const unsigned char M_0_2b[]  PROGMEM="ZEGARA          ";

const unsigned char M_0_3a[]  PROGMEM="POMOC           ";
const unsigned char M_0_3b[]  PROGMEM="                ";

const unsigned char M_1_0a[]  PROGMEM="USTAW GRANICE   ";
const unsigned char M_1_0b[]  PROGMEM="CZASU           ";

const unsigned char M_1_1a[]  PROGMEM="USTAW JASNOSC   ";
const unsigned char M_1_1b[]  PROGMEM="DLA CZASOW      ";

const unsigned char M_2_0a[]  PROGMEM="ZAKRES CZASU    ";
const unsigned char M_2_0b[]  PROGMEM="   >>1<<        ";

const unsigned char M_2_1a[]  PROGMEM="ZAKRES CZASU    ";
const unsigned char M_2_1b[]  PROGMEM="   >>2<<        ";

const unsigned char M_2_2a[]  PROGMEM="ZAKRES CZASU    ";
const unsigned char M_2_2b[]  PROGMEM="   >>3<<        ";

const unsigned char M_2_3a[]  PROGMEM="JASNOSC DLA     ";
const unsigned char M_2_3b[]  PROGMEM="CZASU >>1<<     ";

const unsigned char M_2_4a[]  PROGMEM="JASNOSC DLA     ";
const unsigned char M_2_4b[]  PROGMEM="CZASU >>2<<     ";

const unsigned char M_2_5a[]  PROGMEM="JASNOSC DLA     ";
const unsigned char M_2_5b[]  PROGMEM="CZASU >>3<<     ";
//-----------------------------------------------------

// definicja typu strukturalnego dla menu--------------
typedef struct
{
	unsigned char next_state[5];							//przejœcia do nastêpnych stanów
	void (*callback)(unsigned char event);					//funkcja zwrotna
	const unsigned char* first_line;						//tekst dla 1. linii LCD
	const unsigned char* second_line;						//tekst dla 2. linii LCD
} menu_item;
//-----------------------------------------------------

//callback w definicji i deklaracji musi mieæ przekazywany argument (unsigned char event)
const menu_item menu[] = {
	// IL UP DN OK EX  funkcja              text_lcd     numer zdarzenia
	{{ 0, 0, 1, 0, 0}, NULL,			    M_0_0a,NULL  },//(0) idle menu
	{{ 0, 1, 2, 4, 0}, NULL, 				M_0_1a,M_0_1b},//(1) USTAWIENIA STEROWANIA
	{{ 0, 1, 3,18, 0}, NULL, 				M_0_2a,M_0_2b},//(2) USTAW ZEGAR
	{{ 0, 2, 3, 3, 0}, NULL, 				M_0_3a,M_0_3b},//(3) POMOC
	{{ 0, 4, 5, 6, 1}, NULL,        		M_1_0a,M_1_0b},//(4) USTAWIANIE GRANIC CZASOW
	{{ 0, 4, 5, 9, 1}, NULL,            	M_1_1a,M_1_1b},//(5) USTAWIANIE JASNOŒCI
	{{ 0, 6, 7,12, 4}, NULL,		  		M_2_0a,M_2_0b},//(6) ZAKRES CZASU 1
	{{ 0, 6, 8,13, 4}, NULL,            	M_2_1a,M_2_1b},//(7) ZAKRES CZASU 2
	{{ 0, 7, 8,14, 4}, NULL,            	M_2_2a,M_2_2b},//(8) ZAKRES CZASU 3
	{{ 0, 9,10,15, 5}, NULL,            	M_2_3a,M_2_3b},//(9) JASNOŒÆ DLA CZASU 1
	{{ 0, 9,11,16, 5}, NULL,            	M_2_4a,M_2_4b},//(10) JASNOŒÆ DLA CZASU 2
	{{ 0,10,11,17, 5}, NULL,            	M_2_5a,M_2_5b},//(11) JASNOŒÆ DLA CZASU 3
	{{ 0, 4, 4, 4, 6}, ust_zak_1,           NULL  ,NULL  },//(12) CALLBACK ZAKRES CZASU 1
	{{ 0, 4, 4, 4, 7}, ust_zak_2,           NULL  ,NULL  },//(13) CALLBACK ZAKRES CZASU 2
	{{ 0, 4, 4, 4, 8}, ust_zak_3,           NULL  ,NULL  },//(14) CALLBACK ZAKRES CZASU 3
	{{ 0, 5, 5, 9, 9}, ust_jas_1,          	NULL  ,NULL  },//(15) CALLBACK JASNOŒÆ DLA CZASU 1
	{{ 0, 5, 5,10,10}, ust_jas_2,           NULL  ,NULL  },//(16) CALLBACK JASNOŒÆ DLA CZASU 2
	{{ 0, 5, 5,11,11}, ust_jas_3,           NULL  ,NULL  },//(17) CALLBACK JASNOŒÆ DLA CZASU 3
	{{ 0, 2, 2, 2, 2}, ust_czas ,           NULL  ,NULL  },//(18) CALLBACK USTAWIANIE CZASU
};


//#################################################################################################################
///                                           	/* G£ÓWNA FUNKCJA PROGRAMU */
//#################################################################################################################
int main(void)
{
//ustawienie predkosci magistrali I2C
	i2cSetBitrate(100);

//zerowanie zmiennych
	jasnosc1=0;
	jasnosc2=0;
	jasnosc3=0;

//Konfiguracja portów dla przycisków-------------------------------------------------
	DDRC &= ~(Lprz | Rprz | EXprz | ENprz); //przyciski jako wejscia
	PORTC |= ( Rprz | EXprz | ENprz | Lprz );//podciagniecie pinów do VCC
//-----------------------------------------------------------------------------------

//Konfiguracja TIMERÓW SPRZÊTOWYCH  -------------------------------------------------
	//TIMER2
	// Przerwania co 10ms dla zegarów systemowych (tykacze)
	TCCR2 	= (1<<WGM21);			// tryb pracy CTC
	TCCR2 	= (1<<CS22)|(1<<CS20);	// preskaler = 1024
	OCR2 	= 108;					// przerwanie porównania co 10ms (100Hz)
	TIMSK 	= (1<<OCIE2);			// Odblokowanie przerwania CompareMatch

    // Przerwanie INT0 (odczyt RTC)
	MCUCR |= (1<<ISC01);	// wyzwalanie zboczem opadaj¹cym
	GICR |= (1<<INT0);		// odblokowanie przerwania
	PORTD |= (1<<PD2);		// podci¹gniêcie pinu INT0 do VCC

	//TIMER1
	//SPRZÊTOWY PWM - 1 KANA£ OCR1A (PB1)
	// ustawienie koñcówki OCR1A (PB1) sprzêtowy PWM jako WYJŒCIE
	DDRB |= (1<<PB1);
	// ustawienie w tryb Fast PWM 8bit
	TCCR1A |= (1<<WGM10);
	TCCR1B |= (1<<WGM12);
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1);				// clear at TOP
    TCCR1B |= (1<<CS10) | (1<<CS11);					// preskaler = 1
	OCR1A=255;							// wygaszenie diody w kanale PWM

lcd_init();            //inicjalizacja LCD
menu_event= E_IDDLE;   //poczatkowa wartosc dla menu
change_menu();         //aktualizacja menu
sei();                 // globalne za³¹czenie przerwañ

//------------------------------Wstêpny odczyt zegara----------------------------------
TWI_read_buf( PCF8583_ADDR, 0x01, 4, bufor ); //czytaj z uk³adu RTC 4 bajty od adresu 0x01
sekundy = bcd2dec( bufor[ss] );
minuty = bcd2dec( bufor[mm] );
godziny = bcd2dec( bufor[hh] );
//-------------------------------------------------------------------------------------

PRIORYTET=zwolniony;   //czyszczenie priorytetu dla zakresów

//#################################################################################################################
///                                           	/* PÊTLA NIESKOÑCZONA WHILE(1) */
//#################################################################################################################
	while(1)
	{
		obsluga_menu();                    //funkcja zmieniaj¹ca stany menu w zale¿noœci od wciœniêtego przycisku
		if (current_menu==0)			   //obs³uga wyswietlania czasu i sterowania oœwietleniem jeœli poza menu
		{
		wysw_czas();  					   //wyswietlanie zegara RTC dla ekranu g³ównego
		priorytety();                      //wygaszenie oœwietlenia w zale¿nosci od zarezerwowanego priorytetu
		PRIORYTET=zwolniony;               //czyszczenie priorytetu
		oswietlenie();                     //sterowanie oœwietleniem
		}
	}//end_while(1)
}//end_main(void)


//#################################################################################################################
///                                           	/* DEFINICJE FUNKCJI */
//#################################################################################################################
//funkcja zmieniaj¹ca menu
void change_menu()
{
		if (!Timer3){
		Timer3=100;							//Tykacz co kilkaset ms
		//przejdz do nastepnego stanu
		current_menu = menu[current_menu].next_state[menu_event];	//stan zale¿ny od wciœniêtego przycisku obs³uga w funkcji keyread()

		//wyœwietl 1-sz¹ liniê
		if (menu[current_menu].first_line)
		{
			lcd_locate(0,0);
			lcd_str_P((char*)menu[current_menu].first_line);
		}
		//wyœwietl 2-g¹ liniê
		if (menu[current_menu].second_line)
		{
			lcd_locate(1,0);
			lcd_str_P((char*)menu[current_menu].second_line);
		}
		//wywolaj funkcje zwrotna
		if (menu[current_menu].callback) // jeœli zarejestrowano jak¹œ funkcjê
			menu[current_menu].callback(menu_event);
		}//end_if_(!Timer3)
}

//czytaj klawiature
void keyread (void)
{
	unsigned char key;
	if (LKEY)  key = E_DOWN;
	if (RKEY)  key = E_UP;
	if (ENKEY) key = E_OK;
	if (EXKEY) key = E_EXIT;
	if (NOKEY) key = E_IDDLE;
	menu_event = key;
}

//obs³uga menu na timerze programowym
void obsluga_menu (void){
	keyread();
	if(!Timer2){
	Timer2=10;					//Tykacz co kilkadziesi¹t ms

	if (menu_event)// je¿eli menu_event jest ró¿ne od 0 czyli wciœniêty jakiœ przycisk
		{
		change_menu();
		}
	}
}

// -----------------------------------FUNKCJA IDLE----------------------------------------
void IDLE (unsigned char event){
}

// -----------------------------------ustawianie jasnoœci 1-------------------------------
void ust_jas_1 (unsigned char event){
	uint8_t plock=0, llock=0;
    uint8_t a=0;
    a=jasnosc1;
    lcd_cls();
    //pêtla obs³ugi poziomu menu
    while (!EXKEY) { // dopóki u¿ytkownik nie nacisnie EXIT
    	//wyswietlanie opisu
		lcd_locate(0,0);
		lcd_str("Jasnosc czas 1  ");
		lcd_locate(1,8);
		lcd_str("(0-255)");

		// formatownie wyswietlania zmiennej
		if (a<100){
			if(a>9){
				lcd_locate(1,0);
				lcd_str("0");
				lcd_locate(1,1);
				lcd_int(a);
			}
			if (a<10 && a!=0){
				lcd_locate(1,0);
				lcd_str("00");
				lcd_locate(1,2);
				lcd_int(a);
			}
			if (a==0){
				lcd_locate(1,0);
				lcd_str("OFF");
			}
		}else{
			lcd_locate(1,0);
			lcd_int(a);
		}

		//wyswietl ! jesli zmienna w pamieci rozni sie od ustawionej
		if (a!=jasnosc1){
			lcd_locate(1,3);
			lcd_str("!");
		} else {
			lcd_locate(1,3);
			lcd_str(" ");
		}

		// inkrementacja i dekrementacja zmiennej
		if (  RKEY  &&  !plock){ // inkrementacja z prostym autorepeat'em
			plock = fastar;
			if (a!=255){
			a=a+5;
			}
		}else if (plock) plock++;

		if (  LKEY  &&  !llock){ // dekrementacja z prostym autorepeat'em
			llock = fastar;
			if (a!=0){
			a=a-5;
			}
		}else if (llock) llock++;
		OCR1A=255-a;
		if (ENKEY) jasnosc1 = a; // potwierdzenie zmiany wartoœci ENTEREM

	}
}


// -----------------------------------ustawianie jasnoœci 2-------------------------------
void ust_jas_2 (unsigned char event){
	uint8_t plock=0, llock=0;
    uint8_t a=0;
    a=jasnosc2;
    lcd_cls();
    //pêtla obs³ugi poziomu menu
    while (!EXKEY) { // dopóki u¿ytkownik nie nacisnie EXIT
    	//wyswietlanie opisu
		lcd_locate(0,0);
		lcd_str("Jasnosc czas 2  ");
		lcd_locate(1,8);
		lcd_str("(0-255)");

		// formatownie wyswietlania zmiennej
		if (a<100){
			if(a>9){
				lcd_locate(1,0);
				lcd_str("0");
				lcd_locate(1,1);
				lcd_int(a);
			}
			if (a<10 && a!=0){
				lcd_locate(1,0);
				lcd_str("00");
				lcd_locate(1,2);
				lcd_int(a);
			}
			if (a==0){
				lcd_locate(1,0);
				lcd_str("OFF");
			}
		}else{
			lcd_locate(1,0);
			lcd_int(a);
		}

		//wyswietl ! jesli zmienna w pamieci rozni sie od ustawionej
		if (a!=jasnosc2){
			lcd_locate(1,3);
			lcd_str("!");
		} else {
			lcd_locate(1,3);
			lcd_str(" ");
		}

		// inkrementacja i dekrementacja zmiennej
		if (  RKEY  &&  !plock){ // inkrementacja z prostym autorepeat'em
			plock = fastar;
			if (a!=255){
			a=a+5;
			}
		}else if (plock) plock++;

		if (  LKEY  &&  !llock){ // dekrementacja z prostym autorepeat'em
			llock = fastar;
			if (a!=0){
			a=a-5;
			}
		}else if (llock) llock++;
		OCR1A=255-a;
		if (ENKEY) jasnosc2 = a; // potwierdzenie zmiany wartoœci ENTEREM
	}
}
// -----------------------------------ustawianie jasnoœci 3-------------------------------
void ust_jas_3 (unsigned char event){
	uint8_t plock=0, llock=0;
    uint8_t a=0;
    a=jasnosc3;
//    a=a-5;
    lcd_cls();
    //pêtla obs³ugi poziomu menu
    while (!EXKEY) { // dopóki u¿ytkownik nie nacisnie EXIT
    	//wyswietlanie opisu
		lcd_locate(0,0);
		lcd_str("Jasnosc czas 3  ");
		lcd_locate(1,8);
		lcd_str("(0-255)");

		// formatownie wyswietlania zmiennej
		if (a<100){
			if(a>9){
				lcd_locate(1,0);
				lcd_str("0");
				lcd_locate(1,1);
				lcd_int(a);
			}
			if (a<10 && a!=0){
				lcd_locate(1,0);
				lcd_str("00");
				lcd_locate(1,2);
				lcd_int(a);
			}
			if (a==0){
				lcd_locate(1,0);
				lcd_str("OFF");
			}
		}else{
			lcd_locate(1,0);
			lcd_int(a);
		}

		//wyswietl ! jesli zmienna w pamieci rozni sie od ustawionej
		if (a!=jasnosc3){
			lcd_locate(1,3);
			lcd_str("!");
		} else {
			lcd_locate(1,3);
			lcd_str(" ");
		}

		// inkrementacja i dekrementacja zmiennej
		if (  RKEY  &&  !plock){ // inkrementacja z prostym autorepeat'em
			plock = fastar;
			if (a!=255){
			a=a+5;
			}
		}else if (plock) plock++;

		if (  LKEY  &&  !llock){ // dekrementacja z prostym autorepeat'em
			llock = fastar;
			if (a!=0){
			a=a-5;
			}
		}else if (llock) llock++;
		OCR1A=255-a;
		if (ENKEY) jasnosc3 = a; // potwierdzenie zmiany wartoœci ENTEREM

	}
}
// -----------------------------------ustawianie zakresu 1-------------------------------
void ust_zak_1 (unsigned char event){
	uint8_t flaga_stanu=0;
	static uint8_t first_loop_lock=0;
	uint8_t plock=0, llock=0, entlock=0;
	uint8_t godziny1tmp,minuty1tmp,godziny1ktmp,minuty1ktmp; // zmienne tymczasowe dla ustawiania czasów

	//przepisanie stanu zmiennych globalnych do zmiennych tymczasowych
	godziny1tmp=godziny1;
	minuty1tmp=minuty1;
	godziny1ktmp=godziny1k;
	minuty1ktmp=minuty1k;

	first_loop_lock=1;
	lcd_cls();
    //pêtla obs³ugi poziomu menu
    while (!EXKEY) { // dopóki u¿ytkownik nie nacisnie EXIT

    	//wyswietlanie opisu i sta³ych znaków
		lcd_locate(0,0);
		lcd_str("Zakr1 od...do  ");
		lcd_locate(1,3);
		lcd_str(":");
		lcd_locate(1,7);
		lcd_str("|");
		lcd_locate(1,11);
		lcd_str(":");

		//sygnalizacja wartoœci edytowanej
		if (flaga_stanu==0){
			lcd_locate(1,0);
			lcd_str(">");
		}

		if (flaga_stanu==1){
			lcd_locate(1,6);
			lcd_str("<");
		}

		if (flaga_stanu==2){
			lcd_locate(1,8);
			lcd_str(">");
		}

		if (flaga_stanu==3){
			lcd_locate(1,14);
			lcd_str("<");
		}

		// wyswietlanie zmiennych
		if (godziny1tmp<10){
			lcd_locate(1,1);
			lcd_int(0);
			lcd_locate(1,2);
		}
		if (godziny1tmp>9)lcd_locate(1,1);
		lcd_int(godziny1tmp);

		if(minuty1tmp<10){
			lcd_locate(1,4);
			lcd_int(0);
			lcd_locate(1,5);
		}
		if(minuty1tmp>9)lcd_locate(1,4);
		lcd_int(minuty1tmp);

		if (godziny1ktmp<10){
			lcd_locate(1,9);
			lcd_int(0);
			lcd_locate(1,10);
		}
		if (godziny1ktmp>9)lcd_locate(1,9);
		lcd_int(godziny1ktmp);

		if(minuty1ktmp<10){
			lcd_locate(1,12);
			lcd_int(0);
			lcd_locate(1,13);
		}
		if(minuty1ktmp>9)lcd_locate(1,12);
		lcd_int(minuty1ktmp);


		// inkrementacja i dekrementacja zmiennej
		if (  RKEY  &&  !plock){ // inkrementacja z prostym autorepeat'em
			if (flaga_stanu==0 || flaga_stanu==2)plock = slowar;// prêdkoœæ autorepeat'a dla godzin
			if (flaga_stanu==1 || flaga_stanu==3)plock = fastar;// prêdkoœæ autorepeat'a dla minut

			switch (flaga_stanu)
			{
			case 0:
				if (godziny1tmp<23)godziny1tmp++;
			break;
			case 1:
				if (minuty1tmp<59)minuty1tmp++;
			break;
			case 2:
				if (godziny1ktmp<23)godziny1ktmp++;
			break;
			case 3:
				if (minuty1ktmp<59)minuty1ktmp++;
			break;
			}
		}else if (plock) plock++;

		if (  LKEY  &&  !llock){ // dekrementacja z prostym autorepeat'em
			if (flaga_stanu==0 || flaga_stanu==2)llock = slowar;// prêdkoœæ autorepeat'a dla godzin
			if (flaga_stanu==1 || flaga_stanu==3)llock = fastar;// prêdkoœæ autorepeat'a dla minut
			switch (flaga_stanu)
			{
			case 0:
				if (godziny1tmp>0)godziny1tmp--;
			break;
			case 1:
				if (minuty1tmp>0)minuty1tmp--;
			break;
			case 2:
				if (godziny1ktmp>0)godziny1ktmp--;
			break;
			case 3:
				if (minuty1ktmp>0)minuty1ktmp--;
			break;
			}
		}else if (llock) llock++;

		if (ENKEY && !entlock && !first_loop_lock){
			entlock=slowar;
			lcd_cls();
			flaga_stanu++;

			//przepisanie stanu zmiennych tymczasowych do zmiennych globalnych
			godziny1=godziny1tmp;
			minuty1=minuty1tmp;
			godziny1k=godziny1ktmp;
			minuty1k=minuty1ktmp;

			if (flaga_stanu > 3) flaga_stanu=0;
		} else if(!ENKEY && entlock) entlock++;
		if (!ENKEY) first_loop_lock=0;// kasowanie blokady inkrementacji flagi stanu podczas wejœcia do menu
	}
}

// -----------------------------------ustawianie zakresu 2-------------------------------
void ust_zak_2 (unsigned char event){
	uint8_t flaga_stanu=0;
	static uint8_t first_loop_lock=0;
	uint8_t plock=0, llock=0, entlock=0;
	uint8_t godziny2tmp,minuty2tmp,godziny2ktmp,minuty2ktmp; // zmienne tymczasowe dla ustawiania czasów

	//przepisanie stanu zmiennych globalnych do zmiennych tymczasowych
	godziny2tmp=godziny2;
	minuty2tmp=minuty2;
	godziny2ktmp=godziny2k;
	minuty2ktmp=minuty2k;

	first_loop_lock=1;
	lcd_cls();
    //pêtla obs³ugi poziomu menu
    while (!EXKEY) { // dopóki u¿ytkownik nie nacisnie EXIT
    	//wyswietlanie opisu i sta³ych znaków
		lcd_locate(0,0);
		lcd_str("Zakr2 od...do  ");
		lcd_locate(1,3);
		lcd_str(":");
		lcd_locate(1,7);
		lcd_str("|");
		lcd_locate(1,11);
		lcd_str(":");

		//sygnalizacja wartoœci edytowanej
		if (flaga_stanu==0){
			lcd_locate(1,0);
			lcd_str(">");
		}

		if (flaga_stanu==1){
			lcd_locate(1,6);
			lcd_str("<");
		}

		if (flaga_stanu==2){
			lcd_locate(1,8);
			lcd_str(">");
		}

		if (flaga_stanu==3){
			lcd_locate(1,14);
			lcd_str("<");
		}

		// wyswietlanie zmiennych
		if (godziny2tmp<10){
			lcd_locate(1,1);
			lcd_int(0);
			lcd_locate(1,2);
		}
		if (godziny2tmp>9)lcd_locate(1,1);
		lcd_int(godziny2tmp);

		if(minuty2tmp<10){
			lcd_locate(1,4);
			lcd_int(0);
			lcd_locate(1,5);
		}
		if(minuty2tmp>9)lcd_locate(1,4);
		lcd_int(minuty2tmp);

		if (godziny2ktmp<10){
			lcd_locate(1,9);
			lcd_int(0);
			lcd_locate(1,10);
		}
		if (godziny2ktmp>9)lcd_locate(1,9);
		lcd_int(godziny2ktmp);

		if(minuty2ktmp<10){
			lcd_locate(1,12);
			lcd_int(0);
			lcd_locate(1,13);
		}
		if(minuty2ktmp>9)lcd_locate(1,12);
		lcd_int(minuty2ktmp);


		// inkrementacja i dekrementacja zmiennej
		if (  RKEY  &&  !plock){ // inkrementacja z prostym autorepeat'em
			if (flaga_stanu==0 || flaga_stanu==2)plock = slowar;// prêdkoœæ autorepeat'a dla godzin
			if (flaga_stanu==1 || flaga_stanu==3)plock = fastar;// prêdkoœæ autorepeat'a dla minut

			switch (flaga_stanu)
			{
			case 0:
				if (godziny2tmp<23)godziny2tmp++;
			break;
			case 1:
				if (minuty2tmp<59)minuty2tmp++;
			break;
			case 2:
				if (godziny2ktmp<23)godziny2ktmp++;
			break;
			case 3:
				if (minuty2ktmp<59)minuty2ktmp++;
			break;
			}
		}else if (plock) plock++;

		if (  LKEY  &&  !llock){ // dekrementacja z prostym autorepeat'em
			if (flaga_stanu==0 || flaga_stanu==2)llock = slowar;// prêdkoœæ autorepeat'a dla godzin
			if (flaga_stanu==1 || flaga_stanu==3)llock = fastar;// prêdkoœæ autorepeat'a dla minut
			switch (flaga_stanu)
			{
			case 0:
				if (godziny2tmp>0)godziny2tmp--;
			break;
			case 1:
				if (minuty2tmp>0)minuty2tmp--;
			break;
			case 2:
				if (godziny2ktmp>0)godziny2ktmp--;
			break;
			case 3:
				if (minuty2ktmp>0)minuty2ktmp--;
			break;
			}
		}else if (llock) llock++;

		if (ENKEY && !entlock && !first_loop_lock){
			entlock=slowar;
			lcd_cls();
			flaga_stanu++;

			//przepisanie stanu zmiennych tymczasowych do zmiennych globalnych
			godziny2=godziny2tmp;
			minuty2=minuty2tmp;
			godziny2k=godziny2ktmp;
			minuty2k=minuty2ktmp;

			if (flaga_stanu > 3) flaga_stanu=0;
		} else if(!ENKEY && entlock) entlock++;
		if (!ENKEY) first_loop_lock=0;// kasowanie blokady inkrementacji flagi stanu podczas wejœcia do menu

	}
}

// -----------------------------------ustawianie zakresu 3-------------------------------
void ust_zak_3 (unsigned char event){
	uint8_t flaga_stanu=0;
	static uint8_t first_loop_lock=0;
	uint8_t plock=0, llock=0, entlock=0;
	uint8_t godziny3tmp,minuty3tmp,godziny3ktmp,minuty3ktmp; // zmienne tymczasowe dla ustawiania czasów

	//przepisanie stanu zmiennych globalnych do zmiennych tymczasowych
	godziny3tmp=godziny3;
	minuty3tmp=minuty3;
	godziny3ktmp=godziny3k;
	minuty3ktmp=minuty3k;

	first_loop_lock=1;
	lcd_cls();
    //pêtla obs³ugi poziomu menu
    while (!EXKEY) { // dopóki u¿ytkownik nie nacisnie EXIT
    	//wyswietlanie opisu i sta³ych znaków
		lcd_locate(0,0);
		lcd_str("Zakr3 od...do  ");
		lcd_locate(1,3);
		lcd_str(":");
		lcd_locate(1,7);
		lcd_str("|");
		lcd_locate(1,11);
		lcd_str(":");

		//sygnalizacja wartoœci edytowanej
		if (flaga_stanu==0){
			lcd_locate(1,0);
			lcd_str(">");
		}

		if (flaga_stanu==1){
			lcd_locate(1,6);
			lcd_str("<");
		}

		if (flaga_stanu==2){
			lcd_locate(1,8);
			lcd_str(">");
		}

		if (flaga_stanu==3){
			lcd_locate(1,14);
			lcd_str("<");
		}

		// wyswietlanie zmiennych
		if (godziny3tmp<10){
			lcd_locate(1,1);
			lcd_int(0);
			lcd_locate(1,2);
		}
		if (godziny3tmp>9)lcd_locate(1,1);
		lcd_int(godziny3tmp);

		if(minuty3tmp<10){
			lcd_locate(1,4);
			lcd_int(0);
			lcd_locate(1,5);
		}
		if(minuty3tmp>9)lcd_locate(1,4);
		lcd_int(minuty3tmp);

		if (godziny3ktmp<10){
			lcd_locate(1,9);
			lcd_int(0);
			lcd_locate(1,10);
		}
		if (godziny3ktmp>9)lcd_locate(1,9);
		lcd_int(godziny3ktmp);

		if(minuty3ktmp<10){
			lcd_locate(1,12);
			lcd_int(0);
			lcd_locate(1,13);
		}
		if(minuty3ktmp>9)lcd_locate(1,12);
		lcd_int(minuty3ktmp);


		// inkrementacja i dekrementacja zmiennej
		if (  RKEY  &&  !plock){ // inkrementacja z prostym autorepeat'em
			if (flaga_stanu==0 || flaga_stanu==2)plock = slowar;// prêdkoœæ autorepeat'a dla godzin
			if (flaga_stanu==1 || flaga_stanu==3)plock = fastar;// prêdkoœæ autorepeat'a dla minut

			switch (flaga_stanu)
			{
			case 0:
				if (godziny3tmp<23)godziny3tmp++;
			break;
			case 1:
				if (minuty3tmp<59)minuty3tmp++;
			break;
			case 2:
				if (godziny3ktmp<23)godziny3ktmp++;
			break;
			case 3:
				if (minuty3ktmp<59)minuty3ktmp++;
			break;
			}
		}else if (plock) plock++;

		if (  LKEY  &&  !llock){ // dekrementacja z prostym autorepeat'em
			if (flaga_stanu==0 || flaga_stanu==2)llock = slowar;// prêdkoœæ autorepeat'a dla godzin
			if (flaga_stanu==1 || flaga_stanu==3)llock = fastar;// prêdkoœæ autorepeat'a dla minut
			switch (flaga_stanu)
			{
			case 0:
				if (godziny3tmp>0)godziny3tmp--;
			break;
			case 1:
				if (minuty3tmp>0)minuty3tmp--;
			break;
			case 2:
				if (godziny3ktmp>0)godziny3ktmp--;
			break;
			case 3:
				if (minuty3ktmp>0)minuty3ktmp--;
			break;
			}
		}else if (llock) llock++;

		if (ENKEY && !entlock && !first_loop_lock){
			entlock=slowar;
			lcd_cls();
			flaga_stanu++;

			//przepisanie stanu zmiennych tymczasowych do zmiennych globalnych
			godziny3=godziny3tmp;
			minuty3=minuty3tmp;
			godziny3k=godziny3ktmp;
			minuty3k=minuty3ktmp;

			if (flaga_stanu > 3) flaga_stanu=0;
		} else if(!ENKEY && entlock) entlock++;
		if (!ENKEY) first_loop_lock=0;// kasowanie blokady inkrementacji flagi stanu podczas wejœcia do menu

	}
}

void ust_czas (unsigned char event){
		uint8_t flaga_stanu=0;
		static uint8_t first_loop_lock=0;
		uint8_t plock=0, llock=0, entlock=0;
		uint8_t godzinatmp,minutatmp,sekundatmp; // zmienne tymczasowe dla ustawiania czasów

		//przepisanie stanu zmiennych globalnych do zmiennych tymczasowych
		godzinatmp=godziny;
		minutatmp=minuty;
		sekundatmp=0;

		first_loop_lock=1;
		lcd_cls();
	    //pêtla obs³ugi poziomu menu
	    while (!EXKEY) { // dopóki u¿ytkownik nie nacisnie EXIT
	    	//wyswietlanie opisu i sta³ych znaków
			lcd_locate(0,0);
			lcd_str("Ustaw godzine   ");
			lcd_locate(1,4);
			lcd_str(":");
			lcd_locate(1,9);
			lcd_str(":");

			//sygnalizacja wartoœci edytowanej
			if (flaga_stanu==0){
				lcd_locate(1,0);
				lcd_str(">");
				lcd_locate(1,3);
				lcd_str("<");
			}

			if (flaga_stanu==1){
				lcd_locate(1,5);
				lcd_str(">");
				lcd_locate(1,8);
				lcd_str("<");
			}

			if (flaga_stanu==2){
				lcd_locate(1,10);
				lcd_str(">");
				lcd_locate(1,13);
				lcd_str("<");
			}

			// wyswietlanie zmiennych
			if (godzinatmp<10){
				lcd_locate(1,1);
				lcd_int(0);
				lcd_locate(1,2);
			}
			if (godzinatmp>9)lcd_locate(1,1);
			lcd_int(godzinatmp);

			if(minutatmp<10){
				lcd_locate(1,6);
				lcd_int(0);
				lcd_locate(1,7);
			}
			if(minutatmp>9)lcd_locate(1,6);
			lcd_int(minutatmp);

			if (sekundatmp<10){
				lcd_locate(1,11);
				lcd_int(0);
				lcd_locate(1,12);
			}
			if (sekundatmp>9)lcd_locate(1,11);
			lcd_int(sekundatmp);


			// inkrementacja i dekrementacja zmiennej
			if (  RKEY  &&  !plock){ // inkrementacja z prostym autorepeat'em
				if (flaga_stanu==0 )plock = slowar;// prêdkoœæ autorepeat'a dla godzin
				if (flaga_stanu==1 || flaga_stanu==2)plock = fastar;// prêdkoœæ autorepeat'a dla minut

				switch (flaga_stanu)
				{
				case 0:
					if (godzinatmp<23)godzinatmp++;
				break;
				case 1:
					if (minutatmp<59)minutatmp++;
				break;
				case 2:
					if (sekundatmp<59)sekundatmp++;
				break;
				}
			}else if (plock) plock++;

			if (  LKEY  &&  !llock){ // dekrementacja z prostym autorepeat'em
				if (flaga_stanu==0 )llock = slowar;// prêdkoœæ autorepeat'a dla godzin
				if (flaga_stanu==1 || flaga_stanu==2)llock = fastar;// prêdkoœæ autorepeat'a dla minut
				switch (flaga_stanu)
				{
				case 0:
					if (godzinatmp>0)godzinatmp--;
				break;
				case 1:
					if (minutatmp>0)minutatmp--;
				break;
				case 2:
					if (sekundatmp>0)sekundatmp--;
				break;
				}
			}else if (llock) llock++;

			if (ENKEY && !entlock && !first_loop_lock){
				entlock=slowar;
				lcd_cls();
				flaga_stanu++;

				set_time(godzinatmp,minutatmp,sekundatmp);

				if (flaga_stanu > 2) flaga_stanu=0;
			} else if(!ENKEY && entlock) entlock++;
			if (!ENKEY) first_loop_lock=0;// kasowanie blokady inkrementacji flagi stanu podczas wejœcia do menu

		}
}

//------------------------------------------------------RTC--------------------------------------------------------
// konwersja liczby dziesiêtnej na BCD
uint8_t dec2bcd(uint8_t dec) {
return ((dec / 10)<<4) | (dec % 10);
}

// konwersja liczby BCD na dziesiêtn¹
uint8_t bcd2dec(uint8_t bcd) {
    return ((((bcd) >> 4) & 0x0F) * 10) + ((bcd) & 0x0F);
}

//ustaw czas
void set_time(uint8_t ust_godz, uint8_t ust_min, uint8_t ust_sec){
	uint8_t bufor[4];		// rezerwacja bufora 4 bajty
	bufor[0] = 0;			// setne czêœci sekundy
	bufor[1] = dec2bcd(ust_sec);	// sekundy
	bufor[2] = dec2bcd(ust_min);	// minuty
	bufor[3] = dec2bcd(ust_godz);	// godziny
	// zapis 4 bajtów z bufora pod adres 0x01 w pamiêci RAM naszego RTC
	TWI_write_buf( PCF8583_ADDR, 0x01, 4, bufor );
}

void wysw_czas (void){
	lcd_locate(0,0);
	lcd_str_P((char*)menu[current_menu].first_line);
	//czyszczenie pozosta³osci po menu
	lcd_locate(1,0);
	lcd_str("    ");
	lcd_locate(1,12);
	lcd_str("    ");
	// wyœwietlenie czasu na LCD
	lcd_locate(1,4);
	if( godziny < 10 ) lcd_str("0");
	lcd_int(godziny);
	lcd_str(":");
	if( minuty < 10 ) lcd_str("0");
	lcd_int(minuty);
	lcd_str(":");
	if( sekundy < 10 ) lcd_str("0");
	lcd_int(sekundy);
	lcd_locate(1,12);
	lcd_str(" ");
	}
//-------------------------------STEROWANIE OŒWIETLENIEM----------------------------------------------------------
void oswietlenie_1 (void){

if (godziny1<=godziny1k){//WARUNKI KIEDY CZAS "OD" JEST MNIEJSZY LUB RÓWNY CZASU "DO"
//WARUNKI:
	//[1] godzina aktualna wiêksza od pocz¹tkowej mniejsza od koñcowej
	//[2] godzina aktualna równa pocz¹tkowej i koñcowej, minuty w zakresie wiêksze równe "od" ale mniejsze od "do"
	//[3] godzina pocz¹tkowa ró¿na od koñcowej, godzina aktualna równa godzinie "od" , minuty aktualne wiêksze lub równe minutom "od"
	//[4] godzina pocz¹tkowa ró¿na od koñcowej, godzina aktualna równa godzinie "do" , minuty aktualne mniejsze ni¿ minuty "do"
	if (
	/*1*/	((godziny1<godziny) && (godziny<godziny1k))	   ||
	/*2*/	((godziny==godziny1) && (godziny==godziny1k) && (minuty>=minuty1) && (minuty<minuty1k))    ||
	/*3*/	((godziny1 != godziny1k)  &&  (godziny==godziny1) &&  (minuty>=minuty1))   ||
	/*4*/	((godziny1 != godziny1k) &&  (godziny==godziny1k)&&  (minuty<minuty1k))
		)//koniec warunków
	{
		OCR1A=255-jasnosc1; //PWM
		PRIORYTET = zakres1; // rezerwuj zakres
	}
}

if (godziny1>godziny1k){// WARUNKI KIEDY ZAKRES DOTYCZY PRZEJŒCIA PRZEZ 23:59 -> 00:00
//WARUNKI:
	//[1] przed pó³noc¹ i godzina aktualna wiêksza od pocz¹tkowej
	//[2] przed pó³noc¹, godzina aktualna równa pocz¹tkowej i czekamy na minuty
	//[3] po pó³nocy, godzina aktualna mniejsza od koñcowej
	//[4] po pó³nocy, godzina aktualna równa koñcowej, czekamy na minuty
	if(
	/*1*/	(godziny>godziny1)  ||
	/*2*/	((godziny==godziny1)  &&  minuty>=minuty1)  ||
	/*3*/	(godziny<godziny1k)  ||
	/*4*/	((godziny==godziny1k) && (minuty<minuty1k))
	  ) //koniec warunków
	{
		OCR1A=255-jasnosc1; //PWM
		PRIORYTET = zakres1; // rezerwuj zakres
	}
}
}//koniec funkcji


void oswietlenie_2 (void){

	if (godziny2<=godziny2k){//WARUNKI KIEDY CZAS "OD" JEST MNIEJSZY LUB RÓWNY CZASU "DO"
	//WARUNKI:
		//[1] godzina aktualna wiêksza od pocz¹tkowej mniejsza od koñcowej
		//[2] godzina aktualna równa pocz¹tkowej i koñcowej, minuty w zakresie wiêksze równe "od" ale mniejsze od "do"
		//[3] godzina pocz¹tkowa ró¿na od koñcowej, godzina aktualna równa godzinie "od" , minuty aktualne wiêksze lub równe minutom "od"
		//[4] godzina pocz¹tkowa ró¿na od koñcowej, godzina aktualna równa godzinie "do" , minuty aktualne mniejsze ni¿minuty "do"
		if (
		/*1*/	((godziny2<godziny) && (godziny<godziny2k))	   ||
		/*2*/	((godziny==godziny2) && (godziny==godziny2k) && (minuty>=minuty2) && (minuty<minuty2k))    ||
		/*3*/	((godziny2 != godziny2k)  &&  (godziny==godziny2) &&  (minuty>=minuty2))   ||
		/*4*/	((godziny2 != godziny2k) &&  (godziny==godziny2k)&&  (minuty<minuty2k))
			)//koniec warunków
		{
			OCR1A=255-jasnosc2; //PWM
			PRIORYTET = zakres2; // rezerwuj zakres
		}
	}

	if (godziny2>godziny2k){// WARUNKI KIEDY ZAKRES DOTYCZY PRZEJŒCIA PRZEZ 23:59 -> 00:00
	//WARUNKI:
		//[1] przed pó³noc¹ i godzina aktualna wiêksza od pocz¹tkowej
		//[2] przed pó³noc¹, godzina aktualna równa pocz¹tkowej i czekamy na minuty
		//[3] po pó³nocy, godzina aktualna mniejsza od koñcowej
		//[4] po pó³nocy, godzina aktualna równa koñcowej, czekamy na minuty
		if(
		/*1*/	(godziny>godziny2)  ||
		/*2*/	((godziny==godziny2)  &&  minuty>=minuty2)  ||
		/*3*/	(godziny<godziny2k)  ||
		/*4*/	((godziny==godziny2k) && (minuty<minuty2k))
		  ) //koniec warunków
		{
			OCR1A=255-jasnosc2; //PWM
			PRIORYTET = zakres2; // rezerwuj zakres
		}
	}
}//koniec funkcji

void oswietlenie_3 (void){

	if (godziny3<=godziny3k){//WARUNKI KIEDY CZAS "OD" JEST MNIEJSZY LUB RÓWNY CZASU "DO"
	//WARUNKI:
		//[1] godzina aktualna wiêksza od pocz¹tkowej mniejsza od koñcowej
		//[2] godzina aktualna równa pocz¹tkowej i koñcowej, minuty w zakresie wiêksze równe "od" ale mniejsze od "do"
		//[3] godzina pocz¹tkowa ró¿na od koñcowej, godzina aktualna równa godzinie "od" , minuty aktualne wiêksze lub równe minutom "od"
		//[4] godzina pocz¹tkowa ró¿na od koñcowej, godzina aktualna równa godzinie "do" , minuty aktualne mniejsze ni¿minuty "do"
		if (
		/*1*/	((godziny3<godziny) && (godziny<godziny3k))	   ||
		/*2*/	((godziny==godziny3) && (godziny==godziny3k) && (minuty>=minuty3) && (minuty<minuty3k))    ||
		/*3*/	((godziny3 != godziny3k)  &&  (godziny==godziny3) &&  (minuty>=minuty3))   ||
		/*4*/	((godziny3 != godziny3k) &&  (godziny==godziny3k)&&  (minuty<minuty3k))
			)//koniec warunków
		{
			OCR1A=255-jasnosc3; //PWM
			PRIORYTET = zakres3; // rezerwuj zakres
		}
	}

	if (godziny3>godziny3k){// WARUNKI KIEDY ZAKRES DOTYCZY PRZEJŒCIA PRZEZ 23:59 -> 00:00
	//WARUNKI:
		//[1] przed pó³noc¹ i godzina aktualna wiêksza od pocz¹tkowej
		//[2] przed pó³noc¹, godzina aktualna równa pocz¹tkowej i czekamy na minuty
		//[3] po pó³nocy, godzina aktualna mniejsza od koñcowej
		//[4] po pó³nocy, godzina aktualna równa koñcowej, czekamy na minuty
		if(
		/*1*/	(godziny>godziny3)  ||
		/*2*/	((godziny==godziny3)  &&  minuty>=minuty3)  ||
		/*3*/	(godziny<godziny3k)  ||
		/*4*/	((godziny==godziny3k) && (minuty<minuty3k))
		  ) //koniec warunków
		{
			OCR1A=255-jasnosc3; //PWM
			PRIORYTET = zakres3; // rezerwuj zakres
		}
	}
}//koniec funkcji


void priorytety (void){
	//Jeœli wszystkie zakresy ustawione s¹ na 00:00
	if ((!godziny1) && (!godziny1k) && (!minuty1) && (!minuty1k) && (!godziny2) && (!godziny2k) && (!minuty2) && (!minuty2k) && (!godziny3) && (!godziny3k) && (!minuty3) && (!minuty3k)){
		OCR1A=255;
	}
	//Jeœli nie przyznany priorytet
	if ((PRIORYTET != zakres1) &&  (PRIORYTET != zakres2) && (PRIORYTET != zakres3)){
		OCR1A=255; // wygaszenie diody w przypadku braku rezerwacji priorytetu
	}
}

void oswietlenie (void){
	oswietlenie_1 ();
	oswietlenie_2 ();
	oswietlenie_3 ();

}
//#################################################################################################################
///                                           	/* PROCEDURY OBS£UGI PRZERWAÑ */
//#################################################################################################################
// procedura obs³ugi przerwania dla timerów systemowych
ISR(TIMER2_COMP_vect)
{
	uint16_t n;

	n = Timer1;		//SuperDebounce
	if (n) Timer1 = --n;
	n = Timer2;		//timer dla pêtli g³ównej
	if (n) Timer2 = --n;
	n = Timer3;		//3hz dla rep'a
	if (n) Timer3 = --n;

}

// procedura obs³ugi przerwania INT0 wyjscie INT z PCF8583 (1 Hz)
ISR( INT0_vect ) {
	TWI_read_buf( PCF8583_ADDR, 0x01, 4, bufor );
	sekundy = bcd2dec( bufor[ss] );
	minuty = bcd2dec( bufor[mm] );
	godziny = bcd2dec( bufor[hh] );
}





