/*
 * main.c			F_CPU = 20MHz
 *
 *  Created on: 2011-11-11
 *       Autor: Miros³aw Kardaœ
 */
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "PetitFS/diskio.h"
#include "PetitFS/pff.h"
//#include "LCD/lcd44780.h"

// obs³uga Timer0 z preskalerem = 8
#define TMR_START TCCR0 |= (1<<CS01)
#define TMR_STOP TCCR0 &= ~(1<<CS01)

//*************** makra i zmienne na potrzeby obs³ugi PetitFAT
// proszê pamiêtaæ tak¿e o zmianie tych wartoœci w pliku mmc.c !!!!
#define SCK 	PB7
#define MOSI 	PB5
#define MISO 	PB6
#define CS 		PB4

// definicja struktury z parametrami WAV
typedef struct {
	uint8_t stereo:1;
	uint8_t prescaler:1;
	uint8_t resolution;
	uint16_t khz;
} _FLAGS;

volatile _FLAGS FLAGS;	// definicja struktury

volatile uint8_t can_read;

FATFS Fs;			/* File system object */
DIR Dir;			/* Directory object */
FILINFO Fno;		/* File information */

WORD rb;

static UINT play ( const char *fn );

#define BUF_SIZE 512			// maksymalny rozmiar pojedynczego bufora

uint8_t  buf[2][BUF_SIZE];		// podwójny bufor do odczytu z karty SD

volatile uint8_t nr_buf;		// indeks aktywnego buforu

volatile uint16_t xaxis=700;
volatile uint16_t yaxis=700;
volatile uint16_t zaxis=700;

volatile uint8_t hitpriority=0;
volatile uint8_t breakdisablehigh=0;
volatile uint8_t breakdisablelow=0;
volatile int hit[10] = {1,3,2,1,3,2,1,3,1,1};
volatile uint8_t hitnumber = 1;
volatile uint8_t hitcounter = 0;
volatile int swing[10] = {1,3,2,1,3,2,1,3,1,1};
volatile uint8_t swingnumber = 1;
volatile uint8_t swingcounter = 0;
volatile uint8_t startup = 1;
volatile uint8_t playstop = 0;
//witam
//pozdrawiam


#define swingthresholdhigh 	550
#define swingthresholdlow 	500
#define swingthresholdhigh1 860
#define swingthresholdlow1 	810

BYTE rcv_spi (void)
{
	SPDR = 0xFF;
	loop_until_bit_is_set(SPSR, SPIF);
	return SPDR;
}

//! **************** main() ***********************************************
int main(void) {

	DDRD  |= (1<<PD5)|(1<<PD4);			// ustaw piny PWM1 (OC1A) oraz PWM2 (OC1B) jako wyjœcia WA¯NE !!!
	PORTB = 0b00001111;					// podci¹gniêcie PORTB do VCC

	DDRB |= (1<<PB1);				// wyjœcie do sterowania zasilaniem karty SD

	//PORTD |= (1<<PD2)|(1<<PD3);		// podci¹gniêcie wyjœæ
	//PORTB |= (1<<PB0);				// klawiszy do VCC

	// init SPI
	DDRB |= (1<<CS)|(1<<MOSI)|(1<<SCK)|(1<<CS);
	PORTB |= (1<<CS);
	SPCR |= (1<<SPE)|(1<<MSTR);		
	SPSR |= (1<<SPI2X);				// masymalny zegar SCK

	// konfiguracja PWM (Timer1) noœna
	TCCR1A = (1<<WGM10)|(1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1);//|(1<<COM1B0);
	TCCR1B = (1<<CS10);

	// konfiguracja Timer0 (samplowanie)
	TCCR0 = (1<<WGM01);		// tryb CTC
	TIMSK = (1<<OCIE0);		// zezwolenie na przerwanie CompareMatch

	sei();		// globalne zezwolenie na przerwania
	
	OCR0 = (uint8_t)44;						//bo F_CPU/8/samplerate	
	
	
	// **************** akcelerometr **********************************
	
	/*lcd_init();
	lcd_locate(0,0);
	lcd_str("X:");
	lcd_locate(2,0);
	lcd_str("Y:");
	lcd_locate(3,0);
	lcd_str("Z:");*/
	
	ADCSRA |= 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0;
	ADMUX |= 1<<REFS0 | 1<<REFS1; // | 1<<ADLAR;
	ADCSRA |= 1<<ADIE;
	ADCSRA |= 1<<ADEN;
	
	//ADCSRA |= 1<<ADSC;



	// **************** pêtla g³ówna **********************************
	while(1) {
		if (pf_mount(&Fs)) continue;	/* Initialize FS */

		for (;;) {

			if (pf_opendir(&Dir, "")) break;	/* Open sound file directory (root dir) */

			//while (!pf_readdir(&Dir, &Fno) && Fno.fname[0]) {	/* Play all wav files in the dir */
				//if (!(Fno.fattrib & (AM_DIR|AM_HID))
					//&& strstr(Fno.fname, ".WAV") ) {

					//if (play(Fno.fname)) break;	// odtwarzaj plik WAV

				//}
			//}
		//if (play("star3.wav")) break;		
		//play("startup.wav");
		//for (;;) play("hum.wav");
		if (startup==1)
		{
			if (play("startup.wav")) break;
			startup=0;
		}
		
		
		if (hitpriority==1)
		{
			breakdisablehigh=1;
			breakdisablelow=1;
			swingnumber=swing[swingcounter];
			if (swingnumber==1)
			{
				play("star3.wav");
			}
			if (swingnumber==2)
			{
				play("swing2.wav");
			}
			if (swingnumber==3)
			{
				play("swing3.wav");
			}
			hitpriority=0;
			breakdisablehigh=0;
			breakdisablelow=0;
			swingcounter++;
			if (swingcounter==9)
			{
				swingcounter=0;
			}
		}
		if (hitpriority==2)
		{
			breakdisablehigh=1;
			breakdisablelow=0;
			hitnumber=hit[hitcounter];
			if (hitnumber==1)
			{
				play("lasrhit1.wav");
			}
			if (hitnumber==2)
			{
				play("lasrhit2.wav");
			}
			if (hitnumber==3)
			{
				play("lasrhit3.wav");
			}
			hitpriority=0;
			breakdisablehigh=0;	
			hitcounter++;
			if (hitcounter==9)
			{
				hitcounter=0;
			}
		}
		else
		{
			if (play("hum.wav")) break;	
		}
		}

	} // koniec while(1)
}
//********************************************************************************************


//***************** przerwanie TIMER0 - samplowanie ******************************************
ISR(TIMER0_COMP_vect) {
	
	
	static uint16_t buf_idx;		// indeks w pojedynczym buforze
	static uint8_t v1, v2;			// zmienne do przechowywania próbek
	
				v1 = buf[nr_buf][buf_idx++];		// pobieramy próbkê MONO do zmiennej v1
				v2 = v1;							// to samo na dwa kana³y/wyjœcia

	OCR1A = v1;									// próbka na wyjœcie PWM1, kana³ L
	OCR1B = v2;									// próbka na wyjœcie PWM2, kana³ R

	if( buf_idx > BUF_SIZE-1 ) {
		buf_idx=0;								// reset indeksu bufora
		can_read = 1;							// flaga = 1
		nr_buf ^= 0x01;							// zmiana bufora na kolejny
	}
	ADCSRA |= 1<<ADSC;	
}
// *************************** koniec przerwania ****************************************

// ******************  funkcja  P L A Y  ********************************
static UINT play ( const char *fn ) {

	FRESULT res;

	if ((res = pf_open(fn)) == FR_OK) {

		FLAGS.resolution = 8;
		FLAGS.stereo = 0;

		pf_lseek(0);

		pf_read(&buf[0][0], BUF_SIZE , &rb);	// za³aduj pierwsz¹ czêœæ bufora
		pf_read(&buf[1][0], BUF_SIZE , &rb);	// za³aduj drug¹ czêœæ bufora

		TMR_START;		// start Timera0 (samplowanie)

		//DDRD  |= (1<<PD5)|(1<<PD4);			// ustaw piny PWM1 (OC1A) oraz PWM2 (OC1B) jako wyjœcia WA¯NE !!!

		while(1) {
			if( can_read ) {				// jeœli flaga ustawiona w obs³udze przerwania

				pf_read(&buf[ nr_buf ^ 0x01 ][0], BUF_SIZE , &rb);	// odczytaj kolejny bufor
				if( rb < BUF_SIZE ) break;		// jeœli koniec pliku przerwij pêtlê while(1)

				// klawisz do zmiany utworu na nastêpny
				//if( !(PINB & (1<<PB0)) ) break;

				can_read = 0;
			}
			
			/*if(((xaxis<swingthresholdhigh && xaxis>swingthresholdlow) || (yaxis<swingthresholdhigh && yaxis>swingthresholdlow) || (zaxis<swingthresholdhigh && zaxis>swingthresholdlow) || (xaxis<swingthresholdhigh1 && xaxis>swingthresholdlow1) || (yaxis<swingthresholdhigh1 && yaxis>swingthresholdlow1) || (zaxis<swingthresholdhigh1 && zaxis>swingthresholdlow1)) && breakdisablehigh != 1 && breakdisablelow !=1)
			{
				hitpriority=1;
				break;
			}
			if((xaxis<swingthresholdlow || yaxis<swingthresholdlow || zaxis<swingthresholdlow || xaxis>swingthresholdhigh1 || yaxis>swingthresholdhigh1 || zaxis>swingthresholdhigh1) && (breakdisablehigh != 1 || breakdisablelow==1))
			{
				hitpriority=2;
				break;
			}*/
			if (playstop==1) 
			{
				playstop=0;
				break;
			}
		}

		//DDRD  &= ~((1<<PD5)|(1<<PD4));		// ustaw piny PWM1 (OC1A) oraz PWM2 (OC1B) jako wejœcia WA¯NE !!!

		TMR_STOP;	// wy³¹czenie Timera0 (samplowania)
		//_delay_ms(500);						// przerwa 0,5s
	}

	return res;
}

ISR(ADC_vect)
{
	
	char adcResult[4];
	uint8_t theLow = ADCL;
	uint16_t theTenBitResult = ADCH<<8 | theLow;
	//itoa(theTenBitResult, adcResult, 10);
	switch (ADMUX)
	{
		case 0b11000000:
		xaxis=theTenBitResult;
		//lcd_locate(0,3);
		//lcd_str(adcResult);
		ADMUX = 0b11000001;
		break;
		case 0b11000001:
		yaxis=theTenBitResult;
		//lcd_locate(2,3);
		//lcd_str(adcResult);
		ADMUX = 0b11000010;
		break;
		case 0b11000010:
		zaxis=theTenBitResult;
		//lcd_locate(3,3);
		//lcd_str(adcResult);
		ADMUX = 0b11000000;
		break;
		default:
		//Default code
		break;
	}
	if(((xaxis<swingthresholdhigh && xaxis>swingthresholdlow) || (yaxis<swingthresholdhigh && yaxis>swingthresholdlow) || (zaxis<swingthresholdhigh && zaxis>swingthresholdlow) || (xaxis<swingthresholdhigh1 && xaxis>swingthresholdlow1) || (yaxis<swingthresholdhigh1 && yaxis>swingthresholdlow1) || (zaxis<swingthresholdhigh1 && zaxis>swingthresholdlow1)) && breakdisablehigh != 1 && breakdisablelow !=1)
	{
		hitpriority=1;
		playstop=1;
		//break;
	}
	if((xaxis<swingthresholdlow || yaxis<swingthresholdlow || zaxis<swingthresholdlow || xaxis>swingthresholdhigh1 || yaxis>swingthresholdhigh1 || zaxis>swingthresholdhigh1) && (breakdisablehigh != 1 || breakdisablelow==1))
	{
		hitpriority=2;
		playstop=1;
		//break;
	}
	//ADCSRA |= 1<<ADSC;
}
