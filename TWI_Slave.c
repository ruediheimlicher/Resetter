//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
//#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>

//#include "twislave.c"
//#include "lcd.c"

//#include "adc.c"

//***********************************
//Reset							*
//									*
//***********************************

#define TWI_PORT		PORTB
#define TWI_PIN		PINB
#define TWI_DDR		DDRB

#define RESETPIN              4        // Schaltet Relais
#define LOOPLEDPIN            0        // Blink-LED
//#define VCCPIN              1

#define SDAPIN                3        // Eingang von SDA/I2C
#define OSZIPIN               1
#define WEBSERVERPIN				2			// Eingang von WebServer
#define RESETCOUNT            0x0200   // Fehlercounter: Zeit bis Reset ausgeloest wird
#define RESETDELAY            0x0200   // Waitcounter: Blockiert wiedereinschalten 
#define WEBSERVERRESETDELAY   0x0800
#define WAIT                  0

void delay_ms(unsigned int ms);

volatile uint16_t	loopcount0=0;
volatile uint16_t	loopcount1=0;

volatile uint16_t	resetcount=0;
volatile uint16_t	delaycount=0; // Zaehlt wenn WAIT gesetzt ist: Delay fuer Relais

volatile uint16_t	webserverresetcount=0; // Zeit, die der Resetrequest vom Webserver dauert

volatile uint8_t statusflag=0;

volatile uint16_t	overflowcount=0;

void slaveinit(void)
{
    
    CLKPR |= (1<<3);
    TWI_DDR |= (1<<LOOPLEDPIN);
    TWI_DDR |= (1<<RESETPIN);       // Ausgang: Schaltet Reset-Relais fuer Zeit RESETDELAY
    TWI_PORT &= ~(1<<RESETPIN);     // LO	
    
    TWI_DDR |= (1<<OSZIPIN);        // Ausgang
    TWI_PORT |= (1<<OSZIPIN);       // HI
    
    TWI_DDR &= ~(1<<SDAPIN);        // Eingang: Verbunden mit SDA, misst LO-Zeit, um Stillstand zu erkennen
    TWI_PORT |= (1<<SDAPIN);        // HI
    

   TWI_DDR &= ~(1<<WEBSERVERPIN);        // Eingang: Verbunden mit Webserver, empfŠngt Signal zum reset
   TWI_PORT |= (1<<WEBSERVERPIN);        // HI

   
   
   //TWI_DDR &= ~(1<<VCCPIN);	// Eingang, Abfragen von VCC von Master
    //TWI_PORT |= (1<<VCCPIN);	// HI
    
    //TWI_DDR &= ~(1<<SCLPIN);	// Eingang
    //TWI_PORT |= (1<<SCLPIN);	// HI
    
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms)
	{
		_delay_ms(0.96);
		ms--;
	}
}

/* Initializes the hardware timer to generate 1 millisecond ticks */
void timer_init(void)
{
	/* Set timer to CTC mode */
	//TCCR0A = (1 << WGM01);
	/* Set prescaler to 8 */
	TCCR0B = (1 << CS00)|(1 << CS02);
	/* Set output compare register for 1ms ticks */
	//OCR0A = (F_CPU / 8) / 1000;
	/* Enable output compare A interrupts */
	TIMSK0 = (1 << TOIE0);
}

/* Interrupt Service Routine for compare match A interrupts */
ISR(TIM0_COMPA_vect)
{


}


void main (void) 
{
	
	wdt_disable();
	MCUSR &= ~(1<<WDRF);
	wdt_reset();
	WDTCR |= (1<<WDCE) | (1<<WDE);
	WDTCR = 0x00;
	slaveinit();
	/* initialize the LCD */
	
    
    
	//Zaehler fuer Zeit von (SDA || SCL = LO)
	//uint16_t twi_LO_count0=0;
	//uint16_t twi_LO_count1=0;
	//uint8_t SlaveStatus=0x00; //status
    
	//Zaehler fuer Zeit von (SDA && SCL = HI)
	//uint16_t twi_HI_count0=0;
	
	//uint8_t twicontrol=0;
	
#pragma mark while
	while (1)
	{	
		//wdt_reset();
		//Blinkanzeige
		loopcount0++;
		if (loopcount0>=0x00AF)
		{
			//lcd_gotoxy(0, 0);
			//lcd_putint(loopcount1);
			loopcount0=0;
			
			loopcount1++;
         
         /*
          WEBSERVERPIN 
          */
         
			if (TWI_PIN & (1<<SDAPIN) &&  (!(statusflag & (1<<WAIT)))) // alles OK. SDA ist HI,  WAIT ist nicht gesetzt
			{
				resetcount=0;                    // resetcounter zuruecksetzen
				TWI_PORT &= ~(1<<RESETPIN);
            
         }
			else 
			{
				resetcount++;                    // resetcounter inkrement
				if (resetcount > RESETCOUNT)     // Zeit erreicht
				{
               //TWI_PORT |=(0<<OSZIPIN);
					TWI_PORT |= (1<<RESETPIN);    // Resetpin Hi, Relais schaltet Bus aus
					statusflag |= (1<<WAIT);      // WAIT ist gesetzt, Relais wird von SDA_HI nicht sofort wieder zurueckgesetzt
				}
            
				if (resetcount > (RESETCOUNT + RESETDELAY))
				{
               //TWI_PORT |=(1<<OSZIPIN);
               
					TWI_PORT &= ~(1<<RESETPIN);   // Relais faellt ab
					statusflag &= ~(1<<WAIT);     // WAIT zurueckgesetzt, SDA_HI ist wieder wirksam
					resetcount =0;
               
            }
            
			}
         
         
			
         // WEBSERVERPIN abfragen: Reset wenn LO
         
         if (TWI_PIN & (1 << WEBSERVERPIN))
         {
            //HI, alles OK
            webserverresetcount =0;
            delaycount=0;
            statusflag &= ~(1<<WAIT);
            TWI_PORT &= ~(1<<RESETPIN);
         }
         else // webserverreset inc, reset wenn Eingang vom Webserver lange genug LO ist: Fehlerfall auf Webserver
         {
            webserverresetcount++;
            if (webserverresetcount > WEBSERVERRESETDELAY)
            {
               TWI_PORT |= (1<<RESETPIN);    // Resetpin Hi, Relais schaltet aus
					statusflag |= (1<<WAIT);      // WAIT ist gesetzt, Relais wird von SDA_HI nicht zurueckgesetzt
               
            }
            
            if (resetcount > (WEBSERVERRESETDELAY + RESETDELAY))
            {
               //TWI_PORT |=(1<<OSZIPIN);
               TWI_PORT &= ~(1<<RESETPIN);
               statusflag &= ~(1<<WAIT);// WAIT zurueckgesetzt, SDA_HI ist wieder wirksam
               webserverresetcount =0;
               
            }
            
         }
         
         
			if (loopcount1 >0x8F)
			{
				TWI_PORT ^=(1<<LOOPLEDPIN);
				loopcount1=0;
			}
			
		}
		
		//delaycount
      
      
		
	}//while
    
    
    //return 0;
}
