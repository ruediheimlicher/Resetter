//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
//#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>


//***********************************
//Reset							*
//									*
//***********************************

#define TWI_PORT		PORTB
#define TWI_PIN		PINB
#define TWI_DDR		DDRB


#define LOOPLEDPIN            0        // Blink-LED

#define SDAPIN                1        // Eingang von SDA/I2C
#define WEBSERVERPIN				2			// Eingang vom WebServer

#define OSZIPIN               3
#define REPORTPIN             3       // Wie OSZI. Meldet Reset an Webserver, active LO

#define RELAISPIN             4        // Schaltet Relais


#define RESETCOUNT            0x200   // Fehlercounter: Zeit bis Reset ausgeloest wird
#define RESETDELAY            0x08   // Waitcounter: Blockiert wiedereinschalten
#define WEBSERVERRESETDELAY   0x010
#define WAIT                  0
#define CHECK                 1 // in ISR gesetzt, resetcount soll erhoeht werden

#define SDA_LO_RESET          2 // gesetzt, wenn SDA zulange LO ist
#define SDA_HI_RESET          3 // gesetzt, wenn SDA zulange HI ist

#define SDA_LO_MAX            0x8000
#define SDA_HI_MAX            0xFFFF

void delay_ms(unsigned int ms);

volatile uint16_t	loopcount0=0;
volatile uint16_t	loopcount1=0;

volatile uint16_t	resetcount=0;
volatile uint16_t	delaycount=0; // Zaehlt wenn WAIT gesetzt ist: Delay fuer Relais

volatile uint16_t	webserverresetcount=0; // Zeit, die der Resetrequest vom Webserver dauert

volatile uint8_t statusflag=0;

volatile uint16_t	overflowcount=0;

volatile uint16_t	SDA_LO_counter=0;
volatile uint16_t	SDA_HI_counter=0;


void slaveinit(void)
{
    
    CLKPR |= (1<<3);
    TWI_DDR |= (1<<LOOPLEDPIN);
    TWI_DDR |= (1<<RELAISPIN);       // Ausgang: Schaltet Reset-Relais fuer Zeit RESETDELAY
    TWI_PORT &= ~(1<<RELAISPIN);     // LO	
    
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

/* Initializes the hardware timer  */
void timer_init(void)
{
	/* Set timer to CTC mode */
	//TCCR0A = (1 << WGM01);
	/* Set prescaler */
	TCCR0B = (1 << CS00)|(1 << CS02); // clock/1024
	/* Set output compare register for 1ms ticks */
	//OCR0A = (F_CPU / 8) / 1000;
	/* Enable output compare A interrupts */
	TIMSK0 = (1 << TOIE0); // TOV0 Overflow
}

ISR(TIM0_OVF_vect) // Aenderung an SDA
{
   statusflag |= (1<<CHECK);


}



ISR(INT0_vect) // Potential-Aenderung von SDA
{
   if ((!(statusflag & (1<<WAIT))))// WAIT verhindert, dass Relais von SDA_HI nicht sofort wieder zurueckgesetzt wird
   {
      // counter zuruecksetzen, alles OK
      resetcount=0;
      SDA_HI_counter=0;
      SDA_LO_counter=0;
   }
   
}

/*
ISR (SPI_STC_vect) // Neue Zahl angekommen
{
   OSZI_B_LO;
   if (inindex==0)
   {
      //OSZI_B_LO;
      //OSZI_B_HI;
      //isrcontrol = spi_txbuffer[inindex] ;
   }
   isrcontrol++;
   spi_rxbuffer[inindex] = SPDR;
   //isrcontrol = inindex;
   //isrcontrol +=inindex;
   SPDR = spi_txbuffer[inindex];
   //uint8_t input = SPDR;
   
   spi_rxdata=1;
   //inindex = inc(&inindex);
   inindex++;
   //inindex &= 0x0F;
   //SPI_Data_counter++;
   OSZI_B_HI;
}
*/


void main (void) 
{
	
	wdt_disable();
	MCUSR &= ~(1<<WDRF);
	wdt_reset();
	WDTCR |= (1<<WDCE) | (1<<WDE);
	WDTCR = 0x00;
	slaveinit();
   MCUCR |= (1<<ISC00);
   GIMSK |= (1<<INT0);
   timer_init();
 //  timer1_init();
   sei();
   
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
         if (loopcount1 >0x8F)
         {
            TWI_PORT ^=(1<<LOOPLEDPIN);
            loopcount1=0;
            //           TWI_PORT ^= (1<<RELAISPIN);
            
         }
         
      }
      /*
       Checken, ob SDA zu lange auf gleichem Wert blieb
       */
      
      // impulsdauer von SDA checken
      if (PINB & (1<<SDAPIN)) // HI, darf lange dauern
      {
         SDA_HI_counter++;
         
      }
      else // LO
      {
         SDA_LO_counter++;
         if (SDA_LO_counter >= SDA_LO_MAX)
         {
            statusflag |= (1<<SDA_LO_RESET);
         }
      }
      
      

      if (statusflag & (1<<CHECK))// Timer gibt Takt der Anfrage an
      {
         
         /// TWI_PORT ^=(1<<OSZIPIN);
         statusflag &= ~(1<<CHECK);
         // resetcount wird bei Aenderungen am SDA  in ISR von INT0 zurueckgesetzt. (Normalbetrieb)
         
         
         if ((resetcount > RESETCOUNT)  || (statusflag & (1<<SDA_LO_RESET)))     // Zeit erreicht, VON SDA LO ODER SDA HI
         {
            TWI_PORT &= ~(1<<REPORTPIN); // Meldung an Webserver LO
            //TWI_PORT |=(0<<OSZIPIN);
            
            TWI_PORT |= (1<<RELAISPIN);    // RELAISPIN Hi, Relais schaltet Bus aus
            statusflag |= (1<<WAIT);      // WAIT ist gesetzt, Relais wird von SDA_HI nicht sofort wieder zurueckgesetzt
            delaycount = 0;
         }
         
         
         
         
         if (statusflag |= (1<<WAIT))
         {
            delaycount++; // Counter fuer Dauer Relais_on
            
            if (delaycount > RESETDELAY)
            {
               
            }
            
         }
         else
         {
            // resetcounter inkrement
            resetcount++;
         }
         
         
         
         
         // Reset durch Webserver: WEBSERVERPIN abfragen: Reset wenn LO
         
         if (TWI_PIN & (1 << WEBSERVERPIN))
         {
            //HI, alles OK
            webserverresetcount =0;
            delaycount=0;
            statusflag &= ~(1<<WAIT);
            //           TWI_PORT &= ~(1<<RELAISPIN);
         }
         else // webserverreset inc, reset wenn Eingang vom Webserver lange genug LO ist: Fehlerfall auf Webserver
         {
            webserverresetcount++;
            TWI_PORT ^=(1<<OSZIPIN);
            if (webserverresetcount > WEBSERVERRESETDELAY)
            {
               TWI_PORT |= (1<<RELAISPIN);    // RELAISPIN Hi, Relais schaltet aus
               statusflag |= (1<<WAIT);      // WAIT ist gesetzt, Relais wird von SDA_HI nicht zurueckgesetzt
               
            }
            
            if (webserverresetcount > (WEBSERVERRESETDELAY + RESETDELAY))
            {
               //TWI_PORT |=(1<<OSZIPIN);
               TWI_PORT &= ~(1<<RELAISPIN);
               statusflag &= ~(1<<WAIT);// WAIT zurueckgesetzt, SDA_HI ist wieder wirksam
               webserverresetcount =0;
               resetcount =0;
            }
            
         }
      } // if check
      
   }//while
   
   
    //return 0;
}
