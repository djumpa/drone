/*
Testprogramm für den Slave
Der Buffer wird mit Werten gefüllt. Dann wird er fortlaufend über die serielle Schnittstelle ausgegeben.
Nun kann man dort sehen, wenn der Master einen Wert ändert
*/
#define F_CPU 8000000UL
#include <util/twi.h> 	    //enthält z.B. die Bezeichnungen für die Statuscodes in TWSR
#include <avr/interrupt.h>  //dient zur Behandlung der Interrupts
#include <stdint.h> 	    //definiert den Datentyp uint8_t
#include "twislave.h"
#include <stdlib.h>         //nötig für Zahlumwandlung mit itoa
#include <util/delay.h>


#define BAUD 9600 //Baudrate
#define SLAVE_ADRESSE 0x11 //Die Slave-Adresse
#define TWI_CMD_MASTER_WRITE 0x10
#define TWI_CMD_MASTER_READ  0x20
#define TWI_CMD_MASTER_READ_ADC  0x21

#define START_TIMER2 TCCR2 |= (0 << CS22)|(1 << CS21)|(0 << CS20)//CLK/8
#define STOP_TIMER2  TCCR2 &= ~0x07
#define CLEAR_TIMER2 TCNT2 = 0

#define START_TIMER1 TCCR1B |= (0 << CS12)|(0 << CS11)|(1 << CS10)//CLK/1
#define STOP_TIMER1  TCCR1B &= ~0x07
#define CLEAR_TIMER1 TCNT1 = 0

int i = 0;
int hb = 0;
int rise = 0;
uint8_t now;
uint8_t last;
uint8_t start_count;

volatile unsigned char servo[8];
volatile int pulse_width_us[8];
volatile unsigned char adc0[2];


int main (void)
{
	DDRB=0xFD;
	PORTB=0xFF;
	
	volatile unsigned char data[i2c_buffer_size];
	
	//TWI als Slave mit Adresse slaveadr starten
	init_twi_slave(SLAVE_ADRESSE);
	
	OCR1A = 11999;
	TCCR1B |= (1 << WGM12);
	// Mode 4, CTC on OCR1A
	TIMSK |= (1 << OCIE1A);
	//Set interrupt on compare match

	
	//Timer 2
	OCR2 = 99;
	TCCR2 |= (1 << WGM21);
	// Set to CTC Mode
	TIMSK |= (1 << OCIE2);
	//Set interrupt on compare match
	START_TIMER2;
	// set prescaler to 64 and starts PWM
	

	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0); // Set ADC prescaler
	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
	ADMUX |= (0 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading
	// No MUX values needed to be changed to use ADC0
	
	ADCSRA |= (1 << ADFR);  // Set ADC to Free-Running Mode
	ADCSRA |= (1 << ADEN);  // Enable ADC
	
	ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt
	sei();	// Enable Global Interrupts
	
	ADCSRA |= (1 << ADSC);  // Start A2D Conversions
	
	//i2cdatamit Werten füllen, die der Master auslesen und ändern kann
	for(uint8_t i=0;i<i2c_buffer_size;i++)
	{
		i2cdata[i]=0;
		data[i]=0;
	}

	pulse_width_us[0]=1500;//rudder
	pulse_width_us[1]=1000;//thurst
	pulse_width_us[2]=1500;//elevator
	pulse_width_us[3]=1000;//aileron
	
	

	//in einer Endlosschleife den Inhalt der Buffer ausgeben
	while(1)
	{
		
		
		if (i2cdata[0] == TWI_CMD_MASTER_WRITE)
		{
			//PORTB = messageBuf[1];
			//duty = messageBuf[1];
			//data[0] = messageBuf[0];
			
			
			data[0] = i2cdata[1];
			data[1] = i2cdata[2];
			data[2] = i2cdata[3];
			data[3] = i2cdata[4];
			
			pulse_width_us[0] = 10*data[0]+1000;
			pulse_width_us[1] = 10*data[1]+1000;
			pulse_width_us[2] = 10*data[2]+1000;
			pulse_width_us[3] = 10*data[3]+1000;
			
			//i2cdata[0]=0x00;
		}
		
// 		for(uint8_t i=0;i<i2c_buffer_size;i++)
// 		{
// 			//uart_puti(i2cdata[i]);
// 			PORTB = i2cdata[0];
// 			
// 		}

// TWI_CMD_MASTER_READ prepares the data from PINB in the transceiver buffer for the TWI master to fetch.
if (i2cdata[0] == TWI_CMD_MASTER_READ)
{
	//messageBuf[0] = duty;
	
	//TWI_write_slave(data[3]);
	
	//i2cdata[0]=0x00;
	i2cdata[1]=data[0];
	i2cdata[2]=data[1];
	i2cdata[3]=data[2];
	i2cdata[4]=data[3];
	
}

if (i2cdata[0] == TWI_CMD_MASTER_READ_ADC)
{
	//messageBuf[0] = duty;
	
	//TWI_write_slave(data[3]);
	
	//i2cdata[0]=0x00;
	i2cdata[1]=adc0[0];
	i2cdata[2]=adc0[1];
	
}
		
		//_delay_ms(500);
		
	} 
	
	
	//end.while
} //end.main

ISR (TIMER2_COMP_vect)
{
	if(i==0)
	{
		PORTB |= (1 << 2);
		//PORTB &= ~(1 << 0);
		OCR1A=pulse_width_us[0]*8-1;
		START_TIMER1;
		servo[0] =1;
	}
	if(i==25)
	{
		PORTB |= (1 << 3);
		//PORTB &= ~(1 << 1);
		OCR1A=pulse_width_us[1]*8-1;
		START_TIMER1;
		servo[1] =1;
	}
	if(i==50)
	{
		PORTB |= (1 << 4);
		//PORTB &= ~(1 << 0);
		OCR1A=pulse_width_us[2]*8-1;
		START_TIMER1;
		servo[2] =1;
	}
	if(i==75)
	{
		PORTB |= (1 << 5);
		//PORTB &= ~(1 << 1);
		OCR1A=pulse_width_us[3]*8-1;
		START_TIMER1;
		servo[3] =1;
	}
	i++;
	
	if(i>=200)
	{
		i=0;
		//PORTB |= (1 << 0);
		//PORTB |= (1 << 1);
	}
	
	//heartbeat
	if(hb==0)
	{
		PORTB |= (1 << 0);
	}
	if(hb == 500)
	{
		PORTB &= ~(1 << 0);
	}
	hb++;
	if(hb >= 5000)
	{
		hb=0;
	}
	
	/*
	now = PINB & 0x02;
	if (now != last) {
		if (now > 0){
			//PORTB |= (1 << 0);//rising edge, do something
			//rise++;
			//PORTB |= (1 << 0);
			start_count = 1;
			rise = 0;
		}
		if (now == 0){
			//falling edge
			//rise++;
			//PORTB &= ~(1 << 0);
			start_count = 0;
			if (rise>2){
				PORTB |= (1 << 0);
			}
			else PORTB &= ~(1 << 0);
		}
		last = now; //remember button state for next pass
	} //end of now != last 
	
	if (start_count){
		rise++;
	}
	
	*/
}
	
	


ISR (TIMER1_COMPA_vect)
{
	
	if(servo[0])
	{
		//PORTB |= (1 << 0);
		PORTB &= ~(1 << 2);
		STOP_TIMER1;
		servo[0]=0;
	}
	if(servo[1])
	{
		//PORTB |= (1 << 0);
		PORTB &= ~(1 << 3);
		STOP_TIMER1;
		servo[1]=0;
	}
	if(servo[2])
	{
		//PORTB |= (1 << 0);
		PORTB &= ~(1 << 4);
		STOP_TIMER1;
		servo[2]=0;
	}
	if(servo[3])
	{
		//PORTB |= (1 << 0);
		PORTB &= ~(1 << 5);
		STOP_TIMER1;
		servo[3]=0;
	}
}

ISR(ADC_vect)
{
	adc0[0] = ADCL;
	adc0[1] = ADCH;
}