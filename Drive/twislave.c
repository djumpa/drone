#include <util/twi.h> 	    //enthält z.B. die Bezeichnungen für die Statuscodes in TWSR
#include <avr/interrupt.h>  //dient zur Behandlung der Interrupts
#include <stdint.h>         //definiert den Datentyp uint8_t
#include "twislave.h"

//%%%%%%%% Globale Variablen, die vom Hauptprogramm genutzt werden %%%%%%%%
/*Der Buffer, in dem die Daten gespeichert werden.
Aus Sicht des Masters läuft der Zugrif auf den Buffer genau wie bei einem I2C-EEPROm ab.
Für den Slave ist es eine globale Variable
*/
volatile uint8_t buffer_adr; //"Adressregister" für den Buffer

/*Initaliserung des TWI-Inteface. Muss zu Beginn aufgerufen werden, sowie bei einem Wechsel der Slave Adresse
Parameter adr: gewünschte Slave-Adresse
*/
void init_twi_slave(uint8_t adr)
{
	TWAR= adr; //Adresse setzen
	TWCR &= ~(1<<TWSTA)|(1<<TWSTO);
	TWCR|= (1<<TWEA) | (1<<TWEN)|(1<<TWIE);
	buffer_adr=0xFF;
	sei();
}


//Je nach Statuscode in TWSR müssen verschiedene Bitmuster in TWCR geschreiben werden(siehe Tabellen im Datenblatt!).
//Makros für die verwendeten Bitmuster:

//ACK nach empfangenen Daten senden/ ACK nach gesendeten Daten erwarten
#define TWCR_ACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);

//NACK nach empfangenen Daten senden/ NACK nach gesendeten Daten erwarten
#define TWCR_NACK TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC);

//switch to the non adressed slave mode...
#define TWCR_RESET TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC);


/*ISR, die bei einem Ereignis auf dem Bus ausgelöst wird. Im Register TWSR befindet sich dann
ein Statuscode, anhand dessen die Situation festgestellt werden kann.
*/
ISR (TWI_vect)
{
	//uint8_t data=0;

	switch (TW_STATUS) //TWI-Statusregister prüfen und nötige Aktion bestimmen
	{

		// Slave Receiver

		case TW_SR_SLA_ACK: // 0x60 Slave Receiver, Slave was addresed
		TWCR_ACK; // received next data byte, send ACK
		buffer_adr=0; //buffer position is undefined
		break;
		
		case TW_SR_DATA_ACK: // 0x80 Slave Receiver, ein Datenbyte wurde empfangen
		
		//data=TWDR; //Reading received data
		/*if (buffer_adr == 0xFF) //first access, set buffer position
		{
			//Control whether desired address within the permissible range
			if(data<i2c_buffer_size+1)
			{
				buffer_adr= data; //Set as addressed Buffer position
			}
			else
			{
				buffer_adr=0; //Address set to zero. Does that make sense? TO DO!
			}
			TWCR_ACK;	// receive next byte of data, ACK then to request next byte
		}
		else //Further access, after the position has been set in the buffer. Now ready to receive and store the data
		{*/

			if(buffer_adr<i2c_buffer_size+1)
			{
				i2cdata[buffer_adr]=TWDR; //Write data in Buffer
				
			}
		
			buffer_adr++; //Buffer address continue to count on the next write access
			TWCR_ACK;
		//}
		break;


		//Slave transmitter

		case TW_ST_SLA_ACK: //0xA8 Slave was addressed in read mode and has an ACK returned.
		//Here there is no break! The following code so it is also carried out!
			buffer_adr=0;
		
		case TW_ST_DATA_ACK: //0xB8 Slave Transmitter, Data has been requested

		
			//TWDR=buffer_adr++;
			TWDR = i2cdata[buffer_adr++]; //Send data byte
			//buffer_adr++; //further include buffer address for Next Byte
		
		
		TWCR_ACK;
		break;
		case TW_SR_STOP:
		TWCR_ACK;
		break;
		case TW_ST_DATA_NACK: // 0xC0 Demanded no more data
		TWCR_ACK
		break;
		case TW_SR_DATA_NACK: // 0x88
		case TW_ST_LAST_DATA: // 0xC8  Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received
			TWCR =   (1<<TWSTO)|(1<<TWINT);   //Recover from TWI_BUS_ERROR, this will release the SDA and SCL pins thus enabling other devices to use the bus
		break;
		default:
		TWCR_ACK;
		break;
		
	} //end.switch (TW_STATUS)
} //end.ISR(TWI_vect)

////Ende von twislave.c////