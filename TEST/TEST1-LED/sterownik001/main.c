/*
 * sterownik001.c
 *
 * Created: 15.06.2021 12:28:52
 * Author : tom13
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>



void diodeInit();
void turnOnDiode();
void turnOffDiode();
unsigned char checkDiode();



int main(void)
{
    /* Replace with your application code */
	
	diodeInit();

	
	TCCR0A = 0b00000000;
	TCCR0B = 0b00000011;
	TIMSK0 = 0b00000001;
	
	//bezposrednio przed while w³aczam przerwania
	sei();
	
	turnOnDiode();

    while (1) 
    {

		
    }
}


ISR(TIMER0_OVF_vect)  //przerwanie od przepe³nienia licznika timer0
{
	if(checkDiode())
	{
		turnOffDiode();
	}
	else{
		turnOnDiode();
	}
}




void diodeInit()
{
	DDRH |= 1<<5;
}
void turnOnDiode()
{
	PORTH |= 1<<5;
}
void turnOffDiode()
{
	PORTH &= ~(1<<5);
}
unsigned char checkDiode()
{
	return PORTH & 1<<5; // 1<<5 = 0b00100000
}

