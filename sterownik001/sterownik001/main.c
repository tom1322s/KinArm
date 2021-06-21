/*
 * sterownik001.c
 *
 * Created: 15.06.2021 12:28:52
 * Author : tom13
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#define MOTOR_SEL0 (1<<4)
#define MOTOR_INA (1<<5)
#define MOTOR_INB (1<<6)
#define MOTOR_PWM (1<<7)


volatile char ledFlag;
volatile unsigned int counter;


int main(void)
{
    /* Replace with your application code */
	ledFlag = 0;
	counter = 0;
	
	DDRH = 1<<5;
	
	DDRB = MOTOR_INA | MOTOR_INB | MOTOR_PWM | MOTOR_SEL0;
	
	PORTB |= MOTOR_SEL0;

	
	
	
	TCCR0A = 0b00000000;
	TCCR0B = 0b00000101;
	TIMSK0 = 0b00000001;
	
	//bezposrednio przed while w³aczam przerwania
	sei();
	
    while (1) 
    {
		
		if(ledFlag)
		{
			if(PORTH & 1<<5)
			{
				PORTH &= ~(1<<5);
				
				PORTB |= MOTOR_INA;
				PORTB &= ~MOTOR_INB;
				PORTB |= MOTOR_PWM;
			}
			else
			{
				PORTH |= 1<<5;
				
				PORTB |= MOTOR_INB;
				PORTB &= ~MOTOR_INA;
				PORTB |= MOTOR_PWM;
			}
			ledFlag = 0;
		}
    }
}


ISR(TIMER0_OVF_vect)  //przerwanie od przepe³nienia licznika timer0
{
	if(counter>10)
	{
		ledFlag = 1;
		counter = 0;
	}
	else counter++;
	
}

