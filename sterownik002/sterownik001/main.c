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
volatile short encoderCounter;

void motorInit();
void motorRight();
void motorLeft();
void motorStop();
void motorPWM(int pwm);

void encoderInit();


int main(void)
{
    /* Replace with your application code */
	ledFlag = 0;
	counter = 0;
	
	DDRH = 1<<5;
	
	motorInit();
	encoderInit();
	
	TCCR0A = 0b00000000;
	TCCR0B = 0b00000101;
	TIMSK0 = 0b00000001;
	
	//bezposrednio przed while w³aczam przerwania
	sei();
	
	
	motorPWM(100);
    while (1) 
    {
		
		/*if(ledFlag)
		{
			if(PORTH & 1<<5)
			{
				PORTH &= ~(1<<5);
				
				motorPWM(100);
			}
			else
			{
				PORTH |= 1<<5;
				
				motorPWM(-100);
			}
			ledFlag = 0;
		}*/
		
		if(encoderCounter>8400) motorPWM(-100);
		if(encoderCounter<0) motorPWM(100);
		
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

ISR(INT6_vect) //przerwamie zewenetrze od enkodera
{
	unsigned char En;
	En = (PINE & 0b11000000); //maska do wyluskania sygnalu z pinow enkodera
	switch (En)
	{
		case 0b01000000:
		{
			encoderCounter++;
		}
		break;
		case 0b10000000:
		{
			encoderCounter++;
		}
		break;
		case 0b11000000:
		{
			encoderCounter--;
		}
		break;
		case 0b00000000:
		{
			encoderCounter--;
		}
		break;
	}
}

ISR(INT7_vect) //przerwamie zewenetrze od enkodera
{
	unsigned char En;
	En = (PINE & 0b11000000); //maska do wyluskania sygnalu z pinow enkodera
	switch (En)
	{
		case 0b11000000:
		{
			encoderCounter++;
		}
		break;
		case 0b00000000:
		{
			encoderCounter++;
		}
		break;
		case 0b10000000:
		{
			encoderCounter--;
		}
		break;
		case 0b01000000:
		{
			encoderCounter--;
		}
		break;
	}
}


void motorInit()
{
	DDRB |= MOTOR_INA | MOTOR_INB | MOTOR_PWM | MOTOR_SEL0;  //setting pins as output
	PORTB |= MOTOR_SEL0; //
	
	//configurate OC1C
	TCCR1A |=0b00001011; // 23 bits as clear on compare , Fast pwm 10 bit 10
	TCCR1B |=0b00000010; // 43 fast pwm 10, 210 prescaler 8
}
void motorRight()
{
	PORTB |= MOTOR_INA;
	PORTB &= ~MOTOR_INB;
	OCR1C = 1023;
	//PORTB |= MOTOR_PWM;
}
void motorLeft()
{
	PORTB |= MOTOR_INB;
	PORTB &= ~MOTOR_INA;
	OCR1C = 1023;
	//PORTB &= MOTOR_PWM;
}
void motorStop()
{
	PORTB &= MOTOR_INB;
	PORTB &= ~MOTOR_INA;
}
void motorPWM(int pwm)
{
	if(pwm == 0)
	{
		motorStop();
	}
	else if(pwm>0)
	{
		if(pwm>1023) pwm = 1023;
		PORTB |= MOTOR_INA;
		PORTB &= ~MOTOR_INB;
		OCR1C = pwm;
	}
	else
	{
		pwm = -pwm;
		if(pwm>1023) pwm = 1023;
		PORTB |= MOTOR_INB;
		PORTB &= ~MOTOR_INA;
		OCR1C = pwm;
	}
}

void encoderInit()
{
	EICRB |= 0b01010000;
	EIMSK |= 0b11000000;
	encoderCounter = 0;
}

