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

volatile unsigned int counter = 0;
volatile unsigned char flag = 0;


void motorInit();
void motorRight();
void motorLeft();
void motorStop();
void motorPWM(int pwm);

void diodeInit();
void turnOnDiode();
void turnOffDiode();
unsigned char checkDiode();



int main(void)
{
    /* Replace with your application code */
	
	diodeInit();
	
	motorInit();
	
	TCCR0A = 0b00000000;
	TCCR0B = 0b00000011;
	TIMSK0 = 0b00000001;
	
	//bezposrednio przed while w�aczam przerwania
	sei();

	int pwm = 300;

    while (1) 
    {
		if(flag)
		{
			flag = 0;
			
			motorPWM(pwm);
			pwm = - pwm;
			
			if(checkDiode())
			{
				turnOffDiode();
			}
			else{
				turnOnDiode();
			}
			
		}
		
    }
}


ISR(TIMER0_OVF_vect)  //przerwanie od przepe�nienia licznika timer0
{
	
	
	if (counter>100)
	{
		counter=0;
		flag = 1;
	}
	counter++;
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
