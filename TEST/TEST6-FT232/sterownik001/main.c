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

#define REGULATOR_P 0.2
#define BUFFOR_SIZE 10

typedef struct FIFO{
	char buffer[BUFFOR_SIZE + 1];
	unsigned int head;
	unsigned int tail;
}FIFO;


volatile char regulatorFlag;
volatile short encoderCounter;
volatile short alfaZad;
volatile char flag = 0;

FIFO *readBuffer;
FIFO *writeBuffer;

volatile unsigned int receiveErrorCounter = 0;

void motorInit();
void motorRight();
void motorLeft();
void motorStop();
void motorPWM(int pwm);

void encoderInit();

void diodeInit();
void turnOnDiode();
void turnOffDiode();
unsigned char checkDiode();

void limitSwitchesInit();

void FIFO_init(FIFO *fifo);
int FIFO_push(FIFO *fifo, char data);
int FIFO_pop(FIFO *fifo, char *data);
void FIFO_empty(FIFO *fifo);

void asyncTransmisionInit();


int main(void)
{
    /* Replace with your application code */
	regulatorFlag = 0;
	
	diodeInit();
	
	motorInit();
	encoderInit();
	limitSwitchesInit();
	
	FIFO fifoR;
	FIFO fifoW;
	readBuffer = &fifoR;
	writeBuffer = &fifoW;
	FIFO_init(readBuffer);
	FIFO_init(writeBuffer);
	
	asyncTransmisionInit();
	
	TCCR0A = 0b00000000;
	TCCR0B = 0b00000011;
	TIMSK0 = 0b00000001;
	
	//bezposrednio przed while w³aczam przerwania
	sei();
	
	alfaZad = 8400;
	

    while (1) 
    {
		
		/*if(regulatorFlag)
		{
			short u = REGULATOR_P * (alfaZad - encoderCounter);
			motorPWM(u);
			regulatorFlag = 0;
		}
		
		if(encoderCounter>8400) turnOnDiode();*/
		
		if(flag)
		{
			if(checkDiode())
			{
				turnOffDiode();
			}
			else{
				turnOnDiode();
			}
			flag = 0;
		}

		
    }
}


ISR(TIMER0_OVF_vect)  //przerwanie od przepe³nienia licznika timer0
{
	regulatorFlag = 1;
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

ISR(INT4_vect)
{
	alfaZad = 0;
}

ISR(INT5_vect)
{
	alfaZad = 8400;
}

ISR(USART0_RX_vect)  //przerwanie od odbioru
{
	char c=UDR0;
	//UDR0 = c+1;
	if (c)
	{
		//if(FIFO_push(readBuffer,c)==-1) receiveErrorCounter++;
		//if(c=='a') flag = 1;
		flag = 1;
		
	}
	else receiveErrorCounter++;
	

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

void limitSwitchesInit()
{
	EICRB |= 0b00001111;
	EIMSK |= 0b00110000;
}


void FIFO_init(FIFO *fifo)
{
	fifo->head = 0;
	fifo->tail = 0;
}

int FIFO_push(FIFO *fifo, char data)
{
	if ((fifo->tail - fifo->head) == 1 || (fifo->head - fifo->tail) == BUFFOR_SIZE)
	{
		return -1;
	}
	fifo->buffer[fifo->head] = data;
	fifo->head = (fifo->head + 1) & BUFFOR_SIZE;
	return 1;
}

int FIFO_pop(FIFO *fifo, char *data)
{
	if (fifo->head != fifo->tail)
	{
		*data = fifo->buffer[fifo->tail];
		fifo->tail = (fifo->tail + 1) & BUFFOR_SIZE;
		return 1;
	}
	return -1;
}

void FIFO_empty(FIFO *fifo)
{
	fifo->head = fifo->tail;
}

void asyncTransmisionInit()
{
	UCSR0A = 0b00000010;
	UCSR0B = 0b10011000;
	UCSR0C = 0b00000110;
	UBRR0  = 16;
}
