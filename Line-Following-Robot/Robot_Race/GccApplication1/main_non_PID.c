/*
 * Robot_Race.c
 *
 * Created: 9/10/2025 1:05:37 pm
 * Author : zjp1369
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/*---------------------MACROS---------------------*/
/*-----------------BOARD OUTPUTS------------------*/
//led definitions
#define ledLeftOff (PORTA &= ~(1<<1))
#define ledLeftOn (PORTA |= (1<<1))
#define ledRightOff (PORTC &= ~(1<<4))
#define ledRightOn (PORTC |= (1<<4))

//Motor Definitions
//Left
#define LeftMotorForward {PORTB |= (1<<0); PORTA &= ~(1<<5);} //Left Motor Forwards  [dir1 = 1; pwm1 = 0]
#define LeftMotorBackward {PORTB &= ~(1<<0); PORTA |= (1<<5);} //Left Motor Backwards  [dir1 = 0; pwm1 = 0]
//Right
#define RightMotorForward {PORTB |= (1<<3); PORTC &= ~(1<<0);} // Right Motor Backwards [dir2 = 1; pwm2 = 0]
#define RightMotorBackward {PORTB &= ~(1<<3); PORTC |= (1<<0);} //Right Motot Forwards [dir2 = 0; pwm2 = 1]
//Stopping
#define LeftMotorStop {PORTB &= ~(1<<0); PORTA &= ~(1<<5);} //Left Motor Stop  [dir1 = 0; pwm1 = 0]
#define RightMotorStop {PORTB &= ~(1<<3); PORTC &= ~(1<<0);} // Right Motor Stop [dir2 = 0; pwm2 = 0]


/*-----------------BOARD INPUTS------------------*/

//switches
#define leftBumper !(PINA & (1<<2)) //active low
#define rightBumper !(PINA & (1<<1))//active low

//infra-red sensors
#define leftSensor !(PINA & (1<<7))
#define centreSensor !(PINA & (1<<6))
#define rightSensor !(PINA & (1<<3))

//direction definitions
#define CCW 1 // counterclockwise (0 or 1)
#define CW 0 // clockwise (0 or 1)

/*--------------GLOBAL VARIABLES-----------------*/
#define LEFT_MAX 190
#define RIGHT_MAX 190

#define SLIGHT_UPPER 0.7f
#define SLIGHT_LOWER 0.2f

float left_motor = 0;
float right_motor = 0;


/*-----------FUNCTION DECLERATIONS---------------*/
void Setup(void);
void SetLeftMotorSpeedandDirection(unsigned char Speed,char Direction);
void SetRightMotorSpeedandDirection(unsigned char Speed,char Direction);
void MotorSpeed(void);

void LeftCollision(void);
void RightCollision(void);

/*------------------MAIN LOOP--------------------*/
int main(void)
{
	Setup();
	
	while(1)
	{
 		ledLeftOff;
 		ledRightOff;
		
		//RightCollision();
		//LeftCollision();
		
		char sensor_state = ((leftSensor << 2) | (centreSensor << 1) | (rightSensor));
		
		switch (sensor_state)
		{
			case (0b010):
				left_motor = LEFT_MAX;
				right_motor = RIGHT_MAX;
				SetLeftMotorSpeedandDirection(left_motor, CW);
				SetRightMotorSpeedandDirection(right_motor, CW);
				break;
			case (0b001): //hard right urn
				ledRightOn;
				left_motor = LEFT_MAX;
				right_motor = 0;
				SetLeftMotorSpeedandDirection((left_motor), CW);
				SetRightMotorSpeedandDirection((right_motor), CW);
				break;
			case (0b100): //hard left turn
				ledLeftOn;
				left_motor = 0;
				right_motor = RIGHT_MAX;
				SetLeftMotorSpeedandDirection((left_motor), CW);
				SetRightMotorSpeedandDirection((right_motor), CW);
				break;
			case (0b110): // left + middle
				left_motor = (255*0.3)-(255-LEFT_MAX);
				right_motor = (255*0.7)-(255-RIGHT_MAX);
				SetLeftMotorSpeedandDirection(left_motor, CW);
				SetRightMotorSpeedandDirection(right_motor, CW);
				break;
			case (0b011): // middle + right
				left_motor = (255*0.7)-(255-LEFT_MAX);
				right_motor = (255*0.3)-(255-RIGHT_MAX);
				SetLeftMotorSpeedandDirection(left_motor, CW);
				SetRightMotorSpeedandDirection(right_motor, CW);
				break;
			case (0b000):
			case (0b101):
			case (0b111):
			default:
				SetLeftMotorSpeedandDirection(left_motor, CW);
				SetRightMotorSpeedandDirection(right_motor, CW);
				break;
		}
				        
	}
}
void Setup(void)
{
    /*----------SETTING INPUT OR OUTPUT--------------*/
    // Set ALL initial directions to INPUT
    DDRA = 0x00; 
    DDRB = 0x00;
    DDRC = 0x00;

    //Switches as Inputs
	DDRA &= ~(1<<0); //SW1
	DDRA &= ~(1<<1); //SW2
	
	//InfraRed Sensors as Inputs
	DDRA &= ~(1<<7); //Left
	DDRA &= ~(1<<6); //Center
	DDRA &= ~(1<<3); //Right

	//LEDS as Outputs
    DDRA |= (1 << 1); //Left Led
    DDRC |= (1 << 4); //Right Led

	//Motors as Outputs
	DDRB |= (1<<0); //Dir1
	DDRB |= (1<<3); //Di2
	DDRA |= (1<<5); //PWM1
	DDRC |= (1<<0); //PWM2

	/*---------------------TIMERS---------------------*/
	/* Timer setup for Motors*/
	TCCR0A = (1 << WGM01) | (1 << WGM00); // fast PWM channel A and B, Top = 0xFF
	TCCR0B = (1 << CS01) | (1 << CS00); // pre-scaling = /256
	OCR0A = 0; // Right motor off
	OCR0B = 0; // Left motor off
}

/***********************Left motor control****************************/
void SetLeftMotorSpeedandDirection(unsigned char Speed, char Direction) //speed 1-255;direction CW or CCW
{
    /*----------------SPEED CONSTRAINTS----------------*/
	if(Speed>255){Speed = 255;} // limit the maximum speed to 255
	if(Speed <= 0) // if 0 speed, stop the motor and disable the PWM
	{
		TCCR0A &= ~((1<< COM0B1) | (1<< COM0B0)); // Disconnect hardware PWM signal controlling the speed (OCR0B)
		LeftMotorStop; //Stops Left Motor
		OCR0B = 0; //Resets duty cycle
	}
    /*--------SETTING MOTOR SPEED+DIRECTION-----------*/
	else
	{
		if(Direction == CCW)
		{
			TCCR0A |= (1<< COM0B1);
			TCCR0A &= ~(1<< COM0B0); // clear OCR0B on compare match
			OCR0B = Speed; //sets the pwm duty cycle to be [speed/255*100]%
			LeftMotorBackward;
		}
		else if (Direction == CW)
		{
			TCCR0A |= (1 << COM0B1) | (1 << COM0B0); // set OCR0B on compare match
			OCR0B = Speed; //sets the pwm duty cycle to be [speed/255*100]%
			LeftMotorForward;// set left motor control line high
		}
	}
}
/************************Right motor control*******************************/
void SetRightMotorSpeedandDirection(unsigned char Speed, char Direction) //speed 1-255;direction CW or CCW
{
    /*----------------SPEED CONSTRAINTS----------------*/
	if(Speed>255) {Speed = 255;}// limit the maximum speed value to 100%
	if(Speed <= 0) // if 0 speed, stop the motor and disable the PWM
    {
        TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0)); // Disconnect hardware PWM signal controlling the speed (OCR0A)
		RightMotorStop; //Stops Right Motor
		OCR0A = 0; // resets duty cycle
	}
    /*--------SETTING MOTOR SPEED+DIRECTION-----------*/
	else
	{
		
		if(Direction == CCW)
		{
			TCCR0A |= (1 << COM0A1) | (1 << COM0A0); // set OCR0A on compare match
			OCR0A = Speed; //sets the pwm duty cycle to be [speed/255*100]%
			RightMotorForward; // set right motor control line high
		}
		else if(Direction == CW)
		{
			TCCR0A |= (1 << COM0A1);
			TCCR0A &= ~(1 << COM0A0); // clear OCR0A on compare match
			OCR0A = Speed; //sets the pwm duty cycle to be [speed/255*100]%
			RightMotorBackward; // Clear right motor control line high
		}
	}
}

// void LeftCollision(void)
// {
//     if (leftBumper)
//     {
//         // Stop briefly and then reverse both motors at a medium speed.
//         SetLeftMotorSpeedandDirection(0, CW);
//         SetRightMotorSpeedandDirection(0, CW);
//         _delay_ms(50);
//         
//         SetLeftMotorSpeedandDirection(LEFT_MAX - 100, CCW); // Reverse
//         SetRightMotorSpeedandDirection(RIGHT_MAX -150, CCW); // Reverse
// 
//         int timeout_counter = 300;
// 
//         // Keep reversing as long as all three sensors cannot see the line.
//         while (timeout_counter > 0)
//         {
//             // Check if ANY of the line sensors have detected the line.
//             if (leftSensor || centreSensor || rightSensor)
//             {
//                 break; // Line found, so exit the loop immediately.
//             }
// 
//             timeout_counter--; // Decrement the timer.
//             _delay_ms(10);     // Wait for 10ms before the next check.
//         }
// 
//         // As soon as the line is found, stop the motors.
//         SetLeftMotorSpeedandDirection(0, CW);
//         SetRightMotorSpeedandDirection(0, CW);
//     }
// }

//void RightCollision(void)
// {
//     if (rightBumper)
//     {
//         // Stop briefly and then reverse both motors at a medium speed.
//         SetLeftMotorSpeedandDirection(0, CW);
//         SetRightMotorSpeedandDirection(0, CW);
//         _delay_ms(50);
//         
//         SetLeftMotorSpeedandDirection(LEFT_MAX - 150, CCW); // Reverse
//         SetRightMotorSpeedandDirection(RIGHT_MAX - 100, CCW); // Reverse
// 
//         int timeout_counter = 300;
// 
//         // Keep reversing as long as all three sensors cannot see the line.
//         while (timeout_counter > 0)
//         {
//             // Check if ANY of the line sensors have detected the line.
//             if (leftSensor || centreSensor || rightSensor)
//             {
//                 break; // Line found, so exit the loop immediately.
//             }
// 
//             timeout_counter--; // Decrement the timer.
//             _delay_ms(10);     // Wait for 10ms before the next check.
//         }
// 
//         // As soon as the line is found, stop the motors.
//         SetLeftMotorSpeedandDirection(0, CW);
//         SetRightMotorSpeedandDirection(0, CW);
//     }
// }