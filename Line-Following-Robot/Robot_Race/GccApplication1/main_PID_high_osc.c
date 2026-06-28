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
#define CORRECTION_LIMIT 125.0f
#define INTEGRAL_LIMIT 50.0f
#define D_FILTER_TAU 0.05f
#define DT 0.008192f
#define alpha 0.004096f //(DT / (D_FILTER_TAU + DT));

#define Kp 17.0 //best 17
#define Ki 0.01
#define Kd 1.2

float base_speed = 0.0;
float left_speed = 0.0;
float right_speed = 0.0;
float correction = 0.0;
float last_error = 0.0;
float lastest_error = 0.0;
float lastestest_error = 0.0;
float lastestestest_error = 0.0;
float last_error1 = 0.0;
float current_error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float last_filtered_derivative = 0.0;

/*-----------FUNCTION DECLERATIONS---------------*/
void Setup(void);
void BasicTestCode(void);
void SensorTest(void);
void MotorTest(void);
void SetLeftMotorSpeedandDirection(unsigned char Speed,char Direction);
void SetRightMotorSpeedandDirection(unsigned char Speed,char Direction);
float CurrentErrorCalc(void);
float CorrectionCalc(float current_error);
void SpeedCalc(float correction);

/*------------------MAIN LOOP--------------------*/
int main(void)
{
	Setup();
	
	while(1)
	{
		ledLeftOff;
		ledRightOff;
				        
		current_error = CurrentErrorCalc();
        correction = CorrectionCalc(current_error);
        SpeedCalc(correction);

        // set motor speed and direction
        SetLeftMotorSpeedandDirection(left_speed, CW);
        SetRightMotorSpeedandDirection(right_speed, CW);
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
	TCCR0B = (1 << CS02); // pre-scaling = /256
	OCR0A = 0; // Right motor off
	OCR0B = 0; // Left motor off

    /*----------INTERRUPT SETUP FOR BUMPER----------*/
    // Enable Pin Change Interrupts for the group that contains PINA1
    GIMSK |= (1 << PCIE0); 
    // Unmask the specific pin (PINA1) to allow it to trigger an interrupt
    PCMSK0 |= (1 << PCINT1);
    
    // Enable global interrupts
/*    sei();*/
}

void BasicTestCode()
{
    /*-----------------Basic Test Code----------------*/
    //Sensors test
    if (!leftBumper || !leftSensor || !centreSensor)  
    {
        ledLeftOn;
    }
	_delay_ms(100);
	ledLeftOff;
	
	
	if (!rightBumper || !rightSensor || !centreSensor)
    {
        ledRightOn;
    }
	_delay_ms(100);
    ledRightOff;
}

void SensorTest(void)
{
	if (!rightSensor)
	{
		SetRightMotorSpeedandDirection(100, CW);
		ledRightOn;
		_delay_ms(200);
		ledRightOff;
		RightMotorStop;
		SetRightMotorSpeedandDirection(0, CW);
		_delay_ms(100);
	}
	if (!leftSensor)
	{
		SetLeftMotorSpeedandDirection(100, CW);
		ledLeftOn;
		_delay_ms(200);
		ledLeftOff;
		LeftMotorStop;
		SetLeftMotorSpeedandDirection(0, CW);
		_delay_ms(100);
	}
	if (!centreSensor)
	{
		SetRightMotorSpeedandDirection(100, CCW);
		SetLeftMotorSpeedandDirection(100, CCW);
		_delay_ms(200);
	}
}

void MotorTest(void)
{
	//test motors
	SetLeftMotorSpeedandDirection(255, CW);
	SetRightMotorSpeedandDirection(255, CW);
	_delay_ms(100);
	//change direction
	SetLeftMotorSpeedandDirection(255, CCW);
	SetRightMotorSpeedandDirection(255, CCW);
	_delay_ms(100);
	//turn motors off
	SetLeftMotorSpeedandDirection(0, CCW);
	SetRightMotorSpeedandDirection(0, CCW);
	_delay_ms(100);
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

float CurrentErrorCalc(void)
{
	char sensor_state = ((leftSensor << 2) | (centreSensor << 1) | (rightSensor));
	
	switch (sensor_state)
	{
		case (0b010):
			return 0.0;
		case (0b001): //hard right urn
			ledRightOn;
			return -20.0;
		case (0b100): //hard left turn
			ledLeftOn;
			return 20.0;
		case (0b110): // left + middle
			 return 15.0;
		case (0b011): // middle + right
			return -15.0;
		case (0b000):
		case (0b101):
		case (0b111):
		default:
			return last_error;
	}
}

float CorrectionCalc(float current_error)
{
    float c = 0; //initialise c

    derivative = (current_error - last_error) / DT;
    
    float filtered_derivative = (1.0f - alpha) * last_filtered_derivative + alpha * derivative;

    integral = integral + current_error * DT; //calculate integral

    //limit intergral
    if (integral > INTEGRAL_LIMIT){integral = INTEGRAL_LIMIT;}
    else if (integral < -INTEGRAL_LIMIT){integral = -INTEGRAL_LIMIT;}

    c = ((Kp * current_error) + (Ki * integral) + (Kd * derivative)); //calculate correction

    //set last error
    last_error = current_error;
// 	if (last_error > 100){last_error = 100;}
// 	if (last_error < -100){last_error = -100;}

    last_filtered_derivative = filtered_derivative;

    if (c > 255) c = 255;
    if (c < -255) c = -255;

    return c;
}

void SpeedCalc(float correction)
{
	if ((correction >= (15*Kp)) || (correction <= (-15*Kp)))
	{
		base_speed = 100;
	}
	else if ((correction >= (5*Kp)) || (correction <= (-5*Kp)))
	{
		base_speed = 170;
	}
	else
	{
		base_speed = 255;
	}
	
    left_speed = base_speed + correction; // If correction is positive (turn left), slow down left motor
    right_speed = base_speed - correction; // and speed up right motor

    //Speed Constraints
    if (left_speed < 0) left_speed = 0; 
    if (right_speed < 0) right_speed = 0;
    if (left_speed > 255) left_speed = 255;
    if (right_speed > 255) right_speed = 255;
}

ISR(PCINT0_vect)
{
    // First, check if the right bumper is actually pressed.
    // This prevents the interrupt from running accidentally.
    if ((rightBumper) || (leftBumper))
    {
        // Stop briefly and then reverse both motors at a medium speed.
        SetLeftMotorSpeedandDirection(0, CW);
        SetRightMotorSpeedandDirection(0, CW);
        _delay_ms(50);
        
        SetLeftMotorSpeedandDirection(150, CCW); // Reverse
        SetRightMotorSpeedandDirection(150, CCW); // Reverse

        // Keep reversing as long as all three sensors cannot see the line.
        while (!leftSensor && !centreSensor && !rightSensor)
        {
            // Do nothing, just wait and let the robot roll back.
        }

        // As soon as the line is found, stop the motors.
        SetLeftMotorSpeedandDirection(0, CW);
        SetRightMotorSpeedandDirection(0, CW);
    }
}
