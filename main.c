/************************************************************************************
Team Id : eYRC-RM#2073
 * Author List : Shivam B. Waghela, Akshdeep Rungta
 * Filename: main.c
 * Theme: Recyclable Waste Management
 * Functions: follow(), checkObject(), pickWaste(), removeObstacle(), dump()
 * Global Variables: flag_drop_pos, blockCount, front_sharp_sensor, left_side_sharp_sensor,
 * object_found_by_side_sharp_sensor, ShaftCountRight, ShaftCountLeft
************************************************************************************/

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>

#include "rwm-headers.h"

unsigned char flag_drop_pos = 0;
volatile unsigned long int ShaftCountRight = 0;
volatile unsigned long int ShaftCountLeft = 0;
unsigned int blockCount = 0;
unsigned int object_found_by_side_sharp_sensor = 0;
unsigned int front_sharp_sensor = 11;
unsigned int left_side_sharp_sensor = 9;


/*
 * Function Name:	main
 * Input:		None
 * Output:		void
 * Logic:		Contains complete procedure to traverse the arena.
 *
 * Example Call:		Called automatically by the Operating System
 *
 */

int main()
{
	unsigned int rotate_right_90 = 90;
	unsigned int rotate_left_90 = 90;
	unsigned int rotate_right_45 = 45;
	unsigned int rotate_left_45 = 45;
	unsigned int rotate_right_135 = 135;
	unsigned int rotate_right_180 = 180;
	unsigned int rotate_left_180 = 180;
	
	init_devices();
	
	// set the initial position of all servo motors
	servo_1(180); // initial container position
	servo_2(180); // initial arm position (up position)
	servo_3(180); // initial gripper position (open position) 
	
	// Initially the bot will be facing in west direction 
	
	// Go to NODE 0
	follow();

	
	// NODE 0

	// check for object between NODE 0 and 3 and remember it in variable -> object_found_by_side_sharp_sensor
	checkObject(left_side_sharp_sensor);  // uses left side sharp sensor
	
	// check object in the front
	checkObject(front_sharp_sensor);  // uses front sharp sensor
	follow();
	
	
	// NODE 1
	checkObject(front_sharp_sensor);
	left_degrees(rotate_left_90); 
	checkObject(front_sharp_sensor);
	follow();
	
	
	// NODE 2
	right_degrees(rotate_right_135);
	checkObject(front_sharp_sensor);
	left_degrees(rotate_left_180);
	checkObject(front_sharp_sensor);
	left_degrees(rotate_left_45);
	checkObject(front_sharp_sensor);
	follow();
	
	
	// NODE 3
	
	// If blockCount is 6 then get ready for dumping the wastes at the nearest junction
	if (blockCount == 6)
	{
		// Go to south junction
		right_degrees(rotate_right_90);
		_delay_ms(500);
		follow();
		right_degrees(rotate_right_90);
		dump();
	}
	
	// Check if there was a block between NODE 0 and 3
	if (object_found_by_side_sharp_sensor == 1)
	{
		object_found_by_side_sharp_sensor = 0;
		left_degrees(rotate_left_90);
		checkObject(front_sharp_sensor);
		right_degrees(rotate_right_90);
		_delay_ms(500);
	}
	
	// If blockCount is 6 then get ready for dumping the wastes at the nearest junction
	if (blockCount == 6)
	{
		// Go to south junction
		right_degrees(rotate_right_90);
		follow();
		right_degrees(rotate_right_90);
		dump();
	}
	
	right_degrees(rotate_right_90);
	checkObject(front_sharp_sensor);
	
	// If blockCount is 6 then get ready for dumping the wastes at the nearest junction
	if (blockCount == 6)
	{
		// Go to south junction 
		follow();
		right_degrees(rotate_right_90);
		dump();
	}
	
	left_degrees(rotate_left_90);
	checkObject(front_sharp_sensor);
	follow();
	
	
	// NODE 4
	
	right_degrees(rotate_right_135);
	checkObject(front_sharp_sensor);
	
	// If blockCount is 6 then get ready for dumping the wastes at the nearest junction
	if (blockCount == 6)
	{
		// Go to south junction
		follow();
		right_degrees(rotate_right_45);
		dump();
	}
	
	left_degrees(rotate_left_180);
	checkObject(front_sharp_sensor);
	
	// If blockCount is 6 then get ready for dumping the wastes at the nearest junction
	if (blockCount == 6)
	{
		// Go to east junction
		follow();
		right_degrees(rotate_right_135);
		dump();
	}
	
	left_degrees(rotate_left_45);
	checkObject(front_sharp_sensor);
	follow();
	
	
	// NODE 5
	
	// If blockCount is 6 then get ready for dumping the wastes at the nearest junction
	if (blockCount == 6)
	{
		// Go to east junction
		right_degrees(rotate_right_90);
		follow();
		right_degrees(rotate_right_90);
		dump();
	}
	
	right_degrees(rotate_right_90);
	// change dropping position to right side
	flag_drop_pos = 1;
	checkObject(front_sharp_sensor);
	
	// If blockCount is 6 then get ready for dumping the wastes at the nearest junction
	if (blockCount == 6)
	{
		// Go to east junction
		follow();
		right_degrees(rotate_right_90);
		dump();
	}
	
	left_degrees(rotate_left_180);
	checkObject(front_sharp_sensor);
	
	// If blockCount is 6 then get ready for dumping the wastes at the nearest junction
	if (blockCount == 6)
	{
		// Go to east junction
		right_degrees(rotate_right_180);
		follow();
		right_degrees(rotate_right_90);
		dump();
	}
	follow();
	
	
	// NODE 0
	right_degrees(rotate_right_90);
	checkObject(front_sharp_sensor);
	follow();
	
	// If blockCount is 6 then get ready for dumping the wastes at the nearest junction
	if (blockCount == 6)
	{
		// Go near to north junction
		follow();
		follow();
		right_degrees(rotate_right_90);
		dump();
	}
	
	
	// NODE 6
	
	// check for object in west direction by using left side sensor
	checkObject(left_side_sharp_sensor);
	
	if (object_found_by_side_sharp_sensor == 1)
	{
		// object found by left side sensor
		object_found_by_side_sharp_sensor = 0;
		left_degrees(rotate_left_90);
		back_mm(15);
		checkObject(front_sharp_sensor);
		right_degrees(rotate_right_90);
	}
	
	// If blockCount is 6 then get ready for dumping the wastes at the nearest junction
	if (blockCount == 6)
	{
		// Go near to north junction
		follow();
		follow();
		right_degrees(rotate_right_90);
		dump();
	}
	
	right_degrees(rotate_right_90);
	back_mm(30);
	checkObject(front_sharp_sensor);
	_delay_ms(500);
	left_degrees(rotate_left_90);

	// If blockCount is 6 then get ready for dumping the wastes at the nearest junction
	if (blockCount == 6)
	{
		// Go near to north junction
		follow();
		follow();
		right_degrees(rotate_right_90);
		dump();
	}
		
	follow();
		
	
	// NODE 7
	
	// check for object in west direction
	checkObject(left_side_sharp_sensor);
	
	if (object_found_by_side_sharp_sensor == 1)
	{
		// object found by left side sensor
		object_found_by_side_sharp_sensor = 0;
		left_degrees(rotate_left_90);
		back_mm(40);
		checkObject(front_sharp_sensor);
		right_degrees(rotate_right_90);
	}
	
	if (blockCount == 6)
	{
		// Go near to north junction
		follow();
		right_degrees(rotate_right_90);
		dump();
	}
	
	forward_mm(20);
	right_degrees(rotate_right_90);
	checkObject(front_sharp_sensor);
	_delay_ms(500);
	left_degrees(rotate_left_90);
	
	// If blockCount is 6 then get ready for dumping the wastes at the nearest junction
	if (blockCount == 6)
	{
		// Go near to north junction
		follow();
		right_degrees(rotate_right_90);
		dump();
	}
	
	// Go near to north junction for dumping the wastes	
	follow();
	_delay_ms(500);
	right_degrees(rotate_right_90);
	dump();
	
}


/*
 * Function Name:	checkObject
 * Input:		sharp sensor to be used
 * Output:		void
 * Logic:		This function check for the object and if object is found it detects it type (i.e. whether it is a Waste or Obstacle). 
				It then call the pickWaste() or removeObstacle depending upon the type of block detected.
 *
 * Example Call:		checkObject(front_sharp_sensor) or checkObject(left_side_sharp_sensor)
 *
 */

void checkObject(int sharp_sensor)
{
	unsigned char sharp;
	unsigned int forward_dist = 0;  // used to store the distance in mm the robot has moved forward
	_delay_ms(500);
	sharp = ADC_Conversion(sharp_sensor); //Stores the Analog value of sharp sensor into variable "sharp"
	if ((sharp >= 60 && sharp_sensor == front_sharp_sensor) || (sharp >= 60 && sharp_sensor == left_side_sharp_sensor))
	{
		if (sharp >= 150)
		{
			back_mm(20);
		}
		else if (sharp <= 60)
		{
			forward_mm(50);
			forward_dist += 50;
		}
		if (sharp_sensor == left_side_sharp_sensor)
		{
			// remember that the the side sharp sensor has detected an object
			object_found_by_side_sharp_sensor = 1;
			return;
		}
		
		// keep the count of object detected	
		blockCount += 1;
		
		// get ready for picking up the object
		servo_2(0); // Lower the arm to pick
		_delay_ms(1000);
		servo_3(0);  // Grab the object
		 
		// detect the type of object (Waste/Obstacle)
		if (green_read() >= 5000)
		{
			buzzer_on();
			_delay_ms(1000);
			buzzer_off();
			
			// pick up the waste
			pickWaste();
		}
		else
		{
			buzzer_on();
			_delay_ms(1000);
			buzzer_off();
			
			// set aside the obstacle from the path
			removeObstacle();
		}
		
		back_mm(forward_dist - 20);
		
		_delay_ms(200);
		return;	
	}
	return;
}

/*
 * Function Name:	pickWaste
 * Input:		None
 * Output:		void
 * Logic:		Used to pick the waste and put it inside the container on the collection vehicle. 
 *
 * Example Call:		pickWaste()
 *
 */
void pickWaste()
{
	// servo_1() for container
	// servo_2() for arm
	// servo_3() for gripper
	_delay_ms(500);
	servo_2(180); // Get to dumping position
	_delay_ms(1500);
	servo_3(180); // dump it
	_delay_ms(500);
	
}


/*
 * Function Name:	removeObstacle
 * Input:		None
 * Output:		void
 * Logic:		Used to put the obstacle off the path
 *
 * Example Call:		removeObstacle()
 *
 */
void removeObstacle()
{
	// servo_1() for container
	// servo_2() for arm
	// servo_3() for gripper
	servo_2(30); // raise arm 
	
	
	// rotate 
	if (flag_drop_pos == 1) // change the default drop position to right side
	{
		left_degrees(55);
		_delay_ms(100);
		servo_3(180); // release the object
		_delay_ms(1000);
		right_degrees(55);
		
	}
	else
	{
		right_degrees(55);
		_delay_ms(100);
		servo_3(180); // release the object
		_delay_ms(1000);
		left_degrees(55);
	}
	
	_delay_ms(500);
	servo_2(180);
	_delay_ms(1000);
	
	// set the drop position to default (i.e to left side)
	flag_drop_pos = 0;
}

/*
 * Function Name:	dump
 * Input:		None
 * Output:		void
 * Logic:		Used to dump the waste from the container of collection vehicle to the garbage truck when the truck is detected by left side sharp sensor.
 *				It waits until the garbage truck is detected and then dumps all the waste blocks.
 * Example Call:		dump()
 *
 */
void dump(void)
{
	unsigned char sharp;
	
	// lower the arm so that it cannot hinder the dumping process
	servo_2(50);
	
	while (1)
	{
		sharp = ADC_Conversion(9);
		if (sharp >= 100)
		{
			break;
		}
	}
	_delay_ms(800);
	servo_1(60);
	_delay_ms(1000);
	servo_1(175);
	
	buzzer_on();
	_delay_ms(5000);
	buzzer_off();
	
	// THE END
	exit(0); // exit the program with status 0 
}

/*
 *
 * Function Name: follow
 * Input: void
 * Output: void
 * Logic: Uses PID line following algorithm to follow line from one node to another
 * Example Call: follow();
 *
*/
void follow()
 {
	unsigned char center_sensor = 0; //stores count from center white line sensor
	unsigned char left_sensor = 0; //stores count from center white line sensor
	unsigned char right_sensor = 0; //stores count from center white line sensor
	unsigned char black_threshold = 200;
	int lastproportional = 0, integral = 0;
	int error = 0;
	float kp = 1.8, ki = 0, kd = 4.75;
	while (1) 
	{
		center_sensor = ADC_Conversion(2);
		left_sensor = ADC_Conversion(3);
		right_sensor = ADC_Conversion(1);
		int sum = (center_sensor + left_sensor + right_sensor);
		if (sum >= black_threshold) 
		{
			stop();
			_delay_ms(10);
			forward();
			velocity(252, 255);
			stop();
			break;
		}
		else 
		{
				if (right_sensor <= left_sensor)
				{
					int position = (((right_sensor * 2) + center_sensor) * 250) / sum;
					int set_point = 250;
					int proportional = position - set_point;
					integral += proportional;
					int derivative = proportional - lastproportional;
					lastproportional = proportional;
					error = (proportional * kp + integral * ki + derivative * kd);
					int left_speed, right_speed;
					//Restricting the error value between +256.
					if (error < -255)
					error = -255;

					if (error > 255)
					error = 255;
					if (error < 0) {
						right_speed = 255 + error;
						left_speed = 255;
				}
				// If error_value is greater than zero calculate left turn values
				else
				{
					right_speed = 255;
					left_speed = 255 - error;
				}
				forward();
				velocity(right_speed, left_speed);
			}
			else
			{
				int position = (((left_sensor * 2) + center_sensor) * 250) / sum;
				int set_point = 250;
				int proportional = position - set_point;
				integral += proportional;
				int derivative = proportional - lastproportional;
				lastproportional = proportional;
				error = (proportional * kp + integral * ki + derivative * kd);
				int left_speed, right_speed;
				//Restricting the error value between +256.
				if (error < -255)
					error = -255;
				if (error > 255)
					error = 255;
				if (error < 0) 
				{
					right_speed = 255;
				left_speed = 255 + error;
				}
				// If error_value is greater than zero calculate left turn values
				else 
				{
					right_speed = 255 - error;
					left_speed = 255;
				}
				forward();
				velocity(right_speed, left_speed);	
			}
		}
	}
}


//ADC pin configuration
void adc_pin_config (void)
{
  DDRF = 0x00;  //set PORTF direction as input
  PORTF = 0x00; //set PORTF pins floating
  DDRK = 0x00;  //set PORTK direction as input
  PORTK = 0x00; //set PORTK pins floating
}

//Function to Initialize PORTS
void port_init()
{
	adc_pin_config();
	motion_pin_config();	
	left_encoder_pin_config();
	right_encoder_pin_config();
	buzzer_pin_config();
	color_sensor_pin_config();//color sensor pin configuration
	servo1_pin_config();
	servo2_pin_config();
	servo3_pin_config();
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref = 5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN = 1 --- ADIE = 1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function to configure ports to enable robot's motion
void motion_pin_config(void) {
	DDRA = DDRA | 0x0F;   //set direction of the PORTA 3 to PORTA 0 pins as output
	PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}


// Timer 5 initialized in PWM mode for velocity control
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}
//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibble for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibble to 0
 PortARestore |= Direction; // adding lower nibble for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}


void forward (void) 
{
  motion_set (0x06);
}

void stop (void)
{
  motion_set (0x00);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer1_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	color_sensor_pin_interrupt_init();
	sei();   //Enables the global interrupts
}


void left_encoder_pin_config(void)
{
	DDRE = DDRE & 0xEF;
	PORTE = PORTE | 0x10;
}
void right_encoder_pin_config(void)
{
	DDRE = DDRE & 0xDF;
	PORTE = PORTE | 0x20;
}
void left_position_encoder_interrupt_init(void)
{
	cli();
	EICRB = EICRB | 0x02;
	EIMSK = EIMSK | 0x10;
	sei();
}
void right_position_encoder_interrupt_init()
{
	cli();
	EICRB = EICRB | 0x08;
	EIMSK = EIMSK | 0x20;
}
//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++; //increment right shaft position count
	
}

//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++; //increment left shaft position count
}

void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
			break;
	}
	stop(); //Stop robot
}
void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	//velocity(220, 223);
	left(); //Turn left
	angle_rotate(Degrees);
}
void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	//velocity(220, 223);
	right(); //Turn right
	angle_rotate(Degrees);
}


// buzzer code 
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}


void linear_distance_mm(unsigned int DistanceInMM) {
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	ShaftCountRight = 0;
	while (1) {
		if (ShaftCountRight > ReqdShaftCountInt) {
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM) {
	forward();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM) {
	back();
	linear_distance_mm(DistanceInMM);
}


int has_turned = 0;
int last_turned = 0;


void timer1_init(void) {
    TCCR1B = 0x00; //stop
    TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
    TCNT1L = 0x01; //Counter low value to which OCR1xH value is to be compared with
    OCR1AH = 0x03; //Output compare Register high value for servo 1
    OCR1AL = 0xFF; //Output Compare Register low Value For servo 1
    OCR1BH = 0x03; //Output compare Register high value for servo 2
    OCR1BL = 0xFF; //Output Compare Register low Value For servo 2
    OCR1CH = 0x03; //Output compare Register high value for servo 3
    OCR1CL = 0xFF; //Output Compare Register low Value For servo 3
    ICR1H = 0x03;
    ICR1L = 0xFF;
    TCCR1A = 0xAB;
    /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
     					For Overriding normal port functionality to OCRnA outputs.
    				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
    TCCR1C = 0x00;
    TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}

//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
	float PositionServo = 0;
	PositionServo = ((float)degrees / 1.86) + 35.0;
	OCR1CH = 0x00;
	OCR1CL = (unsigned char) PositionServo;
}

//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free(void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free(void) //makes servo 2 free rotating
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free(void) //makes servo 3 free rotating
{
	OCR1CH = 0x03;
	OCR1CL = 0xFF; //Servo 3 off
}

	
// color sensor code
volatile unsigned long int pulse = 0; //to keep the track of the number of pulses generated by the color sensor
volatile unsigned long int green;       // variable to store the pulse count when read_green function is called

void color_sensor_pin_config(void)
{
	DDRD  = DDRD | 0xFE; //set PD0 as input for color sensor output
	PORTD = PORTD | 0x01;//Enable internal pull-up for PORTD 0 pin
}

void color_sensor_pin_interrupt_init(void) //Interrupt 0 enable
{
	cli(); //Clears the global interrupt
	EICRA = EICRA | 0x02; // INT0 is set to trigger with falling edge
	EIMSK = EIMSK | 0x01; // Enable Interrupt INT0 for color sensor
	sei(); // Enables the global interrupt
}

//Filter Selection
void filter_green(void)	//Used to select green filter
{
	//Filter Select - green filter
	PORTD = PORTD | 0x40; //set S2 High
	PORTD = PORTD | 0x80; //set S3 High
}

//Color Sensing Scaling
void color_sensor_scaling()		//This function is used to select the scaled down version of the original frequency of the output generated by the color sensor, generally 20% scaling is preferable, though you can change the values as per your application by referring datasheet
{
	//Output Scaling 20% from datasheet
	//PORTD = PORTD & 0xEF;
	PORTD = PORTD | 0x10; //set S0 high
	//PORTD = PORTD & 0xDF; //set S1 low
	PORTD = PORTD | 0x20; //set S1 high
}

long int green_read(void) // function to select green filter and display the count generated by the sensor on LCD. The count will be more if the color is green. The count will be very less if its blue or red.
{
	//Green
	filter_green(); //select filter
	pulse=0; //reset the count to 0
	_delay_ms(100); //capture the pulses for 100 ms or 0.1 second
	green = pulse;  //store the count in variable called green
	return green;
}

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
	DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
	PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

ISR(INT0_vect)
{
	pulse++; //increment on receiving pulse from the color sensor
}