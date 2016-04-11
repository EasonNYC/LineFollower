/*
 Name:		LineFollower.ino
 Created:	3/20/2012 4:03pm
 Author:	EpicZero

 This code was the code used for the LineFollower robot I used in the Suny Tyesa 2012 Line Following competition. Small edits
 have been made here and there to make it more readable (beginner programmer at the time), but here it is in all it's glory.
 Some notable things: I made a state machine somehow without knowing what that was. Also, the tuning PID live on the track over
 XBee radios is also a pretty cool feature. Not bad for a beginner. Also notice, there are no libraries. I wrote most of it
 from scratch and in some cases based my code around popular line following implimentation methods used by line followers.
 */

///////////////QUICK SETTINGS//////////////

//(NOTE: DEBUG should be kept turned off for competition runs)
#define DEBUG              
#define MOTORS
#define EIGHTSENSORS
#include <arduino.h>

//intitial values for kp and kd
int Kp = 7;
int Ki = 0;
int Kd = 95;
int TOPSPEED = 83;
int region = 0; //starting region

//////////////////////////////////////////


/*Currently works.

						pin reservations

						0- go button, rx for debug
						1- tx for debug
						2- sens 0
						3~ sens 1
						4- sens 2
						5~ M1-ENA-PWM ->p9 on the robot
						6~ M2-ENB-PWM ->p10
						7~ sens 3
						8- sens 4
						9(pwm)~-sens 5 (do not use for pwm?)
						10(pwm)~-sens 6 (do not use for pwm?)
						11(pwm)~-sens 7
						12-M1-HIGH ->P8/m1/high
						13-LED
						A0-M1 LOW ->A2/m1/low
						A1-M2 HIGH ->A1
						A2-M2 LOW ->A0
						A3-Servo/Cradle
						A4-front sensor
						A5-back sensor
						*/

/*
						sensor plug pins
						 43
						 52
						|61
						 70
						*GndVcc

						*/


//////////////////////BEGIN CODE//////////////////////////////////


#include <Servo.h> //really needed? can we just use pwm?


//Designate LED pin (not used currently)
int LED = 13;
int BOARD_BUTTON_PIN = 0;

//define calibration time
int calTime = 15000; //allow 15 seconds for calibration time (must coordinate this time with Master chip)

//define line color variables
byte blackorwhite; //holds what color line we are currently on
byte black = 0;
byte white = 1;
//sensor timeout
int timeout = 300; //directly affects PID calc frequency

#ifdef DEBUG
//counter variables for determining line reading latency
byte cCounts;
unsigned long cCounter = 0;
unsigned long cStartTime;
#endif

//what pins go where?
#ifdef EIGHTSENSORS
int sensorPins[] = {2,3,4,7,8,9,10,11}; //write the pins as an entire port
int numSensors = 8;
#endif

#define NUMSENSORS sizeof(sensorPins)  //autodefine numsensors doesnt work as well for some things, so it's not always used
unsigned int sensorValues[NUMSENSORS ];
int calSensorValue[NUMSENSORS ];

//calibration array
int useMin[2][NUMSENSORS ];
int useMax[2][NUMSENSORS ];

//calculate center position and choose default positional values on startup
int centerPosition = ((numSensors - 1) * 1000) / 2; //will report 2500 for 6 sensors, 3500 for 8
int lastValue = centerPosition;
int pos = centerPosition;
int senstotal = 0;

#ifdef DEBUG
byte posByte = 0B11011011; //for intersection detection
#endif

//more PID stuff
int kerror = 0;
int lastkerror = 0;
long integral = 0;

//SPEED variables
unsigned int m1Speed = 0;
unsigned int m2Speed = 0;
int motorSpeed;
//int oldspeed = TOPSPEED;

//serial variables
#ifdef DEBUG
int inByte1; //byte read from Serial
#endif

//region variables
int evidence = 0; //for region detection
int evidenceID = 0; //for intersection detection

void blinkLED(byte, int); //forward declaration
//////////////////////////////////////////////////////////////////SETUP////////////////////////////////////////////////////////////////

void setup()
{
	pinMode( LED, OUTPUT );

	blinkLED( 1, 100 );

	    DDRD = B00000010; //sets RX and TX (rightmost bits) correctly to give a chance to upload a new sketch on startup
		delay( 3000 );

#ifdef DEBUG 
		Serial.begin( 9600 );
#endif
		////////////////////////INITIALIZE MOTORS AND SERVOS

		blinkLED( 2, 100 );

		//initialize servo
		Servo myservo; 
		int servoPos = 0;  
		myservo.attach( A3 ); // servo on pin A3 
		myservo.write( 85 ); // write servo to angle 85 on startup

		//set motor pins as outputs
		pinMode( 5, OUTPUT );
		pinMode( 6, OUTPUT );
		pinMode( 12, OUTPUT );
		pinMode( 0, OUTPUT );
		pinMode( 1, OUTPUT );
		pinMode( 2, OUTPUT );

		//Set M1 direction to forward
		digitalWrite( 12, HIGH ); //-->p8/to go forward = high
		digitalWrite( A0, LOW ); //-->A2/forward = low
		
		//set M2 direction to forward 
		digitalWrite( A1, HIGH ); // --->A1/Forward = high
		digitalWrite( A2, LOW ); //--->A0/Forward = low

		/////////////////////////////////START OF LINE SENSOR CALIBRATION ROUTINE///////////////////////////////
		
		//calibrate black 
		while (millis() < calTime - 7000)
		{
			calibrateSensors( black );
		}

		blinkLED( 3, 100 );

#ifdef DEBUG
		Serial.println( "switch lines!" );
#endif
		delay( 3000 );


		//calibration white
		while (millis() < calTime)
		{
			calibrateSensors( white );
		}

		//signal end of calibration
		blinkLED( 4, 100 );

		//for pre run PID tuning 
#ifdef DEBUG
		Serial.println( "" );
		Serial.println( "Calibration complete." );
		Serial.println( "Adjust PID and press SHIFT-P to start." );
		Serial.println( "" );
		Serial.print( "Kp" );
		Serial.print( Kp, DEC );
		Serial.print( " Kd" );
		Serial.print( Kd, DEC );
		Serial.print( " TS" );
		Serial.println( TOPSPEED, DEC );
#endif


		//wait around for button press for robot to start it's run (LOW==not pressed)
		while (BOARD_BUTTON_PIN == LOW)
		{
		
		//tune PID while we wait
		#ifdef DEBUG 
			checkSerial();
		#endif
		}

		//adjust this delay to give time to lift finger off the button
		delay( 1000 );

		//LED indicator signal "go"
		digitalWrite( LED, HIGH ); // set the LED on

		//START WITH TURBO BOOST!
		analogWrite( 5, 255 ); //write to motor
		analogWrite( 6, 255 ); //write to motor
		cStartTime = millis(); //(for tracking latency out of the gate)
		delay( 150 );
	}


	///////////////////////////////////////////////MAIN LOOP////////////////////////////////////////////


	void loop()
			{
				/*big for loop for calculating jitter. Also uncomment the end of the for loop at end of main to enable.
				unsigned long tstartTime = micros();
				for(int i=0;i<10000;i++){
				counter(); */

				//read the line
				readSensors();

				//sum those readings to figure out which color line we are on...
				blackorwhite = whatLineAreWeOn();

				//calibrate our readings differently depending on the color of the line
				calResults( blackorwhite );

				//generate a position from these calibrated readings
				pos = getPosition( blackorwhite );
				
				//PID Controller
				kerror = pos - 3500; //convert our position to one which is relative to 0 as the midpoint and +-3500 as bounds
				motorSpeed = (Kp * kerror) + Ki*(integral+=kerror) + (Kd * (kerror - lastkerror));

				//constrain motor speeds to byte values between 0 and a pre-set TOPSPEED
				constrain(m1Speed,0,TOPSPEED);
				constrain(m2Speed,0,TOPSPEED);

				m1Speed = TOPSPEED + motorSpeed;
				m2Speed = TOPSPEED - motorSpeed;

				//update motors
#ifdef MOTORS
				analogWrite( 5, m1Speed ); //write to motor1
				analogWrite( 6, m2Speed ); //write to motor2
#endif
				//remember our last error for PID
				lastkerror = kerror;

				//where are we on the track
				checkRegion();

#ifdef DEBUG
				//debugCal();
				//debugRaw();
				//debugData();
				checkSerial(); //check for new PID values while running
#endif
			}

	/*uncomment for jitter debugging/tracking
	unsigned long result = (micros()-tstartTime)/10000;
	Serial.println(result, DEC);
	}*/

	/////////////////////////////////////////////////////////FUNCTIONS////////////////////////////////////////////////////////
	
	/*reads the sensors and collects raw sensor data*/
	void readSensors()
			{

				//charge the sensor capacitors
				for (int i = 0; i < numSensors; i++)
				{ 
					pinMode( sensorPins[i], OUTPUT );
				}

				for (int i = 0; i < numSensors; i++)
				{
					digitalWrite( sensorPins[i], HIGH );
				}

				// charge capacitors for 3 us
				delayMicroseconds( 3 ); 

				// set pins as inputs (but internal pull-up will be enabled...)
				for (int i = 0; i < numSensors; i++)
				{
					pinMode( sensorPins[i], INPUT ); 
				}
				for (int i = 0; i < numSensors; i++)
				{
					digitalWrite( sensorPins[i], LOW ); // so disable internal pull-up resistor
				}
				//end capacitor charging on sensor bar

				//reading the sensors...
				unsigned long startTime = micros();
				while (micros() - startTime < timeout)
				{
					// keep time and check all of the sensor pins (end after timeout)
					unsigned int time = micros() - startTime;

					for (int i = 0; i < numSensors; i++)
					{
						//keep the time as long as you read the pin as high , skip the pin if you read low (until timeout)

						if (digitalRead( sensorPins[i] ) == HIGH)
						{
							sensorValues[i] = time;
						}
					}
				}
			}


	/*function which figures out what color line the robot is on based on (RAW) sensor readings and track region transitions*/
	byte whatLineAreWeOn()
			{
				
				//holds the total sum of all uncalibrated sensor values
				senstotal = 0;

				//add all sensor values up
				for (int i = 0; i < numSensors; i++)
				{
					senstotal += sensorValues[i];
				}


				//if senstotal is less than half the sum of all sensors 
				if (senstotal < timeout * 4)
				{
					return black; //we are on the black line
				}
				else
				{
					return white; //we are on the white line
				}
			}

	/*calibrates the raw line readings into relational numeric values */
	void calResults( byte blackorwhite )
			{				

				//map values to a new array with 0 to 1000 resolution
				for (int i = 0; i < numSensors; i++)
				{
					//throw out results below a certain threshold (optional): 
					if (sensorValues[i] < 50)
					{
						calSensorValue[i] = 0;
					}

					//else map them to a value of 0 to 1000. these are the final values used for positional calculations, etc. 
					else
					{
						calSensorValue[i] = map( sensorValues[i], useMin[blackorwhite][i], useMax[blackorwhite][i], 0, 1000 ); //map takes a long time, consider just a simple x5 multiplication when we lower the caps to 2.2nf
					}
				}
			}

	/*computes a numerical positional value based on calibrated sensor values. */
	int getPosition( byte blackorwhite )
			{
	
				//for temp use in the function
				byte tempPosbyte = 0;

				//for math related uses
				long avg = 0;
				long sum = 0;

				//start off with the assumption that we are not on the line anymore
				byte ontheline = 0;

				//do this once for each sensor
				for (int i = 0; i < numSensors; i++)
				{
					int value = calSensorValue[i];

					//so that there are no numbers over 1000 (redundant?)		
					constrain( value, 0, 1000 );

					if (blackorwhite == white)
					{
						value = 1000 - value;
					}

					//prepare tempByte for new info
					tempPosbyte = tempPosbyte << 1;

					// do we see the line (accumilates into the answer when this forloop ends)
					if (value > 200) {
						ontheline = 1; //yes
					}
					else {
						/*to correctly compose the position byte (posByte) into the easiest type of visual aid to read in the serial monitor,
						we add a 1 to tempPos anywhere we "dont" see the line. We could have used a zero instead, but leftmost
						zeros in a byte disapear when they are presented to the serial monitor, so that would make it really
						hard to read when debugging. for example: 11110011 would look like 1100 on the serial monitor had we used 0.*/
						tempPosbyte = tempPosbyte + 1;
					}

					// only average in values that are above a noise threshold (50)
					if (value > 50)
					{
						avg += (long)(value) * (i * 1000);
						sum += value;
					}
				}

				//write the byte to posByte
				posByte = tempPosbyte;

				//when not on the line at all (helps to re-find the line)
				if (!ontheline)
				{
					// If it last read was to the left of center, return 0 (far left).
					if (lastValue < 3500)
					{
						return 0;
					}
					// If it last read to the right of center, return the max line position. (note make more effecient)
					else
					{
						return 7000;
					}
				}

				//calculate the average
				lastValue = avg / sum;

				//report the line position
				return lastValue;
			}

	/* blinks an LED numBlinks times with a set delay in ms between blinks*/
	void blinkLED( byte numBlinks, int delay_ms )
			{
				// LED off to start
				digitalWrite( LED, LOW ); 
				for (int i = 0; i < numBlinks; i++)
				{
					digitalWrite( LED, HIGH ); // set the LED on
					delay(delay_ms); 
					digitalWrite( LED, LOW ); // set the LED off
					delay(delay_ms);
				}
			}

	/*setup code used during calibration. This takes the track color and reads the sensors and figures out the min max values for each area of track we are on. */
	void calibrateSensors( byte color )
			{

				//initial read of the line sensors 
				readSensors();

				for (int i = 0; i < numSensors; i++)
				{
					if (sensorValues[i] > useMax[color][i])
					{
						useMax[color][i] = constrain( sensorValues[i], 0, 1000 );
					}
					else if (sensorValues[i] < useMin[color][i])
					{
						useMin[color][i] = constrain( sensorValues[i], 0, 1000 );
					}
				}

#ifdef DEBUG
				//print the min
				if (color == black)
				{
					Serial.print( "bMIN: " );
				}
				else
				{
					Serial.print( "wMIN: " );
				}

				for (int i = 0; i < numSensors; i++)
				{
					Serial.print( useMin[color][i], DEC );
					Serial.print( " " );
				}
				Serial.println( "" );
				//print the max
				if (color == black)
				{
					Serial.print( "bMAX: " );
				}
				else
				{
					Serial.print( "wMAX: " );
				}
				for (int i = 0; i < numSensors; i++)
				{
					Serial.print( useMax[color][i], DEC );
					Serial.print( " " );
				}
				Serial.println( "" );
#endif
		}//end calibrate sensors function

/*
//this function figure out the latency of the line readings/PID loop (in readings per second)
void counter(){
	
	if ((millis() - cStartTime) < 1000)
	{
		cCounter++;
	}

	else
	{
		cCounts = cCounter;
		cCounter = 0;
		cStartTime = millis();
	}
#endif 
}
*/

#ifdef DEBUG
/*prints calibrated data for debugging. (i.e. black and white appear the same).*/
void debugCal()
{

	if (blackorwhite == 0)
	{
		//for black line
		for (int i = 0; i < numSensors; i++)
		{
			Serial.print( calSensorValue[i] );
			Serial.println( " " );
		}
		Serial.println( "" );
	}
	else
	{
		//for white line
		for (int i = 0; i < numSensors; i++)
		{
			Serial.print( 1000 - calSensorValue[i] );
			Serial.println( " " );
		}
		Serial.println( "" );
	}
}

/*prints raw values of the sensors for debugging*/
void debugRaw()
{
	for (int i = 0; i < numSensors; i++)
	{
		Serial.print( sensorValues[i] );
		Serial.print( " " );
	}
	Serial.println( "" );
}

/*prints line color, position, and motor data for debugging*/
void debugData()
{
	Serial.print( "CLR:" );
	Serial.print( blackorwhite, DEC );
	Serial.print( " tot:" );
	Serial.print( senstotal, DEC );
	Serial.print( " MSP:" );
	Serial.print( motorSpeed, DEC );
	Serial.print( " p:" );
	Serial.print( pos, DEC );
	Serial.print( " p:" );
	Serial.print( posByte, BIN ); //visual of position
	Serial.print( " m1:" );
	Serial.print( m1Speed, DEC );
	Serial.print( " m2:" );
	Serial.println( m2Speed, DEC );
}
#endif

/*sets motors to go forward with a given speed */
void setForward(int TS)
{
	//set m1 direction to forward 
	digitalWrite(12, HIGH); //-->p8/forward = high
	digitalWrite(A0, LOW); //-->A2/forward = low

						   //set M2 
	digitalWrite(A1, HIGH); // --->A1/Forward = high
	digitalWrite(A2, LOW); //--->A0/Forward = low

						   //write new top speed
	TOPSPEED = TS;
}

//reverse driving direction with a given speed
void setReverse(int TS)
{
	//sets motors to reverse with given speed 

	//m1direction to reverse 
	digitalWrite(12, LOW); //-->p8/forward = high
	digitalWrite(A0, HIGH); //-->A2/forward = low

							//set M2 
	digitalWrite(A1, LOW); // --->A1/Forward = high
	digitalWrite(A2, HIGH); //--->A0/Forward = low

							//write new top speed
	TOPSPEED = TS;
}

//brakes(coasts) for a given duration
void Brake(int dur)
{

	//brake M1
	digitalWrite(12, LOW); //-->pin12/forward = high
	digitalWrite(A0, LOW); //-->A0/forward = low

						   //Brake M2  
	digitalWrite(A1, LOW); // --->A1/Forward = high
	digitalWrite(A2, LOW); //--->A2/Forward = low

						   //Brake for this many milliseconds
	delay(dur);

}

//This function brakes(coasts) and then stops.
void Stop()
{
	//brake and then stop
	Brake(200); //brakes/coasts for __ milliseconds
	TOPSPEED = 0; //full stop
}



/* This is a large state machine to track the colored regions on the track and change the robot's behavior when it enters them. 
There is/was a much better way to do this, but I didnt know what a statemachine was at the time or how to impliment a really nice 
function pointer based one (still a beginner at the time). I'm leaving this here for legacy reason's but might change this implimentation 
in the future when I have time.*/
void checkRegion()
{
	switch (region)
	{
	case 0:	//take off with the ball/(start for robot waiting)

		//intersection detection
		if (senstotal > 1000 && senstotal < 1500)
		{
			evidenceID++;
			if (evidenceID > 40)
			{
				evidenceID = 0;
				region = 2; //skip case 1 (which is used for the start for robot 2)

				#ifdef DEBUG
				Serial.print( "R:" );
				Serial.println( region, DEC );
				#endif
			}
		}
		break;

	case 1: //wait for ball (start for robot 2)
		
		if (senstotal  > 1000 && senstotal < 1500){
		evidenceID++;

			if (evidenceID > 40){
			evidenceID = 0;
			region = 2;

			#ifdef DEBUG
			Serial.print("R:");
			Serial.println(region, DEC);
			#endif		

			}
		}
		break;

	case 2: //now in turn 1

		if (blackorwhite == white)
		{
			
			evidence++;
			if (evidence > 50) {
				region = 3;
				evidence = 0;
				//TOPSPEED = oldspeed;
				//cStartTime = millis();
			#ifdef DEBUG
					Serial.print("R:");
					Serial.println(region, DEC);
			#endif
			}
		}
		break;

	case 3: //now in turn 2
		if(blackorwhite == black) {

			evidence++;
			if (evidence > 40) {

				//on exit of 3
				region = 4;
				evidence = 0;

				#ifdef DEBUG
						Serial.print("R:");
						Serial.println(region, DEC);
				#endif
			}
		}
		break;

	case 4: //thru the middle U turn

		if(blackorwhite==white)
		{
			evidence++;
			if (evidence > 50) {
				region = 5;
				evidence = 0;

			#ifdef DEBUG
					Serial.print("R:");
					Serial.println(region, DEC);
			#endif
			}
		}
		break;

	case 5: //in and around turn 3

		if (blackorwhite==black) {
			evidence++;
			if (evidence > 50) {


				region = 6;
				evidence = 0;

				//control speed on exit of region 5
				// TOPSPEED = 60;                     

			#ifdef DEBUG
				Serial.print("R:");
				Serial.println(region, DEC);
			#endif
			}
		}
		break;

	case 6: //coming around into turn 4 and aproaching start finish area
		if ((senstotal > 1000) && (senstotal<1500))
		{
			evidenceID++;
			if (evidenceID > 30) {
				/*
						unsigned long thisTime = millis();
						unsigned long laptime = thisTime - cStartTime;
						Serial.println(laptime, DEC);
						*/
				evidenceID = 0;

				//on entering start finish  
				region = 7;
				TOPSPEED = 40;
				
			#ifdef DEBUG
					Serial.print("R:");
					Serial.println(region, DEC);
			#endif
			}
		}
		break;

	case 7:
		//stop for now till bumper sensor placed on the robots
		if ((senstotal > 1000) && (senstotal<1500))
		{
			evidenceID++;
			if (evidenceID > 30) {

				//on exit of 7  
				evidenceID = 0;
				stop();
				region = 0;

		#ifdef DEBUG
				Serial.print("R:");
				Serial.println(region, DEC);
		#endif
			}
		}
		break;

	case 8: //stop if at the finish line (placeholder code)
		if(senstotal > 1000 && senstotal<1500)
		{
			evidenceID++;
			if (evidenceID > 30) {

				//on exit 
				evidenceID = 0;
				stop(); //
				region = 0;

		#ifdef DEBUG
				Serial.print("R:");
				Serial.println(region, DEC);
		#endif
			}
		}
		region=1;
		break;

	}//end switch
}//end checkRegion function



#ifdef DEBUG
/*checkSerial is the main workhorse for receiving PID parameters and adjustments and sending debug data
live over UART to the wireless xbee radio while the prototype robot was running. I should have split this into at least 
two smaller seperate smaller functions, but I didn't develop that programming habit until the follwoing semester after I 
completed this project*/
void checkSerial()
{
	// integerVar = (byte1Var * 256) + byte2Var //receive an int
	// Read what came in over Serial from laptop, adjust requested variable, send back new paramter:
	if (Serial.available() > 0)
	{
		inByte1 = Serial.read();

		switch (inByte1)
		{
		case 'q': // increase kp
			Kp++;
			Serial.println( "" );
			Serial.print( "kp" );
			Serial.println( Kp, DEC );
			break;
		case 'a': // decrease kp
			Kp--;
			Serial.println( "" );
			Serial.print( "kp" );
			Serial.println( Kp, DEC );
			break;
		case 'Q': // increase kp
			Kp += 10;
			Serial.println( "" );
			Serial.print( "kp" );
			Serial.println( Kp, DEC );
			break;
		case 'A': // decrease kp
			Kp -= 10;
			Serial.println( "" );
			Serial.print( "kp" );
			Serial.println( Kp, DEC );
			break;
		case 'w': // increase kd
			Kd++;
			Serial.println( "" );
			Serial.print( "kd" );
			Serial.println( Kd, DEC );
			break;
		case 's': // decrease kd
			Kd--;
			Serial.println( "" );
			Serial.print( "kd" );
			Serial.println( Kd, DEC );
			break;
		case 'W': // increase kd
			Kd += 10;
			Serial.println( "" );
			Serial.print( "kd" );
			Serial.println( Kd, DEC );
			break;
		case 'S': // decrease kd
			Kd -= 10;
			Serial.println( "" );
			Serial.print( "kd" );
			Serial.println( Kd, DEC );
			break;
		case 't': // increase Maxspeed
			TOPSPEED++;
			Serial.println( "" );
			Serial.print( "TS" );
			Serial.println( TOPSPEED, DEC );
			break;
		case 'T': // increase Maxspeed
			TOPSPEED += 10;
			Serial.println( "" );
			Serial.print( "TS" );
			Serial.println( TOPSPEED, DEC );
			break;
		case 'g': // decrease maxspeed
			TOPSPEED--;
			Serial.println( "" );
			Serial.print( "TS" );
			Serial.println( TOPSPEED, DEC );
			break;
		case 'G': // decrease maxspeed
			TOPSPEED -= 10;
			Serial.println( "" );
			Serial.print( "TS" );
			Serial.println( TOPSPEED, DEC );
			break;
		case 'z': // show kp
			Serial.println( "" );
			Serial.print( "Kp" );
			Serial.println( Kp, DEC );
			break;
		case 'x': // show kd
			Serial.println( "" );
			Serial.print( "Kd" );
			Serial.println( Kd, DEC );
			break;
		case 'b': // show topspeed
			Serial.println( "" );
			Serial.print( "TS" );
			Serial.println( TOPSPEED, DEC );
			break;
		case 'v': // show all 3
			Serial.println( "" );
			Serial.print( "Kp" );
			Serial.print( Kp, DEC );
			Serial.print( " Kd" );
			Serial.print( Kd, DEC );
			Serial.print( " TS" );
			Serial.print( TOPSPEED, DEC );
			break;
		case 'P': // unpause
			Serial.println( "" );
			Serial.print( "UNPAUSE:" );
			Serial.print( "Kp" );
			Serial.print( Kp, DEC );
			Serial.print( " Kd" );
			Serial.print( Kd, DEC );
			Serial.print( " TS" );
			Serial.print( TOPSPEED, DEC );
			Serial.println( "" );
			// lastkerror = 0;
			BOARD_BUTTON_PIN = 1;
			cStartTime = millis();
			break;

		case 'p': // pause
			#ifdef MOTORS
						//set motors to nuetral/off
						analogWrite( 5, 0 ); //write to motor
						analogWrite( 6, 0 ); //write to motor
			#endif

			Serial.println( "" );
			Serial.print( "PAUSED:" );
			Serial.print( "Kp" );
			Serial.print( Kp, DEC );
			Serial.print( " Kd" );
			Serial.print( Kd, DEC );
			Serial.print( " TS" );
			Serial.print( TOPSPEED, DEC );
			region = 1;
			BOARD_BUTTON_PIN = 0;
			while (BOARD_BUTTON_PIN == LOW)
			{
				//insert check serial  w/ no pause here

				if (Serial.available() > 0)
				{
					inByte1 = Serial.read();

					switch (inByte1)
					{
					case 'q': // increase kp
						Kp++;
						Serial.println( "" );
						Serial.print( "kp" );
						Serial.println( Kp, DEC );
						break;
					case 'a': // decrease kp
						Kp--;
						Serial.println( "" );
						Serial.print( "kp" );
						Serial.println( Kp, DEC );
						break;
					case 'Q': // increase kp
						Kp += 10;
						Serial.println( "" );
						Serial.print( "kp" );
						Serial.println( Kp, DEC );
						break;
					case 'A': // decrease kp
						Kp -= 10;
						Serial.println( "" );
						Serial.print( "kp" );
						Serial.println( Kp, DEC );
						break;
					case 'w': // increase kd
						Kd++;
						Serial.println( "" );
						Serial.print( "kd" );
						Serial.println( Kd, DEC );
						break;
					case 's': // decrease kd
						Kd--;
						Serial.println( "" );
						Serial.print( "kd" );
						Serial.println( Kd, DEC );
						break;
					case 'W': // increase kd
						Kd += 10;
						Serial.println( "" );
						Serial.print( "kd" );
						Serial.println( Kd, DEC );
						break;
					case 'S': // decrease kd
						Kd -= 10;
						Serial.println( "" );
						Serial.print( "kd" );
						Serial.println( Kd, DEC );
						break;
					case 't': // increase Maxspeed
						TOPSPEED++;
						Serial.println( "" );
						Serial.print( "TS" );
						Serial.println( TOPSPEED, DEC );
						break;
					case 'T': // increase Maxspeed
						TOPSPEED += 10;
						Serial.println( "" );
						Serial.print( "TS" );
						Serial.println( TOPSPEED, DEC );
						break;
					case 'g': // decrease maxspeed
						TOPSPEED--;
						Serial.println( "" );
						Serial.print( "TS" );
						Serial.println( TOPSPEED, DEC );
						break;
					case 'G': // decrease maxspeed
						TOPSPEED -= 10;
						Serial.println( "" );
						Serial.print( "TS" );
						Serial.println( TOPSPEED, DEC );
						break;
					case 'z': // show kp
						Serial.println( "" );
						Serial.print( "Kp" );
						Serial.println( Kp, DEC );
						break;
					case 'x': // show kd
						Serial.println( "" );
						Serial.print( "Kd" );
						Serial.println( Kd, DEC );
						break;
					case 'b': // show topspeed
						Serial.println( "" );
						Serial.print( "TS" );
						Serial.println( TOPSPEED, DEC );
						break;
					case 'v': // show all 3
						Serial.println( "" );
						Serial.print( "Kp" );
						Serial.print( Kp, DEC );
						Serial.print( " Kd" );
						Serial.print( Kd, DEC );
						Serial.print( " TS" );
						Serial.print( TOPSPEED, DEC );
						break;
					case 'P': // unpause
						Serial.println( "" );
						Serial.print( "UNPAUSE:" );
						Serial.print( "Kp" );
						Serial.print( Kp, DEC );
						Serial.print( " Kd" );
						Serial.print( Kd, DEC );
						Serial.print( " TS" );
						Serial.print( TOPSPEED, DEC );
						Serial.println( "" );
						cStartTime = millis();
						region = 1;
						lastkerror = 0; //restartPID
						BOARD_BUTTON_PIN = 1; //exit while loop
						//go for 1 second
						analogWrite( 5, 255 ); //write to motor
						analogWrite( 6, 255 ); //write to motor
						delay( 100 );
						break;
					}
				}
			}
			break;
		}
	}
}
#endif
