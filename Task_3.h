//Included Libraries
#ifndef Task_3_h
#define Task_3_h
#include <IntervalCheckTimer.h>
#include <InterruptBasedSpeedMeasure.h>
#include <DCmotor.h>
#include <InterruptBasedSpeedMeasure.h>
#include <SystemControl_Unit.h>												

class task_3{
public:
//Classes and Objects
IntervalCheckTimer timer;													//Object "timer" declared from class "IntervalCheckTimer"
InterruptSpeedMeasure rotation_counter_L, rotation_counter_R;				//Objects "rotation_counter L&R" declared from class "InterruptSpeedMeasure"
SystemControler MotorLeft, MotorRight;										//Objects "Motor L&R" declared from class "SystemControler"
HBridgeDCmotor hbdcmotor_R, hbdcmotor_L;									//Objects "hbdcmotor L&R" declared from class "HBridgeDCmotor"

//System Constants
const int inp_interr_on_circle=49;											//Interupt Number
const int Wc = 6;															//Width of car	

//Global Variables
double RPM_R;
double RPM_L;

//Setup Functions
void setup_Hardware()
{
    int motorpin_L=5;														//Left PWM Pin
    int directionpin_L=7;													//Left Direction Pin
    int motorpin_R=8;														//Right PWM Pin
    int directionpin_R=10;													//Right Direction Pin
    MotorLeft.PID_SetupHardware(motorpin_L, directionpin, int_0);
    MotorRight.PID_SetupHardware(motorpin_R, directionpin, int_1);
    MotorRight.setup_HBridgeDCmotor(motorpin_R,directionpin_R);
	MotorLeft.setup_HBridgeDCmotor(motorpin_L,directionpin_L);
}

void setup_speed_measurement(inp_interr_on_circle)
{
	rotation_counter_L.setupSpeedMeasure(int_0,inp_interr_on_circle);		//Initialise Left Speed Counter
	rotation_counter_R.setupSpeedMeasure(int_1,inp_interr_on_circle);		//Initialise Right Speed counter
}

void setup_PID()
{
	double lowerbound = 0, upperbound = 255;  								//Bounds for PID output
    const int control_interval_ms=30;         								//PID control interval
	double Kp = 2, Ki = 0, Kd = 0.17; 										//Unscaled PID Values
	MotorLeft.setTunings(Kp,Ki,Kd); 										//Initialise Gain Parameters L
	MotorRight.setTunings(Kp,Ki,Kd);  										//Initialise Gain Parameters R
	MotorLeft.PID_SetupControl(lowerbound, upperbound, control_interval_ms);
	MotorRight.PID_SetupControl(lowerbound, upperbound, control_interval_ms);
}

//Opperational Functions
void delta_speed_fun(int inp_speed, float delta_percent, int time_durr, bool add)
{
	float new_output_speed;													//Blank Local Variable
	timer.setInterCheck(time_durr);											//Initialised Check Timer 
	if(timer.isMinChekTimeElapsed())										//Boolean Timer Check
	{
		if (delta_percent >= 100)											//conditional dissalows invalid selection i.e 110%
		{
			new_output_speed = inp_speed;									//Return Input speed as safety measure
		}
		if(add == true){													//If add flag is set to true...
			new_output_speed = inp_speed + (inp_speed*(delta_percent/100));	//Add Percentage
			return new_output_speed;										//Return Calculated Value
		}else if(add==false){												//If add flag is set to false
			new_output_speed = inp_speed - (inp_speed*(delta_percent/100));	//Subtract Percentage
		}
		
		if(timer.isMinChekTimeElapsedAndUpdate())
		{
			RPM_R=rotation_counter_R.getRPMandUpdate();						//Get Speed of Right Motor in RPM
			RPM_L=rotation_counter_L.getRPMandUpdate();						//Get Speed of Left Motor in RPM
		}
		
		double PWM_R=MotorRight.ComputePID_output(new_output_speed,RPM_R);	//Compute Required PWM output Left
		hbdcmotor_L.start();												//Start Left Motor
		hbdcmotor_L.setSpeedPWM(PWM_L);										//Set Speed in PWM
		
		double PWM_L=MotorLeft.ComputePID_output(new_output_speed,RPM_L);	//Compute Required PWM output Right
		hbdcmotor_R.start();												//Start Right Motor
		hbdcmotor_R.setSpeedPWM(PWM_R);										//Set Speed in PWM
	}
}

void rotate_counter_clockwise(int radius)
{
	hbdcmotor_R.stop();														//Stop Right Motor
	hbdcmotor_L.stop();														//Stop Left Motor
	float circumfrance = 2*(3.1415926539)*radius;							//Calculate circumfrance
	if(timer.isMinChekTimeElapsedAndUpdate())
	{
		RPM_L=rotation_counter_L.getRPMandUpdate();							//Get Speed of Left Motor in RPM
		RPM_R=rotation_counter_R.getRPMandUpdate();							//Get Speed of Right Motor in RPM
	}
	RPM_L =  ((Wc+radius)/radius)*RPM_R;				    				//Calculate Left Wheel Velocity
	double PWM_L=MotorLeft.ComputePID_output(speed_left,RPM_L);				//Compute Required PWM output
	hbdcmotor_L.start();													//Start Left Motor
	hbdcmotor_L.setSpeedPWM(PWM_L);											//Set Speed in PWM
	//Use circumfrance as limit inside an if statement i.e if(distance>=circumfrance) {motor.stop();}
}

void rotate_clockwise(int radius)
{
	hbdcmotor_R.stop();														//Stop Right Motor
	hbdcmotor_L.stop();														//Stop Left Motor
	float circumfrance = 2*(3.1415926539)*radius;							//Calculate Circumfrance
	if(timer.isMinChekTimeElapsedAndUpdate())
	{
		RPM_L=rotation_counter_L.getRPMandUpdate();							//Get Speed of Left Motor in RPM
		RPM_R=rotation_counter_R.getRPMandUpdate();							//Get Speed of Right Motor in RPM
	}
	RPM_R =  ((Wc+radius)/radius)*RPM_L;				    				//Calculate Left Wheel Velocity
	double PWM_R=MotorRight.ComputePID_output(speed_right,RPM_R);			//Compute Required PWM output
	hbdcmotor_R.start();													//Start Right Motor
	hbdcmotor_R.setSpeedPWM(PWM_R);											//Set Speed in PWM
	//Use circumfrance as limit inside an if statement i.e if(distance>=circumfrance) {motor.stop();}
}

float turn_90_L()
{
	hbdcmotor_R.stop();														//Stop Right Motor
	hbdcmotor_L.stop();														//Stop Left Motor
	float quarter_turn_distance = 0.5*(3.1415926539)*radius;				//Calculate Distance
	if(timer.isMinChekTimeElapsedAndUpdate())
	{
		RPM_L=rotation_counter_L.getRPMandUpdate();							//Get Speed of Left Motor in RPM
	}
	double PWM_L=MotorLeft.ComputePID_output(speed_left,RPM_L);				//Compute Required PWM output
	hbdcmotor_L.start();													//Start Left Motor
	hbdcmotor_L.setSpeedPWM(PWM_L);											//Set Speed in PWM
	//Use quarter_turn_distance as limit inside an if statement i.e if(distance>=quarter_turn_distance) {motor.stop();}	
}

float turn_90_R()
{
	hbdcmotor_R.stop();														//Stop Right Motor
	hbdcmotor_L.stop();														//Stop Left Motor
	float quarter_turn_distance = 0.5*(3.1415926539)*radius;				//Calculate Distance
	hbdcmotor_R.start();													//Start Right Motor
	if(timer.isMinChekTimeElapsedAndUpdate())
	{
		RPM_R=rotation_counter_R.getRPMandUpdate();							//Get Speed of Right Motor in RPM
	}
	double PWM_R=MotorRight.ComputePID_output(speed_right,RPM_R);			//Compute Required PWM output
	hbdcmotor_R.setSpeedPWM(PWM_R);											//Set Speed in PWM
	//Use quarter_turn_distance as limit inside an if statement i.e if(distance>=quarter_turn_distance) {motor.stop();}	
}
};
#endif																		//End Class Decleration
