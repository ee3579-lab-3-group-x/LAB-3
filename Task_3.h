//Included Libraries
#ifndef Task_3_h
#define Task_3_h
#include "IntervalCheckTimer.h"
#include "InterruptBasedSpeedMeasure.h"
#include "DCmotor.h"
#include "SystemControl_Unit.h"												

class task_3{
public:
//Classes and Objects
IntervalCheckTimer timer;													                  //Object "timer" declared from class "IntervalCheckTimer"
InterruptSpeedMeasure rotation_counter_L, rotation_counter_R;				//Objects "rotation_counter L&R" declared from class "InterruptSpeedMeasure"
SystemControler MotorLeft, MotorRight;										          //Objects "Motor L&R" declared from class "SystemControler"
HBridgeDCmotor hbdcmotor_R, hbdcmotor_L;									          //Objects "hbdcmotor L&R" declared from class "HBridgeDCmotor"
basic_speed_PID motor_L, motor_R;
//System Constants
const int Wc = 100;															                      //Width of car
const int wheel_diameter = 570;	                                    //Diameter of wheel (mm)
const int default_PWM = 60;                                         //Default PWM Value
const int default_speed = 800;                                      //Default Speed (RPM)

//Global Variables
double RPM_R;                                                       
double RPM_L;

//Setup Functions
void setup_Hardware()
{
    int motorpin_L=5;														                      //Left PWM Pin
    int directionpin_L=7;													                    //Left Direction Pin
    int motorpin_R=8;														                      //Right PWM Pin
    int directionpin_R=10;													                  //Right Direction Pin
    MotorLeft.PID_SetupHardware(motorpin_L, directionpin_L, int_0);   //Initialise Direction and PWM Pins for Left Motor
    MotorRight.PID_SetupHardware(motorpin_R, directionpin_R, int_1);  //Initialise Direction and PWM Pins for Left Motor
    hbdcmotor_R.setup_HBridgeDCmotor(motorpin_R,directionpin_R);      //Initialise Direction and PWM Pins for Left Motor
	  hbdcmotor_L.setup_HBridgeDCmotor(motorpin_L,directionpin_L);      //Initialise Direction and PWM Pins for Right Motor
}

void setup_speed_measurement(const int inp_interr_on_circle)          //Initialise Hall Effect Iteupt Readings
{
	rotation_counter_L.setupSpeedMeasure(int_0,inp_interr_on_circle);		//Initialise Left Speed Counter
	rotation_counter_R.setupSpeedMeasure(int_1,inp_interr_on_circle);		//Initialise Right Speed counter
}

void setup_PID()                                                              //Function to intialise PID Controller
{
	double lowerbound = 0, upperbound = 255;  								                  //Bounds for PID output
    const int control_interval_ms=30;         								                //PID control interval
	double Kp = 2, Ki = 0, Kd = 0.17; 										                      //Unscaled PID Values
	MotorLeft.setTunings(Kp,Ki,Kd); 										                        //Initialise Gain Parameters L
	MotorRight.setTunings(Kp,Ki,Kd);  										                      //Initialise Gain Parameters R
	MotorLeft.PID_SetupControl(lowerbound, upperbound, control_interval_ms);    //Set PWM Limits and Control Interval
	MotorRight.PID_SetupControl(lowerbound, upperbound, control_interval_ms);   //Set PWM Limits and Control Interval
}

void master_Setup()                                                           //Used Becuase I'm too dumb for constructors :/
{
	setup_Hardware();                                                           //Call Function to setup Hardware
	setup_speed_measurement(49);                                                //Call Function to initilase spped measurement
	setup_PID();                                                                //Call Function to setup PID Controller
}

//Calculation Functions
double distance_traveled_L()                                            //Function to calculate distance travelled by left wheel
{
	double rotations = rotation_counter_L.GetkDistanceCount();            //Count and Store Number of Rotations
	double distance = (rotations/49)*((3.1415926539)*wheel_diameter);	    //Calculate and Store Distance
	return distance;													                            //Return to allow for external acess
}

double distance_traveled_R()                                            //Function to calculate distance travelled by left wheel
{
	double rotations = rotation_counter_R.GetkDistanceCount();            //Count and Store Number of Rotations
	double distance = (rotations/49)*((3.1415926539)*wheel_diameter);     //Calculate and Store Distance
	return distance;                                                      //Return to allow for external acess
}

//Opperational Functions
void delta_speed_fun(int inp_speed, float delta_percent, int time_durr, bool add, int itterative_counter)
{
	float new_output_speed;													                        //Blank Local Variable
	timer.setInterCheck(time_durr);											                    //Initialised Check Timer 
	if(timer.isMinChekTimeElapsed())										                    //Boolean Timer Check
	{
		if (delta_percent >= 100)											                        //conditional dissalows invalid selection i.e 110%
		{
			new_output_speed = inp_speed;									                      //Return Input speed as safety measure
		}
		if(add == true){													                            //If add flag is set to true...
			new_output_speed = inp_speed + (inp_speed*(delta_percent/100));	    //Add Percentage
			return new_output_speed;										                        //Return Calculated Value
		}else if(add==false){												                          //If add flag is set to false
			new_output_speed = inp_speed - (inp_speed*(delta_percent/100));	    //Subtract Percentage
		}
		
		if(timer.isMinChekTimeElapsedAndUpdate())                             //If check Timer has elapsed...
		{
			RPM_R=rotation_counter_R.getRPMandUpdate();						              //Get Speed of Right Motor in RPM
			RPM_L=rotation_counter_L.getRPMandUpdate();						              //Get Speed of Left Motor in RPM
		}
		double PWM_L=motor_L.ComputePID_output(new_output_speed,RPM_L);       //Compute Required PWM output Right
   
		hbdcmotor_L.start();												                          //Start Left Motor
		hbdcmotor_L.setSpeedPWM(PWM_L);										                    //Set Speed in PWM
		
		double PWM_R=motor_R.ComputePID_output(new_output_speed,RPM_R);       //Compute Required PWM output Left
		hbdcmotor_R.start();												                          //Start Right Motor
		hbdcmotor_R.setSpeedPWM(PWM_R);										                    //Set Speed in PWM
	}
}

void rotate_clockwise(int radius, int inp_RPM_R)
{
  hbdcmotor_R.stop();                                       //Stop Right Motor
  hbdcmotor_L.stop();                                       //Stop Left Motor
  double circumfrance = 2*(3.1415926539)*radius;              //Calculate Circumfrance
  if(timer.isMinChekTimeElapsedAndUpdate())
  {
    RPM_L=rotation_counter_L.getRPMandUpdate();             //Get Speed of Left Motor in RPM
    RPM_R=rotation_counter_R.getRPMandUpdate();             //Get Speed of Right Motor in RPM
  }

  double distance = distance_traveled_R();                          //Calculate and Store Distance traveled
  if (distance <= circumfrance){                                     //If distance travelled is >= desired path
   hbdcmotor_R. set_jumpstart(true);                                //Jumpstart Right Motor
   hbdcmotor_L. set_jumpstart(true);                                //Jumpstart Left Motor
  double PWM_R=motor_R.ComputePID_output(inp_RPM_R,RPM_R);          //Compute Required PWM output
  double Target_RPM_L =  ((Wc+radius)/radius)*RPM_R;                //Calculate Left Wheel Velocity
  double PWM_L=motor_L.ComputePID_output(Target_RPM_L,RPM_L);       //Compute Required PWM output
  hbdcmotor_R.start();                                              //Start Left Motor
  hbdcmotor_R.setSpeedPWM(default_PWM);                             //Set Speed in PWM
  hbdcmotor_L.start();                                              //Start Right Motor
  hbdcmotor_L.setSpeedPWM(PWM_R);                                   //Set Speed in PWM
  }else{
  hbdcmotor_R.stop();                                                         //Stop Right Motor
  hbdcmotor_L.stop();                                                         //Stop Left Motor
  }
}

void rotate_counter_clockwise(int radius, int inp_RPM_L)
{
	hbdcmotor_R.stop();														            //Stop Right Motor
	hbdcmotor_L.stop();														            //Stop Left Motor
	double circumfrance = 2*(3.1415926539)*radius;							//Calculate Circumfrance
	if(timer.isMinChekTimeElapsedAndUpdate())
	{
		RPM_L=rotation_counter_L.getRPMandUpdate();							//Get Speed of Left Motor in RPM
		RPM_R=rotation_counter_R.getRPMandUpdate();							//Get Speed of Right Motor in RPM
	}

	double distance = distance_traveled_R();                          //Calculate and Store Distance traveled
	if (distance <= circumfrance){                                     //If distance travelled is >= desired path
   hbdcmotor_R. set_jumpstart(true);                                //Jumpstart Right Motor
   hbdcmotor_L. set_jumpstart(true);                                //Jumpstart Left Motor
  double PWM_L=motor_L.ComputePID_output(inp_RPM_L,RPM_L);          //Compute Required PWM output
  double Target_RPM_R =  ((Wc+radius)/radius)*RPM_L;                //Calculate Left Wheel Velocity
  double PWM_R=motor_R.ComputePID_output(Target_RPM_R,RPM_R);       //Compute Required PWM output
  hbdcmotor_L.start();                                              //Start Left Motor
  hbdcmotor_L.setSpeedPWM(default_PWM);                             //Set Speed in PWM
  hbdcmotor_R.start();                                              //Start Right Motor
  hbdcmotor_R.setSpeedPWM(PWM_R);                                   //Set Speed in PWM
	}else{
  hbdcmotor_R.stop();                                                         //Stop Right Motor
  hbdcmotor_L.stop();                                                         //Stop Left Motor
	}
}

void turn_90_L(int inp_RPM_L)                                                 //Function to turn 90 degrees to the left
{
	hbdcmotor_R.stop();														                              //Stop Right Motor
	hbdcmotor_L.stop();														                              //Stop Left Motor
	float quarter_turn_distance = 0.5*(3.1415926539)*(wheel_diameter/2);				//Calculate Distance
  double distance = distance_traveled_L();                                    //Calculate and Store Distance traveled
 
	if(timer.isMinChekTimeElapsedAndUpdate())                                   //If check timer has expired...
	{
		RPM_L=rotation_counter_L.getRPMandUpdate();							                  //Get Speed of Left Motor in RPM
	}
  if (distance <= quarter_turn_distance)                                      //If distance travelled is >= desired path
  {
  	double PWM_L=motor_L.ComputePID_output(inp_RPM_L,RPM_L);			              //Compute Required PWM output
    hbdcmotor_L. set_jumpstart(true);                                //Jumpstart Left Motor
  	hbdcmotor_L.start();													                              //Start Left Motor
  	hbdcmotor_L.setSpeedPWM(PWM_L);											                        //Set Speed in PWM
  }else{
    hbdcmotor_R.stop();													//Stop Right Motor
		hbdcmotor_L.stop();													//Stop Left Motor
	}
}

void turn_90_R(int inp_RPM_R)                                               //Function to turn 90 degrees to the right
{
  hbdcmotor_R.stop();                                                         //Stop Right Motor
  hbdcmotor_L.stop();                                                         //Stop Left Motor
  float quarter_turn_distance = 0.5*(3.1415926539)*(wheel_diameter/2);        //Calculate Distance
  double distance = distance_traveled_R();                                    //Calculate and Store Distance traveled
 
  if(timer.isMinChekTimeElapsedAndUpdate())                                   //If check timer has expired...
  {
    RPM_R=rotation_counter_R.getRPMandUpdate();                               //Get Speed of Left Motor in RPM
  }
  if (distance <= quarter_turn_distance)                                      //If distance travelled is >= desired path
  {
    double PWM_R=motor_R.ComputePID_output(inp_RPM_R,RPM_R);                    //Compute Required PWM output
    hbdcmotor_R. set_jumpstart(true);                                //Jumpstart Left Motor
    hbdcmotor_R.start();                                                        //Start Left Motor
    hbdcmotor_R.setSpeedPWM(PWM_R);                                             //Set Speed in PWM
  }else{
    hbdcmotor_R.stop();                         //Stop Right Motor
    hbdcmotor_L.stop();                         //Stop Left Motor
  }
}
};
#endif																		                                  //End Class Decleration						                                  //End Class Decleration
