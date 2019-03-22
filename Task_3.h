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
const int Wc = 100;															                     //Width of car (mm)
const int wheel_diameter = 57;	                                    //Diameter of wheel (mm)

//Global Variables
double RPM_R;                                                       
double RPM_L;
double PWM_R;
double PWM_L;

//Setup Functions
void setup_Hardware(int motorpin_L,int directionpin_L,int motorpin_R,int directionpin_R)
{
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

void setup_PID(double Kp_R,double Kp_L,double Ki,double Kd,double lowerbound,double upperbound,const int control_interval_ms)    //Function to intialise PID Controller
{
	MotorLeft.setTunings(Kp_L,Ki,Kd); 										                        //Initialise Gain Parameters L
	MotorRight.setTunings(Kp_R,Ki,Kd);  										                      //Initialise Gain Parameters R
	MotorLeft.PID_SetupControl(lowerbound, upperbound, control_interval_ms);    //Set PWM Limits and Control Interval
	MotorRight.PID_SetupControl(lowerbound, upperbound, control_interval_ms);   //Set PWM Limits and Control Interval
}

void Setup_Hardware_PID(const int motorpin_L,const int directionpin_L,const int motorpin_R,const int directionpin_R,const int inp_interr_on_circle,double Kp_L,double Kp_R,double Ki,double Kd,double lowerbound,double upperbound,const int control_interval_ms)                                                           //Used Becuase I'm too dumb for constructors :/
{
	setup_Hardware(motorpin_L,directionpin_L, motorpin_R,directionpin_R);       //Call Function to setup Hardware
	setup_speed_measurement(inp_interr_on_circle);                                                //Call Function to initilase spped measurement
	setup_PID(Kp_R,Kp_L,Ki,Kd,lowerbound,upperbound,control_interval_ms);              //Call Function to setup PID Controller
}

//Calculation Functions
double distance_traveled_L()                                                  //Function to calculate distance travelled by left wheel
{
	double distance = rotation_counter_L.GetkDistanceCount();             //Count and Store Number of Interupts
	//double distance = (interupt_count/49)*((3.1415926539)*wheel_diameter);	    //Calculate and Store Distance (mm)
	return distance;													                                  //Return to allow for external acess
  //return interupt_count;
}

double distance_traveled_R()                                                  //Function to calculate distance travelled by left wheel
{
	double distance = rotation_counter_R.GetkDistanceCount();            //Count and Store Number of Rotations
	//double distance = (interupt_count/49)*((3.1415926539)*wheel_diameter);     //Calculate and Store Distance (mm)
	return distance;                                                           //Return to allow for external acess
 //return interupt_count;
}

//Opperational Functions
void delta_speed_fun(int inp_speed, float delta_percent, int time_durr, bool add, int itterative_counter, int RPM_LIMIT)
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
     for (int i = 0; i < itterative_counter; i++)
     if(RPM_LIMIT>=RPM_L||RPM_LIMIT>=RPM_R)
     {
        hbdcmotor_R.stop();                                                   //Stop Right Motor
        hbdcmotor_L.stop();                                                   //Stop Left Motor
     }else{
    		double PWM_L=motor_L.ComputePID_output(new_output_speed,RPM_L);       //Compute Required PWM output Right
        hbdcmotor_L. set_jumpstart(true);                                     //Jumpstart Left Motor
    		hbdcmotor_L.start();												                          //Start Left Motor
    		hbdcmotor_L.setSpeedPWM(PWM_L);										                    //Set Speed in PWM
    		double PWM_R=motor_R.ComputePID_output(new_output_speed,RPM_R);       //Compute Required PWM output Left
        hbdcmotor_R. set_jumpstart(true);                                      //Jumpstart Right Motor
    		hbdcmotor_R.start();												                          //Start Right Motor
    		hbdcmotor_R.setSpeedPWM(PWM_R);										                    //Set Speed in PWM
        i++;
    }
	}
}
void drive_straight(int inp_RPM, int distance)
{
    hbdcmotor_R.stop();                                       //Stop Right Motor
    hbdcmotor_L.stop();                                       //Stop Left Motor
    if(timer.isMinChekTimeElapsedAndUpdate())
  {
    RPM_L=rotation_counter_L.getRPMandUpdate();             //Get Speed of Left Motor in RPM
    RPM_R=rotation_counter_R.getRPMandUpdate();             //Get Speed of Right Motor in RPM
  }
  double distance_R = distance_traveled_R();                          //Calculate and Store Distance traveled
  double distance_L = distance_traveled_L();                          //Calculate and Store Distance traveled
  Serial.println("Interput Count");
  Serial.print(distance_R);
  if ((distance_R <= distance)||(distance_L <= distance))
  {

   PWM_R=motor_R.ComputePID_output(inp_RPM,RPM_R);            //Compute Required PWM output
   PWM_L=motor_L.ComputePID_output(inp_RPM,RPM_L);            //Compute Required PWM output
  }else{
  PWM_R=0;
  PWM_L=0;
  }
  hbdcmotor_R. set_jumpstart(true);                                 //Jumpstart Right Motor
  hbdcmotor_L. set_jumpstart(true);                                 //Jumpstart Left Motor
  hbdcmotor_R.start();                                              //Start Right Motor
  hbdcmotor_L.start();                                              //Start Left Motor
  hbdcmotor_R.setSpeedPWM(PWM_R);                                   //Set Speed in PWM
  hbdcmotor_L.setSpeedPWM(PWM_L);                                   //Set Speed in PWM
}

void rotate_clockwise(int radius, int inp_RPM_R, int itterations)
{
  hbdcmotor_R.stop();                                       //Stop Right Motor
  hbdcmotor_L.stop();                                       //Stop Left Motor
  double circumfrance = 2*(3.1415926539)*radius;              //Calculate Circumfrance
  if(timer.isMinChekTimeElapsedAndUpdate())
  {
    RPM_L=rotation_counter_L.getRPMandUpdate();             //Get Speed of Left Motor in RPM
    RPM_R=rotation_counter_R.getRPMandUpdate();             //Get Speed of Right Motor in RPM
  }
  double distance = distance_traveled_L();                          //Calculate and Store Distance traveled
  if (distance <= circumfrance*itterations){                        //If distance travelled is >= desired path
   double Target_RPM_L =  ((Wc+radius)/radius)*inp_RPM_R;                //Calculate Left Wheel Velocity
   PWM_R=motor_R.ComputePID_output(inp_RPM_R,RPM_R);          //Compute Required PWM output
   PWM_L=motor_L.ComputePID_output(Target_RPM_L,RPM_L);       //Compute Required PWM output

  }else{
  hbdcmotor_R.stop();                                               //Stop Right Motor
  hbdcmotor_L.stop();                                               //Stop Left Motor
  }
  hbdcmotor_R. set_jumpstart(true);                                 //Jumpstart Right Motor
  hbdcmotor_L. set_jumpstart(true);                                 //Jumpstart Left Motor
  hbdcmotor_R.start();                                              //Start Right Motor
  hbdcmotor_L.start();                                              //Start Left Motor
  hbdcmotor_R.setSpeedPWM(PWM_R);                                   //Set Speed in PWM
  hbdcmotor_L.setSpeedPWM(PWM_L);                                   //Set Speed in PWM
}

void rotate_counter_clockwise(int radius, int inp_RPM_L, int itterations)
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
	if (distance <= circumfrance*itterations){                        //If distance travelled is >= desired path
    double Target_RPM_R =  ((Wc+radius)/radius)*inp_RPM_L;            //Calculate Left Wheel Velocity
   PWM_L=motor_L.ComputePID_output(inp_RPM_L,RPM_L);          //Compute Required PWM output
   PWM_R=motor_R.ComputePID_output(Target_RPM_R,RPM_R);       //Compute Required PWM output
	}else{
  PWM_R=0;
  PWM_L=0;
	}
  hbdcmotor_R. set_jumpstart(true);                                 //Jumpstart Right Motor
  hbdcmotor_L. set_jumpstart(true);                                 //Jumpstart Left Motor
  hbdcmotor_R.start();                                              //Start Right Motor
  hbdcmotor_L.start();                                              //Start Left Motor
  hbdcmotor_R.setSpeedPWM(PWM_R);                                   //Set Speed in PWM
  hbdcmotor_L.setSpeedPWM(PWM_L);                                   //Set Speed in PWM
}

void turn_L(int inp_RPM_L,int angle)                                                 //Function to turn 90 degrees to the left
{
	hbdcmotor_R.stop();														                              //Stop Right Motor
	hbdcmotor_L.stop();														                              //Stop Left Motor
  float turn_distance = (2*(3.1415926539)*(Wc))*(angle/360);                  //Calculate Distance
  double distance = distance_traveled_L();                                    //Calculate and Store Distance traveled
 
	if(timer.isMinChekTimeElapsedAndUpdate())                                   //If check timer has expired...
	{
		RPM_L=rotation_counter_L.getRPMandUpdate();							                  //Get Speed of Left Motor in RPM
	}
  if (distance <= turn_distance)                                      //If distance travelled is >= desired path
  {
  	double PWM_L=motor_L.ComputePID_output(inp_RPM_L,RPM_L);			              //Compute Required PWM output

  }else{
  PWM_L=0;
	}
     hbdcmotor_L. set_jumpstart(true);                                //Jumpstart Left Motor
   hbdcmotor_L.start();                                                        //Start Left Motor
    hbdcmotor_L.setSpeedPWM(PWM_L);                                             //Set Speed in PWM
}

void turn_R(int inp_RPM_R,int angle)                                               //Function to turn 90 degrees to the right
{
  hbdcmotor_R.stop();                                                         //Stop Right Motor
  hbdcmotor_L.stop();                                                         //Stop Left Motor
  float turn_distance = (2*(3.1415926539)*(Wc))*(angle/360);        //Calculate Distance
  double distance = distance_traveled_R();                                    //Calculate and Store Distance traveled
 
  if(timer.isMinChekTimeElapsedAndUpdate())                                   //If check timer has expired...
  {
    RPM_R=rotation_counter_R.getRPMandUpdate();                               //Get Speed of Left Motor in RPM
  }
  if (distance <= turn_distance)                                      //If distance travelled is >= desired path
  {
    double PWM_R=motor_R.ComputePID_output(inp_RPM_R,RPM_R);                    //Compute Required PWM output
  }else{
  PWM_R=0;
  }
    hbdcmotor_R. set_jumpstart(true);                                //Jumpstart Left Motor
    hbdcmotor_R.start();                                                        //Start Left Motor
    hbdcmotor_R.setSpeedPWM(PWM_R);                                             //Set Speed in PWM
}

void task_4_fun(int radius, int inp_RPM, int itterations, int distance,int inp_turning_RPM)
{
  int total_length = (((2*(distance))+(2*(3.1415926539)*radius))*itterations);
  double distance_R = distance_traveled_R();                          //Calculate and Store Distance traveled
  double distance_L = distance_traveled_L();                          //Calculate and Store Distance traveled
  Serial.print(distance_L);
  if ((distance_R <= total_length)||(distance_L <= total_length))     
  {
  drive_straight(inp_RPM,distance);
  rotate_clockwise(radius, inp_turning_RPM, 1);
  drive_straight(inp_RPM,distance);
  rotate_clockwise(radius, inp_turning_RPM, 1);
  }else{
  hbdcmotor_R.stop();                                               //Stop Right Motor
  hbdcmotor_L.stop();                                               //Stop Left Motor
  }
}
};
#endif																		                                  //End Class Decleration
