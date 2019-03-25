//Included Library
#include "Task_3.h"
//Classes and Objects
task_3 obj;
//System Constants
const int turning_radius = 80;                                        //Radius of circle (mm)
const int distance = 300;
const int input_RPM = 600;                                              //Default Input RPM
const int percent_variation = 10;                                       //Percentage Variation in speed
const int timer = 1;                                                   //RPM Check Timer
const int speed_itterations = 10;                                       //Itteration of delta_speed_fun(); loop
const int circle_itterations = 1;                                       //Itteration of driving in donut
const int turning_angle = 90;                                           //Angle of turing
int RPM_LIMIT = 1000;                                                   //Maximum RPM Limit
bool add_flag = true;                                                   //Addition Flag
const int task_4_itterations = 4;
const int inp_turning_RPM = 100;

//Hardware Parameters
const int motorpin_L=5;                                                  //Left PWM Pin
const int directionpin_L=7;                                              //Left Direction Pin
const int motorpin_R=8;                                                  //Right PWM Pin
const  int directionpin_R=10;                                            //Right Direction Pin
const int inp_interr_on_circle = 49;                                     //Magnets on circle
//PID Parameters
double lowerbound = 0, upperbound = 255;                                 //Bounds for PID output
const int control_interval_ms=1;                                        //PID control interval
double Kp_L =1,Kp_R =1.04, Ki = 0.25, Kd = 0.2;                       //Unscaled PID Values     
//Setup Function
void setup() {
obj.Setup_Hardware_PID(motorpin_L,directionpin_L,motorpin_R,directionpin_R,inp_interr_on_circle,Kp_L,Kp_R,Ki,Kd,lowerbound,upperbound,control_interval_ms);
Serial.begin(19200);
}

//Main Functions
void loop() {
  //obj.drive_straight(300,200);
  //obj.delta_speed_fun(input_RPM,percent_variation,timer,add_flag,speed_itterations,RPM_LIMIT);
  //obj.rotate_counter_clockwise(turning_radius,input_RPM,circle_itterations);                      //Working!!!
  //obj.rotate_clockwise(turning_radius,input_RPM,circle_itterations);                      //Working!!!
  //obj.turn_R(300, 90);                                                                    //Friction Is Problem!!
  //obj.turn_L(800, 90);                                                    //Friction Is Problem!!
  obj.task_4_fun(turning_radius,input_RPM,task_4_itterations,distance,inp_turning_RPM);         //Warning (mm) only!!!
}
