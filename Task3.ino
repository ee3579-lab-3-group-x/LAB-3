#include "Task_3.h"
task_3 obj;
const int turning_radius = 100;
const int input_RPM = 900;
const int percent_variation = 10;
const int timer = 10;
const int speed_itterations = 10;
const int circle_itterations = 4; 
const int turning_angle = 90;
int RPM_LIMIT = 1000;
bool add_flag = true;

void setup() {
obj.master_Setup();
Serial.begin(19200);
}

void loop() {
  //obj.delta_speed_fun(input_RPM,percent_variation,timer,add_flag,speed_itterations,RPM_LIMIT);
  obj.rotate_counter_clockwise(turning_radius,input_RPM,circle_itterations);
  //obj.rotate_clockwise(turning_radius,input_RPM);
  //obj.turn_L(input_RPM, turning_angle);
  //obj.turn_R(input_RPM, turning_angle);
}