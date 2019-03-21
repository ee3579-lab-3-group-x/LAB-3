#include "Task_3.h"
task_3 obj;
const int turning_radius = 100;
const int input_RPM = 900;
const int percent_variation = 10;
const int timer = 10;
bool add_flag = true;

void setup() {
obj.master_Setup();
Serial.begin(19200);
}

void loop() {
  //obj.delta_speed_fun(input_RPM,percent_variation,timer,add_flag);
  //obj.rotate_counter_clockwise(turning_radius,input_RPM);
  //obj.rotate_clockwise(turning_radius,input_RPM);
  obj.turn_90_L(input_RPM);
  //obj.turn_90_R(input_RPM);
}
