//Fabio Lab 3 
//Task 1; PID Controlled Dual Motor
//Date 14/03/2019

#include <Arduino.h>
#include <SystemControl_Unit.h>

#include <IntervalCheckTimer.h>

SystemControler MotorLeft, MotorRight;
InterruptSpeedMeasure rotation_counter_L, rotation_counter_R;
IntervalCheckTimer speed_check, speed_change, dir_change;
bool motor_r, motor_l;

void setup()
{
  
  cli();// Disable Interrupts  
  motor_r = true;
  motor_l = true;
  Serial.begin(9600);  
  // Left motor
  int motorpin_L=5;// PWM
  int directionpin_L=7;// digital ON/OFF for the direction

  // Right motor
  int motorpin_R=8;// PWM
  int directionpin_R=10;// digital ON/OFF for the direction

  
  MotorLeft.PID_SetupHardware(motorpin_L, directionpin, int_0);
  MotorRight.PID_SetupHardware(motorpin_R, directionpin, int_1);  


  int inp_interr_on_circle=49;
  // enable the interrupt (int_0 works via pin2)
  // enable the interrupt (int_1 works via pin3)  
  rotation_counter_L.setupSpeedMeasure(int_0,inp_interr_on_circle);
  rotation_counter_R.setupSpeedMeasure(int_1,inp_interr_on_circle);

  MotorRight.setup_HBridgeDCmotor(motorpin_R,directionpin_R);
  MotorLeft.setup_HBridgeDCmotor(motorpin_L,directionpin_L);

  int initial_speed_pwm=60;
  if(motor_r)
  {
    MotorRight.start();
    MotorRight.setSpeedPWM(initial_speed_pwm);
  }
  if(motor_l)
  {
    MotorLeft.start();
    MotorLeft.setSpeedPWM(initial_speed_pwm);
  }

  // Setup The PID Controller
  unsigned long PIDUpdate_ms=100;
  unsigned long RPMUpdate_ms=50;
  unsigned long VerboseTime_ms=1000;   

  double lowerbound = 0, upperbound = 255;  // Bounds for PID output
  int control_interval_ms=30;         // PID control interval
  double Kp = 2, Ki = 0, Kd = 0.17; // Unscaled PID Values  

  MotorLeft.setPID_interval_vals(PIDUpdate_ms, RPMUpdate_ms, VerboseTime_ms);
  MotorLeft.PID_SetupControl(lowerbound, upperbound, control_interval_ms);
  MotorLeft.setSMAsize_Speedomotor(100);
  MotorLeft.setTunings(Kp,Ki,Kd);  

  MotorRight.setPID_interval_vals(PIDUpdate_ms, RPMUpdate_ms, VerboseTime_ms);
  MotorRight.PID_SetupControl(lowerbound, upperbound, control_interval_ms);
  MotorRight.setSMAsize_Speedomotor(100);
  MotorRight.setTunings(Kp,Ki,Kd);  
  sei();   
}

void loop()
{
    MotorLeft.CheckInputs_and_Control();
    MotorRight.CheckInputs_and_Control();    
}
