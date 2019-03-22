//#include <Arduino.h>
#include <IntervalCheckTimer.h>
#include <InterruptBasedSpeedMeasure.h>
#include <DCmotor.h>
#include <CarPIDController.h>
#include <PID_MotorControl.h>
#include <basic_speed_PID.h>


CARController Motor_l, Motor_r;

void setup()
{

  Serial.begin(9600);
  Serial.setTimeout(10);

  int pwmmotorpin_l = 8;
  int directionpin_l = 10;
  Motor_l.PID_SetupHardware(pwmmotorpin_l, directionpin_l, int_0);
  int pwmmotorpin_r = 5;
  int directionpin_r = 7;
  Motor_r.PID_SetupHardware(pwmmotorpin_r, directionpin_r, int_1); 
   
  // Setup The PID Controller
  unsigned long PIDUpdate_ms=50;
  unsigned long RPMUpdate_ms=50;
  unsigned long VerboseTime_ms=1000; 

  double lowerbound = 0, upperbound = 255;  // Bounds for PID output
  int control_interval_ms=30;         // PID control interval
  double Kp_l = 0.5, Ki_l = 0.025, Kd_l = 0.2; // Unscaled PID Values for left motor  
  double Kp_r = 0.6, Ki_r = 0.025, Kd_r = 0.2; // Unscaled PID Values for right motor    

  Motor_l.setPID_interval_vals(PIDUpdate_ms, RPMUpdate_ms, VerboseTime_ms);
  Motor_l.PID_SetupControl(lowerbound, upperbound, control_interval_ms);
  Motor_l.setSMAsize_Speedomotor(100);
  Motor_l.setTunings(Kp_l,Ki_l,Kd_l);
  
  Motor_r.setPID_interval_vals(PIDUpdate_ms, RPMUpdate_ms, VerboseTime_ms);
  Motor_r.PID_SetupControl(lowerbound, upperbound, control_interval_ms);
  Motor_r.setSMAsize_Speedomotor(100);
  Motor_r.setTunings(Kp_r,Ki_r,Kd_r);    

  Motor_l.jumpstartCar();
  Motor_r.jumpstartCar();  
}

void loop()
{
  Motor_l.CheckInputs_and_Control();
  Motor_r.CheckInputs_and_Control();
    

}
