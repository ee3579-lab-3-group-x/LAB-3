//#include <Arduino.h>
#include <IntervalCheckTimer.h>
#include <InterruptBasedSpeedMeasure.h>
#include <DCmotor.h>

HBridgeDCmotor hbdcmotor_R, hbdcmotor_L;

InterruptSpeedMeasure rotation_counter_L, rotation_counter_R;
IntervalCheckTimer speed_check, speed_change, dir_change;


//FV
int target_index;
bool test_R, test_L;
//

void setup()
{
  
  ///// ===== paramters selection start ====
  //set to true to test the L motor
  test_L=true;

  //set to true to test the R motor
  test_R=true;
  
  
  // Left motor
  int motorpin_L=5;// PWM
  int directionpin_L=7;// digital ON/OFF for the direction

  // Right motor
  int motorpin_R=8;// PWM
  int directionpin_R=10;// digital ON/OFF for the direction

  // set 49 for the car; 6 for the small motors
  int inp_interr_on_circle=49;
  // enable the interrupt (int_0 works via pin2)
  // enable the interrupt (int_1 works via pin3)  
  rotation_counter_L.setupSpeedMeasure(int_0,inp_interr_on_circle);
  rotation_counter_R.setupSpeedMeasure(int_1,inp_interr_on_circle);
  ///// ===== paramters selection stop ====



  
  hbdcmotor_R.setup_HBridgeDCmotor(motorpin_R,directionpin_R);
  hbdcmotor_L.setup_HBridgeDCmotor(motorpin_L,directionpin_L);
  //hbdcmotor.enableEcho();
  
  // jump start the motor
  bool jumpstartmotor=false;
  if(jumpstartmotor)
  {
    hbdcmotor_R. set_jumpstart(jumpstartmotor);
    hbdcmotor_L. set_jumpstart(jumpstartmotor);
  }
  
  int initial_speed_pwm=60;
  if(test_R)
  {
    hbdcmotor_R.start();
    hbdcmotor_R.setSpeedPWM(initial_speed_pwm);
  }
  if(test_L)
  {
    hbdcmotor_L.start();
    hbdcmotor_L.setSpeedPWM(initial_speed_pwm);
  }
    

  
  
  // timer to perform speed measurement and control at given interval:
  // set the time between speed measurements/control)
  int speed_measurement_ms=1000;  
  speed_check.setInterCheck(speed_measurement_ms);
  
  unsigned long int speed_change_ms=4000;
  speed_change.setInterCheck(speed_change_ms);
  
  unsigned long int dir_change_ms=40000;
  dir_change.setInterCheck(dir_change_ms);
    
  Serial.begin(9600);  
  
  target_index=-1;
  
  
}

void loop()
{
  const int tot_speed_vals=8;
  int pwmspeedval[tot_speed_vals]={40, 70, 80, 140, 135, 45, 55, 45};

  int speed_pwm;
  
  if(speed_check.isMinChekTimeElapsedAndUpdate())
  {
    double RPM_L=rotation_counter_L.getRPMandUpdate();
    double RPM_R=rotation_counter_R.getRPMandUpdate();

    if(test_L)
    {
      Serial.print(" Left Revs per min  = ");
      Serial.println(RPM_L);
    }
    if(test_R)
    {
      Serial.print(" Right Revs per min  = ");
      Serial.println(RPM_R);    
    }
    Serial.println("");    
  }
  
  if(speed_change.isMinChekTimeElapsedAndUpdate())
  {
    target_index++;
    if(target_index>=tot_speed_vals)
      target_index=0;

    speed_pwm=pwmspeedval[target_index];
    
    Serial.print("Setting PWM to ");
    Serial.println(speed_pwm);
    
    if(test_L)
       hbdcmotor_L.setSpeedPWM(speed_pwm);
    if(test_R)
       hbdcmotor_R.setSpeedPWM(speed_pwm);
  }
 
 
 
    if(dir_change.isMinChekTimeElapsedAndUpdate())
    {
        Serial.println("-->Changing direction");
       if(test_L)
          hbdcmotor_L.stop();
       if(test_R)
          hbdcmotor_R.stop();        
        
        delay(1500);

        if(test_L)
        {
          hbdcmotor_L.changedir();
          hbdcmotor_L.start();
          hbdcmotor_L.setSpeedPWM(speed_pwm);
        }
        if(test_R)
        {
          hbdcmotor_R.changedir();
          hbdcmotor_R.start();
          hbdcmotor_R.setSpeedPWM(speed_pwm);
        }
    }
    
  
}
