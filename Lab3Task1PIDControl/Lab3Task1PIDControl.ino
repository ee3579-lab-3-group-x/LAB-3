// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       Task2.ino
    Created:  25/02/2019 14:05:12
    Author:     LAPTOP-OLI\olive
*/

#include <Arduino.h>
//#include <PID_v1.h>
#include "SystemControl_Unit.h"
// Define User Types below here or use a .h file
//


// Define Function Prototypes that use User Types below here or use a .h file
//
SystemControler Motor1, Motor2;

// Define Functions below here or use other .ino or cpp files
//


// The setup() function runs once each time the micro-controller starts
void setup()
{
  
  cli();// Disable Interrupts
  Serial.begin(9600);
  Serial.setTimeout(10);

  // add three pushbuttons (pin and label)
  // change the pin number as appropriate but keep the labels (switch_on, switch_off, change_spin_dir)
  Motor1.add_buttonpin_and_label(6, switch_on);
  Motor1.add_buttonpin_and_label(4, switch_off);
  Motor1.add_buttonpin_and_label(7, change_spin_dir);
  Motor2.add_buttonpin_and_label(6, switch_on);
  Motor2.add_buttonpin_and_label(4, switch_off);
  Motor2.add_buttonpin_and_label(7, change_spin_dir);  
  // Sets Up POT Speed Control
  //Motor1.SetupSpeedLED(8,9,10);
  //Motor1.setupSetPoints(800, 1600, 2400);
  //Motor1.setupPOT(A2);
  //Motor2.SetupSpeedLED(8,9,10);
  //Motor2.setupSetPoints(800, 1600, 2400);
  //Motor2.setupPOT(A2);
  //I don't think we need the potentiometer assignments, as we aren't using a pot to read the speed measurement.

  // setup the H-Bridge Motor and Hall Effect Sensor Interrupt
  int pwmmotorpin_l = 5;
  int directionpin_l = 10;
  int pwmmotorpin_r = 8;
  int directionpin_r = 7;  
  Motor1.PID_SetupHardware(pwmmotorpin_l, directionpin_l, int_0);
  Motor2.PID_SetupHardware(pwmmotorpin_r, directionpin_r, int_0); 

  // Setup The PID Controller
  unsigned long PIDUpdate_ms=100;
  unsigned long RPMUpdate_ms=50;
  unsigned long VerboseTime_ms=1000;

  double lowerbound = 0, upperbound = 255;  // Bounds for PID output
  int control_interval_ms=30;         // PID control interval
  double Kp = 2, Ki = 0, Kd = 0.17; // Unscaled PID Values
  //double Kp = 4, Ki = 0, Kd = 0;  // Scaled PID Values


  Motor1.setPID_interval_vals(PIDUpdate_ms, RPMUpdate_ms, VerboseTime_ms);
  Motor1.PID_SetupControl(lowerbound, upperbound, control_interval_ms);
  //Motor1.PID_SetScalling(1500, false);
  Motor1.setSMAsize_Speedomotor(100);
  Motor1.setTunings(Kp,Ki,Kd);

  Motor2.setPID_interval_vals(PIDUpdate_ms, RPMUpdate_ms, VerboseTime_ms);
  Motor2.PID_SetupControl(lowerbound, upperbound, control_interval_ms);
  //Motor1.PID_SetScalling(1500, false);
  Motor2.setSMAsize_Speedomotor(100);
  Motor2.setTunings(Kp,Ki,Kd);  
  
  
  // Serial Print Options
  Motor1.setVerbose(true); // Formated and time for User Monitoring
  Motor1.setMatlabSerialPrint(false); // Formated for data logging
  Motor2.setVerbose(true); // Formated and time for User Monitoring
  Motor2.setMatlabSerialPrint(false); // Formated for data logging  
  //Motor1.setTunings(1, 1, 1);
  sei();  //enable interrupts
}

// Add the main program code into the continuous loop() function
void loop()
{
  Motor2.CheckInputs_and_Control();
  Motor1.CheckInputs_and_Control();
  
  //Motor1.logSpeedValues(1);

}
