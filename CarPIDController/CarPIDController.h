//Write a new PID controller that doesn't require an update via buttons but still uses timers.
#ifndef CARPIDCONTROLLER_H
#define CARPIDCONTROLLER_H

#include "Basic_Input.h"
#include "IntervalCheckTimer.h"
#include "PID_MotorControl.h"
#include <Arduino.h>

#define SERIAL_OUTPUT_REPEAT 1000  
#define LOGGERSAMPLES 100

class CARController {
protected:

	PID_Motor pidcontroler;
	static const int matlab_verbose = 0;
	static const int buttons_check = 1;
	static const int verbose_time = 2;
	static const int TOT_TIMERS = 3;
	IntervalCheckTimer timer_array[TOT_TIMERS];

	bool isTimeToCheckTimer(int timer_index)
	{
		if (timer_index >= 0 && timer_index < TOT_TIMERS)
		{
			bool result = timer_array[timer_index].isMinChekTimeElapsed();
			// reset timer when true
			if (result)
				timer_array[timer_index].updateCheckTime();
			return result;
		}
		else
			return false;
	}


public:
	CarPIDConrtoller() {
		// these are the default values for intervals between: 
		//two speed-adjustments; two taget-speed checks; two button checks; 
		int buttons_check_interval_ms = 50;
		int verbosetime_ms = 1000;
		int matlab_verbose_ms = 2000;
		set_interval_vals(buttons_check_interval_ms);
	}
	void PID_SetupHardware(int motorpin, int directionpin, ArduinoInterruptNames IntPin = int_0) {
		pidcontroler.SetupHardware(motorpin, directionpin, IntPin);
	}
	void PID_SetupControl(double lower_bound, double upper_bound, int ref_con_interval_ms) {
		pidcontroler.SetupControl(lower_bound, upper_bound, ref_con_interval_ms);
	}
	void jumpstartCar()
	{
		pidcontroler.setTargetSpeed(350);
		pidcontroler.motorStart();
	}
	void setSMAsize_Speedomotor(int smasize) {
		pidcontroler.setSMAsize_Speedomotor(smasize);
	}

	void set_interval_vals(unsigned long buttons_check_interval_ms)
	{
		timer_array[buttons_check].setInterCheck(buttons_check_interval_ms);
	}
	//Don't forget to change the timer parameters.
	bool isTimeToUpdateMotor() { return isTimeToCheckTimer(buttons_check); }
	//We will start the motor in the INO file.
	//From here we are conrolling the motor by altering the PWM values.
	//We alter the PWM values by using a PID controller.

	void setPID_interval_vals(unsigned long PIDUpdate_ms,
		unsigned long RPMUpdate_ms, unsigned long VerboseTime_ms) {
		pidcontroler.set_interval_vals(PIDUpdate_ms, RPMUpdate_ms, VerboseTime_ms);
	}

	void setTunings(double Kp, double Ki, double Kd) {
		pidcontroler.setTunings(Kp, Ki, Kd);
	}
	void CheckInputs_and_Control() {
		// check buttons and react to that;
		pidcontroler.setTargetSpeed(500);
		if (isTimeToUpdateMotor()) {
			pidcontroler.runSystem();
			Serial.print("\t CheckInputs_and_Control() Runs:\n ");
			Serial.print("Target Speed: ");
			Serial.print(pidcontroler.getTargetSpeed());
			Serial.print("\t Current Speed: ");
			Serial.print(pidcontroler.getCurrentSpeed());
			Serial.print("\t PID Output: ");
			Serial.print(pidcontroler.getPIDOutput());
			Serial.println("");
		}
	}
};
#endif // !CARPIDCONTROLLER_H