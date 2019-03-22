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

	void checkButtonsAndUpdateMotor()
	{
		command_list_enum in_smpl_cmd;
		bool success;
		//check the buttons; any command found is written to in_smpl_cmd
		success = motor_pushbuttons.check_n_get_command(in_smpl_cmd);
		if (success)
		{
			switch (in_smpl_cmd) {
			case switch_on:
				if (verbose)
					Serial.println("Motor Start");
				pidcontroler.motorStart();
				break;
			case switch_off:
				if (verbose)
					Serial.println("Motor Stop");
				pidcontroler.motorStop();
				break;
			case change_spin_dir:
				if (verbose)
					Serial.println("Motor Reverse");
				pidcontroler.motorChangeDir();
				break;
			default:
				if (verbose)
					Serial.println("Unknown button pressed");
			}
			if (verbose)
				Serial.println("");
		}
	}
public:


	void set_interval_vals(unsigned long buttons_check_interval_ms)
	{
		timer_array[motor_check].setInterCheck(buttons_check_interval_ms);
	}
	//Don't forget to change the timer parameters.
	bool isTimeToUpdateMotor() { return isTimeToCheckTimer(motor_check); }
	bool isTimeToRunPID() { return isTimeToCheckTimer(PID_check); }
	//We will start the motor in the INO file.
	//From here we are conrolling the motor by altering the PWM values.
	//We alter the PWM values by using a PID controller.

	void CheckInputs_and_Control() {
		// check buttons and react to that;
		if (isTimeToUpdateMotor()) {
			pidcontroler.runSystem();
		}
	}
};