#ifndef SYSTEMCONTROL_UNIT_H
#define SYSTEMCONTROL_UNIT_H

#include "Basic_Input.h"
#include "IntervalCheckTimer.h"
#include "PID_MotorControl.h"
#include <Arduino.h>

#define SERIAL_OUTPUT_REPEAT 1000  
#define LOGGERSAMPLES 100


class SystemControler {
protected:

	inputs motor_pushbuttons;
	in_analog motor_potentiometer;
	PID_Motor pidcontroler;
	// Button Label Statics
	static const int matlab_verbose = 0;
	static const int buttons_check = 1;
	static const int verbose_time = 2;
	static const int TOT_TIMERS = 3;
	IntervalCheckTimer timer_array[TOT_TIMERS];
	
	bool verbose = false;				// Print to serial consol controler actions and mesurments
	bool matlabverboseenabled = false; // Tab Delineated Values including time

	double setpoint1, setpoint2, setpoint3; // Motor Set-points
	int LED3_pin;	// Led's To indicate SEtpoint
	int LED2_pin;
	int LED1_pin;		

	// Data Logger Varibles
	float values[LOGGERSAMPLES];  //array used for logging, size defined by LOGGERSAMPLES
	int nValuePointer = 0;        //used for setting the array location to be used 
	long dataloggermillis = 0; //timer used for spacing samples
	int nLogSampleTime = 100; //time to space samples by 
	unsigned long nRepeatMillis = 0; //timer used for the direct serial print section of the code
	   	  
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

	SystemControler() {
		// these are the default values for intervals between: 
		//two speed-adjustments; two taget-speed checks; two button checks; 
		int buttons_check_interval_ms = 200;
		int verbosetime_ms = 1000;
		int matlab_verbose_ms = 2000;
		set_interval_vals(matlab_verbose_ms, buttons_check_interval_ms, verbosetime_ms);
	}
	void PID_SetupHardware(int motorpin, int directionpin, ArduinoInterruptNames IntPin = int_0) {
		pidcontroler.SetupHardware(motorpin, directionpin, IntPin);
	}
	void PID_SetupControl(double lower_bound, double upper_bound, int ref_con_interval_ms) {
		pidcontroler.SetupControl(lower_bound, upper_bound, ref_con_interval_ms);
	}
	
	void setSMAsize_Speedomotor(int smasize) {
		pidcontroler.setSMAsize_Speedomotor(smasize);
	}
	// these allow setting values other than the default for each timer
	void set_interval_vals(unsigned long matlab_verbose_ms, unsigned long buttons_check_interval_ms, unsigned long verbosetime_ms)
	{
		timer_array[matlab_verbose].setInterCheck(matlab_verbose_ms);
		timer_array[buttons_check].setInterCheck(buttons_check_interval_ms);
		timer_array[verbose_time].setInterCheck(verbosetime_ms);
	}

	bool isTimeToCheckInputButtons() { return isTimeToCheckTimer(buttons_check); }
	bool isTimeToMatlabVerbose() { return isTimeToCheckTimer(matlab_verbose); }
	bool isTimeToPrintVerbose() { return isTimeToCheckTimer(verbose_time); }

	void setupPOT(int POT_Pin) {
		motor_potentiometer.setup_in_analog(POT_Pin);
	}

	// this is to add the pin and "label" of a new push button
	void add_buttonpin_and_label(int butpinnum, command_list_enum butlabel)
	{
		// temporary variable of type in_push_button
		in_push_button new_button(butpinnum, butlabel);
		// the temp button is now copied into motor_pushbuttons
		motor_pushbuttons.add_in_push_button(new_button);
	}
	//Add a setup for the LED, where we define the pins 8-10. 
	void setTargetSpeed(double TargetSpeed) {
		pidcontroler.setTargetSpeed(TargetSpeed);
	}
	/*
	void PID_SetScalling(double scalling_max_RPM, bool enabled) {
		pidcontroler.setscalling(scalling_max_RPM, enabled);
	}*/
	void setPID_interval_vals(unsigned long PIDUpdate_ms, 
		unsigned long RPMUpdate_ms, unsigned long VerboseTime_ms)	{
		pidcontroler.set_interval_vals(PIDUpdate_ms, RPMUpdate_ms, VerboseTime_ms);
	}
	void setVerbose(bool verbose_mode)
	{
		verbose = verbose_mode;
		if (verbose)
		Serial.begin(9600);
		Serial.setTimeout(10);
	}

	void setMatlabSerialPrint(bool matlabenable) {
		matlabverboseenabled = matlabenable;
		if (matlabverboseenabled) {
			Serial.begin(9600);
			Serial.setTimeout(10);
		}
	}

	double getTargetSpeed() { return pidcontroler.getTargetSpeed(); };
	double getCurrentSpeed() { return pidcontroler.getCurrentSpeed(); };
	double getPWM() { return pidcontroler.getPIDOutput(); };

	void PrintControlValues() {
		if ((verbose == true) && isTimeToPrintVerbose()) {
			Serial.print("Target Speed: ");
			Serial.print(pidcontroler.getTargetSpeed());
			Serial.print("\t Current Speed: ");
			Serial.print(pidcontroler.getCurrentSpeed());
			Serial.print("\t PID Output: ");
			Serial.print(pidcontroler.getPIDOutput());
			Serial.println("");
		}
	}
	void SetupSpeedLED(int outpin1, int outpin2, int outpin3) {
		LED1_pin = outpin1;
		LED2_pin = outpin2;
		LED3_pin = outpin3;
			pinMode(LED1_pin, OUTPUT);
			digitalWrite(LED1_pin, LOW);
			pinMode(LED2_pin, OUTPUT);
			digitalWrite(LED2_pin, LOW);
			pinMode(LED3_pin, OUTPUT);
			digitalWrite(LED3_pin, LOW);
	}
	void setupSetPoints(double SP1, double SP2, double SP3)
	{
		setpoint1 = SP1;
		setpoint2 = SP2;
		setpoint3 = SP3;
	}
	void logSpeedValues(char countdowntoreadout = 0) {
		//if countdowntoreadout is anything other than 0, enable the logging part otherwise just dump to serial.
		if (countdowntoreadout)  //this is the code that runs if we're logging response and dumping to serial afterwards
		{
			if (countdowntoreadout == 3)  //third mode of data logging: dump data to serial, and switch off motor
			{
				Serial.println("");
				Serial.println("");
				Serial.println("");
				Serial.println("");
				Serial.println("");
				Serial.println("");
				Serial.println("");  //empty space
				for (int i = 0; i < LOGGERSAMPLES; i++)
				Serial.println(values[i]);

				pidcontroler.setTargetSpeed(0); //disabled motor after data logging
				countdowntoreadout = 4;  //next mode will be endless loop doing nothing
			}
			if (countdowntoreadout == 2)  //second mode of data logging: actual logging
			{
				if (millis() - dataloggermillis > nLogSampleTime)
				{
					dataloggermillis = millis();
					values[nValuePointer] = pidcontroler.getCurrentSpeed();
					nValuePointer++;
				}
				if (nValuePointer >= LOGGERSAMPLES)      //if we've fillerd our sample array
					countdowntoreadout = 3;  //next mode  
			}
			if (countdowntoreadout == 1)  //first mode of data logging: initialise
			{
				dataloggermillis = millis();  //time started (so next sample occurs at correct time)
				values[0] = pidcontroler.getCurrentSpeed();  //first value
				nValuePointer = 1;  //point to next value

				countdowntoreadout = 2; //next mode
			}
		}
		//If not doing a data logging session, execute following code for serial port I/O
		else  //this code runs for normal operation - monitors serial input, and outputs setpoint / current point to console periodically
		{
			//This bit of code monitors the serial interface for numbers.
			if (Serial.available() > 0)
			{ //serial data available...
				int nVal = Serial.parseInt();          //get an int - this will be a target RPM
				int nVal2 = Serial.parseInt();          //get a second int. If present, then trigger countdown to readout...

				if (nVal2)
				{
					countdowntoreadout = 1;    //if nonzero second value trigger readout
					nLogSampleTime = nVal2;    //and set sample time to second value (millis)
				}
				pidcontroler.setTargetSpeed((float)nVal);
				while (Serial.read() != -1);            //and flush input buffer by reading until empty (gets rid of pesky CR or LF)
			}

			//this bit of code deals with computing RPM and dumping it to serial monitor
			if (millis() - nRepeatMillis > SERIAL_OUTPUT_REPEAT)  //fire off this bit of code every SERIAL_OUTPUT_REPEAT milliseconds
			{
				nRepeatMillis = millis();          //this resets the trigger for the next 1000 milliseconds
				Serial.print("Current speed:");
				Serial.print(pidcontroler.getCurrentSpeed());                    //dump reading to console
				Serial.print(" Target speed:");
				Serial.println(pidcontroler.getTargetSpeed());          //dump reading to console
			}
		}
	}

	double CheckPotReading() {
		//Check the pre-built function for the motor potentiometer for the current setting of the potentiometer between 0-1023.
		//Based upon this current measurement we can return a speed.
		//This speed will then be read by the setTargetSpped() function below to adjust the motors current spee.
		int potvalue;
		motor_potentiometer.read_input(potvalue);
		if (potvalue < 350) {
			digitalWrite(LED1_pin, HIGH);
			digitalWrite(LED2_pin, LOW);
			digitalWrite(LED3_pin, LOW);
			return setpoint1;
		}
		else if (potvalue < 700) {
			digitalWrite(LED1_pin, HIGH);
			digitalWrite(LED2_pin, HIGH);
			digitalWrite(LED3_pin, LOW);
			return setpoint2;
		}
		else  {
			digitalWrite(LED1_pin, HIGH);
			digitalWrite(LED2_pin, HIGH);
			digitalWrite(LED3_pin, HIGH);
			return setpoint3;
		}
	}
	void setTunings(double Kp, double Ki, double Kd) {
		pidcontroler.setTunings(Kp,Ki,Kd);
	}
	void CheckInputs_and_Control() {
		// check buttons and react to that;
		if (isTimeToCheckInputButtons()) {
			checkButtonsAndUpdateMotor();
			setTargetSpeed(CheckPotReading());
		}
		if (isTimeToMatlabVerbose() && matlabverboseenabled) {
			pidcontroler.matlabSerialPrint();
		}
		
		if (isTimeToPrintVerbose() && verbose) PrintControlValues();
		pidcontroler.runSystem();
		//logSpeedValues(0);
	}
};
#endif // !SYSTEMCONTROL_UNIT_H

