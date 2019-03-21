#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include "DCmotor.h"
#include "basic_speed_PID.h"
#include "InterruptBasedSpeedMeasure.h"

class PID_Motor {

protected:
	basic_speed_PID PIDControler;
	InterruptSpeedMeasure_SMA Speedometer;
	HBridgeDCmotor Motor;
	// Control Variables
	double SetPoint_Store, Setpoint, Output_RPM, Input_PWM;
	//double scaled_RPM, scaled_Setpoint;
	//double Scalling_MAXRPM;
	bool motorison = false;
	bool verbose = true;
	//bool usedscaledvalues = false;
	// Timers
	static const int PIDUpdate  = 0;
	static const int RPMUpdate = 1;
	static const int verbosetime = 2;
	static const int TOT_TIMERS = 3;
	IntervalCheckTimer timer_array[TOT_TIMERS];

public:

	PID_Motor() {
		// Setup Timers
		//set_interval_vals(100,100, 500);
		Setpoint = 750;
		//Scalling_MAXRPM = 3000;
		// Setup Controller
		PIDControler.set_bounds(150, 255);
		PIDControler.set_ref_control_interval_ms(5);
		PIDControler.set_gainvals(0.3, 0, 0);
		PIDControler.set_echopidcontrol(false);
		// Setup Motor
		Motor.set_jumpstart(false);
		Motor.stop();
		Motor.setup_HBridgeDCmotor(11, 12);
		// Setup Speed Measure
		Speedometer.setupSpeedMeasure(int_0);
		int smasize = 8;
		Speedometer.setupSMAarray(smasize);
	}
	void SetupHardware(int motorpin, int directionpin, ArduinoInterruptNames IntPin = int_0) {
		Motor.set_jumpstart(false);
		Motor.stop();
		Motor.setup_HBridgeDCmotor(motorpin, directionpin);
		Speedometer.setupSpeedMeasure(int_0);
		int smasize = 5;
		Speedometer.setupSMAarray(smasize);
	}

	void SetupControl(double lower_bound, double upper_bound, int ref_con_interval_ms) {
		// Setup Controller
		PIDControler.set_bounds(lower_bound, upper_bound);
		PIDControler.set_ref_control_interval_ms(ref_con_interval_ms);
	}
	/*
	PID_Motor(int motorpin, int directionpin, ArduinoInterruptNames intname){
		// Setup Controller
		PIDControler.set_bounds(0, 255);
		PIDControler.set_ref_control_interval_ms(50);
		PIDControler.set_gainvals(1, 0, 0);
		PIDControler.set_echopidcontrol(false);
		// Setup Motor
		Motor.stop();
		Motor.setup_HBridgeDCmotor(motorpin, directionpin);
		Motor.set_jumpstart(false);
		// Setup Speed Measure
		Speedometer.setupSpeedMeasure(intname);
		int smasize = 5;
	}	
	*/
	// Control Functions
	void runSystem() {
		if (isTimeToUpdateRPM()) {
			Output_RPM = Speedometer.getRPMandUpdate();
			//scaled_RPM = mapDouble(Output_RPM, 0, Scalling_MAXRPM, 0, 255);
		}
		if (isTimeToUpdatePID()) {
			//if(usedscaledvalues) Input_PWM = PIDControler.ComputePID_output(scaled_Setpoint, scaled_RPM);
			Input_PWM = PIDControler.ComputePID_output(Setpoint, Output_RPM);
			Motor.setSpeedPWM((int)Input_PWM);
		}
		if (isTimeToPrintVerbose()) matlabSerialPrint();//PrintControlValues();
	}
	void motorStart() {
		motorison = true;
		Setpoint = SetPoint_Store;
		//scaled_Setpoint = mapDouble(Setpoint, 0, Scalling_MAXRPM, 0, 255);
		Motor.start();
		//Code Below: Commented out as we are testing a starting PWM.
		//Motor.setSpeedPWM((int)Input_PWM);
		Motor.setSpeedPWM(150);

	}
	void motorStop() {
		Setpoint=0;
		motorison = false;
		Motor.stop();
	} 
	void motorChangeDir() {
		Motor.changedir();
	}
	// Calculation Functions
	void calculateRPM() {
		Output_RPM = Speedometer.getRPMandUpdate();
		//scaled_RPM = mapDouble(Output_RPM, 0, Scalling_MAXRPM, 0, 255);
	}
	// Setup Functions
	void set_interval_vals(unsigned long PIDUpdate_ms, unsigned long RPMUpdate_ms, unsigned long VerboseTime_ms)
	{
		timer_array[PIDUpdate].setInterCheck(PIDUpdate_ms);
		timer_array[RPMUpdate].setInterCheck(RPMUpdate_ms);
		timer_array[verbosetime].setInterCheck(VerboseTime_ms);
	}
	void setTunings(double Kp, double Ki, double Kd) {
		PIDControler.set_gainvals(Kp, Ki, Kd);
	}
	void setSMAsize_Speedomotor(int smasize) {
		Speedometer.setupSMAarray(smasize);
	}
	void setTargetSpeed(double spoint) {
		SetPoint_Store = spoint;
		if (motorison) {
			Setpoint = SetPoint_Store;
			//scaled_Setpoint = mapDouble(Setpoint, 0, Scalling_MAXRPM, 0, 255);
		}
	}
	/*void setscalling(double scalling_max_RPM,bool enabled) {
		Scalling_MAXRPM = scalling_max_RPM;
		usedscaledvalues = enabled;

	}*/
	bool isTimeToUpdatePID() { return isTimeToCheckTimer(PIDUpdate); }
	bool isTimeToPrintVerbose() { return isTimeToCheckTimer(verbosetime); }
	bool isTimeToUpdateRPM() { return isTimeToCheckTimer(RPMUpdate); }
	// Timer Check Functions
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
	double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
	// Serial Info
	// Gets

	double getTargetSpeed() { return Setpoint; };
	double getCurrentSpeed() { return Output_RPM; };
	double getPIDOutput() { return Input_PWM; };
	// Serial Info

	void PrintControlValues() {
		if (verbose) {
			
			Serial.print("Target Speed: ");
			Serial.print(Setpoint);
			Serial.print("\t\t Current Speed: ");
			Serial.print(Output_RPM);
			Serial.print("\t PID Output: ");
			Serial.print(Input_PWM);
			//if (usedscaledvalues) Serial.print("\t\t Scaled RPM: ");
			//if (usedscaledvalues) Serial.print(scaled_RPM);
			//if (usedscaledvalues) Serial.print("\t Scaled Set Point: ");
			//if (usedscaledvalues) Serial.print(scaled_Setpoint);
			Serial.println("");
			//pidcontroler.printPIDvalues();
			
		}
	}
	void matlabSerialPrint() {
		Serial.print((float)millis() / 1000.0, 3);
		Serial.print("\t");
		Serial.print(Setpoint);
		Serial.print("\t");
		Serial.print(Input_PWM);
		Serial.print("\t");
		Serial.println(Output_RPM);
	}
};



#endif // !PID_CONTROL_H
