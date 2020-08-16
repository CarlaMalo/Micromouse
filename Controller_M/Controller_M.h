// Controller_M.h

#ifndef _CONTROLLER_M_h
#define _CONTROLLER_M_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
class Controlador {
public:
	Controlador(float time_ms);
	void setK(float kp, float ki, float kd);
	float getPID(float measurement);
	void setReference(float setpoint);
	void updateValues(float array_in[]);
	void antiWindup();
	float setpoint;
	float error;
	float _integral;

private:
	float _kp;
	float _ki;
	float _kd;
	float _dt;
	float _error_ant;
	float _setpoint;
	float _error_to_integrate;
	float control_output;
};

#endif

