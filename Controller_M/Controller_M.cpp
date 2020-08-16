// 
// 
// 

#include "Controller_M.h"


Controlador::Controlador(float time_ms)
{
	_dt = time_ms / 1000.0f;
	_error_ant = 0;
	_integral = 0;
	_error_to_integrate = 0;
	_kp = 0;
	_ki = 0;
	_kd = 0;

}

void Controlador::setK(float kp, float ki, float kd)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;

}

float Controlador::getPID(float measurement)
{
	float control_output;
	error = _setpoint - measurement;
	float error_derivative = (error - _error_ant) / _dt;
	_error_to_integrate = error;
	antiWindup();
	_integral = _integral + _error_to_integrate * _dt;
	_error_ant = error;
	control_output = (_kp * error + _ki * _integral + _kd * error_derivative);


	return control_output;
}

void Controlador::setReference(float setpoint)
{
	_setpoint = setpoint;
}

void Controlador::updateValues(float array_in[])
{
	_kp = array_in[0];
	_ki = array_in[1];
	_kd = array_in[2];
	_setpoint = array_in[3];
}

void Controlador::antiWindup()
{
	if (control_output >= 100 && error>=0 ) {
		_error_to_integrate = 0;
	}
	else if (control_output <= -100 && error <= 0) {
		_error_to_integrate = 0;
	}
}
