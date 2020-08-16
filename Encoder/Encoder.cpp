// 
// 
// 

#include "Encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB, float degrees_per_count)
{

	_pinA = pinA;
	_pinB = pinB;
	_degrees_per_count = degrees_per_count;
	pinMode(_pinA, INPUT_PULLUP);
	pinMode(_pinB, INPUT_PULLUP);
	attachInterrupt(_pinA, std::bind(&Encoder::handle, this), CHANGE);
	attachInterrupt(_pinB, std::bind(&Encoder::handle, this), CHANGE);
}

Encoder::~Encoder()
{
	detachInterrupt(_pinA);
	detachInterrupt(_pinB);
}

float Encoder::getSpeed()
{
	//_speed = 1000000 * _degrees_per_count / _elapsed_time;
	//_speed = LAMBDA * _speed + (1 - LAMBDA)*_speed_prev;
	//_speed_prev = _speed;

	_speed = _counter;
	_counter = 0;

	return (_speed);
}

long Encoder::getCounts()
{
	return _counter;
}

float Encoder::getAngle()
{
	return _counter*_degrees_per_count;

}

bool Encoder::getInterrupt()
{

	if (_flag) {
		_flag = false;
		return true;
	}
	else {
		return false;
	}
}

volatile unsigned long Encoder::getElapsedtime()
{

	return _elapsed_time;
}



void IRAM_ATTR Encoder::handle()
{

	//_elapsed_time = micros()-_micros_prev;
	//_micros_prev = micros();


	_A_new = digitalRead(_pinA);
	_B_new = digitalRead(_pinB);

	 _state_A_lead = _A_new ^ _B_last;
	 _state_B_lead = _B_new ^ _A_last;

	if (_state_A_lead && _state_B_lead)
		_errors += 1;
	if (_state_A_lead && !_state_B_lead) {
		_counter += 1;
	}
	if (!_state_A_lead  && _state_B_lead) {
		_counter -= 1;
	}
	_A_last = _A_new;
	_B_last = _B_new;

	//if (abs(_counter) > 909)
		//_counter = 0;

	_flag = true;
	

}

