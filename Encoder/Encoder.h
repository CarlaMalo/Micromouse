// Encoder.h

#ifndef _ENCODER_h
#define _ENCODER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <FunctionalInterrupt.h>
#define LAMBDA 0.05f

class Encoder
{
	public:
		Encoder(uint8_t pinA, uint8_t pinB, float degrees_per_count);
		~Encoder();
		float getSpeed();
		long getCounts();
		float getAngle();
		bool getInterrupt();
		volatile unsigned long getElapsedtime();
		volatile long _counter;
	private:
		uint8_t _pinA;
		uint8_t _pinB;
		ulong _micros;
		float _degrees_per_count;
		//VARIABLES ENCODER
		volatile bool _A_new;
		volatile bool _A_last;
		volatile bool _B_new;
		volatile bool _B_last;
		volatile long _errors;
		volatile bool _state_A_lead;
		volatile bool _state_B_lead;
		volatile unsigned long _micros_prev;
		volatile unsigned long _elapsed_time;
		volatile bool _flag;
		float _speed_prev;
		float _speed;


		void  IRAM_ATTR handle();
};

#endif

