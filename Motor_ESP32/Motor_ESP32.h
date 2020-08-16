// Motor_ESP32.h

#ifndef _MOTOR_ESP32_h
#define _MOTOR_ESP32_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <PWM_ESP32.h>
class HBridgeMosfet {
public:
	HBridgeMosfet(uint8_t channel1, uint8_t channel2, uint8_t a1_pin, uint8_t a2_pin);
	~HBridgeMosfet();
	void setSpeed(int speed);
	void setBitSpeed(int speed);
	PWM pwm1;
	PWM pwm2;
protected:
	uint8_t _a1_pin;
	uint8_t _a2_pin;
};

#endif

