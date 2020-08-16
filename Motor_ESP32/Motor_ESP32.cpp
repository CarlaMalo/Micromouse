// 
// 
// 

#include "Motor_ESP32.h"

HBridgeMosfet::HBridgeMosfet(uint8_t channel1, uint8_t channel2, uint8_t a1_pin, uint8_t a2_pin)
{
	pwm1.setup(a1_pin, channel1, 1000.0f, 8, HIGH); //100
	pwm2.setup(a2_pin, channel2, 1000.0f, 8, HIGH); //100

}

HBridgeMosfet::~HBridgeMosfet()
{
}

void HBridgeMosfet::setSpeed(int speed)
{
	if (speed < 0) {
		pwm1.setPWM(abs(speed));
		pwm2.setPWM(0);
	}
	else {
		pwm2.setPWM(abs(speed));
		pwm1.setPWM(0);
	}

}

void HBridgeMosfet::setBitSpeed(int speed)
{
	if (speed < 0){
	pwm1.setBits(speed);
	pwm2.setBits(0);
	}
	else {
	pwm2.setBits(speed);
	pwm1.setBits(0);
	}
}

