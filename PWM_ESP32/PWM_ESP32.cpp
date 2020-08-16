#include "PWM_ESP32.h"

PWM::PWM()
{
}

PWM::~PWM()
{
	ledcDetachPin(_pin);
}

void PWM::setup(uint8_t pin, uint8_t channel, double frequency, uint8_t bits_resolution, bool on_state)
{
	_pin = pin;
	_channel = channel;
	_frequency = frequency;
	_on_state = on_state;
	_resolution = pow(2, bits_resolution) - 1;
	pinMode(_pin, OUTPUT);
	// configure LED PWM functionalitites
	ledcSetup(_channel, _frequency, bits_resolution);
	// attach the channel to the GPIO to be controlled
	ledcAttachPin(_pin, _channel);
	setPWM(0);
}

void PWM::setPWM(float duty_cycle)
{
	if (duty_cycle > 100)
		duty_cycle = 100;
	else if (duty_cycle < 0)
		duty_cycle = 0;
	setBits((_resolution * duty_cycle) / 100);
}

void PWM::setBits(uint32_t bits)
{
	if (bits > _resolution)
		bits = _resolution;
	if (_on_state == LOW)
		bits = _resolution - bits; //inverted logic
	_duty_cycle = (bits * 100) / _resolution;
	ledcWrite(_channel, bits);
}

void PWM::setFrequency(float frequency)
{
	_frequency = frequency;
	ledcWriteTone(_channel, _frequency);
}

void PWM::attachPin(uint8_t pin)
{
	pinMode(pin, OUTPUT);
	ledcAttachPin(pin, _channel);
}

