// Profile_Velocity.h

#ifndef _PROFILE_VELOCITY_h
#define _PROFILE_VELOCITY_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <vector>

using namespace std;

class Profile
{
public:
	Profile(float a_max, uint8_t v_max);
	std::vector<float> v_list;
	void setPerfil(float ticks);
	void setSampling(float sampling_time);
	void setProfile(float a_max, uint8_t v_max);

private:
	uint16_t _ticks;
	float _a;
	float _v_max;
	float _sampling_time;
};

#endif

