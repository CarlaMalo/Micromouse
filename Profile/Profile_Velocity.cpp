// 
// 
// 

#include "Profile_Velocity.h"

Profile::Profile(float a_max, uint8_t v_max)
{
	_a = a_max;
	_v_max = v_max;

}

void Profile::setPerfil(float ticks){
	double v = 0;
	double x = abs(ticks);
	float dt = _sampling_time;
	uint8_t t = 0;
	v_list.push_back(v);
	while (x > 0) {
		if (pow(v,2) < 2 * _a*x) {
			v = v + _a * dt;
			if (v > _v_max) {
				v = _v_max;
			}
		}
		else {
			v = v - _a * dt;
		}
		x = x - v * dt;
		t = t + dt;
		if (ticks < 0) {
			v_list.push_back(-v);
		}
		else {
			v_list.push_back(v);
		}
		//Serial.printf("%.2f,%.2f,%.2f,%.3f,%.3f\n", _a, _v_max, v, x, dt);
	}

}

void Profile::setSampling(float sampling_time)
{
	_sampling_time = sampling_time;
}

void Profile::setProfile(float a_max, uint8_t v_max)
{
	_a = a_max;
	_v_max = v_max;
}
