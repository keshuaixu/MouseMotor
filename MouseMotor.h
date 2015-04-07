#ifndef MouseMotor_h
#define MouseMotor_h
#include "Energia.h"
class MouseMotor{
public:
	MouseMotor(int ph, int en, int nsleep, long* encoder);
	void go(int speed);
	void brake();
	long getEncoder();
private:
	int ph;
	int en;
	int nsleep;
	long* encoder;
};
#endif