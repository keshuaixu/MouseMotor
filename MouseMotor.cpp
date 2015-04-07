#include "MouseMotor.h"

MouseMotor::MouseMotor(int ph, int en, int nsleep, long* encoder){
	this->ph = ph;
	this->en = en;
	this->nsleep = nsleep;
	this->encoder = encoder;
}

void MouseMotor::go(int speed){
	digitalWrite(en, 1);
	digitalWrite(ph, (speed > 0) ? 0: 1);
	analogWrite(nsleep, abs(speed));
}

void MouseMotor::brake(){
	digitalWrite(en, 0);
}

long MouseMotor::getEncoder(){
	return *encoder;
}