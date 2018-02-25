// 
// 
// 

#include "DeltaLeg.h"

#include <math.h>

/* * * * * * DEFS  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Set debug enabled for this class
static const bool debug = false;


/* * * * * * STATIC  * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// trigonometric constants
const float sqrt3 = sqrt(3.0);
const float pi = 3.141592653;    // PI
const float sin120 = sqrt3/2.0;
const float cos120 = -0.5;
const float tan60 = sqrt3;
const float sin30 = 0.5;
const float tan30 = 1/sqrt3; 

/* * * * * * C'TORS * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

DeltaLeg::DeltaLeg() {
	e  = 10.0;
	f  = 60.0;
	re = 70.0;
	rf = 35.0;
}

/* * * * * * PUBLIC * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void DeltaLeg::init(int s0, int s1, int s2, bool reversed) {
	mServo[0].init(s0);
	mServo[1].init(s1);
	mServo[2].init(s2);

	mServo[0].setReversed(reversed);
	mServo[1].setReversed(reversed);
	mServo[2].setReversed(reversed);

	DeltaServo::enable(false);		
}

void DeltaLeg::setParameters(float _e, float _f, float _re, float _rf) {
	e = _e; f = _f; re = _re; rf = _rf;
}

bool DeltaLeg::setAngles(float theta0, float theta1, float theta2) {
	servo(0).set(theta0);
	servo(1).set(theta1);
	servo(2).set(theta2);
	return recompute();
}

bool DeltaLeg::setPosition(float x, float y, float z) {
	float theta[3];
	if(delta_calcInverse(x, y, z, theta[0], theta[1], theta[2])) return false;
	
	for(int i=0; i<3; i++) servo(i).set(theta[i]);
	mX = x; mY = y; mZ = z;
	return true;
}

void DeltaLeg::getPosition(float &x, float &y, float &z) {
	x = mX; y = mY, z = mZ;
}

bool DeltaLeg::recompute() {
	float theta[3];
	for(int i=0; i<3; i++) theta[i] = servo(i).get();
	return !delta_calcForward(theta[0], theta[1], theta[2], mX, mY, mZ);
}

bool DeltaLeg::forward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
	return !delta_calcForward(theta1, theta2, theta3, x0, y0, z0);
}

bool DeltaLeg::inverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
	return !delta_calcInverse(x0, y0, z0, theta1, theta2, theta3);
}

/* * * * * * PROTECTED  * * * * * * * * * * * * * * * * * * * * * * * * * * */

// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
int DeltaLeg::delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
	float t = (f-e)*tan30/2;
	float dtr = pi/(float)180.0;
	
	theta1 *= dtr;
	theta2 *= dtr;
	theta3 *= dtr;
	
	float y1 = -(t + rf*cos(theta1));
	float z1 = -rf*sin(theta1);
	
	float y2 = (t + rf*cos(theta2))*sin30;
	float x2 = y2*tan60;
	float z2 = -rf*sin(theta2);
	
	float y3 = (t + rf*cos(theta3))*sin30;
	float x3 = -y3*tan60;
	float z3 = -rf*sin(theta3);
	
	float dnm = (y2-y1)*x3-(y3-y1)*x2;
	
	float w1 = y1*y1 + z1*z1;
	float w2 = x2*x2 + y2*y2 + z2*z2;
	float w3 = x3*x3 + y3*y3 + z3*z3;
	
	// x = (a1*z + b1)/dnm
	float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
	float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
	
	// y = (a2*z + b2)/dnm;
	float a2 = -(z2-z1)*x3+(z3-z1)*x2;
	float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
	
	// a*z^2 + b*z + c = 0
	float a = a1*a1 + a2*a2 + dnm*dnm;
	float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
	float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
	
	// discriminant
	float d = b*b - (float)4.0*a*c;
	if (d < 0) return -1; // non-existing point
	
	z0 = -(float)0.5*(b+sqrt(d))/a;
	x0 = (a1*z0 + b1)/dnm;
	y0 = (a2*z0 + b2)/dnm;
	return 0;
}

// helper functions, calculates angle theta1 (for YZ-pane)
int DeltaLeg::delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
	float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
	y0 -= 0.5 * 0.57735    * e;    // shift center to edge
	// z = a + b*y
	float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
	float b = (y1-y0)/z0;
	// discriminant
	float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf);
	if (d < 0) return -1; // non-existing point
	float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
	float zj = a + b*yj;
	theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
	return 0;
}

// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=non-existing position
int DeltaLeg::delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
	theta1 = theta2 = theta3 = 0;
	int status = delta_calcAngleYZ(x0, y0, z0, theta1);
	if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
	if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
	return status;
}


