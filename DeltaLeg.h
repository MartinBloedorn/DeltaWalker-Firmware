// DeltaLeg.h

#ifndef _DELTALEG_h
#define _DELTALEG_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "DeltaServo.h"

class DeltaLeg {
public: 
	DeltaLeg();	
	
	void init(int s0, int s1, int s2, bool reversed = false);

	void setParameters(float _e, float _f, float _re, float _rf);

	DeltaServo & servo(int i) { return mServo[constrain(i, 0, 2)]; }

	// Set servo angles at once, updates kinematics
	bool setAngles(float theta0, float theta1, float theta2);
	// Set foot coordinates; returns true if reachable
	bool setPosition(float x, float y, float z);
	// Get foot coordinates
	void getPosition(float &x, float &y, float &z);

	// Updates current X,Y,Z position based on current effector angles
	// Returns false if forward kin failed
	bool recompute();	
	// Only computes forward kin, no action. Returns true if position is valid
	bool forward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0);
	// Only computes inverse kin, no action. Returns true if position is valid
	bool inverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3);

	
protected:
	DeltaServo	mServo[3];	

	// Delta robot's parameters
	// http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/	
	float e, f, re, rf;	

	// Current foot coordinates
	float mX, mY, mZ;

	// Kinematics functions
	int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0);
	int delta_calcAngleYZ(float x0, float y0, float z0, float &theta);
	int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3);
};

#endif

