// DeltaServo.h

#ifndef _DELTASERVO_h
#define _DELTASERVO_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// Use multiplexer board for servo implementation
#define DELTA_SERVO_USE_PCA9685		1
// Multilplexer board I2C address
#define DELTA_SERVO_PCA9685_ADDR	0x40
// PCA9685 update rate (Hz)
#define DELTA_SERVO_PCA9685_HZ		60
// PCA9685 ~OE pin 
#define DELTA_SERVO_PCA9685_OE_PIN	23

class DeltaServo {
public:
	DeltaServo();
	DeltaServo(int id);
	
	void init(int id = -1);		

	// Set position (offset added to <deg>)
	void  set(float deg);
	// Get current servo angle (w/ offset)
	float get() { return mPos; }
	// Current servo angle w/o offset
	float raw() { return mRaw; }
	
	// Position limits; applied after offset computation
	void setLimits(float degMin, float degMax);		
	// Set if servo should be reversed 
	void setReversed(bool r) { mIsReversed = r; }
	bool isReversed() { return mIsReversed; }

	void  setOffset(float offs);
	float getOffset() { return mOffs; }

	static void enable(bool en) {
		digitalWrite(DELTA_SERVO_PCA9685_OE_PIN, !en);
	}
protected:
	int		mID;
	float	mMin, mMax, mOffs, mPos, mRaw;	
	bool	mIsReversed;

	// For PCA9685 only
	const float	mUpdateRate = DELTA_SERVO_PCA9685_HZ;	
};

#endif

