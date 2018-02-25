// 
// 
// 

#include "DeltaServo.h"

#if DELTA_SERVO_USE_PCA9685 == 1
	#include <Wire.h>
	#include <Adafruit_PWMServoDriver.h>

	static Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(DELTA_SERVO_PCA9685_ADDR);
#else
	#include <Servo.h>	
#endif

/* * * * * * DEFS  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 

// Only init PWM shield once
static bool pwm_is_init = false;

// Set debug enabled for this class
static const bool debug = false;

/* * * * * * C'TORS * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 

DeltaServo::DeltaServo() : DeltaServo(-1) {}

DeltaServo::DeltaServo(int id) :
	mID(id),
	mMin(0.0),
	mMax(180.0),
	mOffs(0.0),
	mIsReversed(false)
{
	
}

/* * * * * * PUBLIC * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void DeltaServo::init(int id) {
	mID = (id > -1? id : mID);
	if(mID == -1) return;
	
#if DELTA_SERVO_USE_PCA9685 == 1
	if(!pwm_is_init) {
		Wire.setSDA(17);
		Wire.setSCL(16);
	
		pwm.begin();
		pwm.setPWMFreq(mUpdateRate);

		pinMode(DELTA_SERVO_PCA9685_OE_PIN, OUTPUT);
		pwm_is_init = true;
	}
#else
	servo.attach(mID);
#endif
}

void DeltaServo::set(float deg) {	
	mPos = constrain(deg, mMin, mMax);			// 'Virtual' angle
	mRaw = constrain(mPos + mOffs, 0.0, 180.0); // Effective servo angle

#if DELTA_SERVO_USE_PCA9685 == 1	
	double w = (4096.0/(1000000.0/mUpdateRate))*map(mRaw, 0.0, 180.0, 
													(mIsReversed? 2000.0 : 1000.0), 
													(mIsReversed? 1000.0 : 2000.0));	
	pwm.setPWM(mID, 0, int(w));	

	if(debug) {
		Serial.print("Set ID "); 
		Serial.print(mID);
		Serial.print(" to us = ");
		Serial.println(w);
	}
#else
	
#endif	
}

void DeltaServo::setOffset(float offs) {
	mOffs = offs;
}

void DeltaServo::setLimits(float degMin, float degMax) {
	mMin = degMin;
	mMax = degMax;
}

/* * * * * * PROTECTED  * * * * * * * * * * * * * * * * * * * * * * * * * * */