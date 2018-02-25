// DeltaBot.h

#ifndef _DELTABOT_h
#define _DELTABOT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "DeltaLeg.h"
#include "RGBLEDBlender.h"
#include "NVMManager.h"

class DeltaBot { 
public:	
	inline static DeltaBot & instance() {
		static DeltaBot d;
		return d;
	}

	void init();
	
	// Get a pointer to a particular leg
	DeltaLeg   & leg(int i) { return mLeg[constrain(i, 0, 3)]; }
	// Get a pointer to a particular servo in a leg
	DeltaServo & servo(int l, int s) { return leg(l).servo(s); }

	// Enable/disable servos
	void enableMotors(bool en) { DeltaServo::enable(en); }	

	// Returns battery voltage; optionally returns cells' voltages
	float getBatteryVoltage(float * c0 = 0, float * c1 = 0);

	// Puts legs on zeroed position, clears current walk state
	void centerLegs();

	// Saves current leg position as center; use -1 index to save all
	void saveCurrentLegPositionAsCenter(int idx);

	// Set leg's position, with vector referenced in {O}
	bool setLegPosition(int i, float x, float y, float z);
	// Get leg's position, referenced in {O}
	void getLegPosition(int i, float &x, float &y, float &z);

	// Translates body to the position, by moving all legs together; returns true if motion is possible
	// If incremental, will add the [x, y, z] to the current leg positions
	bool translate(float x, float y, float z, bool incremental=false);

	// Walk in [x y]^{O} direction
	void walk(float x, float y);

protected:
	DeltaBot();

	NVMManager		NVM;
	DeltaLeg		mLeg[4];
	RGBLEDBlender	mRGB;

	float	mLegOffset[12]; // Angle offsets for each leg
	const float	mLegRot[4] = {45.0, 135.0, -135.0, -45.0}; // Leg rotations in relation to {O}

	float	mX0, mY0, mZ0;	// Generic foot position in center angle 
	
	float mgX, mgY, mgZ;	// Direction vector of current gait cycle
	int mLegSeq[4];			// Leg sequence for current gait cycle
	int mStepsTaken;		// How many steps were already performed in current gait cycle

	enum walkstate_e {
		LEG0, LEG1, LEG2, LEG3,		// Moving each leg
		RETILT,						// Swing to the right side during gait		
		FINISH,						// Doing the final translation
		IDLE						// Not started
	};
	int mWalkState;
};

#endif

