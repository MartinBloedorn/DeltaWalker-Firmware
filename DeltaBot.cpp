// 
// 
// 

#include "DeltaBot.h"

#include "NVMManager.h"

/* * * * * * DEFS  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Set debug enabled for this class
static const bool debug = false;

static const int APIN_BAT_0		= 0;
static const int APIN_BAT_1		= 1;
static const int PIN_LED_R		= 4;
static const int PIN_LED_G		= 3;
static const int PIN_LED_B		= 2;

static float norm(float x, float y, float z) {
	return sqrt(x*x + y*y + z*z);
}

static void rotz(float x, float y, float deg, float &_x, float &_y) {
	float rad = M_PI*deg/180.0;
	_x = x*cos(rad) - y*sin(rad);
	_y = x*sin(rad) + y*cos(rad);
}

static void printv(float x, float y, float z) {
	Serial.print("["); 
	Serial.print(x); Serial.print(", ");
	Serial.print(y); Serial.print(", ");
	Serial.print(z); Serial.println("]");
}

/* Returns in which quadrant the vector is in. [0,1] \in 0, [1,0] \in 1, and so on.
 *            3  0   
 *            2  1
 */
static int quadrant(float x, float y) {
	if(x >= 0.0) {
		if(y >= 0.0) return 0;
		else return 1;
	} else {
		if(y >= 0.0) return 3;
		else return 2;
	}
}

/* * * * * * C'TORS * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

DeltaBot::DeltaBot() :
	NVM(NVMManager::instance())	
{
	mWalkState = walkstate_e::IDLE;
}

void DeltaBot::init() {
	// Initializing legs' servo IDs	
	mLeg[0].init( 4, 5, 6, true);	// Front left, reversed servos
	mLeg[1].init( 0, 1, 2);			// Back left
	mLeg[2].init(12,13,14, true);	// Back right, reversed servos
	mLeg[3].init( 8, 9,10);			// Front right

	// Add offset parameters for each leg 
	for(int i=0; i<12; i++) 
		NVM.addEntry(mLegOffset[i]);
	// Load all parameters
	NVM.begin();
	// Configure all legs offsets and limits
	for(int iL=0; iL<4; iL++) {
		for(int iS=0; iS<3; iS++) {
			mLeg[iL].servo(iS).setOffset(mLegOffset[iL*3 + iS]);
			mLeg[iL].servo(iS).setLimits(-180.0, 180.0);			
		}		
	}
	// Centralize legs
	centerLegs();
	// Compute neutral leg position
	leg(0).forward(0.0, 0.0, 0.0, mX0, mY0, mZ0);			
}

/* * * * * * PUBLIC * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void DeltaBot::centerLegs() {
	for(int iL=0; iL<4; iL++)  mLeg[iL].setAngles(0.0, 0.0, 0.0);
	mWalkState = IDLE;
}

bool DeltaBot::setLegPosition(int i, float x, float y, float z) {
	i = constrain(i, 0, 3);
	float _x, _y, th0, th1, th2;
	rotz(x, y, mLegRot[i], _x, _y);
	return leg(i).setPosition(_x, _y, z);	
}

void DeltaBot::getLegPosition(int i, float &x, float &y, float &z) {
	i = constrain(i, 0, 3);
	float _x, _y;
	leg(i).getPosition(_x, _y, z);
	rotz(_x, _y, -mLegRot[i], x, y);
}

float DeltaBot::getBatteryVoltage(float * c0 /* = 0 */, float * c1 /* = 0 */) {
	float bat_all = (float(analogRead(APIN_BAT_0))/1024.0)*(3.3/0.3125); // 22k + 10k divider
	float bat_c0  = (float(analogRead(APIN_BAT_1))/1024.0)*(3.3/0.5);    // 10k + 10k divider
	float bat_c1  = bat_all - bat_c0;

	if(c0) *c0 = bat_c0;
	if(c1) *c1 = bat_c1;
	return bat_all;
}

void DeltaBot::saveCurrentLegPositionAsCenter(int idx) {
	int first, last;
	if(idx == -1) { first = 0; last = 4; }
	else { idx = constrain(idx, 0, 3); first = idx, last = idx+1; }

	for(int iL=first; iL<last; iL++) 
		for(int iS=0; iS<3; iS++) {
			float o = mLeg[iL].servo(iS).raw();
			mLegOffset[iL*3 + iS] = o;
			// Set offset and reset position
			mLeg[iL].servo(iS).setOffset(o);
			mLeg[iL].servo(iS).set(0.0);
		}

	NVM.save();
}

bool DeltaBot::translate(float x, float y, float z, bool incremental) {	
	float lx[4], ly[4], lz[4];	// Starting leg positions
	float tx[4], ty[4], tz[4];	// Target leg positions
	float _x, _y;				// Buffer variables for rotations
	float th[4][3];				// Leg angles
	
	for(int iL=0; iL<4; iL++) {
		if(incremental)
			leg(iL).getPosition(lx[iL], ly[iL], lz[iL]);
		else {
			lx[iL] = mX0;
			ly[iL] = mY0;
			lz[iL] = mZ0;
		}
		// Rotate [x, y, z] to match each leg's CS
		rotz(x, y, mLegRot[iL], _x, _y);
		tx[iL] = lx[iL] - _x;
		ty[iL] = ly[iL] - _y;
		tz[iL] = lz[iL] -  z;		
		/*
		if(incremental) {
			Serial.print("Pos   = "); printv(lx[iL],ly[iL],lz[iL]);
			Serial.print("Input = "); printv(x, y, z);
			Serial.print("RotZ  = "); printv(_x, _y, z);
			Serial.print("Target= "); printv(tx[iL],ty[iL],tz[iL]);
		}
		*/
		// Verify accessibility
		if(!leg(iL).inverse(tx[iL], ty[iL], tz[iL], th[iL][0], th[iL][1], th[iL][2])) return false;
	}

	// If we're here, positions are reacheable. Apply them:
	for(int iL=0; iL<4; iL++) leg(iL).setAngles(th[iL][0], th[iL][1], th[iL][2]);

	return true;
}

void DeltaBot::walk(float x, float y) {		
	float wf = 35.0;				// Walk scaling factor
	float ta = 35.0;				// Walk tilting angle
	float wz = 30.0;				// Walk Z height
	float n;						// Norm of direction vector
	float lx, ly, lz, _lx, _ly;		// Leg target position for each step
	float tx, ty;					// Body translation offsets	
	int stepDelay = 150;			// Delay between each motion		

	float zCorrections[] = {0.0, 0.0, -3.0, 0.0}; // Manual step depth correction if needed

	switch(mWalkState) {
		// Starting a gait cycle
		case IDLE:			
			// Normalize direction vector
			n = norm(x, y, 0.0);
			mgX = x/n;
			mgY = y/n;
			mgZ = 0.0;

			// Change direction slightly to the left
			rotz(x, y, ta, tx, ty);	
			// Translate body
			translate(wf*tx, wf*ty, 10.0);			
			delay(stepDelay*2);

			Serial.print("Tilting to "); printv(wf*tx, wf*ty, wz);

			// Get the first leg that should move (opposite to the current heading)			
			mStepsTaken = 0;
			mLegSeq[0] = quadrant(-tx, -ty);
			// The next leg should be -45 off the current heading
			rotz(x, y, -45.0, tx, ty);
			mLegSeq[1] = quadrant(tx, ty);
			// Ensure nothing gets f*cked up due to numerical issues 
			if(mLegSeq[0] == mLegSeq[1]) {
				rotz(x, y, -60.0, tx, ty);
				mLegSeq[1] = quadrant(tx, ty);
			} 
			// Now get the remaining legs
			mLegSeq[2] = (mLegSeq[0] + 1)%4;
			mLegSeq[3] = (mLegSeq[1] + 3)%4;

			mWalkState = mLegSeq[0];
			break;		
		case RETILT:			
			// Incrementally translate the body to the right 
			rotz(mgX, mgY, -90.0, tx, ty);
			tx *= cos((90.0 - ta)*PI/180.0);
			ty *= cos((90.0 - ta)*PI/180.0);
			// Translate body
			if(translate(2*wf*tx, 2*wf*ty, 0.0, true)) {
				Serial.print("Re-tilt increment "); printv(2*wf*tx, 2*wf*ty, 0.0);
			} else 
				Serial.println("Unable to re-tilt!");
			delay(stepDelay*2);
			
			// Keep walking 
			mWalkState = mLegSeq[2];
			break;
		case LEG0:			
		case LEG1:			
		case LEG2:			
		case LEG3:						
			Serial.print("Moving leg "); 
			Serial.println(mLegSeq[mStepsTaken]);			
			
			// Get the current Z position of the foot we'll move
			getLegPosition(mWalkState, lx, ly, lz);
			// Raise leg
			leg(mWalkState).setPosition(0.0, 0.0, -35.0);
			delay(stepDelay);
			// Lower leg in walking direction
			//setLegPosition(mWalkState, wf*mgX, wf*mgY, lz + zCorrections[mWalkState]);			

			// Rotate current leg position 90 deg
			rotz(lx, ly, (mStepsTaken < 2? 90.0 : -90.0), _lx, _ly);

			Serial.print("Got position "); printv(lx, ly, lz);
			Serial.print("Rotated to "); printv(_lx, _ly, lz);

			setLegPosition(mWalkState, _lx, _ly, lz + zCorrections[mWalkState]);
			delay(stepDelay);			

			mStepsTaken++;
			if(mStepsTaken == 2) mWalkState = RETILT;
			else if(mStepsTaken == 4) mWalkState = FINISH;
			else mWalkState = mLegSeq[mStepsTaken];
			break;
		case FINISH:
			Serial.println("Finished gait cycle");
			translate(0.0, 0.0, wz);
			delay(stepDelay*2);
			mWalkState = IDLE;
			break;
	}	

	/*
	switch(mWalkState) {
		case IDLE:
			translate(wf*x/n, wf*y/n, 30.0);			
			mWalkState = LEG1;
			delay(stepDelay*2);
			break;
		case LEG0:
			//tx = tf*-1.0; ty = tf*-1.0;
		case LEG1:
			//tx = tf*-1.0; ty = tf*1.0;
		case LEG2:
			//tx = tf*1.0; ty = tf*1.0;
		case LEG3:
			//tx = tf*1.0; ty = tf*-1.0;
			// Tilt away from the leg we'll use
			//translate(wf*x/n + tx, wf*y/n + ty, 30.0);
			//delay(stepDelay/2);

			// Get the current Z position of the foot we'll move
			leg(mWalkState).getPosition(lx, ly, lz);
			// Raise leg
			leg(mWalkState).setPosition(0.0, 0.0, -35.0);
			delay(stepDelay/2);
			// Lower leg (rotate target position)
			rotz(wf*x/n, wf*y/n, mLegRot[mWalkState], lx, ly);
			leg(mWalkState).setPosition(lx, ly, lz);
			delay(stepDelay);
			// Next leg (opposite)
			if(mWalkState == LEG2)
				mWalkState = FINISH;
			else
				mWalkState = legSeq[mWalkState];
			break;
		case FINISH:
			translate(0.0, 0.0, 30.0);
			delay(stepDelay);
			mWalkState = IDLE;
			break;
	}	
	*/
}

/* * * * * * PROTECTED  * * * * * * * * * * * * * * * * * * * * * * * * * * */


