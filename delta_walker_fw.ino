#include "DeltaBot.h"
#include "NVMManager.h"

#include <IRremote.h>
#include <string.h>

#define sgn(x)			((x > 0) - (x < 0))
#define NIBBLE(x)		((0x0F & x))
#define BATT_CELL_LOW	3.4

#define MSG_LEN       24 // in bits
#define MSG_HEADER    0xF
#define MSG_WALK      0x6
#define MSG_TRANSLATE 0xB
#define MSG_ROTATE    0x5
#define MSG_TOGGLE    0xA
#define MSG_PING      0xF
#define MSG_RESET     0x1

/* * * * * * VARS * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

IRrecv irrecv(9);
decode_results results;

DeltaBot  dBot = DeltaBot::instance();
NVMManager NVM = NVMManager::instance();

RGBLEDBlender rgb(4, 3, 2);
Color currColor;
String cmdStr;

// Loop times for pseudo-tasks
uint32_t loopt_check_bat	= 0;
uint32_t tstamp_walk_until	= 0;

// Current direction of walk/translation/rotation
float dx, dy, dz;
float tx, ty, tz;

// Current leg, servo and position indexes
int cLeg=0, cSrv=0, cPos=0, cIncr=0;
bool motorEn = false;
bool isWalking = false;

/* * * * * * STATIC * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Converts nibble to [-8 7] range
int nibble2int(uint8_t n) {
	n = NIBBLE(n);
	if(n & 0x8) {
		return (((int)~0x7) | n);
	} else
	return (int)n;
}

// Converts from [-8 7] range to nibble
uint8_t int2nibble(int i) {
	i = constrain(i, -8, 7);
	return NIBBLE(i);
}

bool verifyMessage(uint32_t m, int &cmd, int &a, int &b, int &c) {	
	// Must be only 24 bits long
	if(m & 0xFF000000) return false;
	// Header is 0xF
	if(NIBBLE(m >> 4*5) != 0xF) return false;
	// Extracting parts
	cmd = NIBBLE(m >> 4*4);
	uint8_t chk	= NIBBLE(m);	
	uint8_t _a  = NIBBLE(m >> 4*3);
	uint8_t _b  = NIBBLE(m >> 4*2);
	uint8_t _c  = NIBBLE(m >> 4*1);
	// Verify checksum
	uint8_t ver = NIBBLE((0xA + cmd + _a + _b + _c));	
	if(ver != chk) return false;
	// Convert payload
	a = nibble2int(_a);
	b = nibble2int(_b);
	c = nibble2int(_c);
	return true;
}

/* * * * * * SETUP  * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void setup() {
	Serial.begin(115200);	
	Serial.setTimeout(1);
	
	dBot.init();		
	dBot.centerLegs();

	irrecv.enableIRIn();	
	
	Serial.println("Hi there.");		
}

/* * * * * * LOOP * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void loop() {				
	uint32_t irCmd = 0;
	int recvCmd, payload[3];
	bool recvCmdIsValid = false;
	
	// Pseudo-threads * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	// Check battery level
	if(millis() - loopt_check_bat > 5000) {
		float c[2];
		dBot.getBatteryVoltage(&c[0], &c[1]);
		for(int i=0; i<2; i++)
		if(c[i] > 1.0 && c[i] < BATT_CELL_LOW) {
			Serial.print("Battery cell ");
			Serial.print(i);
			Serial.print(" has low voltage! (");
			Serial.print(c[i]);
			Serial.println("v)");
			rgb.Hold(_RED);
		}
		loopt_check_bat = millis();
	}

	// Keep walking until timeout
	if(isWalking) {
		if(millis() > tstamp_walk_until || (dx == 0.0 && dy == 0.0)) {
			dBot.centerLegs();
			isWalking = false;
		} else {
			dBot.walk(dx, dy);
			tstamp_walk_until = millis() + 2000;
		}
	}

	// IR command interface * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
	if (irrecv.decode(&results)) {	
		irCmd = results.value;					
		
		if(verifyMessage(irCmd, recvCmd, payload[0], payload[1], payload[2])) {
			Serial.print("Valid message: [");
			Serial.print(recvCmd, HEX);			
			for(int i=0; i<3; i++) {
				Serial.print(", ");
				Serial.print(payload[i]);
			}
			Serial.println("]");
			recvCmdIsValid = true;
			// Briefly blink green
			currColor = rgb.GetColor();
			rgb.Hold(_GREEN);
			delay(100);
			rgb.Hold(currColor);
		} else {
			Serial.print("Unknown code: ");
			Serial.println(irCmd, HEX);
		}

		irrecv.resume(); 
	}
	
	if(!recvCmdIsValid) return;		

	switch(recvCmd) {		
		case MSG_PING: 
			Serial.println("Ping!");
			// Briefly blink orange
			currColor = rgb.GetColor();
			rgb.Hold(_ORANGE);
			delay(500);
			rgb.Hold(currColor);
			break;
		case MSG_RESET:
			Serial.println("Centering legs");
			dBot.centerLegs();
			break;
		case MSG_TOGGLE:
			motorEn = !motorEn;
			if(motorEn)
				Serial.println("Enabling motors");
			else
				Serial.println("Disabling motors");
			dBot.enableMotors(motorEn);
			// Switch the led to green when motors engaged, blue otherwise
			rgb.Hold(motorEn? _BLUE : _BLACK);
			break;
		case MSG_WALK:
			// Walk directions has defined values: [-2 -1 0 1 2]
			dx = float(payload[0])/2.0;
			dy = float(payload[1])/2.0;
			// Walk for 2 second if no other signals come			
			Serial.print("Walking [");
			Serial.print(dx);
			Serial.print(", ");
			Serial.print(dy);
			Serial.println("]");
			tstamp_walk_until = millis() + 2000;
			isWalking = true;
			break;
		case MSG_TRANSLATE: 
			tx = 40.0*(float(payload[0])/2.0);
			ty = 40.0*(float(payload[1])/2.0);
			//tz = 65.0*(float(payload[2])/2.0) - 15.0; //-15 to 50
			tz = (payload[2] > 0? 40.0*(float(payload[2])/2.0) : 15.0*(float(payload[2])/2.0));
			Serial.print("Translating [");
			Serial.print(tx);
			Serial.print(", ");
			Serial.print(ty);
			Serial.print(", ");
			Serial.print(tz);
			Serial.println("]");
			dBot.translate(tx, ty, tz);
			delay(250);
			break;
	}
}
