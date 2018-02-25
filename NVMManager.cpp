// 
// 
// 

#include "NVMManager.h"

/*  One block = 4 bytes.
	First block (@BASE_ADDR): XOR checksum, amount of entries
	
	BASE_ADDR	|  checksum
	+ 1			|  no of entries
	+ 2			|  - not used -
	+ 3			|  - not used -	
*/

/* * * * * * DEFS  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Set debug enabled for this class
static const bool debug = false;

// Base address for storage table
static const int base_addr = 0x10;

//NVMManager::chksumsalt[] = {0xAB, 0x31, 0x26, 0x74};
//NVMManager::chksumsaltlen= 4;  

/* * * * * * STATIC * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static void _debug(const String c, int i, int f) {
	if(!debug) return;
	Serial.print(c);
	Serial.println(i, f);	
}

/* * * * * * C'TORS * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

NVMManager::NVMManager() : mEntryI(0) {
	
}

/* * * * * * PUBLIC * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

bool NVMManager::begin() {	

	if(!restore(false))
		save();
	return true;
}

void NVMManager::save() {
	uint8_t chksum = 0;	

	for(int i=0; i<mEntryI; i++) {		
		// Address of entry 
		uint32_t ad = base_addr + 4*(i+1);
		// Compute checksum of entry
		uint8_t * p = mEntry[i].p;		
		for(int j=0; j<mEntry[i].size; j++) 			
			chksum += *p++;
		
		// Save entry
		switch(mEntry[i].type) {
			case FLOAT   : EEPROM.put(ad, *(float *)mEntry[i].p); break;
			case UINT32_T: EEPROM.put(ad, *(uint32_t *)mEntry[i].p); break;
			case BOOL    :
			case UINT8_T : 
				EEPROM.put(ad, *(uint8_t *)mEntry[i].p); 
				for(int j=1; j<4; j++) EEPROM.update(ad + j, 0);
				break;				
			default: break;
		}		
	}
	// Save checksum
	EEPROM.update(base_addr, chksum);
	// Save no of entries
	EEPROM.update(base_addr + 1, mEntryI);	

	_debug("mEntry = ", mEntryI, DEC);
	_debug("chksum = ", chksum, HEX);
}

bool NVMManager::restore(bool force) {
	//LOG_DEBUG("Base address = ", _NVM::BASE_ADDR, HEX);
	uint8_t nvm_chksum = EEPROM.read(base_addr);
	uint8_t	nvm_entryi = EEPROM.read(base_addr + 1);

	uint8_t chksum = 0;

	for(int i=0; i<4*nvm_entryi; i++) 		
		chksum += EEPROM.read(base_addr + 4 + i);

	_debug("Found variables = ", nvm_entryi, DEC);
	_debug("Found checksum = ", nvm_chksum, HEX);
	_debug("Computed checksum = ", chksum, HEX);	

	if(chksum != nvm_chksum) {		
		if(!force) return false;
	}
	// Restoring
	int i;
	for(i=0; i<nvm_entryi; i++) {
		if(i > mEntryI) break;
		// Address of entry
		uint32_t ad = base_addr + 4*(i+1);
		switch(mEntry[i].type) {
			case BOOL	 : EEPROM.get(ad, *(bool *)mEntry[i].p); break;			
			case FLOAT   : EEPROM.get(ad, *(float *)mEntry[i].p); break;
			case UINT8_T : EEPROM.get(ad, *(uint8_t *)mEntry[i].p); break;			
			case UINT32_T: EEPROM.get(ad, *(uint32_t *)mEntry[i].p); break;
			default: break;			
		}
	}
	//LOG_INFO("Restored variables = ", i, DEC);
	return true;
}

/* * * * * * PROTECTED  * * * * * * * * * * * * * * * * * * * * * * * * * * */