// NVMComponent.h

#ifndef _NVMCOMPONENT_h
#define _NVMCOMPONENT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <EEPROM.h>

class NVMManager {
public:	
	inline static NVMManager & instance() {
		static NVMManager n;
		return n;
	}

	bool begin();	
	
	void addEntry(bool &e)	{ addEntry(e, BOOL); }
	void addEntry(float &e)  { addEntry(e, FLOAT); }
	void addEntry(uint8_t &e) { addEntry(e, UINT8_T); }
	void addEntry(uint32_t &e) { addEntry(e, UINT32_T); }

	void save();
	bool restore(bool force=false);
private:
	NVMManager();

	enum nvm_type_e { BOOL, UINT8_T, UINT32_T, FLOAT };
		
	template <typename T> void addEntry(T &e, nvm_type_e type) {
		mEntry[mEntryI].p = (uint8_t *)&e;
		mEntry[mEntryI].type = type;
		mEntry[mEntryI].size = sizeof(T);
		if(mEntryI < MAX_ENTRIES-1) mEntryI++;
	}	

	typedef struct {
		uint8_t * p;
		uint8_t size;
		nvm_type_e type;
	} nvm_entry_t;

	static const int MAX_ENTRIES = 40;			
	nvm_entry_t mEntry[MAX_ENTRIES];

	uint8_t mEntryI;
};

#endif

