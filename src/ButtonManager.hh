#pragma once

#include <stdint.h>
#include "pindefs.hh"
class ButtonManager {
public:
	ButtonManager();
	virtual ~ButtonManager();
	void Update();
	uint32_t GetShortPresses();
	uint32_t GetLongPresses();
private:
	bool prevSW_ONOFF=true;
	bool prevSW_DOWN=true;
	bool prevSW_SET=true;
	bool prevSW_UP=true;
	bool prevROT_SW=true;
	uint32_t timeROT_SW=0;

	uint32_t shortEvents=0;
	uint32_t longEvents=0;


};
