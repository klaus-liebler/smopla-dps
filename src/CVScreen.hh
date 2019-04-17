#pragma once
#include "Screen.hh"
#include "stdint.h"
#include "spilcd16bit.hh"
#include "printf/printf.hh"

class CVScreen:public Screen {
public:
	CVScreen(SPILCD16bit *lcd);
	virtual ~CVScreen();

	void UpdateVoltageSet(uint32_t voltageSet);
private:
	uint32_t VoltageSet;
	SPILCD16bit *lcd;
};
