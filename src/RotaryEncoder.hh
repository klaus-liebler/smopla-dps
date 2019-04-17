#pragma once
#include <stdint.h>

class RotaryEncoder {
public:
	RotaryEncoder();
	virtual ~RotaryEncoder();
	int32_t GetDiff();
	int32_t GetValue();
	void IrqCallback(uint8_t ab);
private:
	int32_t value=0;
	int32_t prevValue=0;
	uint8_t prevAB=0;
	uint32_t timestamp=0;
};

