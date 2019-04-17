#pragma once
#include <stdint.h>

class RotaryEncoder {
public:
	RotaryEncoder();
	virtual ~RotaryEncoder();
	int32_t GetDiff();
	int32_t GetValue();
	void IrqCallback(bool a, bool b);
private:
	int32_t value=0;
	int32_t prevValue=0;
	bool a0=0;
	bool c0=0;
};

