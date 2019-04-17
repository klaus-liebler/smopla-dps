#include "RotaryEncoder.hh"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"

RotaryEncoder::RotaryEncoder() {
	// TODO Auto-generated constructor stub

}

RotaryEncoder::~RotaryEncoder() {
	// TODO Auto-generated destructor stub
}

int32_t RotaryEncoder::GetDiff()
{
	int32_t tmp=value-prevValue;
	prevValue=value;
	return tmp;
}

int32_t RotaryEncoder::GetValue()
{
	return value;

}

void RotaryEncoder::IrqCallback(bool a, bool b)
{
	if (a != a0) {              // A changed
		a0 = a;
		if (b != c0) {
			c0 = b;
			value+=(a == b)?1:-1;
		}
	}
}
