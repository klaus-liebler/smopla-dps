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

void RotaryEncoder::IrqCallback(uint8_t ab)
{
	if(SysTick->VAL-timestamp < 10)
	{
		return;
	}
	timestamp=SysTick->VAL;
	if((prevAB==0b00 && ab==0b10) || (prevAB==0b10 && ab==0b11) || (prevAB==0b11 && ab==0b01) || (prevAB==0b01 && ab== 0b00))
		value++;
	if((prevAB==0b00 && ab==0b01) || (prevAB==0b01 && ab==0b11) || (prevAB==0b11 && ab==0b10) || (prevAB==0b10 && ab== 0b00))
		value--;
	prevAB=ab;
}
