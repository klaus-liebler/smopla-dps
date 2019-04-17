#include "ButtonManager.hh"
#include "pindefs.hh"

constexpr uint32_t LONG_PRESS_LIMIT=600;

ButtonManager::ButtonManager() {
	// TODO Auto-generated constructor stub

}

ButtonManager::~ButtonManager() {
	// TODO Auto-generated destructor stub
}

void ButtonManager::Update()
{
	bool v=Gpio::Get(SW_ONOFF);
	if(prevSW_ONOFF && !v)//now pressed
	{
		prevSW_ONOFF=false;
	}
	else if(!prevSW_ONOFF && v)
	{
		shortEvents|=0b00001;
		prevSW_ONOFF=true;
	}
	v=Gpio::Get(SW_DOWN);
	if(prevSW_DOWN && !v)//now pressed
	{
		prevSW_DOWN=false;
	}
	else if(!prevSW_DOWN && v)
	{
		shortEvents|=0b00010;
		prevSW_DOWN=true;
	}
	v=Gpio::Get(SW_SET);
	if(prevSW_SET && !v)//now pressed
	{
		prevSW_SET=false;
	}
	else if(!prevSW_SET && v)
	{
		shortEvents|=0b00100;
		prevSW_SET=true;
	}
	v=Gpio::Get(SW_UP);
	if(prevSW_UP && !v)//now pressed
	{
		prevSW_UP=false;
	}
	else if(!prevSW_UP && v)
	{
		shortEvents|=0b01000;
		prevSW_UP=true;
	}

	v=Gpio::Get(ROTENC_SW);
	if(prevROT_SW && !v)//now pressed
	{
		prevROT_SW=false;
		timeROT_SW = SysTick->VAL;
	}
	else if(!prevROT_SW && v)
	{
		if(SysTick->VAL-timeROT_SW > LONG_PRESS_LIMIT)
		{
			longEvents|=0b10000;
		}
		else
		{
			shortEvents|=0b10000;
		}
		prevROT_SW=true;
	}
}
uint32_t ButtonManager::GetShortPresses(){
	uint8_t tmp = shortEvents;
	shortEvents=0;
	return tmp;

}
uint32_t ButtonManager::GetLongPresses()
{
	uint8_t tmp = longEvents;
	longEvents=0;
	return tmp;
}

