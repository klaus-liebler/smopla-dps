#pragma once

#include "gfxfont.h"
#include "gpio.hh"
#include <stddef.h>
#include <stdint.h>
#include <string>

#include "spilcd16bit.hh"

class TFT_ST7789 : public SPILCD16bit{

 public:
	TFT_ST7789(SPI_TypeDef* spi, const uint32_t dmaChannel, const int16_t physicalWidth, const int16_t physicalHeigth, const DisplayRotation rotation, const Pin cspin, const Pin dcpin, const Pin backlightPin, const Pin rstpin);
	virtual ~TFT_ST7789(){}
	void idleMode(bool onOff) override;
	void display(bool onOff) override;
	void sleepMode(bool mode) override;
 protected:
	void chipInit() override;
	void setAddr(uint16_t x_min_incl, uint16_t y_min_incl, uint16_t x_max_incl, uint16_t y_max_incl) override;

 private:
	uint8_t		_Mactrl_Data;
};
