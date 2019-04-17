#include "st7789.hh"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_utils.h"
#include "gpio.hh"
#include <algorithm>    // std::max


constexpr uint8_t  ST77XX_NOP        =0x00;
constexpr uint8_t  ST77XX_SWRESET    =0x01;
constexpr uint8_t  ST77XX_RDDID      =0x04;
constexpr uint8_t  ST77XX_RDDST      =0x09;

constexpr uint8_t  ST77XX_SLPIN      =0x10;
constexpr uint8_t  ST77XX_SLPOUT     =0x11;
constexpr uint8_t  ST77XX_PTLON      =0x12;
constexpr uint8_t  ST77XX_NORON      =0x13;

constexpr uint8_t  ST77XX_INVOFF     =0x20;
constexpr uint8_t  ST77XX_INVON      =0x21;
constexpr uint8_t ST77XX_DISPOFF    =0x28;
constexpr uint8_t  ST77XX_DISPON     =0x29;
constexpr uint8_t  ST77XX_CASET      =0x2A;
constexpr uint8_t ST77XX_RASET      =0x2B;
constexpr uint8_t ST77XX_RAMWR      =0x2C;
constexpr uint8_t ST77XX_RAMRD      =0x2E;

constexpr uint8_t ST77XX_PTLAR      =0x30;
constexpr uint8_t ST77XX_COLMOD     =0x3A;
constexpr uint8_t ST77XX_MADCTL     =0x36;

constexpr uint8_t ST7789_IDMOFF = 0x38;
constexpr uint8_t ST7789_IDMON = 0x39;


constexpr uint8_t ST77XX_MADCTL_MY  =0x80;
constexpr uint8_t ST77XX_MADCTL_MX  =0x40;
constexpr uint8_t ST77XX_MADCTL_MV  =0x20;
constexpr uint8_t ST77XX_MADCTL_ML  =0x10;
constexpr uint8_t ST77XX_MADCTL_RGB =0x00;

constexpr uint8_t ST77XX_RDID1      =0xDA;
constexpr uint8_t ST77XX_RDID2      =0xDB;
constexpr uint8_t ST77XX_RDID3      =0xDC;
constexpr uint8_t ST77XX_RDID4      =0xDD;

constexpr int16_t ST7789_240x240_XSTART =0;
constexpr uint16_t ST7789_240x240_YSTART =80;


constexpr uint8_t init_cmds1[] =
{
		9,                              //  9 commands in list:
		    ST77XX_SWRESET,   SPI_LCD_DELAY_SIGN, //  1: Software reset, no args, w/delay
		      200,                          //    150 ms delay
		    ST77XX_SLPOUT ,   SPI_LCD_DELAY_SIGN, //  2: Out of sleep mode, no args, w/delay
		      255,                          //     255 = 500 ms delay
		    ST77XX_COLMOD , 1+SPI_LCD_DELAY_SIGN, //  3: Set color mode, 1 arg + delay:
		      0x55,                         //     16-bit color
		      10,                           //     10 ms delay
		    ST77XX_MADCTL , 1,              //  4: Mem access ctrl (directions), 1 arg:
		      0x08,                         //     Row/col addr, bottom-top refresh
		    ST77XX_CASET  , 4,              //  5: Column addr set, 4 args, no delay:
		      0x00,
		      ST7789_240x240_XSTART,        //     XSTART = 0
		      (240+ST7789_240x240_XSTART)>>8,
		      (240+ST7789_240x240_XSTART)&0xFF,  //     XEND = 240
		    ST77XX_RASET  , 4,              //  6: Row addr set, 4 args, no delay:
		      0x00,
		      ST7789_240x240_YSTART,             //     YSTART = 0
		      (240+ST7789_240x240_YSTART)>>8,
		      (240+ST7789_240x240_YSTART)&0xFF,  //     YEND = 240
		    ST77XX_INVON  ,   SPI_LCD_DELAY_SIGN,  //  7: hack
		      10,
		    ST77XX_NORON  ,   SPI_LCD_DELAY_SIGN, //  8: Normal display on, no args, w/delay
		      10,                           //     10 ms delay
		    ST77XX_DISPON ,   SPI_LCD_DELAY_SIGN, //  9: Main screen turn on, no args, delay
		    255 };

constexpr uint8_t init_cmds[] = {
		PARAM_BASE+0, 0x13, // partial mode off
		PARAM_BASE+0, 0x21, // display inversion off
		PARAM_BASE+1, 0x36,0x08,	// memory access 0xc0 for 180 degree flipped
		PARAM_BASE+1, 0x3a,0x55,	// pixel format; 5=RGB565
		PARAM_BASE+2, 0x37,0x00,0x00, //
		PARAM_BASE+5, 0xb2,0x0c,0x0c,0x00,0x33,0x33, // Porch control
		PARAM_BASE+1, 0xb7,0x35,	// gate control
		PARAM_BASE+1, 0xbb,0x1a,	// VCOM
		PARAM_BASE+1, 0xc0,0x2c,	// LCM
		PARAM_BASE+1, 0xc2,0x01,	// VDV & VRH command enable
		PARAM_BASE+1, 0xc3,0x0b,	// VRH set
		PARAM_BASE+1, 0xc4,0x20,	// VDV set
		PARAM_BASE+1, 0xc6,0x0f,	// FR control 2
		PARAM_BASE+2, 0xd0, 0xa4, 0xa1, 	// Power control 1
		PARAM_BASE+14, 0xe0, 0x00,0x19,0x1e,0x0a,0x09,0x15,0x3d,0x44,0x51,0x12,0x03,
		0x00,0x3f,0x3f, 	// gamma 1
		PARAM_BASE+14, 0xe1, 0x00,0x18,0x1e,0x0a,0x09,0x25,0x3f,0x43,0x52,0x33,0x03,
		0x00,0x3f,0x3f,		// gamma 2
		PARAM_BASE+0, 0x29,	// display on
	0
};



TFT_ST7789::TFT_ST7789(SPI_TypeDef *spi, const uint32_t dmaChannel, const int16_t physicalWidth, const int16_t physicalHeigth,
		const DisplayRotation rotation, const Pin cspin, const Pin dcpin, const Pin backlightPin, const Pin rstpin)
:SPILCD16bit(spi, dmaChannel, rotation, cspin, dcpin, backlightPin, rstpin)
{

	switch (rotation) {
	case DisplayRotation::ROT0:
		_Mactrl_Data = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB;
		_width = physicalWidth;
		_height = physicalHeigth;
		break;
	case DisplayRotation::ROT180CW:
		_Mactrl_Data = ST77XX_MADCTL_RGB;
		_width = physicalWidth;
		_height = physicalHeigth;
		break;
	case DisplayRotation::ROT90CW:
		_Mactrl_Data =  ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
		_width = physicalHeigth;
		_height = physicalWidth;
		break;
	case DisplayRotation::ROT270CW:
		_Mactrl_Data = ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
		_width = physicalHeigth;
		_height = physicalWidth;
		break;
	}
}


void TFT_ST7789::chipInit() {
	processInitCommandsCompact(init_cmds);
	writecommand(ST77XX_MADCTL);
	writedata(_Mactrl_Data);
	fillScreen();
}

void TFT_ST7789::setAddr(uint16_t x_min_incl, uint16_t y_min_incl, uint16_t x_max_incl, uint16_t y_max_incl) {
	writecommand(ST77XX_CASET); // Column
	writedata16(x_min_incl);
	writedata16(x_max_incl);
	writecommand(ST77XX_RASET); // Page
	writedata16(y_min_incl);
	writedata16(y_max_incl);
	writecommand(ST77XX_RAMWR); //Into RAM
}

void TFT_ST7789::display(bool on) {
	writecommand(on ? ST77XX_DISPON : ST77XX_DISPOFF);
}

void TFT_ST7789::idleMode(bool on) {

	writecommand(on ? ST7789_IDMON : ST7789_IDMOFF);
}

void TFT_ST7789::sleepMode(bool sleepIn) {
	if (sleepIn) {
		if (sleep == 1)
			return; //already sleeping
		sleep = 1;
		writecommand(ST77XX_SLPIN);
		LL_mDelay(5); //needed
	} else {
		if (sleep == 0)
			return; //Already awake
		sleep = 0;
		writecommand(ST77XX_SLPOUT);
		LL_mDelay(120); //needed
	}
}





