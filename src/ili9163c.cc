#include "ili9163c.hh"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_utils.h"
#include "gpio.hh"
#include <algorithm>    // std::max


constexpr uint8_t CMD_NOP = 0x00; //Non operation
constexpr uint8_t CMD_SWRESET = 0x01; //Soft Reset
constexpr uint8_t CMD_SLPIN = 0x10; //Sleep ON
constexpr uint8_t CMD_SLPOUT = 0x11; //Sleep OFF
constexpr uint8_t CMD_PTLON = 0x12; //Partial Mode ON
constexpr uint8_t CMD_NORML = 0x13; //Normal Display ON
constexpr uint8_t CMD_DINVOF = 0x20; //Display Inversion OFF
constexpr uint8_t CMD_DINVON = 0x21; //Display Inversion ON
constexpr uint8_t CMD_GAMMASET = 0x26; //Gamma Set (0x01[1],0x02[2],0x04[3],0x08[4])
constexpr uint8_t CMD_DISPOFF = 0x28; //Display OFF
constexpr uint8_t CMD_DISPON = 0x29; //Display ON
constexpr uint8_t CMD_IDLEON = 0x39; //Idle Mode ON
constexpr uint8_t CMD_IDLEOFF = 0x38; //Idle Mode OFF
constexpr uint8_t CMD_CLMADRS = 0x2A; //Column Address Set
constexpr uint8_t CMD_PGEADRS = 0x2B; //Page Address Set

constexpr uint8_t CMD_RAMWR = 0x2C; //Memory Write
constexpr uint8_t CMD_RAMRD = 0x2E; //Memory Read
constexpr uint8_t CMD_CLRSPACE = 0x2D; //Color Space : 4K/65K/262K
constexpr uint8_t CMD_PARTAREA = 0x30; //Partial Area
constexpr uint8_t CMD_VSCLLDEF = 0x33; //Vertical Scroll Definition
constexpr uint8_t CMD_TEFXLON = 0x35; //Tearing Effect Line ON
constexpr uint8_t CMD_TEFXLOF = 0x34; //Tearing Effect Line OFF
constexpr uint8_t CMD_MADCTL = 0x36; //Memory Access Control
constexpr uint8_t CMD_VSSTADRS = 0x37; //Vertical Scrolling Start address
constexpr uint8_t CMD_PIXFMT = 0x3A; //Interface Pixel Format
constexpr uint8_t CMD_FRMCTR1 = 0xB1; //Frame Rate Control (In normal mode/Full colors)
constexpr uint8_t CMD_FRMCTR2 = 0xB2; //Frame Rate Control(In Idle mode/8-colors)
constexpr uint8_t CMD_FRMCTR3 = 0xB3; //Frame Rate Control(In Partial mode/full colors)
constexpr uint8_t CMD_DINVCTR = 0xB4; //Display Inversion Control

constexpr uint8_t CMD_SDRVDIR = 0xB7; //Source Driver Direction Control
constexpr uint8_t CMD_GDRVDIR = 0xB8; //Gate Driver Direction Control

constexpr uint8_t CMD_PWCTR1 = 0xC0; //Power_Control1
constexpr uint8_t CMD_PWCTR2 = 0xC1; //Power_Control2
constexpr uint8_t CMD_PWCTR3 = 0xC2; //Power_Control3
constexpr uint8_t CMD_PWCTR4 = 0xC3; //Power_Control4
constexpr uint8_t CMD_PWCTR5 = 0xC4; //Power_Control5
constexpr uint8_t CMD_VCOMCTR1 = 0xC5; //VCOM_Control 1
constexpr uint8_t CMD_VCOMCTR2 = 0xC6; //VCOM_Control 2
constexpr uint8_t CMD_VCOMOFFS = 0xC7; //VCOM Offset Control
constexpr uint8_t CMD_PGAMMAC = 0xE0; //Positive Gamma Correction Setting
constexpr uint8_t CMD_NGAMMAC = 0xE1; //Negative Gamma Correction Setting
constexpr uint8_t CMD_GAMRSEL = 0xF2; //GAM_R_SEL

#define Swap2Bytes(val) ( (((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00) )

constexpr uint8_t init_cmds[] =
{
17,             // 17 commands follow
0x01,  0+SPI_LCD_DELAY_SIGN,250,       // Software reset
0x11,  0+SPI_LCD_DELAY_SIGN,250,       // Exit sleep mode
0x3A,  1, 0x05, // Set pixel format
0x26,  1, 0x04, // Set Gamma curve
0xF2,  1, 0x01, // Gamma adjustment enabled
0xE0, 15, 0x3F, 0x25, 0x1C, 0x1E, 0x20, 0x12, 0x2A, 0x90,
          0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, // Positive Gamma
0xE1, 15, 0x20, 0x20, 0x20, 0x20, 0x05, 0x00, 0x15,0xA7,
          0x3D, 0x18, 0x25, 0x2A, 0x2B, 0x2B, 0x3A, // Negative Gamma
0xB1,  2, 0x08, 0x08, // Frame rate control 1
0xB4,  1, 0x07,       // Display inversion
0xC0,  2, 0x0A, 0x02, // Power control 1
0xC1,  1, 0x02,       // Power control 2
0xC5,  2, 0x50, 0x5B, // Vcom control 1
0xC7,  1, 0x40,       // Vcom offset
0x2A,  4+SPI_LCD_DELAY_SIGN, 0x00, 0x00, 0x00, 0x7F, 250, // Set column address
0x2B,  4, 0x00, 0x00, 0x00, 0x7F,            // Set page address
0x36,  1, 0xC8,       // Set address mode
0x29,  0,             // Set display on
};
constexpr uint8_t init_cmds_compact[] =
{
PARAM_BASE + DELAY_500ms +  0, CMD_SWRESET,
PARAM_BASE + DELAY_500ms +  0, CMD_SLPOUT,
PARAM_BASE + DELAY_0ms   +  1, CMD_PIXFMT,   0x05,
PARAM_BASE + DELAY_0ms   +  1, CMD_GAMMASET,   0x04, //select gammy curve 4 (0x04 for 4, 0x02 for 2, 0x01 for 1)
PARAM_BASE + DELAY_0ms   +  1, CMD_GAMRSEL,   0x01,
PARAM_BASE + DELAY_0ms   + 15, CMD_PGAMMAC,  0x3F, 0x25, 0x1C, 0x1E, 0x20, 0x12, 0x2A, 0x90, 0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, // Positive Gamma
PARAM_BASE + DELAY_0ms   + 15, CMD_NGAMMAC,  0x20, 0x20, 0x20, 0x20, 0x05, 0x00, 0x15, 0xA7, 0x3D, 0x18, 0x25, 0x2A, 0x2B, 0x2B, 0x3A, // Negative Gamma
PARAM_BASE + DELAY_0ms   +  2, CMD_FRMCTR1,  0x08, 0x08,
PARAM_BASE + DELAY_0ms   +  1, CMD_DINVCTR, 0x07,
PARAM_BASE + DELAY_0ms   +  2, CMD_PWCTR1,  0x0A, 0x02,
PARAM_BASE + DELAY_0ms   +  1, CMD_PWCTR2,  0x02,
PARAM_BASE + DELAY_0ms   +  2, CMD_VCOMCTR1,  0x50, 0x5B,
PARAM_BASE + DELAY_0ms   +  1, CMD_VCOMOFFS,  0x40,
PARAM_BASE + DELAY_0ms   +  4, CMD_CLMADRS, 0x00, 0x00, 0x00, 0x7F,
PARAM_BASE + DELAY_0ms   +  4, CMD_PGEADRS, 0x00, 0x00, 0x00, 0x7F,
PARAM_BASE + DELAY_0ms   +  1, CMD_MADCTL, 0xC8,
PARAM_BASE + DELAY_0ms   +  0, CMD_IDLEOFF,
PARAM_BASE + DELAY_0ms   +  0, CMD_DISPON,
0,
};

constexpr uint8_t init_cmds_compact_shortened[] =
{
PARAM_BASE + DELAY_150ms + 0, CMD_SWRESET,
PARAM_BASE + DELAY_150ms  + 0, CMD_SLPOUT,
PARAM_BASE + DELAY_0ms   + 0, CMD_IDLEOFF,
//PARAM_BASE + DELAY_0ms   + 2, CMD_PWCTR1,  0x0A, 0x02,
//PARAM_BASE + DELAY_0ms   + 1, CMD_PWCTR2,  0x02,
//PARAM_BASE + DELAY_0ms   + 1, CMD_PWCTR3,  0x01,
//PARAM_BASE + DELAY_0ms   + 1, CMD_PWCTR4,  0x01,
//PARAM_BASE + DELAY_0ms   + 1, CMD_PWCTR5,  0x01,
//PARAM_BASE + DELAY_0ms   + 2, CMD_VCOMCTR1,  0x50, 0x5B,
//PARAM_BASE + DELAY_0ms   + 1, CMD_VCOMOFFS,  0x40,
PARAM_BASE + DELAY_0ms   + 2, CMD_FRMCTR1,  0x08, 0x02,
//PARAM_BASE + DELAY_0ms   + 2, CMD_FRMCTR2,  0x08, 0x02,
//PARAM_BASE + DELAY_0ms   + 2, CMD_FRMCTR3,  0x08, 0x02,
PARAM_BASE + DELAY_0ms   + 1, CMD_PIXFMT,   0x05,
PARAM_BASE + DELAY_0ms   + 1, CMD_GAMMASET,   0x04, //select gammy curve 4 (0x04 for 4, 0x02 for 2, 0x01 for 1)
PARAM_BASE + DELAY_0ms   + 1, CMD_GAMRSEL,   0x01,
PARAM_BASE + DELAY_0ms   + 15, CMD_PGAMMAC,  0x3F, 0x25, 0x1C, 0x1E, 0x20, 0x12, 0x2A, 0x90, 0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, // Positive Gamma
PARAM_BASE + DELAY_0ms   + 15, CMD_NGAMMAC,  0x20, 0x20, 0x20, 0x20, 0x05, 0x00, 0x15, 0xA7, 0x3D, 0x18, 0x25, 0x2A, 0x2B, 0x2B, 0x3A, // Negative Gamma
PARAM_BASE + DELAY_0ms   + 4, CMD_CLMADRS, 0x00, 0x00, 0x00, 0x7F,
PARAM_BASE + DELAY_0ms   + 4, CMD_PGEADRS, 0x00, 0x00, 0x00, 0x7F,
PARAM_BASE + DELAY_0ms   +  1, CMD_MADCTL, 0xC8,
PARAM_BASE + DELAY_0ms   + 1, CMD_DINVCTR, 0x07,
PARAM_BASE + DELAY_0ms   + 0, CMD_NORML,
PARAM_BASE + DELAY_0ms   + 0, CMD_DISPON,
0,
};

TFT_ILI9163C::TFT_ILI9163C(SPI_TypeDef *spi, const uint32_t dmaChannel, const int16_t physicalWidth, const int16_t physicalHeigth,
		const DisplayRotation rotation, const Pin cspin, const Pin dcpin, const Pin backlightPin, const Pin rstpin)
:SPILCD16bit(spi, dmaChannel, rotation, cspin, dcpin, backlightPin, rstpin)
{


	/*
	 7) MY:  1(bottom to top), 0(top to bottom) 	Row Address Order
	 6) MX:  1(R to L),        0(L to R)        	Column Address Order
	 5) MV:  1(Exchanged),     0(normal)        	Row/Column exchange
	 4) ML:  1(bottom to top), 0(top to bottom) 	Vertical Refresh Order
	 3) RGB/BGR Order : 1(BGR), 		   0(RGB)           	Color Space
	 2) MH:  1(R to L),        0(L to R)        	Horizontal Refresh Order
	 1)
	 0)

	 MY, MX, MV, ML,RGB, MH, D1, D0
	 0 | 0 | 0 | 0 | 1 | 0 | 0 | 0	//normal
	 1 | 0 | 0 | 0 | 1 | 0 | 0 | 0	//Y-Mirror
	 0 | 1 | 0 | 0 | 1 | 0 | 0 | 0	//X-Mirror
	 1 | 1 | 0 | 0 | 1 | 0 | 0 | 0	//X-Y-Mirror
	 0 | 0 | 1 | 0 | 1 | 0 | 0 | 0	//X-Y Exchange
	 1 | 0 | 1 | 0 | 1 | 0 | 0 | 0	//X-Y Exchange, Y-Mirror
	 0 | 1 | 1 | 0 | 1 | 0 | 0 | 0	//XY exchange
	 1 | 1 | 1 | 0 | 1 | 0 | 0 | 0
	 */

	switch (rotation) {
	case DisplayRotation::ROT0:
		_Mactrl_Data = 0b11000000;
		_width = physicalWidth;
		_height = physicalHeigth;
		break;
	case DisplayRotation::ROT180CW:
		_Mactrl_Data = 0b00000000;
		_width = physicalWidth;
		_height = physicalHeigth;
		break;
	case DisplayRotation::ROT90CW:
		_Mactrl_Data = 0b10100000;
		_width = physicalHeigth;
		_height = physicalWidth;
		break;
	case DisplayRotation::ROT270CW:
		_Mactrl_Data = 0b01100000;
		_width = physicalHeigth;
		_height = physicalWidth;
		break;
	}
	_Mactrl_Data |= (uint8_t) __COLORSPC;
}


void TFT_ILI9163C::chipInit() {
	processInitCommandsCompact(init_cmds_compact);
	writecommand(CMD_MADCTL);
	writedata(_Mactrl_Data);
	fillScreen();
}

void TFT_ILI9163C::setAddr(uint16_t x_min_incl, uint16_t y_min_incl, uint16_t x_max_incl, uint16_t y_max_incl) {
	writecommand(CMD_CLMADRS); // Column
	writedata16(x_min_incl);
	writedata16(x_max_incl);
	writecommand(CMD_PGEADRS); // Page
	writedata16(y_min_incl);
	writedata16(y_max_incl);
	writecommand(CMD_RAMWR); //Into RAM
}


void TFT_ILI9163C::display(bool on) {
	writecommand(on ? CMD_DISPON : CMD_DISPOFF);
}

void TFT_ILI9163C::idleMode(bool on) {

	writecommand(on ? CMD_IDLEON : CMD_IDLEOFF);
}

void TFT_ILI9163C::sleepMode(bool sleepIn) {
	if (sleepIn) {
		if (sleep == 1)
			return; //already sleeping
		sleep = 1;
		writecommand(CMD_SLPIN);
		LL_mDelay(5); //needed
	} else {
		if (sleep == 0)
			return; //Already awake
		sleep = 0;
		writecommand(CMD_SLPOUT);
		LL_mDelay(120); //needed
	}
}





