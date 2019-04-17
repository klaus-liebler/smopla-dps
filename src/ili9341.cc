#include "ili9341.hh"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_utils.h"
#include "gpio.hh"

constexpr uint8_t CMD_NOP = 0x00; //Non operation
constexpr uint8_t CMD_SWRESET = 0x01; //Soft Reset
constexpr uint8_t CMD_RDDID   =   0x04;     ///< Read display identification information
constexpr uint8_t CMD_RDDST    =  0x09;     ///< Read Display Status
constexpr uint8_t CMD_SLPIN = 0x10; //Sleep ON
constexpr uint8_t CMD_SLPOUT = 0x11; //Sleep OFF
constexpr uint8_t CMD_PTLON = 0x12; //Partial Mode ON
constexpr uint8_t CMD_NORML = 0x13; //Normal Display ON

constexpr uint8_t CMD_RDMODE     =0x0A;     ///< Read Display Power Mode
constexpr uint8_t CMD_RDMADCTL   =0x0B;     ///< Read Display MADCTL
constexpr uint8_t CMD_RDPIXFMT   =0x0C;     ///< Read Display Pixel Format
constexpr uint8_t CMD_RDIMGFMT   =0x0D;     ///< Read Display Image Format
constexpr uint8_t CMD_RDSELFDIAG =0x0F;     ///< Read Display Self-Diagnostic Result

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
constexpr uint8_t CMD_RGBBLK = 0xB5; //RGB Interface Blanking Porch setting
constexpr uint8_t CMD_DFUNCTR = 0xB6; //Display Fuction set 5
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


constexpr uint8_t init_cmds[] = {
		22, // 22 commands follow
  0xEF, 3, 0x03, 0x80, 0x02,
  0xCF, 3, 0x00, 0xC1, 0x30,
  0xED, 4, 0x64, 0x03, 0x12, 0x81,
  0xE8, 3, 0x85, 0x00, 0x78,
  0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  0xF7, 1, 0x20,
  0xEA, 2, 0x00, 0x00,
  CMD_PWCTR1  , 1, 0x23,             // Power control VRH[5:0]
  CMD_PWCTR2  , 1, 0x10,             // Power control SAP[2:0];BT[3:0]
  CMD_VCOMCTR1  , 2, 0x3e, 0x28,       // VCM control
  CMD_VCOMCTR2  , 1, 0x86,             // VCM control2
  CMD_MADCTL  , 1, 0x48,             // Memory Access Control
  CMD_VSSTADRS, 1, 0x00,             // Vertical scroll zero
  CMD_PIXFMT  , 1, 0x55,
  CMD_FRMCTR1 , 2, 0x00, 0x18,
  CMD_DFUNCTR , 3, 0x08, 0x82, 0x27, // Display Function Control
  0xF2, 1, 0x00,                         // 3Gamma Function Disable
  CMD_GAMMASET , 1, 0x01,             // Gamma curve selected
  CMD_PGAMMAC , 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  CMD_NGAMMAC , 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  CMD_SLPOUT  , SPI_LCD_DELAY_SIGN+0, 150,                // Exit Sleep
  CMD_DISPON  , SPI_LCD_DELAY_SIGN+0, 150,               // Display on
};

constexpr uint8_t MADCTL_MY  =0x80;  ///< Bottom to top
constexpr uint8_t MADCTL_MX  =0x40;  ///< Right to left
constexpr uint8_t MADCTL_MV  =0x20;  ///< Reverse Mode
constexpr uint8_t MADCTL_ML  =0x10;  ///< LCD refresh Bottom to top
constexpr uint8_t MADCTL_RGB =0x00;  ///< Red-Green-Blue pixel order
constexpr uint8_t MADCTL_BGR =0x08;  ///< Blue-Green-Red pixel order
constexpr uint8_t MADCTL_MH  =0x04;  ///< LCD refresh right to left

TFT_ILI9341::TFT_ILI9341(SPI_TypeDef *spi, const uint32_t dmaChannel, const int16_t physicalWidth, const int16_t physicalHeigth,
		const DisplayRotation rotation, const Pin cspin, const Pin dcpin, const Pin backlightPin, const Pin rstpin)
:SPILCD16bit(spi, dmaChannel, rotation, cspin, dcpin, backlightPin, rstpin)
{
	switch (rotation) {
	case DisplayRotation::ROT0:
		_Mactrl_Data = (MADCTL_MX | MADCTL_BGR);
		_width = physicalWidth;
		_height = physicalHeigth;
		break;
	case DisplayRotation::ROT90CW:
		_Mactrl_Data = (MADCTL_MV | MADCTL_BGR);
		_width = physicalHeigth;
		_height = physicalWidth;
		break;
	case DisplayRotation::ROT180CW:
		_Mactrl_Data = (MADCTL_MY | MADCTL_BGR);
		_width = physicalWidth;
		_height = physicalHeigth;
		break;

	case DisplayRotation::ROT270CW:
		_Mactrl_Data = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
		_width = physicalHeigth;
		_height = physicalWidth;
		break;
	}
}


void TFT_ILI9341::chipInit() {
	processInitCommands(init_cmds);
	writecommand(CMD_MADCTL);
	writedata(_Mactrl_Data);
	fillScreen();
}

void TFT_ILI9341::setAddr(uint16_t x_min_incl, uint16_t y_min_incl, uint16_t x_max_incl, uint16_t y_max_incl) {
	writecommand(CMD_CLMADRS); // Column
	writedata16(x_min_incl);
	writedata16(x_max_incl);
	writecommand(CMD_PGEADRS); // Page
	writedata16(y_min_incl);
	writedata16(y_max_incl);
	writecommand(CMD_RAMWR); //Into RAM
}


void TFT_ILI9341::display(bool on) {
	writecommand(on ? CMD_DISPON : CMD_DISPOFF);
}

void TFT_ILI9341::idleMode(bool on) {

	writecommand(on ? CMD_IDLEON : CMD_IDLEOFF);
}

void TFT_ILI9341::sleepMode(bool sleepIn) {
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





