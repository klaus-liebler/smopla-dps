#pragma once
#include <stddef.h>
#include <stdint.h>

enum class Colorspace:uint8_t{
	RGB= 0b00000000,
	BGR =0b00001000,
};

constexpr Colorspace __COLORSPC = Colorspace::BGR; // 1:GBR - 0:RGB


#define __GAMMASET3

#if defined(__GAMMASET1)
constexpr uint8_t pGammaSet[15]= {0x36,0x29,0x12,0x22,0x1C,0x15,0x42,0xB7,0x2F,0x13,0x12,0x0A,0x11,0x0B,0x06};
constexpr uint8_t nGammaSet[15]= {0x09,0x16,0x2D,0x0D,0x13,0x15,0x40,0x48,0x53,0x0C,0x1D,0x25,0x2E,0x34,0x39};
#elif defined(__GAMMASET2)
constexpr uint8_t pGammaSet[15]= {0x3F,0x21,0x12,0x22,0x1C,0x15,0x42,0xB7,0x2F,0x13,0x02,0x0A,0x01,0x00,0x00};
constexpr uint8_t nGammaSet[15]= {0x09,0x18,0x2D,0x0D,0x13,0x15,0x40,0x48,0x53,0x0C,0x1D,0x25,0x2E,0x24,0x29};
#elif defined(__GAMMASET3)
constexpr uint8_t pGammaSet[15] = { 0x3F, 0x26, 0x23, 0x30, 0x28, 0x10, 0x55,
		0xB7, 0x40, 0x19, 0x10, 0x1E, 0x02, 0x01, 0x00 };
constexpr uint8_t nGammaSet[15] = { 0x09, 0x18, 0x2D, 0x0D, 0x13, 0x15, 0x40,
		0x48, 0x53, 0x0C, 0x1D, 0x25, 0x2E, 0x24, 0x29 };
#else
constexpr uint8_t pGammaSet[15]= {0x3F,0x25,0x1C,0x1E,0x20,0x12,0x2A,0x90,0x24,0x11,0x00,0x00,0x00,0x00,0x00};
constexpr uint8_t nGammaSet[15]= {0x20,0x20,0x20,0x20,0x05,0x15,0x00,0xA7,0x3D,0x18,0x25,0x2A,0x2B,0x2B,0x3A};
#endif

