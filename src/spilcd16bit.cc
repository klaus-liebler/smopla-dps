#include "spilcd16bit.hh"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_utils.h"
#include "gpio.hh"
#include <algorithm>
#include <stdio.h>
#include <string.h>
#include "printf/printf.h"

SPILCD16bit::SPILCD16bit(SPI_TypeDef *spi, const uint32_t dmaChannel,
		const DisplayRotation rotation, const Pin cspin, const Pin dcpin, const Pin backlightPin, const Pin rstpin) :
		spi(spi), dmaChannel(dmaChannel), rotation(rotation), _cs(cspin), _dc(dcpin), _backlight(backlightPin), _rst(rstpin){
}

void SPILCD16bit::spiTxCompleteCallback() {
	LL_DMA_DisableChannel(DMA1, this->dmaChannel);
	if (this->bufferStep == 0){
		Gpio::Set(_cs, true);
		this->bufferFillerMode=BufferfillerMode::NONE;
		return;
	}

	switch (this->bufferFillerMode) {
	case BufferfillerMode::RECT:
		this->fillRectCb();
		break;
	case BufferfillerMode::STRING:
		this->printStringCb();
		break;
	default:
		return;
	}
}

void SPILCD16bit::backlight(bool on) {
	Gpio::Set(_backlight, on);
}


void SPILCD16bit::writecommand(uint8_t c) {
	while (LL_SPI_IsActiveFlag_BSY(spi))
		__NOP();
	Gpio::Set(_dc, false);
	Gpio::Set(_cs, false);
	LL_SPI_TransmitData8(spi, c);

	Gpio::Set(_cs, true);
}

void SPILCD16bit::writedata(uint8_t c) {

	while (LL_SPI_IsActiveFlag_BSY(spi))
		__NOP();
	Gpio::Set(_dc, true);
	Gpio::Set(_cs, false);
	LL_SPI_TransmitData8(spi, c);
	Gpio::Set(_cs, true);
}

void SPILCD16bit::writedata16(uint16_t d) {

	while (LL_SPI_IsActiveFlag_BSY(spi))
		__NOP();
	Gpio::Set(_dc, true);
	Gpio::Set(_cs, false);
	uint8_t d1 = (uint8_t) (d >> 8);
	LL_SPI_TransmitData8(spi, d1);
	uint8_t d2 = (0xFF) & d;
	while (!LL_SPI_IsActiveFlag_TXE(spi))
		;
	LL_SPI_TransmitData8(spi, d2);
	Gpio::Set(_cs, true);
}

void SPILCD16bit::begin(void) {
	Gpio::ConfigurePin(_backlight, PinMode::OUTPUTPP_2MHz);
	Gpio::ConfigurePin(_dc, PinMode::OUTPUTPP_10MHz);
	Gpio::ConfigurePin(_cs, PinMode::OUTPUTPP_10MHz);
	Gpio::Set(_cs, false);
	if (_rst != Pin::NO_PIN) {
		Gpio::ConfigurePin(_rst, PinMode::OUTPUTPP_2MHz, false);
		LL_mDelay(250);//min 10us according to datasheet
		Gpio::Set(_rst, true);
		LL_mDelay(250); //min 120ms according to datasheet
	}
	LL_DMA_ConfigAddresses(
	DMA1, dmaChannel, (uint32_t) buffer, LL_SPI_DMA_GetRegAddr(spi),
	LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_SPI_Enable(spi);
	chipInit();
}



void SPILCD16bit::processInitCommands(const uint8_t* addr){

	uint8_t  numCommands, numArgs;
	uint8_t  ms;

	numCommands = *addr++;// Number of commands to follow
	while (numCommands--)                           // For each command...
	{
		writecommand(*addr++);    // Read, issue command
		numArgs = *addr++;        // Number of args to follow
		ms = numArgs & SPI_LCD_DELAY_SIGN;      // If hibit set, delay follows args
		numArgs &= ~SPI_LCD_DELAY_SIGN;         // Mask out delay bit
		while (numArgs--)                       // For each argument...
		{
			writedata(*addr++); // Read, issue argument
		}

		if (ms)
		{
			ms = *addr++;     // Read post-command delay time (ms)
			LL_mDelay( 2*ms );
		}
	}
}

bool SPILCD16bit::DMAIdle()
{
	return this->bufferFillerMode == BufferfillerMode::NONE;
}

void SPILCD16bit::processInitCommandsCompact(const uint8_t* addr){

	uint8_t numArgs = *addr++;
	uint16_t delay=0;
	static constexpr uint16_t lookup[] = {0, 10, 150, 500};
	while (numArgs!=0)                           // For each command...
	{
		delay = (numArgs & 0xC0);
		numArgs &=0x3F; //Max 64 Args
		writecommand(*addr++);    // Read, issue command
		while (--numArgs)                       // For each argument...
		{
			writedata(*addr++); // Read, issue argument
		}
		if (delay)
		{
			delay >>=6;
			LL_mDelay(lookup[delay]);
		}
		numArgs = *addr++;
	}
}


void SPILCD16bit::fillScreen() {
	int px;
	setAddr(0x00, 0x00, _width, _height); //go home
	for (px = 0; px < _width*_height; px++) {
		writedata16(foregroundColor);
	}
}

void SPILCD16bit::drawPixel(int16_t x, int16_t y) {
	if (boundaryCheck(x, y))
		return;
	if ((x < 0) || (y < 0))
		return;
	setAddr(x, y, x, y);
	//TODO: Version von SUMOTOY setAddr(x, y, x + 1, y + 1);
	writedata16(this->foregroundColor);
}

void SPILCD16bit::drawFastVLine(int16_t x, int16_t y, int16_t h) {
	// Rudimentary clipping
	if (boundaryCheck(x, y))
		return;
	if (((y + h) - 1) >= _height)
		h = _height - y;
	setAddr(x, y, x, (y + h) - 1);
	while (h-- > 0) {
		writedata16(this->foregroundColor);
	}
}

bool SPILCD16bit::boundaryCheck(int16_t x, int16_t y) {
	if ((x >= _width) || (y >= _height))
		return true;
	return false;
}

void SPILCD16bit::drawFastHLine(int16_t x, int16_t y, int16_t w) {
	// Rudimentary clipping
	if (boundaryCheck(x, y))
		return;
	if (((x + w) - 1) >= _width)
		w = _width - x;
	setAddr(x, y, (x + w) - 1, y);
	while (w-- > 0) {

		writedata16(foregroundColor);
	}

}

// fill a rectangle
void SPILCD16bit::fillRect(int16_t x, int16_t y, int16_t w, int16_t h) {
	if (boundaryCheck(x, y))
		return;
	if (((x + w) - 1) >= _width)
		w = _width - x;
	if (((y + h) - 1) >= _height)
		h = _height - y;

	while (this->bufferFillerMode != BufferfillerMode::NONE)
		__NOP(); //wait till a previous process has been finished;
	setAddr(x, y, (x + w) - 1, (y + h) - 1);
	this->bufferFillerMode = BufferfillerMode::RECT;
	this->bufferStep = w * h;
	while (LL_SPI_IsActiveFlag_BSY(spi)) //as long as the setAddr last byte is transmitted
			__NOP();
	Gpio::Set(_dc, true);
	Gpio::Set(_cs, false);
	this->fillRectCb();

//	for (y = h; y > 0; y--) {
//		for (x = w; x > 0; x--) {
//			writedata16(color);
//		}
//	}

}
//bufferStep zählt, wie viele 16bit-Farben noch zu senden sind
void SPILCD16bit::fillRectCb() {
	size_t length = 0;
	uint16_t *buffer16 = (uint16_t *) buffer;
	while (bufferStep > 0 && length < BUFFER_SIZE_BYTES / 2) {
		buffer16[length] = this->foregroundColor;
		length++;
		bufferStep--;
	}
	length *= 2;
	LL_DMA_SetDataLength(DMA1, this->dmaChannel, length);
	LL_DMA_EnableChannel(DMA1, this->dmaChannel);
}



void SPILCD16bit::setColors(uint16_t foregroundColor, uint16_t backgroundColor)
{
	this->foregroundColor= Swap2Bytes(foregroundColor); //Because of DMA byte order
	this->backgroundColor= Swap2Bytes(backgroundColor);
}

/**************************************************************************/
/*!
    @brief    Helper to determine size of a character with current font/size.
    @param    c     The ascii character in question
    @param    x     Pointer to x location of character
    @param    y     Pointer to y location of character
*/
/**************************************************************************/
void SPILCD16bit::charBounds(char c, int16_t *x) {
	if(c == '\n') { // Newline?
		*x  = 0;    // Reset x to zero, advance y by one line
		return;
	}
	if(c == '\r') { // Not a carriage return; is normal char
		return;
	}
	uint8_t first = font->first;
	uint8_t last  = font->last;
	if((c < first) && (c > last)) { // Char present in this font?
		return;
	}
	GFXglyph *glyph = &((font->glyph)[c - first]);
	*x += glyph->xAdvance;
}

void SPILCD16bit::getTextBounds(const char *str, int16_t x, int16_t y, Anchor anchorType, int16_t *x1, int16_t *y1, int16_t *x2, int16_t *y2) {
	if(anchorType!=Anchor::BOTTOM_LEFT) while (1) __asm__ __volatile__ ("bkpt #0"); //NOT Implemented

	*x1 = x;
	*x2 = x;
	*y2 = y;
	*y1 = y;

    size_t l = strlen(str);
    for(size_t i=0;i<l; i++)
    {
    	char c = str[i];
    	if(c == '\n' || c == '\r') { // Newline?
			continue;
		}

		uint8_t first = font->first;
		uint8_t last  = font->last;
		if((c < first) && (c > last)) { // Char present in this font?
			continue;
		}
		GFXglyph *glyph = &((font->glyph)[c - first]);
		*x2 += glyph->xAdvance;
		*y1 = std::min((int16_t)*y1, (int16_t)(y+glyph->yOffset));
		*y2 = std::max((int16_t)*y1, (int16_t)(y+glyph->yOffset+glyph->height));
    }
}

int16_t SPILCD16bit::getTextPixelLength(const char *str) {

	int16_t x2=0;

    size_t l = strlen(str);
    for(size_t i=0;i<l; i++)
    {
    	char c = str[i];
    	if(c == '\n' || c == '\r') { // Newline?
			continue;
		}

		uint8_t first = font->first;
		uint8_t last  = font->last;
		if((c < first) && (c > last)) { // Char present in this font?
			continue;
		}
		GFXglyph *glyph = &((font->glyph)[c - first]);
		x2 += glyph->xAdvance;

    }
    return x2;
}

void SPILCD16bit::setFont(const GFXfont* font)
{
	this->font=font;
}



PrintStringError SPILCD16bit::printString(int16_t cursor_x, int16_t cursor_y, int16_t xWindowStart, int16_t xWindowEnd, int16_t yWindowStart, int16_t yWindowEnd, Anchor anchorType, const char *format, ...) {

	//X und Y definieren eine Ankerposition. In welche Richtung ab dort der Text geschrieben wird, bestimmt anchorType
	if(!this->font) return PrintStringError::NO_FONT_SET;
	if(anchorType!=Anchor::BOTTOM_LEFT && anchorType!=Anchor::BOTTOM_RIGHT) return PrintStringError::LAYOUT_NOT_IMPLEMENTED;
	if(xWindowEnd<=xWindowStart) return PrintStringError::PARAM_ASEERTION_ERROR;
	if(yWindowEnd<=yWindowStart) return PrintStringError::PARAM_ASEERTION_ERROR;
	if(BUFFER_SIZE_BYTES/2 < this->_width) return PrintStringError::BUFFER_TOO_SMALL;


	while (this->bufferFillerMode != BufferfillerMode::NONE)
				__NOP(); //wait till a previous process has been finished;



	this->xWindowStart=std::max((int16_t)0, xWindowStart);
	this->xWindowEnd=std::min((int16_t)_width,xWindowEnd);
	this->yWindowStart=std::max((int16_t)0,yWindowStart);
	this->yWindowEnd=std::min((int16_t)_height, yWindowEnd);
	this->bufferStep = yWindowEnd-yWindowStart;
	va_list va;
	va_start(va, format);
	this->strLength=vsnprintf(strBuffer, STRING_BUFFER_SIZE_CHARS, format, va);
	va_end(va);
	if(anchorType==Anchor::BOTTOM_LEFT)
	{
		this->cursor_x = cursor_x;
		this->cursor_y = cursor_y;
	}
	else if(anchorType==Anchor::BOTTOM_RIGHT)
	{
		this->cursor_x = cursor_x-getTextPixelLength(strBuffer);
		this->cursor_y = cursor_y;
	}


	setAddr(xWindowStart, yWindowStart, xWindowEnd-1, yWindowEnd-1);
	this->bufferFillerMode = BufferfillerMode::STRING;


	while (LL_SPI_IsActiveFlag_BSY(spi)) //as long as the setAddr last byte is transmitted
			__NOP();
	Gpio::Set(_dc, true);
	Gpio::Set(_cs, false);
	this->printStringCb();
    return PrintStringError::OK;
}


void SPILCD16bit::printString(int16_t cursor_x, int16_t cursor_y, Anchor anchorType, const char *format, ...) {

	//X und Y definieren eine Ankerposition. In welche Richtung ab dort der Text geschrieben wird, bestimmt anchorType
	if(!this->font) return;
	if(anchorType!=Anchor::BOTTOM_LEFT) while (1) __asm__ __volatile__ ("bkpt #0"); //NOT Implemented


	while (this->bufferFillerMode != BufferfillerMode::NONE)
				__NOP(); //wait till a previous process has been finished;
	if(BUFFER_SIZE_BYTES/2 < this->_width)//as we write line by line, we need at least one line in the buffer
	{
		while (1) {
				__asm__ __volatile__ ("bkpt #0");
			}
	}


	va_list va;
	va_start(va, format);
	this->strLength=vsnprintf(strBuffer, STRING_BUFFER_SIZE_CHARS, format, va);
	va_end(va);

	int16_t x1, y1, x2, y2;

	getTextBounds(strBuffer, cursor_x, cursor_y, anchorType, &x1, &y1, &x2, &y2);
	this->cursor_x = cursor_x;
	this->cursor_y = cursor_y;
	this->xWindowStart=std::max((int16_t)0, x1);
	this->xWindowEnd=std::min((int16_t)_width,x2);
	this->yWindowStart=std::max((int16_t)0,y1);
	this->yWindowEnd=std::min((int16_t)_height, y2);


	//this->distanceToBaseline = y1-cursor_y; //negativer Wert, wie glyph->yoffset

	setAddr(xWindowStart, yWindowStart, xWindowEnd-1, yWindowEnd-1);
	this->bufferFillerMode = BufferfillerMode::STRING;
	this->bufferStep = y2-y1;

	while (LL_SPI_IsActiveFlag_BSY(spi)) //as long as the setAddr last byte is transmitted
			__NOP();
	Gpio::Set(_dc, true);
	Gpio::Set(_cs, false);
	this->printStringCb();
    return;
}

void SPILCD16bit::printStringCb() {
	uint16_t *buffer16 = (uint16_t *) buffer;
	size_t length = 0;
	size_t cidx = 0;
	for(int16_t u=0;u<cursor_x-xWindowStart;u++)
	{
		buffer16[length] = this->backgroundColor;
		length++;
	}
	while(cidx<strLength)
	{
		uint8_t c = strBuffer[cidx];
		GFXglyph *glyph  = &(font->glyph[c-font->first]);
		uint8_t  *bitmap = font->bitmap;
		uint16_t bo = glyph->bitmapOffset;
		uint8_t  w  = glyph->width, //of bitmap
				 h  = glyph->height; //of bitmap
		int8_t   xo = glyph->xOffset, //of bitmap
				 yo = glyph->yOffset, //negativ!!
					adv = glyph->xAdvance;


		//BufferStep gibt an, wie viele Zeilen noch fehlen bis zur letzten Zeile

		for(int16_t xx=0; xx<adv;xx++)
		{
			volatile bool bit=false;
			//Prüfung, ob wir innerhalb der Bitmap sind
			int16_t distanceToBaseline= this->yWindowEnd - this->bufferStep - this->cursor_y;
			if(xx>=xo && xx<xo+w && distanceToBaseline >= yo && distanceToBaseline < yo+h)
			{
				volatile int line_in_bitmap = distanceToBaseline-yo;
				int bitindex = line_in_bitmap*w+(xx-xo);
				int byteindex = bitindex >> 3;
				int bitinbyte = bitindex & 0x7;
				bit = (bitmap[bo+byteindex] << bitinbyte) & 0x80;
			}
			buffer16[length] = bit?this->foregroundColor:this->backgroundColor;
			length++;
		}



		cidx++;
	}
	while(length<xWindowEnd-xWindowStart)
	{
		buffer16[length] = this->backgroundColor;
		length++;
	}

	this->bufferStep--;
	length *= 2;
	LL_DMA_SetDataLength(DMA1, this->dmaChannel, length);
	LL_DMA_EnableChannel(DMA1, this->dmaChannel);
}

