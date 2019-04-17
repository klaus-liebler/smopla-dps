#include "Screen.hh"
#include "spilcd16bit.hh"

Screen::Screen(SPILCD16bit *lcd):lcd(lcd) {

}

Screen::~Screen() {}

void Screen::SetActive(void)
{
	this->active=true;
	this->RedrawCompletely();
}
void Screen::SetInactive(void)
{
	this->active=false;
}
