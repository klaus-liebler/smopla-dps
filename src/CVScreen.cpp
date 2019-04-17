#include "CVScreen.hh"
#include "spilcd16bit.hh"
#include "printf/printf.hh"

//Logik
//zu jedem Zeitpunkt ist ein "Screen" aktiv; es sind aber alle instanziert
//auf den aktiven Screen zeigt ein Pointer und dieser enthält auch alle UI-Eingaben
//Beim Wechsel wird ScreenA "inaktiv", dann wird der neue Screen aktiv und dann wird der Pointer umgebogen
//Der Wechsel findet im Hauptthread statt, nicht im IRQ-Kontext
//Nur der aktive Screen darf Kommandos an den Controller senden

CVScreen::CVScreen(SPILCD16bit *lcd):VoltageSet(0),lcd(lcd) {


}

CVScreen::~CVScreen() {
	// TODO Auto-generated destructor stub
}

void CVScreen::UpdateVoltageSet(uint32_t voltageSet)
{
	const int16_t x = 10;
	const int16_t y = 20;
	const Anchor anchor = Anchor::BOTTOM_LEFT;
	if(voltageSet==this->VoltageSet) return;
	this->VoltageSet=voltageSet;
	//lcd->printString()

}
