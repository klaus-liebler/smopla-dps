#pragma once
#include "gpio.hh"

constexpr Pin DBGPIN = Pin::PA06;


constexpr Pin SW_ONOFF = Pin::PB04;
constexpr Pin SW_DOWN = Pin::PA01;
constexpr Pin SW_SET = Pin::PA02;
constexpr Pin SW_UP = Pin::PA03;

constexpr Pin ROTENC_SW = Pin::PB05;

constexpr Pin ROTENC_A = Pin::PB08;
constexpr Pin ROTENC_B = Pin::PB09;


constexpr Pin LCD_SCK = Pin::PB13;
constexpr Pin LCD_MOSI = Pin::PB15;
constexpr Pin LCD_CS = Pin::NO_PIN;
constexpr Pin LCD_DC = Pin::PB14;
constexpr Pin LCD_BL = Pin::PB07;
constexpr Pin LCD_RST = Pin::PB12;

constexpr Pin ADC_VIN = Pin::PB00; //ADC1_IN8
constexpr Pin ADC_VOUT = Pin::PB01; //ADC1_IN9
constexpr Pin ADC_IOUT = Pin::PA07; //ADC1_IN7

constexpr Pin PWR_PWR = Pin::PB11;
constexpr Pin PWR_DAC_U = Pin::PA04;
constexpr Pin PWR_DAC_I = Pin::PA05;

