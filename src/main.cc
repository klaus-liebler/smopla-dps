#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_dac.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_fsmc.h"
#include "gpio.hh"
#include "pindefs.hh"
#include "PowerStageController.hh"
#include "RotaryEncoder.hh"
#include "ili9163c.hh"
#include "ili9341.hh"
#include "st7789.hh"
#define PROGMEM
#include "Fonts/FreeSans12pt7b.h"

/*
 ROT = VCC
 GRÜN RESET
 WEIß=CLK
 BLAU=SWDIO

 */


constexpr uint32_t RANGE_12BITS = 4095; /* Max value with a full range of 12 bits */
constexpr size_t ADCBUFFER_SIZE = 3; /* Size of array containing ADC converted values */

constexpr uint32_t REFERENCE_VOLTAGE_MILLIVOLTS = 3300;
constexpr double mV_digit = (double) REFERENCE_VOLTAGE_MILLIVOLTS
		/ RANGE_12BITS;

alignas(4) __IO uint16_t adcbuffer[ADCBUFFER_SIZE]; // ADC conversion results table of regular group,
__IO uint32_t aCounter = 0;

//TFT_ILI9341 tft(SPI2, 5, 240, 320, DisplayRotation::ROT0, Pin::NO_PIN, Pin::PB14, Pin::PB07, Pin::PB12);
TFT_ILI9163C tft(SPI2, 5, 130, 132, DisplayRotation::ROT0, LCD_CS, LCD_DC,
		LCD_BL, LCD_RST);
//TFT_ST7789 tft(SPI2, 5, 240, 240, DisplayRotation::ROT0, Pin::NO_PIN, Pin::PB14, Pin::PB07, Pin::PB12);

PowerStageController pwst(PWR_PWR);
RotaryEncoder rotenc;

void SystemClock_HSI24MHz_Config(void);

extern "C" void blocking_handler(void) {
	while (1) {
		__asm__ __volatile__ ("bkpt #0");
	}
}

extern "C" void HardFault_Handler(void) {
	while (1) {
		__asm__ __volatile__ ("bkpt #0");
	}
}
extern "C" void MemManage_Handler(void) {
	while (1) {
		__asm__ __volatile__ ("bkpt #0");
	}
}
extern "C" void BusFault_Handler(void) {
	while (1) {
		__asm__ __volatile__ ("bkpt #0");
	}
}
extern "C" void UsageFault_Handler(void) {
	while (1) {
		__asm__ __volatile__ ("bkpt #0");
	}
}
extern "C" void Error_Handler(void) {
	while (1) {
		__asm__ __volatile__ ("bkpt #0");
	}
}

extern "C" void NMI_Handler(void) {
}
extern "C" void SVC_Handler(void) {
}
extern "C" void DebugMon_Handler(void) {
}
extern "C" void PendSV_Handler(void) {
}
extern "C" void SysTick_Handler(void) {
}

extern "C" void DMA1_Channel1_IRQHandler(void) { //ADC_DMA
	if (LL_DMA_IsActiveFlag_TC1(DMA1)) {
		LL_DMA_ClearFlag_TC1(DMA1);
		pwst.Update(adcbuffer);
	} else if (LL_DMA_IsActiveFlag_TE1(DMA1)) {
		while (1) {
			__asm__ __volatile__ ("bkpt #0");
		}
	}
}

extern "C" void DMA1_Channel5_IRQHandler(void) //SPI2_TX
{
	if (LL_DMA_IsActiveFlag_TC5(DMA1)) {
		LL_DMA_ClearFlag_GI5(DMA1);
		tft.spiTxCompleteCallback();
	} else if (LL_DMA_IsActiveFlag_TE5(DMA1)) {
		while (1) {
			__asm__ __volatile__ ("bkpt #0");
		}
	}
}

extern "C" void TIM3_IRQHandler(void) {
	if (LL_TIM_IsActiveFlag_UPDATE(TIM3)) {
		LL_TIM_ClearFlag_UPDATE(TIM3);
	}
}

extern "C" void ADC1_IRQHandler(void)
{
  if(LL_ADC_IsActiveFlag_EOS(ADC1))
  {
    LL_ADC_ClearFlag_EOS(ADC1);
	aCounter++;
  }
  return;
}

extern "C" void EXTI9_5_IRQHandler(void)
{
	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8))
	{
		rotenc.IrqCallback((Gpio::Get(Pin::PB08)<<1) + Gpio::Get(Pin::PB09));
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
	}
	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9))
	{
		rotenc.IrqCallback((Gpio::Get(Pin::PB08)<<1) + Gpio::Get(Pin::PB09));
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
	}
	return;
}

void SystemClock_HSI24MHz_Config(void) {
	//STM32F100 runs with 24MHz and does not need FLASH_LATENCY! FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1;

	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_6);
	LL_RCC_PLL_Enable();
	while (LL_RCC_PLL_IsReady() != 1)
		__NOP();

	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
		__NOP();

	// switch system clock to PLL
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL)
		__NOP();

	LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_8);
	/* Set systick to 1ms in using frequency set to 24MHz */
	LL_Init1msTick(24000000);

	/* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
	LL_SetSystemCoreClock(24000000);
}

void Configure_SPI2(void) {
	/* (1) Enables GPIO clock and configures the SPI2 pins ********************/

	/* Enable the peripheral clock of GPIOB */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

	/* Configure SCK Pin */
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_13, LL_GPIO_PULL_DOWN);

	/* Configure MOSI Pin  */
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_15, LL_GPIO_PULL_DOWN);

	/* (2) Configure SPI2 functional parameters ********************************/
	/* Enable the peripheral clock of GPIOB */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

	/* Configure SPI2 communication */
	LL_SPI_SetBaudRatePrescaler(SPI2, LL_SPI_BAUDRATEPRESCALER_DIV16);
	LL_SPI_SetTransferDirection(SPI2, LL_SPI_HALF_DUPLEX_TX);
	LL_SPI_SetClockPhase(SPI2, LL_SPI_PHASE_1EDGE); //See ILI datasheet, page 22
	LL_SPI_SetClockPolarity(SPI2, LL_SPI_POLARITY_LOW); //See ILI datasheet, page 22
	LL_SPI_SetTransferBitOrder(SPI2, LL_SPI_MSB_FIRST);
	LL_SPI_SetDataWidth(SPI2, LL_SPI_DATAWIDTH_8BIT);
	LL_SPI_SetNSSMode(SPI2, LL_SPI_NSS_SOFT);
	LL_SPI_SetMode(SPI2, LL_SPI_MODE_MASTER);

	/* Configure SPI1 DMA transfer interrupts */
	/* Enable DMA TX Interrupt */

}

void Configure_DMA(void) {
	// (0) Allow SPI2 to use DMA
	LL_SPI_EnableDMAReq_TX(SPI2);

	/* (1) Enable the clock of DMA1*/
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	//Configure DMA for SPI2 (Display)
	NVIC_SetPriority(DMA1_Channel5_IRQn, 2);
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	LL_DMA_ConfigTransfer(DMA1,
			LL_DMA_CHANNEL_5,
			LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
			LL_DMA_MODE_NORMAL |
			LL_DMA_PERIPH_NOINCREMENT |
			LL_DMA_MEMORY_INCREMENT |
			LL_DMA_PDATAALIGN_BYTE |
			LL_DMA_MDATAALIGN_BYTE |
			LL_DMA_PRIORITY_MEDIUM);

	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);

	//Configure DMA for ADC
	NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	LL_DMA_ConfigTransfer(DMA1,
			LL_DMA_CHANNEL_1,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
			LL_DMA_MODE_CIRCULAR |
			LL_DMA_PERIPH_NOINCREMENT |
			LL_DMA_MEMORY_INCREMENT |
			LL_DMA_PDATAALIGN_HALFWORD |
			LL_DMA_MDATAALIGN_HALFWORD |
			LL_DMA_PRIORITY_HIGH);

	LL_DMA_ConfigAddresses(DMA1,
			LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
			(uint32_t) &adcbuffer,
			LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADCBUFFER_SIZE);

	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

void Configure_TIM_TimeBase_ADC_trigger(void) {
	//NVIC_SetPriority(TIM3_IRQn, 1); /* DMA IRQ lower priority than ADC IRQ */
	//NVIC_EnableIRQ(TIM3_IRQn);
	//const uint32_t timer_clock_frequency = 24000000;             /* Timer clock frequency */
	const uint32_t timer_prescaler = 24;//-->after Prescale: 1Mhz                   /* Time base prescaler to have timebase aligned on minimum frequency possible */
	const uint32_t timer_reload = 2000; //-->500Hz                      /* Timer reload value in function of timer prescaler to achieve time base period */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	LL_TIM_SetPrescaler(TIM3, (timer_prescaler - 1));
	LL_TIM_SetAutoReload(TIM3, (timer_reload - 1));
	LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetRepetitionCounter(TIM3, 0);
	LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_UPDATE);
	LL_TIM_EnableCounter(TIM3);
	//LL_TIM_EnableIT_UPDATE(TIM3);
}
#define ADC_DELAY_ENABLE_CALIB_CPU_CYCLES  (LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32)
void Configure_ADC(void) {
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);

	  //NVIC_SetPriority(ADC1_IRQn, 0); /* ADC IRQ greater priority than DMA IRQ */
	  //NVIC_EnableIRQ(ADC1_IRQn);

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

	LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM3_TRGO); //todo hier SW einstellen, wenn es nicht der Timer sein soll
	LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE); //todo: hier continuous einstellen, wenn sich der ACD quais selbst antreiben soll

	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

	LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_ENABLE);
	LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS);

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_8);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_9);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_7);

	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8,
			LL_ADC_SAMPLINGTIME_28CYCLES_5); //todo: hier auf LL_ADC_SAMPLINGTIME_239CYCLES_5 stellen, wenn sich der ADC selbst antreibt
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_9,
			LL_ADC_SAMPLINGTIME_28CYCLES_5);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7,
			LL_ADC_SAMPLINGTIME_28CYCLES_5);
	__IO uint32_t wait_loop_index = 0;


	//LL_ADC_EnableIT_EOS(ADC1);
	LL_ADC_Enable(ADC1);
	wait_loop_index = (ADC_DELAY_ENABLE_CALIB_CPU_CYCLES >> 1);
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}
	LL_ADC_StartCalibration(ADC1);

	while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0) {
		__NOP();
	}
	LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
}


void Configure_DAC(void) {

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

	Gpio::ConfigurePin(PWR_DAC_U, PinMode::INPUT_ANALOG);
	Gpio::ConfigurePin(PWR_DAC_I, PinMode::INPUT_ANALOG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);

	//LL_DAC_SetTriggerSource(DAC1, LL_DAC_CHANNEL_1, LL_DAC_TRIG_SOFTWARE);
	//LL_DAC_SetTriggerSource(DAC1, LL_DAC_CHANNEL_2, LL_DAC_TRIG_SOFTWARE);
	__IO uint32_t wait_loop_index = 0;

	/* Force the default value to be loaded in the data holding register after reset system */
	LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 0x00);
	LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, 0x00);

	/* Enable DAC channel */
	LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
	LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);
	wait_loop_index = ((LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US
			* (SystemCoreClock / (100000 * 2))) / 10);
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}

	//LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_1);
	//LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_2);

}

void Configure_ROTENC_BUTTONS()
{
	Gpio::ConfigurePinInputPullup(SW_DOWN);
	Gpio::ConfigurePinInputPullup(SW_SET);
	Gpio::ConfigurePinInputPullup(SW_UP);
	Gpio::ConfigurePinInputPullup(SW_ONOFF);
	Gpio::ConfigurePinInputPullup(ROTENC_SW);
	Gpio::ConfigurePinInputPullup(ROTENC_A);
	Gpio::ConfigurePinInputPullup(ROTENC_B);

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE8);//ROTENC_A
	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE9);//ROTENC_B
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_8|LL_EXTI_LINE_9);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_9);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_8);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_9);
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_8);
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_9);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_SetPriority(EXTI9_5_IRQn,3);
}

const char* pattern = "Val %d";

int main() {
	SystemClock_HSI24MHz_Config();
	for (uint32_t i = 0; i < ADCBUFFER_SIZE; i++) {
		adcbuffer[i] = 0;
	}
	Configure_SPI2();
	Configure_DAC();
	Configure_DMA();
	Configure_TIM_TimeBase_ADC_trigger();
	Configure_ADC();
	Configure_ROTENC_BUTTONS();

	Gpio::ConfigurePin(DBGPIN, PinMode::OUTPUTPP_10MHz, true);


	Gpio::ConfigurePin(PWR_PWR, PinMode::OUTPUTPP_2MHz, true);
	tft.setColors(MAGENTA, GREEN);
	tft.begin();
	tft.backlight(true);
	tft.fillRect(10, 10, 40, 40);
	tft.setFont(&FreeSans12pt7b);
	int val =42;

	while (true) {
		while(!tft.DMAIdle()){__NOP();}
		tft.printString(20, 100, 10, 128, 80, 120, Anchor::BOTTOM_LEFT, pattern, rotenc.GetValue());
		LL_mDelay(500);
		val++;
	}
}
