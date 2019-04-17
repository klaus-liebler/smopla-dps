#include "PowerStageController.hh"
#include "gpio.hh"
#include "math.h"
#include "stm32f1xx_ll_dac.h"

PowerStageController::PowerStageController(const Pin powerControlPin):
powerControlPin(powerControlPin){

}

PowerStageController::~PowerStageController() {

}

void PowerStageController::Begin(){
    /** Load default calibration constants */
    a_adc_k_coef = A_ADC_K;
    a_adc_c_coef = A_ADC_C;
    a_dac_k_coef = A_DAC_K;
    a_dac_c_coef = A_DAC_C;
    v_adc_k_coef = V_ADC_K;
    v_adc_c_coef = V_ADC_C;
    v_dac_k_coef = V_DAC_K;
    v_dac_c_coef = V_DAC_C;
    vin_adc_k_coef = VIN_ADC_K;
    vin_adc_c_coef = VIN_ADC_C;

    /** Load any calibration constants that maybe stored in non-volatile memory (past) */
    //todo!!

    EnableOutput(false);
}

void PowerStageController::EnableOutput(bool enable)
{
    this->outputEnabled = enable;
    if (enable) {

#ifdef DPS5015
        //gpio_clear(GPIOA, GPIO9); // this is power control on '5015
        gpio_set(GPIOB, GPIO11);    // B11 is fan control on '5015
        gpio_clear(GPIOC, GPIO13);  // C13 is power control on '5015
#else
        Gpio::Set(this->powerControlPin, false);  // B11 is power control on '5005
#endif
    } else {
#ifdef DPS5015
        //gpio_set(GPIOA, GPIO9);    // gpio_set(GPIOB, GPIO11);
        gpio_clear(GPIOB, GPIO11); // B11 is fan control on '5015
        gpio_set(GPIOC, GPIO13);   // C13 is power control on '5015
#else
        Gpio::Set(this->powerControlPin, true);  // B11 is power control on '5005
#endif
    }
}

uint16_t PowerStageController::GetUin()
{
	return pwrctl_calc_uin(this->uInRaw);
}

uint16_t PowerStageController::GetUout(){
	return pwrctl_calc_uout(this->uOutRaw);
}

uint16_t PowerStageController::GetIOut()
{
	return pwrctl_calc_iout(this->iOutRaw);
}

/**
  * @brief Calculate V_in based on raw ADC measurement
  * @param raw value from ADC
  * @retval corresponding voltage in milli volt
  */
uint32_t PowerStageController::pwrctl_calc_uin(uint16_t raw)
{
    float value = vin_adc_k_coef * raw + vin_adc_c_coef;
    if (value <= 0)
        return 0;
    else
        return value + 0.5f; /** Add 0.5f to value so it is correctly rounded when it is truncated */
}

/**
  * @brief Calculate V_out based on raw ADC measurement
  * @param raw value from ADC
  * @retval corresponding voltage in milli volt
  */
uint32_t PowerStageController::pwrctl_calc_uout(uint16_t raw)
{
    float value = v_adc_k_coef * raw + v_adc_c_coef;
    if (value <= 0)
        return 0;
    else
        return value + 0.5f; /** Add 0.5f to value so it is correctly rounded when it is truncated */
}

/**
  * @brief Calculate DAC setting for requested V_out
  * @param v_out_mv requested output voltage
  * @retval corresponding 12 bit DAC value
  */
uint16_t PowerStageController::pwrctl_calc_uout_dac(uint32_t v_out_mv)
{
    float value = v_dac_k_coef * v_out_mv + v_dac_c_coef;
    if (value <= 0)
        return 0;
    else if (value >= 0xfff)
        return 0xfff; /** 12 bits */
    else
        return value + 0.5f; /** Add 0.5f to value so correct rounding is done when truncated */
}

/**
  * @brief Calculate I_out based on raw ADC measurement
  * @param raw value from ADC
  * @retval corresponding current in milliampere
  */
uint32_t PowerStageController::pwrctl_calc_iout(uint16_t raw)
{
    float value = a_adc_k_coef * raw + a_adc_c_coef;
    if (value <= 0)
        return 0;
    else
        return value + 0.5f; /** Add 0.5f to value so correct rounding is done when truncated */
}

/**
  * @brief Calculate expected raw ADC value based on selected I_limit
  * @param i_limit_ma selected I_limit
  * @retval expected raw ADC value
  */
uint16_t PowerStageController::pwrctl_calc_ilimit_adc(uint16_t i_limit_ma)
{
    float value = (i_limit_ma - a_adc_c_coef) / a_adc_k_coef + 1;
    if (value <= 0)
        return 0;
    else
        return value + 0.5f; // Add 0.5 so it is correctly rounded when it is truncated
}

/**
  * @brief Calculate expected raw ADC value based on selected V_limit
  * @param v_limit_mv selected V_limit
  * @retval expected raw ADC value
  */
uint16_t PowerStageController::pwrctl_calc_ulimit_adc(uint16_t v_limit_mv)
{
    float value = (v_limit_mv - v_adc_c_coef) / v_adc_k_coef + 1;
    if (value <= 0)
        return 0;
    else
        return value + 0.5f; // Add 0.5 so it is correctly rounded when it is truncated
}

/**
  * @brief Calculate DAC setting for constant current mode
  * @param i_out_ma requested constant current
  * @retval corresponding 12 bit DAC value
  * @note this formula is valid for the DPS5005 and would probably need changes
  *       for DPS:es capable of higher current output.
  */
uint16_t PowerStageController::pwrctl_calc_iout_dac(uint32_t i_out_ma)
{
    float value = a_dac_k_coef * i_out_ma + a_dac_c_coef;
    if (value <= 0)
        return 0;
    else if (value >= 0xfff)
        return 0xfff; /** 12 bits */
    else
        return value + 0.5f; /** Add 0.5f to value so correct rounding is done when truncated */
}

void PowerStageController::Update(volatile uint16_t *VinVOutIout)
{
	this->uOutRaw = VinVOutIout[1]- VinVOutIout[2];
	this->uInRaw=  VinVOutIout[0];
	this->iOutRaw = VinVOutIout[2];

	if(this->outputEnabled)
	{
		  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, this->uLimitRaw);
		  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, this->iLimitRaw);
	}
	else
	{
		  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, 0);
		  LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, 0);
	}
}

void PowerStageController::Set(uint16_t uLimit, uint16_t iLimit)
{
	//TODO: Gibt es ein Problem wenn man die sequentiell setzt und genau in dem Moment der Regelalgorithmus arbeitet?
	//ggf in einen 32bit-Wert verpacken und gemeinsam setzen

	this->iLimit=std::min(iLimit, (uint16_t)CONFIG_DPS_MAX_CURRENT);
	this->uLimit=std::min(uLimit, (uint16_t)CONFIG_DPS_MAX_VOLTAGE);
	this->iLimitRaw=pwrctl_calc_ilimit_adc(this->iLimit);
	this->uLimitRaw=pwrctl_calc_ulimit_adc(this->uLimit);
}
