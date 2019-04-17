#pragma once
#include <stddef.h>
#include <stdint.h>
#include "dps-model.h"
#include "Gpio.hh"
class PowerStageController {
public:
	PowerStageController(const Pin powerControlPin);
	virtual ~PowerStageController();
	void Begin();
	void Update(volatile uint16_t *UinoutIOut);
	void Set(uint16_t uLimit, uint16_t iLimit);
	void EnableOutput(bool enable);
	uint16_t GetUin();
	uint16_t GetUout();
	uint16_t GetIOut();
private:
	const Pin powerControlPin;
	uint16_t uLimitRaw=0, iLimitRaw=0, uInRaw=0, uOutRaw=0, iOutRaw=0;
	uint16_t uLimit=0, iLimit=0;
	bool outputEnabled=false;
	float a_adc_k_coef = A_ADC_K;
	float a_adc_c_coef = A_ADC_C;
	float a_dac_k_coef = A_DAC_K;
	float a_dac_c_coef = A_DAC_C;
	float v_adc_k_coef = V_ADC_K;
	float v_adc_c_coef = V_ADC_C;
	float v_dac_k_coef = V_DAC_K;
	float v_dac_c_coef = V_DAC_C;
	float vin_adc_k_coef = VIN_ADC_K;
	float vin_adc_c_coef = VIN_ADC_C;
	uint32_t pwrctl_calc_uin(uint16_t);
	uint32_t pwrctl_calc_uout(uint16_t);
	uint16_t pwrctl_calc_uout_dac(uint32_t);
	uint32_t pwrctl_calc_iout(uint16_t x);
	uint16_t pwrctl_calc_ilimit_adc(uint16_t);
	uint16_t pwrctl_calc_ulimit_adc(uint16_t);
	uint16_t pwrctl_calc_iout_dac(uint32_t);
};

