
#include "board.h"
#include "adc.h"

/** V25 (temperature sensor voltage at 25 C) value from STM32F103 datasheet.
 *
 * In [V]. Note that this value is inaccurate and needs calibration.
 * @sa vsnTemperature_calibrateOnePoint */
float vsnTemperature_V25 = 1.543;

static const float vsnTemperature_adcK = 3.3 / (4096.0 - 1.0);

/** Avg_Slope value from STM32F103 datasheet.
 *
 * In [V / K]. Note that while this value is more accurate than
 * V25, it might still need calibration for accurate measurements.
 *
 * @sa vsnTemperature_calibrateTwoPoints */
float vsnTemperature_avgSlope = 4.3e-3;





/** Read raw sensor value. */
uint16_t vsnTemperature_readRaw(void)
{
    adc_init(ADC_0, ADC_RES_8BIT);

/*Set the TSVREFE bit in the ADC_CR2 to wake up the temperature sensor from power down mode*/





	return 1;

}





/** Read sensor value and return temperature in degrees Celsius. */
float vsnTemperature_read(void)
{
	uint16_t raw = vsnTemperature_readRaw();

	return (vsnTemperature_V25 - ((float)raw) * vsnTemperature_adcK) /
		vsnTemperature_avgSlope + 25.0;
}

