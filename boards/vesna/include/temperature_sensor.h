#include "board.h"
#include "adc.h"


extern float vsnTemperature_avgSlope;
extern float vsnTemperature_V25;

uint16_t vsnTemperature_readRaw(void);
float vsnTemperature_read(void);

//void vsnTemperature_calibrateOnePoint(uint16_t raw, float temperature);
//void vsnTemperature_calibrateTwoPoints(uint16_t raw1, float temperature1,
//		uint16_t raw2, float temperature2);
