#ifndef xSW02_h
#define xSW02_h

#include "bsec.h"
#include <Wire.h>

class xSW02
{
public:

	xSW02(void);
	xSW02(uint8_t addr);

	void begin();

	float getRawTemperature();
	float getPressure();
	float getRawHumidity();
	float getGasResistance();
	float getIAQ();
	float getIAQAccuracy();
	float getTemperature();
	float getHumidity();
	float getStaticIAQ();
	float getCO2Equivalent();
	float getBreathVOCEquivalent();

	void updateSubscription();
	bool run();
	void checkIaqSensorStatus();


private:
	Bsec iaqSensor;
	uint8_t i2c_addr;

	float rawTemperature;
	float pressure;
	float rawHumidity;
	float gasResistance;
	float iaq;
	float iaqAccuracy;
	float temperature;
	float humidity;
	float staticIaq;
	float co2Equivalent;
	float breathVocEquivalent;
	String output;

	void errLeds();
	
};

#endif