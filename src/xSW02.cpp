#include "xSW02.h"

xSW02::xSW02()
{
	i2c_addr = BME680_I2C_ADDR_PRIMARY;
}

xSW02::xSW02(uint8_t addr)
{
	i2c_addr = addr;
}

void xSW02::begin()
{
	output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
	iaqSensor.begin(i2c_addr, Wire);
	//Serial.print(output);
	checkIaqSensorStatus();

  	bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  //Serial.println(output);
}

float xSW02::getRawTemperature()
{
	return iaqSensor.rawTemperature;
}

float xSW02::getPressure()
{
	return 	iaqSensor.pressure;
}

float xSW02::getRawHumidity()
{
	return iaqSensor.rawHumidity;
}

float xSW02::getGasResistance()
{
	return iaqSensor.gasResistance;
}

float xSW02::getIAQ()
{
	return iaqSensor.iaq;
}

float xSW02::getIAQAccuracy()
{
	return iaqSensor.iaqAccuracy;
}

float xSW02::getTemperature()
{
	return iaqSensor.temperature;
}

float xSW02::getHumidity()
{
	return iaqSensor.humidity;
}

float xSW02::getStaticIAQ()
{
	return iaqSensor.staticIaq;
}

float xSW02::getCO2Equivalent()
{
	return iaqSensor.co2Equivalent;
}

float xSW02::getBreathVOCEquivalent()
{
	return iaqSensor.breathVocEquivalent;
}

bool xSW02::run()
{
   unsigned long time_trigger = millis();
  if (iaqSensor.run()) { // If new data is available
    output = String(time_trigger);
    output += ", " + String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.pressure);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.gasResistance);
    output += ", " + String(iaqSensor.iaq);
    output += ", " + String(iaqSensor.iaqAccuracy);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.humidity);
    output += ", " + String(iaqSensor.staticIaq);
    output += ", " + String(iaqSensor.co2Equivalent);
    output += ", " + String(iaqSensor.breathVocEquivalent);
    return true;
    //Serial.println(output);
  } else {
    checkIaqSensorStatus();
    return false;
  }
}

void xSW02::checkIaqSensorStatus()
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      //Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      //Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      //Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      //Serial.println(output);
    }
  }
}

void xSW02::errLeds(void)
{
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);
   delay(100);
  // digitalWrite(LED_BUILTIN, LOW);
   delay(100);
}