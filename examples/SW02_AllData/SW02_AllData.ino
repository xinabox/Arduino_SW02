#include <xSW02.h>

xSW02 SW02;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  SW02.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

  if(SW02.run()) {
    Serial.println("Raw Temperature       : " + String(SW02.getRawTemperature()));
    Serial.println("Pressure              : " + String(SW02.getPressure()));
    Serial.println("Raw Humidity          : " + String(SW02.getRawHumidity()));
    Serial.println("Gas Resistance        : " + String(SW02.getGasResistance()));
    Serial.println("IAQ                   : " + String(SW02.getIAQ()));
    Serial.println("IAQ Acuracy           : " + String(SW02.getIAQAccuracy()));
    Serial.println("Temperature           : " + String(SW02.getTemperature()));
    Serial.println("Humidity              : " + String(SW02.getHumidity()));
    Serial.println("CO2 Equivalent        : " + String(SW02.getCO2Equivalent()));
    Serial.println("Breath VOC Equivalent : " + String(SW02.getBreathVOCEquivalent()));
  } else {
        // sensor status being checked inside library
  }
}
