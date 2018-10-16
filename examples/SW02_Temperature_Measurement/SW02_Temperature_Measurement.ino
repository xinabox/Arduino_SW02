/*************************************************************
	This is an examples for the SW02
	Digital Temperature, Pressure and Humidity Sensor
	
	You can buy one on our store!
	-----> https://xinabox.cc/products/SW02
	
	This example request a temperature meausrement to be 
	made by the SW02 and display the data over the Serial
	bus.
	
	Supported on the all ☒CHIPs
	
	The sensor communicates over the I2C Bus.
	
*************************************************************/

#include <xCore.h>
#include <arduino-SW02.h>

const int DELAY_TIME = 1000;

xSW02 SW02;

void setup(){
	// Start the Serial Monitor
	Serial.begin(115200);

	// Set the I2C Pins for CW01
	#ifdef ESP8266
	  Wire.pins(2, 14);
	  Wire.setClockStretchLimit(15000);
	#endif
	
	// Start the I2C Comunication
	Wire.begin();
	
	// Start the  SW02 Sensor
	SW02.begin();
	
	Serial.println("================================");
	Serial.println("    XINABOX SW02 Temperature    ");
	Serial.println("================================");
	
	//Delay for sensor to normalise
	delay(5000);
}

void loop(){
	// Create a variable to store the data read from SW02	
	float tempC;
	float tempF;
	tempC = tempF = 0;
  
	// Read and calculate data from SW02 sensor
	SW02.poll();
  
	// Request SW02 to get the temperature measurement and store in
	// the temperature variable		
	tempC = SW02.getTempC(); // Temperature in Celcuis
	tempF = SW02.getTempF(); // Temperature in Farenheit
	
	// Display the recoreded data over the Serial Monitor		
	Serial.print("Temperature: ");
	Serial.print(tempC);
	Serial.println(" C");
	Serial.print("Temperature: ");
	Serial.print(tempF);
	Serial.println(" F");	

  	// Small delay between sensor reads
	delay(DELAY_TIME);
}
