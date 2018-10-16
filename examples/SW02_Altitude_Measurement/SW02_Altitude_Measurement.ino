/*************************************************************
	This is an examples for the SW02
	Digital Temperature, Pressure and Humidity Sensor
	
	You can buy one on our store!
	-----> https://xinabox.cc/products/SW02/
	
	This example request a altitude meausrement to be 
	made by the SW02 and display the data over the Serial
	bus.
	
	Supported on the all ☒CHIPs
	
	The sensor communicates over the I2C Bus.
	
*************************************************************/

#include <xCore.h>
#include <arduino-SW02.h>

xSW02 SW02;

const int DELAY_TIME = 1000;

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
	
	//Delay for sensor to normalise
	delay(5000);
	
	Serial.println("================================");
	Serial.println("     XINABOX SW02 Altitude      ");
	Serial.println("================================");	
}

void loop(){
	// Create a variable to store the data read from SW01
	float alt;
	alt = 0;

  	// Read and calculate data from SW01 sensor
  	SW02.poll();
  
	// Request SW01 to get the altitude measurement and store in
	// the altitude variable	
	alt = SW02.getAltitude();
	
	// Display the recoreded data over the Serial Monitor	
	Serial.print("Altitude: ");
	Serial.print(alt);
	Serial.println(" m");
	
	// Small delay between sensor reads
	delay(DELAY_TIME);
}
