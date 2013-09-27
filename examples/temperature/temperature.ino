#include <rfm23.h>
/* temperature.ino
 * Created: 07.07.2012 10:51:27
 *  Author: hberg539
 * repackaged Sep 2013, Griff Hamlin
 */ 

rfm23 radio;   // constructor initializes rfm23 

void setup() {
	// init serial
	// 9600 Baud, 8 data, 1 stop bit
	Serial.begin(9600);
	
	// rfm23 test
	// test spi single read/write
	// and burst read/write
   uint8_t i=radio.rfm23_test();
	if (i) {
		Serial.print("rfm23 test successfull.");
	} else {
		Serial.print("rfm23 test failed.");
	}
   Serial.print("returned: "); Serial.print(i); Serial.print("\r\n");
}

void loop() {	
		// get temperature from internal sensor
		uint8_t temp = radio.rfm23_get_temperature();
		
		// temp in C = return_value * 0.5 - 64
		temp = (temp * 0.5) - 64;
		
		Serial.print("rfm23 temperature(C): ");
      Serial.print(temp); Serial.print("\r\n");
		
		delay(2000);
}



