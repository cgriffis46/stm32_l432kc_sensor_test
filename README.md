# stm32_l432kc_sensor_test

This STM32 project is testing various drivers for embedded sensors. 

The drivers all have a single main() function intended to be non-blocking and 
some operations require multiple calls to main(). check newData() to see if 
new data is available. 

the benefit of this is that the MCU is free to perform other tasks.

Mutual exclusion is not implemented yet. Currently it's left to the 
User to ensure atomic bus access. 



Working: 
	BMP280
	BME280
	SHT31
	Si7021
	AHT20
	HTU21DF
	
In Progress:
	MPL3115A2
	BMP390