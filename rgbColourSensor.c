/*
 * rgbColourSensor.c
 *
 *  Created on: Nov 25, 2023
 *      Author: Harry
 */
#include "rgbColourSensor.h"
#include "stm32f407xx.h"
#include "main.h"

void writeData(uint8_t adr, uint8_t reg, uint8_t data){
	char volatile res; //may be needed to reset flags
	int timeout = 1000;
	//write start signal
	I2C1->CR1 |= 1 << 8; //send start signal
	while((I2C1->SR1 & 0x0001) == 0); //wait until start signal flag appears on SR1

	//write peripheral address
	I2C1->DR = adr; //write address to data register
	while((I2C1->SR1 & 0x0002) == 0 && timeout--); //wait until address sent flag appears on SR1
	res = (I2C1->SR2); //reset flags (just in case)
	if(timeout < 0){
		//if peripheral doesnt exist stop trying to transmit
		I2C1->CR1 |= 1 << 9; // Send the I2C stop symbol
		while ((I2C1->SR2 & 0x0002) ==   0); // Wait for stop to complete
	} else {
		timeout = 1000;
		GPIOE->BSRR = 1 << 9;
		//write register address
		I2C1->DR = reg; //write register address to data register
		while((I2C1->SR1 & (1 << 7)) == 0); //wait until transmit empty bit is set
		//write data
		I2C1->DR = data; //write data to data register
		while((I2C1->SR1 & (1 << 7)) == 0); //wait until transmit empty bit is set

		//write a stop signal
		I2C1->CR1 |= 1 << 9; // Send the I2C stop symbol
		while ((I2C1->SR2 & 0x0002) == 0); // Wait for stop to complete
	}
}
char readData(uint8_t adr, uint8_t reg) {
	uint8_t data = 0;
	char volatile res; //may be needed to reset flags
	int timeout = 1000;
	//write start signal
	I2C1->CR1 |= 1 << 8;
	while(!(I2C1->SR1 & 0x0001));
	//write peripheral address
	I2C1->DR = adr; //write address to data register
		while(!(I2C1->SR1 & 0x0002) && timeout--); //wait until address sent flag appears on SR1
		res = (I2C1->SR2); //reset flags (just in case)
		if(timeout < 0){
			//if peripheral doesnt exist stop trying to transmit
			I2C1->CR1 |= 1 << 9; // Send the I2C stop symbol
			while (!(I2C1->SR2 & 0x0002)); // Wait for stop to complete
		} else {
			timeout = 1000;
			//write register address
			I2C1->DR = reg; //write register address to data register
			while(!(I2C1->SR1 & (1 << 7))); //wait until transmit empty bit is set

			//write start signal
			I2C1->CR1 |= 1 << 8;
			while(!(I2C1->SR1 & 0x0001));

			//write peripheral address
			I2C1->DR = adr+1; //write address to data register
			while(!(I2C1->SR1 & 0x0002) && timeout--); //wait until address sent flag appears on SR1
			res = (I2C1->SR2); //reset flags (just in case)
			if(timeout < 0){
				//if peripheral doesnt exist stop trying to transmit
				I2C1->CR1 |= 1 << 9; // Send the I2C stop symbol
				while (!(I2C1->SR2 & 0x0002)); // Wait for stop to complete
			} else {
				timeout = 1000;
				while(!(I2C1->SR1 & (1 << 6)));
				data = I2C1->DR;
			}

			//write a stop signal
			I2C1->CR1 |= 1 << 9; //send stop signal
			while(!(I2C1->SR2 & 0x0002)); //wait for stop signal to be sent
		}

	return data;
}

void readData16(uint8_t adr, uint8_t reg, uint16_t *data) {
	char volatile res; //may be needed to reset flags
	int timeout = 1000;
	//write start signal
	I2C1->CR1 |= 1 << 8;
	while(!(I2C1->SR1 & 0x0001));
	//write peripheral address
	I2C1->DR = adr; //write address to data register
		while(!(I2C1->SR1 & 0x0002) && timeout--); //wait until address sent flag appears on SR1
		res = (I2C1->SR2); //reset flags (just in case)
		if(timeout < 0){
			//if peripheral doesnt exist stop trying to transmit
			I2C1->CR1 |= 1 << 9; // Send the I2C stop symbol
			while (!(I2C1->SR2 & 0x0002)); // Wait for stop to complete
		} else {
			timeout = 1000;
			//write register address
			I2C1->DR = reg; //write register address to data register
			while(!(I2C1->SR1 & (1 << 7))); //wait until transmit empty bit is set

			//write start signal
			I2C1->CR1 |= 1 << 8;
			while(!(I2C1->SR1 & 0x0001));

			//write peripheral address
			I2C1->DR = adr+1; //write address to data register
			while(!(I2C1->SR1 & 0x0002) && timeout--); //wait until address sent flag appears on SR1
			res = (I2C1->SR2); //reset flags (just in case)
			if(timeout < 0){
				//if peripheral doesnt exist stop trying to transmit
				I2C1->CR1 |= 1 << 9; // Send the I2C stop symbol
				while (!(I2C1->SR2 & 0x0002)); // Wait for stop to complete
			} else {
				timeout = 1000;
				I2C1->CR1 |= 1 << 10; // Request acknowledgement
				while (!(I2C1->SR1 & (1 << 6))); // Wait for data to arrive
				data[1] = I2C1->DR; // store the least significant byte in the second element
				I2C1->CR1 &= ~(1 << 10); // Prevent acknowledgement
				while (!(I2C1->SR1 & (1 << 6))); // Wait for data to arrive
				data[0] = I2C1->DR; // store the most significant byte in the first element
			}

			//write a stop signal
			I2C1->CR1 |= 1 << 9; //send stop signal
			while(!(I2C1->SR2 & 0x0002)); //wait for stop signal to be sent
		}

}

void setupDevice(uint8_t integrationTime, uint8_t analogueGain){
	 writeData(TCS34725_ADDR, TCS34725_COMMAND_BIT | ATIME_REG, integrationTime); //sets ADC integration time
	 writeData(TCS34725_ADDR, TCS34725_COMMAND_BIT | CONTROL_REG, analogueGain); //sets ADC analogue gain
}

void enableDevice(){
	writeData(TCS34725_ADDR, TCS34725_COMMAND_BIT | ENABLE_REG, 0x01); //writes to enable register to take the device out of the power off state
	  HAL_Delay(3);
	  writeData(TCS34725_ADDR, TCS34725_COMMAND_BIT | ENABLE_REG, 0x03); //enables ADCs so the device progresses through its states and continually reads the rgb data
	  HAL_Delay(100); //delay for at least integration time so some values are ready to be read
}

void getColourData(uint16_t *rgbColourData){
	uint16_t tempBuf[2];
	readData16(TCS34725_ADDR, TCS34725_COMMAND_BIT | RDATA_REG, (uint16_t *)tempBuf);
	rgbColourData[0] = (tempBuf[0] << 8) + tempBuf[1];
	readData16(TCS34725_ADDR, TCS34725_COMMAND_BIT | GDATA_REG, (uint16_t *)tempBuf);
	rgbColourData[1] = (tempBuf[0] << 8) + tempBuf[1];
	readData16(TCS34725_ADDR, TCS34725_COMMAND_BIT | BDATA_REG, (uint16_t *)tempBuf);
	rgbColourData[2] = (tempBuf[0] << 8) + tempBuf[1];
}
