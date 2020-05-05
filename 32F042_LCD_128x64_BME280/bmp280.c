/*
 *
 * Modified by Alex Sedyshev for STM32F042x6.. (CMSIS )
 *  https://github.com/AlekseySedyshev
 * 
*/

/**
 * Ciastkolog.pl (https://github.com/ciastkolog)
 * 
*/
/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 sheinz (https://github.com/sheinz)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "stm32f0xx.h"
#include "bmp280.h"


/**
 * BMP280 registers
 */
#define NULL	0
#define BMP280_REG_TEMP_XLSB   0xFC /* bits: 7-4 */
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_TEMP        (BMP280_REG_TEMP_MSB)
#define BMP280_REG_PRESS_XLSB  0xF9 /* bits: 7-4 */
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_PRESSURE    (BMP280_REG_PRESS_MSB)
#define BMP280_REG_CONFIG      0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define BMP280_REG_CTRL        0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define BMP280_REG_STATUS      0xF3 /* bits: 3 measuring; 0 im_update */
#define BMP280_REG_CTRL_HUM    0xF2 /* bits: 2-0 osrs_h; */

#define BMP280_REG_ID          0xD0
#define BMP280_REG_CALIB       0x88
#define BMP280_REG_HUM_CALIB   0x88

#define BMP280_REG_RESET       0xE0
#define BMP280_RESET_VALUE     0xB6
//-----------------i2c--------------------------
void 			writeReg8(uint8_t reg, uint8_t value)																						{//Write a 8-bit register
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (BME280_ADDRESS << I2C_CR2_SADD_Pos) | (0x80 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){}; 	 
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = value;	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;	
}
void 			writeReg16(uint8_t reg, uint16_t value)																					{//Write a 16-bit register
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (BME280_ADDRESS << I2C_CR2_SADD_Pos) | (0x80 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){}; 	

	for (uint8_t i = 2; i > 0; i--) {
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; 		
	I2C1->TXDR =  (uint8_t) (value >> (i - 1)) & 0xFF;
	while((I2C1->ISR & I2C_ISR_TC)){};
	}
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;	
}

uint8_t 	readReg8(uint8_t reg)																														{//Read an 8-bit register
	
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (BME280_ADDRESS << I2C_CR2_SADD_Pos) | (0x80 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){};
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;	
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD);	
	I2C1->CR2 |= I2C_CR2_RD_WRN;																				// Direction - data in
	I2C1->CR2 |= (BME280_ADDRESS << I2C_CR2_SADD_Pos) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while(!(I2C1->ISR & I2C_ISR_RXNE)){};		
	uint8_t value=I2C1->RXDR;
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	I2C1->CR2 &=(~I2C_CR2_RD_WRN) &(~I2C_CR2_NACK);
	return value; 		
}



uint16_t 	readReg16(uint8_t reg)																													{//Read a 16-bit register
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (BME280_ADDRESS << I2C_CR2_SADD_Pos) | (0x80 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){};
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;	
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD);	
	I2C1->CR2 |= I2C_CR2_RD_WRN;																				// Direction - data in
	I2C1->CR2 |= (BME280_ADDRESS << I2C_CR2_SADD_Pos) | (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 
	uint16_t value = 0;
	for (uint8_t i = 2; i > 0; i--) {
		while(!(I2C1->ISR & I2C_ISR_RXNE)){};
		value |= (uint16_t) (I2C1->RXDR << (8 * (i - 1)));
	}
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	I2C1->CR2 &=(~I2C_CR2_RD_WRN) &(~I2C_CR2_NACK);
	return value; 
}


void writeMulti(uint8_t reg, uint8_t const * src, uint8_t count)													{// writeMulti
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (BME280_ADDRESS << I2C_CR2_SADD_Pos) | (0xff << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while (count-- > 0) {
		while(!(I2C1->ISR & I2C_ISR_TXE)){};  
		I2C1->TXDR= *(src++);
		while((I2C1->ISR & I2C_ISR_TC)){};
	}
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
}



void readMulti(uint8_t reg, uint8_t * dst, uint8_t count)																	{// readMulti
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (BME280_ADDRESS << I2C_CR2_SADD_Pos) | (0xff << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY	
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD);	
	I2C1->CR2 |= I2C_CR2_RD_WRN;																				// Direction - data in
	I2C1->CR2 |= (BME280_ADDRESS << I2C_CR2_SADD_Pos) | (count << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while (count-- > 0) {
		
		while(!(I2C1->ISR & I2C_ISR_RXNE)){};
		*(dst++) = I2C1->RXDR;
	}
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	I2C1->CR2 &=(~I2C_CR2_RD_WRN) &(~I2C_CR2_NACK);
}





//--------------end of i2c----------------------
void bmp280_init_default_params(bmp280_params_t *params) {
	params->mode = BMP280_MODE_NORMAL;
	params->filter = BMP280_FILTER_OFF;
	params->oversampling_pressure = BMP280_STANDARD;
	params->oversampling_temperature = BMP280_STANDARD;
	params->oversampling_humidity = BMP280_STANDARD;
	params->standby = BMP280_STANDBY_250;
	//params->standby = BMP280_STANDBY_4000;
}


void bmp280_sleep(bmp280_params_t *params) {
	params->mode = BMP280_MODE_SLEEP;
}


static void read_calibration_data(BMP280_HandleTypedef *dev) {

	dev->dig_T1 = readReg8(0x89)<<8 | readReg8(0x88);
	dev->dig_T2 = readReg8(0x8b)<<8 | readReg8(0x8a);
	dev->dig_T3 = readReg8(0x8d)<<8 | readReg8(0x8c);
	dev->dig_P1 = readReg8(0x8f)<<8 | readReg8(0x8e);
	dev->dig_P2	= readReg8(0x91)<<8 | readReg8(0x90);
	dev->dig_P3	= readReg8(0x93)<<8 | readReg8(0x92);
	dev->dig_P4	= readReg8(0x95)<<8 | readReg8(0x94);
	dev->dig_P5	= readReg8(0x97)<<8 | readReg8(0x96);
	dev->dig_P6	= readReg8(0x99)<<8 | readReg8(0x98);
	dev->dig_P7	= readReg8(0x9b)<<8 | readReg8(0x9a);
	dev->dig_P8	= readReg8(0x9d)<<8 | readReg8(0x9c);
	dev->dig_P9	= readReg8(0x9f)<<8 | readReg8(0x9e);

}

static void read_hum_calibration_data(BMP280_HandleTypedef *dev) {
	uint8_t e4,e5,e6;

	dev->dig_H1 = readReg8(0xa1);
	dev->dig_H2	=	readReg8(0xe2)<<8 | readReg8(0xe1);
	dev->dig_H3	=	readReg8(0xe3);
	
					e4	=	readReg8(0xe4);
					e5	=	readReg8(0xe5);
					e6  =	readReg8(0xe6);
	dev->dig_H4 = (e4<<4) | (e5 & 0xf);
	dev->dig_H5 = (e6<<4)	| (e5>>4);
	dev->dig_H6	=	readReg8(0xe7);
	
}


bool bmp280_init(BMP280_HandleTypedef *dev, bmp280_params_t *params) {

	dev->id=readReg8(BMP280_REG_ID);
	if (dev->id != BMP280_CHIP_ID && dev->id != BME280_CHIP_ID) {
	return false;
	}
	writeReg8(BMP280_REG_RESET, BMP280_RESET_VALUE);	// Soft reset.
	while(readReg8(BMP280_REG_STATUS)&0x01){};				// Wait until finished copying over the NVP data.
	
	read_calibration_data(dev);
	read_hum_calibration_data(dev);

	uint8_t config = (params->standby << 5) | (params->filter << 2);
	writeReg8(BMP280_REG_CONFIG, config);

	if (params->mode == BMP280_MODE_FORCED) {
		params->mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
	}

	uint8_t ctrl = (params->oversampling_temperature << 5)
			| (params->oversampling_pressure << 2) | (params->mode);

	if (dev->id == BME280_CHIP_ID) {
		// Write crtl hum reg first, only active after write to BMP280_REG_CTRL.
		uint8_t ctrl_hum = params->oversampling_humidity;
		writeReg8(BMP280_REG_CTRL_HUM, ctrl_hum);
	}
	
	writeReg8(BMP280_REG_CTRL, ctrl);
	return true;
}

void bmp280_force_measurement(BMP280_HandleTypedef *dev) {
	uint8_t ctrl;
	ctrl=readReg8(BMP280_REG_CTRL);
	ctrl &= ~3;  // clear two lower bits
	ctrl |= BMP280_MODE_FORCED;
	writeReg8(BMP280_REG_CTRL, ctrl);
}

bool bmp280_is_measuring(BMP280_HandleTypedef *dev) {
	uint8_t status;
	status=readReg8(BMP280_REG_STATUS);
	if (status & (1 << 3)) {
		return true;
	}
	return false;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in degrees Celsius.
 */
static inline int32_t compensate_temperature(BMP280_HandleTypedef *dev, int32_t adc_T,	int32_t *t_fine) {
	int32_t var1, var2, T;

  var1  = ((((adc_T>>3) - ((int32_t)dev->dig_T1<<1))) * ((int32_t)dev->dig_T2)) >> 11; 
  var2  = (((((adc_T>>4) - (int32_t)dev->dig_T1)) * ((adc_T>>4) - ((int32_t)dev->dig_T1)))>> 12) *((int32_t)dev->dig_T3) >> 14; 
  *t_fine = var1 + var2; 
  T  = (*t_fine * 5 + 128) >> 8; 
  return T; 
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_pressure(BMP280_HandleTypedef *dev, int32_t adc_press, int32_t t_fine) {
	int64_t var1, var2, p;

	var1 = ((int64_t) t_fine) - 128000;
	var2 = var1 * var1 * (int64_t) dev->dig_P6;
	var2 = var2 + ((var1 * (int64_t) dev->dig_P5) << 17);
	var2 = var2 + (((int64_t) dev->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) dev->dig_P3) >> 8)
			+ ((var1 * (int64_t) dev->dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) dev->dig_P1) >> 33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) dev->dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t) dev->dig_P7 << 4);
	return (uint32_t)p;
}

/**
 * Compensation algorithm is taken from BME280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_humidity(BMP280_HandleTypedef *dev, int32_t adc_hum, int32_t t_fine) {
	int32_t v_x1_u32r;

	v_x1_u32r = t_fine - (int32_t) 76800;
	v_x1_u32r = ((((adc_hum << 14) - ((int32_t) dev->dig_H4 << 20)
			- ((int32_t) dev->dig_H5 * v_x1_u32r)) + (int32_t) 16384) >> 15)
			* (((((((v_x1_u32r * (int32_t) dev->dig_H6) >> 10)
					* (((v_x1_u32r * (int32_t) dev->dig_H3) >> 11)
							+ (int32_t) 32768)) >> 10) + (int32_t) 2097152)
					* (int32_t) dev->dig_H2 + 8192) >> 14);
	v_x1_u32r = v_x1_u32r
			- (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
					* (int32_t) dev->dig_H1) >> 4);
	v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
	v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;
	return v_x1_u32r >> 12;
}


void bmp280_read_fixed(BMP280_HandleTypedef *dev, int32_t *temperature, uint32_t *pressure,	uint32_t *humidity) {
	int32_t adc_pressure;
	int32_t adc_temp;
	uint8_t data[8];
	int32_t fine_temp;
	
	// Only the BME280 supports reading the humidity.
	if (dev->id != BME280_CHIP_ID) {*humidity = 0; humidity = NULL;}
		
	
	// Need to read in one sequence to ensure they match.
	uint8_t size = humidity ? 8 : 6;
	readMulti(BMP280_REG_PRESS_MSB, data, size);
	
	adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	adc_temp = data[3] << 12 | data[4] << 4 | data[5] >> 4;

	*temperature = compensate_temperature(dev, adc_temp, &fine_temp);
	
	*pressure = compensate_pressure(dev, adc_pressure, fine_temp)*0.00750063755419211;

	if (humidity) {
		int32_t adc_humidity = data[6] << 8 | data[7];
		//h1=compensate_humidity(dev, adc_humidity, fine_temp);
		*humidity = compensate_humidity(dev, adc_humidity, fine_temp);
	}

}

