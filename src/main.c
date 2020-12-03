/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>

#define I2C_DEV DT_LABEL(DT_ALIAS(i2c_0))

#define LM75A_DEFAULT_ADDRESS   0x48

/**
 * @file Sample app using the Fujitsu MB85RC256V FRAM through ARC I2C.
 */

#define FRAM_I2C_ADDR	0x50

static int write_bytes(const struct device *i2c_dev, uint16_t addr,
		       uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;

	/* Setup I2C messages */

	/* Send the address to write to */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, FRAM_I2C_ADDR);
}

static int read_bytes(const struct device *i2c_dev, uint16_t addr,
		      uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* Now try to read back from FRAM */

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;

	/* Setup I2C messages */

	/* Send the address to read from */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, FRAM_I2C_ADDR);
}


/*float M2M_LM75A::getTemperature()
{
	uint16_t result;
	if (!read16bitRegister(LM75A_REGISTER_TEMP, result))
	{
		return LM75A_INVALID_TEMPERATURE;
	}
	return (float)result / 256.0f;
}*/

/*bool M2M_LM75A::read16bitRegister(const uint8_t reg, uint16_t& response)
{
	uint8_t result;

	Wire.beginTransmission(_i2cAddress);
	Wire.write(reg);
	result = Wire.endTransmission();
	// result is 0-4 
	if (result != 0)
	{
		return false;
	}

	result = Wire.requestFrom(_i2cAddress, (uint8_t)2);
	if (result != 2)
	{
		return false;
	}
	uint8_t part1 = Wire.read();
	uint8_t part2 = Wire.read();
	
	//response = (Wire.read() << 8) | Wire.read();
	uint16_t temp = part1 << 8 | part2;
	response = part1 << 8 | part2;
	return true;
}*/


const float LM75A_DEGREES_RESOLUTION = 0.125;
const int LM75A_REG_ADDR_TEMP = 0;

void main(void)
{   
  /*  // Temperature
  Serial.print(F("Temperature in Celsius: "));
  Serial.print(lm75a.getTemperature());
  Serial.println(F(" *C"));*/
	const struct device *i2c_dev;
	uint8_t cmp_data[16];
	uint8_t data[16];
	int i, ret;
	uint16_t temp = 0x00;
    uint8_t pointer = 0x00;

	i2c_dev = device_get_binding(I2C_DEV);
	if (!i2c_dev) {
		printk("I2C: Device driver not found.\n");
		return;
	}
	printk("I2C LM75A - Pruebas.\n");
	/* ----- */
	
	pointer = 0x03; //Tos register pointer - 5000h por defecto
	i2c_write(i2c_dev, &pointer, 1, LM75A_DEFAULT_ADDRESS);
	k_msleep(5);
	ret = i2c_read(i2c_dev, &data[0], 2, LM75A_DEFAULT_ADDRESS);
	k_msleep(5);
	printk("----->> SetPoint - Tos Register: %x|%x\n\n", data[0], data[1]);
	
	pointer = 0x02; //Thyst register pointer - 4B00h por defecto
	i2c_write(i2c_dev, &pointer, 1, LM75A_DEFAULT_ADDRESS);
	k_msleep(5);
	ret = i2c_read(i2c_dev, &data[0], 2, LM75A_DEFAULT_ADDRESS);
	k_msleep(5);
	printk("----->> Hysteresis - Thyst Register: %x|%x\n\n", data[0], data[1]);
	
	pointer = 0x01; //Configuration register pointer - 00h por defecto
	i2c_write(i2c_dev, &pointer, 1, LM75A_DEFAULT_ADDRESS);
	k_msleep(5);
	ret = i2c_read(i2c_dev, &data[0], 1, LM75A_DEFAULT_ADDRESS);
	k_msleep(5);
	printk("----->>  Configuration - Register %x|%x\n\n", data[0], data[1]);
	
	
	//only 9 bits of the two bytes are used to store the set-point data in 2’s complement format with the resolution of 0.5 °C
	printk("----->>  Writing - Tos Register %x|%x\n", data[0], data[1]);
	pointer = 0x03;
	data[0] = pointer;
	temp = 24.5; //valor de setpoint que será enviado - resolución de 0.5°C solo si es float
	temp = temp*2; //dividir por 0.5 <=> multiplicar por 2
	data[1] = temp >> 8;//MSByte
	data[2] = (uint16_t)temp & 0x0F;//LSByte
	i2c_write(i2c_dev, &data[0], 3, LM75A_DEFAULT_ADDRESS);
	printk("----->>  Tos Register wrote temp: <%x>, MSByte <%x>, LSByte <%x>\n\n", temp, data[1], data[2]);
	k_msleep(5);
	
	//verificando el registro de Tos
	pointer = 0x03; //Tos register pointer - 5000h por defecto
	i2c_write(i2c_dev, &pointer, 1, LM75A_DEFAULT_ADDRESS);
	k_msleep(5);
	ret = i2c_read(i2c_dev, &data[0], 2, LM75A_DEFAULT_ADDRESS);
	k_msleep(5);
	printk("----->> SetPoint - Tos Register: %x|%x\n\n", data[0], data[1]);
	
	pointer = 0x00; // Temp register pointer
    i2c_write(i2c_dev, &pointer, 1, LM75A_DEFAULT_ADDRESS);
    // the pointer doesn't need to be set every i2c read. Once set, pointer is latched and the next readings will be made form the register pointed by pointer
    ret = 0;
    while(1) {
        //int i2c_write(conststructdevice *dev, const uint8_t *buf, uint32_t num_bytes, uint16_t addr)
        
        // int i2c_read(const structdevice *dev, uint8_t *buf, uint32_t num_bytes, uint16_t addr)
        ret = i2c_read(i2c_dev, &data[0], 2, LM75A_DEFAULT_ADDRESS);
        
        //int i2c_reg_read_byte(conststructdevice *dev, uint16_t dev_addr, uint8_t reg_addr, uint8_t *value)
        //ret = i2c_reg_read_byte(i2c_dev, LM75A_DEFAULT_ADDRESS, 0x00, &data[0]);
        
        if(~ret) {
            printk("Succesful read\n");
        }
        else {
            printk("Error read\n");
        }    
        
        printk("data[0]: %X data[1] %X \n", data[0], data[1]);
        
        temp = data[0] << 8 | (data[1] & 0x80 );
        temp = temp >> 5; //toma 11 MSBits. Como son 16 bits en temp, se debe hacer el corrimiento.
        temp = temp * LM75A_DEGREES_RESOLUTION;
        //temp = data[0] << 8 | data[1];
        printk("Temperatura en Celsius: %d\n", temp); //float not supported by printk
        k_msleep(2500);
    }
    
    
	/* Do one-byte read/write */
    /*
	data[0] = 0xAE;
	ret = write_bytes(i2c_dev, 0x00, &data[0], 1);
	if (ret) {
		printk("Error writing to FRAM! error code (%d)\n", ret);
		return;
	} else {
		printk("Wrote 0xAE to address 0x00.\n");
	}

	data[0] = 0x86;
	ret = write_bytes(i2c_dev, 0x01, &data[0], 1);
	if (ret) {
		printk("Error writing to FRAM! error code (%d)\n", ret);
		return;
	} else {
		printk("Wrote 0x86 to address 0x01.\n");
	}

	data[0] = 0x00;
	ret = read_bytes(i2c_dev, 0x00, &data[0], 1);
	if (ret) {
		printk("Error reading from FRAM! error code (%d)\n", ret);
		return;
	} else {
		printk("Read 0x%X from address 0x00.\n", data[0]);
	}

	data[1] = 0x00;
	ret = read_bytes(i2c_dev, 0x01, &data[0], 1);
	if (ret) {
		printk("Error reading from FRAM! error code (%d)\n", ret);
		return;
	} else {
		printk("Read 0x%X from address 0x01.\n", data[0]);
	}
    */
	
	
}
