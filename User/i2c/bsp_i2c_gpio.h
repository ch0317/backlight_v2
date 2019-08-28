#ifndef __BSP_I2C_GPIO_H
#define	__BSP_I2C_GPIO_H


#include "stm32f10x.h"

/*
EN_2D2   PA6
EN_2D    PA7
EN_3D2   PB6
EN_3D1   PB7

SCL_2D2  PB8
SDA_2D2  PB9

SCL_2D   PB10
SDA_2D   PB11

*/


#define I2C_SCL_GPIO_PORT    	GPIOB		              /* GPIO端口 */
#define I2C_SCL_GPIO_CLK 	    RCC_APB2Periph_GPIOB		/* GPIO端口时钟 */
#define I2C_SCL_GPIO_PIN			GPIO_Pin_8			        

#define I2C_SDA_GPIO_PORT    	GPIOB			              /* GPIO端口 */
#define I2C_SDA_GPIO_CLK 	    RCC_APB2Periph_GPIOB		/* GPIO端口时钟 */
#define I2C_SDA_GPIO_PIN			GPIO_Pin_9		        


// NEW ADD PIN
#define I2C_SCL_GPIO_PIN_1			GPIO_Pin_10			        
#define I2C_SDA_GPIO_PIN_1			GPIO_Pin_11

//控制引脚电平
#if 1

	#define EEPROM_I2C_SDA_1()		GPIO_SetBits(I2C_SDA_GPIO_PORT,I2C_SDA_GPIO_PIN)
	#define EEPROM_I2C_SDA_0()		GPIO_ResetBits(I2C_SDA_GPIO_PORT,I2C_SDA_GPIO_PIN)

	#define EEPROM_I2C_SCL_1()		GPIO_SetBits(I2C_SCL_GPIO_PORT,I2C_SCL_GPIO_PIN)
	#define EEPROM_I2C_SCL_0()		GPIO_ResetBits(I2C_SCL_GPIO_PORT,I2C_SCL_GPIO_PIN)

	#define EEPROM_I2C_SDA_READ() GPIO_ReadInputDataBit(I2C_SDA_GPIO_PORT,I2C_SDA_GPIO_PIN)


#define EEPROM_I2C_SDA1_1()		GPIO_SetBits(I2C_SDA_GPIO_PORT,I2C_SDA_GPIO_PIN_1)
#define EEPROM_I2C_SDA1_0()		GPIO_ResetBits(I2C_SDA_GPIO_PORT,I2C_SDA_GPIO_PIN_1)

#define EEPROM_I2C_SCL1_1()		GPIO_SetBits(I2C_SCL_GPIO_PORT,I2C_SCL_GPIO_PIN_1)
#define EEPROM_I2C_SCL1_0()		GPIO_ResetBits(I2C_SCL_GPIO_PORT,I2C_SCL_GPIO_PIN_1)

#define EEPROM_I2C_SDA1_READ() GPIO_ReadInputDataBit(I2C_SDA_GPIO_PORT,I2C_SDA_GPIO_PIN_1)


#else

	#define EEPROM_I2C_SDA_1()		I2C_SDA_GPIO_PORT->BSRR = I2C_SDA_GPIO_PIN  //GPIO_SetBits(I2C_SDA_GPIO_PORT,I2C_SDA_GPIO_PIN)
	#define EEPROM_I2C_SDA_0()		I2C_SDA_GPIO_PORT->BRR = I2C_SDA_GPIO_PIN //GPIO_ResetBits(I2C_SDA_GPIO_PORT,I2C_SDA_GPIO_PIN)

	#define EEPROM_I2C_SCL_1()		I2C_SCL_GPIO_PORT->BSRR = I2C_SCL_GPIO_PIN//GPIO_SetBits(I2C_SCL_GPIO_PORT,I2C_SCL_GPIO_PIN)
	#define EEPROM_I2C_SCL_0()		I2C_SCL_GPIO_PORT->BRR = I2C_SCL_GPIO_PIN //GPIO_ResetBits(I2C_SCL_GPIO_PORT,I2C_SCL_GPIO_PIN)

	#define EEPROM_I2C_SDA_READ() 	((I2C_SDA_GPIO_PORT->IDR & I2C_SDA_GPIO_PIN) != 0) //GPIO_ReadInputDataBit(I2C_SDA_GPIO_PORT,I2C_SDA_GPIO_PIN)

#endif

void i2c_GPIO_Config(void);
void i2c_START(void);
uint8_t i2c_WAIT_ASK(void);
void i2c_WRITE_BYTE(uint8_t data);
void i2c_STOP(void);
void i2c_NASK(void);
uint8_t i2c_READ_BYTE(void);
void i2c_ASK(void);

void i2c1_WRITE_BYTE(uint8_t data);
void i2c1_START(void);
void i2c1_STOP(void);
void i2c1_NASK(void);
uint8_t i2c1_WAIT_ASK(void);
uint8_t i2c1_READ_BYTE(void);
void i2c1_ASK(void);


#endif /* __BSP_I2C_GPIO_H */
