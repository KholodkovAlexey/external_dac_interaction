#ifndef INC_CS43L22_H_
#define INC_CS43L22_H_

#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx.h"

#define I2C_SR(I2C) (((uint32_t)I2C->SR1 | (((uint32_t) I2C->SR2) << 16)))

#define CS43L22_POWER_CONTROL1R 0x02
#define CS43L22_CLOCK_CONTROLR 0x05
#define CS43L22_INTERFACE_CONTROL1R 0x06

#define CS43L22_POWER_CONTROL1R_POWERUP 0x9E
#define CS43L22_CLOCK_CONTROLR_MCLKDIV2 0x01
#define CS43L22_CLOCK_CONTROLR_AUTO 0x80

typedef struct {
	I2C_TypeDef *I2C;
	SPI_TypeDef *SPI;
} CS43L22_CodecTypeDef;

void CS43L22_InitGPIO(GPIO_TypeDef *Reset_Port, uint16_t Reset_Pin,
		GPIO_TypeDef *I2C_Port, uint16_t I2C_Pins, GPIO_TypeDef *I2S_WS_Port,
		uint16_t I2S_WS_Pin, GPIO_TypeDef *I2S_MCK_CK_SD_Port,
		uint16_t I2S_MCK_CK_SD_Pins, uint8_t AF_I2C, uint8_t AF_I2S);
void CS43L22_InitCodec(CS43L22_CodecTypeDef *CS43L22_CodecStruct);
void GetPinDPos(uint8_t Pin_Dpos[], uint16_t Pin, uint8_t Count);
void SetPinAF(GPIO_TypeDef *Port, uint8_t Pin_Dpos, uint8_t AF);
void Send_CS43L22_Control(I2C_TypeDef *I2C, uint8_t reg, uint8_t dat);
void ConfigAudio(CS43L22_CodecTypeDef *CS43L22_CodecStruct, uint32_t frequency,
		uint16_t bits_per_sample);
void Send_CS43L22_Sample(CS43L22_CodecTypeDef *CS43L22_CodecStruct, uint32_t sample, uint16_t bits_per_sample);

#endif /* INC_CS43L22_H_ */
