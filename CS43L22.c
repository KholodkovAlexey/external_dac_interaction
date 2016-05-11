#include "CS43L22.h"

void CS43L22_InitGPIO(GPIO_TypeDef *Reset_Port, uint16_t Reset_Pin,
		GPIO_TypeDef *I2C_Port, uint16_t I2C_Pins, GPIO_TypeDef *I2S_WS_Port,
		uint16_t I2S_WS_Pin, GPIO_TypeDef *I2S_MCK_CK_SD_Port,
		uint16_t I2S_MCK_CK_SD_Pins, uint8_t AF_I2C, uint8_t AF_I2S) {
	uint8_t pin_dpos[3] = { 0, 0, 0 };

	//Reset сигнал для CS43L22
	GetPinDPos(pin_dpos, Reset_Pin, 1);
	//Режим General purpose output mode
	Reset_Port->MODER &= ~(GPIO_MODER_MODER0 << pin_dpos[0]);
	Reset_Port->MODER |= (GPIO_MODER_MODER0_0 << pin_dpos[0]);
	//Тип вывода push-pull
	Reset_Port->OTYPER &= ~((uint32_t) Reset_Pin);
	//Режим Pull-down
	Reset_Port->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << pin_dpos[0]);
	Reset_Port->PUPDR |= (GPIO_PUPDR_PUPDR0_1 << pin_dpos[0]);
	//Скорость вывода hight (50MHz)
	Reset_Port->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << pin_dpos[0]);
	Reset_Port->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0_1 << pin_dpos[0]);

	//Пины для I2C
	GetPinDPos(pin_dpos, I2C_Pins, 2);
	//Режим Alternate function
	I2C_Port->MODER &= ~((GPIO_MODER_MODER0 << pin_dpos[0])
			| (GPIO_MODER_MODER0 << pin_dpos[1]));
	I2C_Port->MODER |= (GPIO_MODER_MODER0_1 << pin_dpos[0])
			| (GPIO_MODER_MODER0_1 << pin_dpos[1]);
	//Тип вывода open-drain
	I2C_Port->OTYPER |= I2C_Pins;
	//Режим No pull-up, pull-down GPIO_PUPDR_PUPDR0
	I2C_Port->PUPDR &= ~((GPIO_PUPDR_PUPDR0 << pin_dpos[0])
			| (GPIO_PUPDR_PUPDR0 << pin_dpos[1]));
	//Скорость вывода hight (50MHz)
	I2C_Port->OSPEEDR &= ~((GPIO_OSPEEDER_OSPEEDR0 << pin_dpos[0])
			| (GPIO_OSPEEDER_OSPEEDR0 << pin_dpos[1]));
	I2C_Port->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0_1 << pin_dpos[0])
			| (GPIO_OSPEEDER_OSPEEDR0_1 << pin_dpos[1]);
	//Устанавливаем alternative function I2C
	SetPinAF(I2C_Port, pin_dpos[0], AF_I2C);
	SetPinAF(I2C_Port, pin_dpos[1], AF_I2C);

	//Пины для I2S
	GetPinDPos(pin_dpos, I2S_WS_Pin, 1);
	//Режим Alternate function
	I2S_WS_Port->MODER &= ~(GPIO_MODER_MODER0 << pin_dpos[0]);
	I2S_WS_Port->MODER |= GPIO_MODER_MODER0_1 << pin_dpos[0];
	//Тип вывода push-pull
	I2S_WS_Port->OTYPER &= ~((uint32_t) I2S_WS_Pin);
	//Режим No pull-up, pull-down
	I2S_WS_Port->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << pin_dpos[0]);
	//Скорость вывода hight (50MHz)
	I2S_WS_Port->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << pin_dpos[0]);
	I2S_WS_Port->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_1 << pin_dpos[0];
	//Устанавливаем alternative function I2S
	SetPinAF(I2S_WS_Port, pin_dpos[0], AF_I2S);

	GetPinDPos(pin_dpos, I2S_MCK_CK_SD_Pins, 3);
	//Режим Alternate function
	I2S_MCK_CK_SD_Port->MODER &= ~((GPIO_MODER_MODER0 << pin_dpos[0])
			| (GPIO_MODER_MODER0 << pin_dpos[1])
			| (GPIO_MODER_MODER0 << pin_dpos[2]));
	I2S_MCK_CK_SD_Port->MODER |= (GPIO_MODER_MODER0_1 << pin_dpos[0])
			| (GPIO_MODER_MODER0_1 << pin_dpos[1])
			| (GPIO_MODER_MODER0_1 << pin_dpos[2]);
	//Тип вывода push-pull
	I2S_MCK_CK_SD_Port->OTYPER &= ~((uint32_t) I2S_MCK_CK_SD_Pins);
	//Режим No pull-up, pull-down
	I2S_MCK_CK_SD_Port->PUPDR &= ~((GPIO_PUPDR_PUPDR0 << pin_dpos[0])
			| (GPIO_PUPDR_PUPDR0 << pin_dpos[1])
			| (GPIO_PUPDR_PUPDR0 << pin_dpos[2]));
	//Скорость вывода hight (50MHz)
	I2S_MCK_CK_SD_Port->OSPEEDR &= ~((GPIO_OSPEEDER_OSPEEDR0 << pin_dpos[0])
			| (GPIO_OSPEEDER_OSPEEDR0 << pin_dpos[1])
			| (GPIO_OSPEEDER_OSPEEDR0 << pin_dpos[2]));
	I2S_MCK_CK_SD_Port->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0_1 << pin_dpos[0])
			| (GPIO_OSPEEDER_OSPEEDR0_1 << pin_dpos[1])
			| (GPIO_OSPEEDER_OSPEEDR0_1 << pin_dpos[2]);
	//Устанавливаем alternative function I2S
	SetPinAF(I2S_MCK_CK_SD_Port, pin_dpos[0], AF_I2S);
	SetPinAF(I2S_MCK_CK_SD_Port, pin_dpos[1], AF_I2S);
	SetPinAF(I2S_MCK_CK_SD_Port, pin_dpos[2], AF_I2S);

	//Поднимаем reset сигнал CS43L22
	Reset_Port->BSRR = (uint32_t) Reset_Pin; //<< 16;
}

void CS43L22_InitCodec(CS43L22_CodecTypeDef *CS43L22_CodecStruct) {
	//Включаем I2C
	CS43L22_CodecStruct->I2C->CR1 |= I2C_CR1_PE;

	//Настраиваем I2C
	//Выключаем I2C
	CS43L22_CodecStruct->I2C->CR1 &= ~I2C_CR1_PE;
	//Записываем частоту переферии APB1 (42MHz)
	CS43L22_CodecStruct->I2C->CR2 &= ~I2C_CR2_FREQ;
	CS43L22_CodecStruct->I2C->CR2 |= (uint16_t) (APB1_CLK / 1000000);
	//частота_APB1 / (частота_I2C << 1)  (i2c base clock = 100000)
	CS43L22_CodecStruct->I2C->CCR &= ~I2C_CCR_CCR;
	CS43L22_CodecStruct->I2C->CCR |= (uint16_t) (APB1_CLK / (200000));
	//частота_APB1 + 1
	CS43L22_CodecStruct->I2C->TRISE &= ~I2C_TRISE_TRISE;
	CS43L22_CodecStruct->I2C->TRISE |= (uint16_t) ((APB1_CLK / 1000000) + 1);
	//Включаем I2C, ставим бит подтверждения
	CS43L22_CodecStruct->I2C->CR1 &= ~(I2C_CR1_SMBUS);

	//I2C1->CR1 |= I2C_CR1_PE | I2C_CR1_ACK;
	CS43L22_CodecStruct->I2C->CR1 |= I2C_CR1_ACK;

	//Поднимаем 14 бит, выставляем собственный адрес 0x33
	CS43L22_CodecStruct->I2C->OAR1 &= ~(I2C_OAR1_ADD1_7 | I2C_OAR1_ADD0);
	CS43L22_CodecStruct->I2C->OAR1 |= (((uint16_t) 0x0001) << 14)
			| ((uint16_t) 0x0033);

	CS43L22_CodecStruct->I2C->CR1 |= I2C_CR1_PE;

	//Настраиваем I2S
	//Выключаем I2S
	CS43L22_CodecStruct->SPI->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
	//Отчищаем SPI_I2SCFGR (возможно, это можно сделат в одну команду с предыдущей строкой, но в документации сказанно, что многие значения должны измнятся толко при выключенном I2S)
	SPI3->I2SCFGR &= ~(SPI_I2SCFGR_I2SCFG | SPI_I2SCFGR_I2SSTD
			| SPI_I2SCFGR_CKPOL | SPI_I2SCFGR_DATLEN);
	//Настраиваем предделитель (SPI_I2SPR_I2SDIV * 2 + SPI_I2SPR_ODD), включаем master clock
	CS43L22_CodecStruct->SPI->I2SPR &= ~(SPI_I2SPR_ODD | SPI_I2SPR_I2SDIV);
	CS43L22_CodecStruct->SPI->I2SPR |= SPI_I2SPR_MCKOE | 12;
	//Задаем режим I2S, включаем I2S, режим мастер-передача, стандарт Phillips, размер данных 16 бит, размер канала 16 бит.
	CS43L22_CodecStruct->SPI->I2SCFGR |= SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SE
			| SPI_I2SCFGR_I2SCFG_1;

	uint32_t i;

	//Данные для инициализации cs43l22
	const uint8_t cs43l22_init_data[] = {
			0x00, 0x99,
			0x47, 0x80,
			0x32, 0xBB,
			0x32, 0x3B,
			0x00, 0x00,

			CS43L22_POWER_CONTROL2R, 0xAF,
			CS43L22_PLAYBACK_CONTROL1R, 0x70,
			CS43L22_INTERFACE_CONTROL1R, 0x07,
			CS43L22_ANALOG_ZC_SR_SETTINGSR, 0x00,
			CS43L22_LIMITTER_CONTROL1R, 0x00,
			CS43L22_PCMA_VOLUMER, 0x0A,
			CS43L22_PCMB_VOLUMER, 0x0A,
			CS43L22_TONE_CONTROLR, 0x0F,
			CS43L22_CLOCK_CONTROLR, CS43L22_CLOCK_CONTROLR_MCLKDIV2
					| CS43L22_CLOCK_CONTROLR_AUTO,
			CS43L22_POWER_CONTROL1R, CS43L22_POWER_CONTROL1R_POWERUP };

	//Заполняем регистры cs43l22 даннными
	for (i = 0; i < sizeof(cs43l22_init_data); i += 2) {
		Send_CS43L22_Control(CS43L22_CodecStruct->I2C, cs43l22_init_data[i],
				cs43l22_init_data[i + 1]);
	}
}

void GetPinDPos(uint8_t Pin_Dpos[], uint16_t Pin, uint8_t Count) {
	uint8_t i, j;
	for (i = 0, j = 0; (Pin != 0) && (j < Count); ++i) {
		if (Pin & 0x01) {
			Pin_Dpos[j++] = (uint8_t) (i << 1);
		}
		Pin = Pin >> 1;
	}
}

void SetPinAF(GPIO_TypeDef *Port, uint8_t Pin_Dpos, uint8_t AF) {
	/*if (Pin_Dpos >= 0x10) {
	 Port->AFR[1] &= ~(uint32_t) (0x0F
	 << (uint32_t) (((uint32_t) Pin_Dpos << 1) - 0x20));
	 Port->AFR[1] |= (uint32_t) (AF
	 << (uint32_t) (((uint32_t) Pin_Dpos << 1) - 0x20));
	 } else {
	 Port->AFR[0] &= ~(uint32_t) (0x0F
	 << (uint32_t) ((uint32_t) Pin_Dpos << 1));
	 Port->AFR[0] |=
	 (uint32_t) (AF << (uint32_t) ((uint32_t) Pin_Dpos << 1));
	 }*/

	uint8_t height = Pin_Dpos >= 0x10, diff = (uint8_t) (height << 5);
	Port->AFR[height] &= ~(uint32_t) (0x0F
			<< (uint32_t) (((uint32_t) Pin_Dpos << 1) - diff));
	Port->AFR[height] |= (uint32_t) (AF
			<< (uint32_t) (((uint32_t) Pin_Dpos << 1) - diff));
}

void Send_CS43L22_Control(I2C_TypeDef *I2C, uint8_t reg, uint8_t dat) {
	const uint32_t DATA_TRANSMITTED = (((uint32_t) (I2C_SR2_BUSY | I2C_SR2_MSL
			| I2C_SR2_TRA)) << 16) | I2C_SR1_TXE, ADDRESS_TRANSMITTED =
			DATA_TRANSMITTED | I2C_SR1_ADDR;

	//Ждем пока шина освободится
	while (I2C->SR2 & I2C_SR2_BUSY) {

	}

	//Генерируем Start condition
	I2C->CR1 |= I2C_CR1_START;
	//Ждем пока сгенерируется Start condition
	while (!(I2C->SR1 & I2C_SR1_SB)) {

	}

	//Шлем адрес 0x94, младший бит в 0 (передаем данные)
	I2C->DR = 0x94 & 0xFE;
	//Ждем отправки адреса (проверяем с помощью маски ADDRESS_TRANSMITTED)
	while ((I2C_SR(I2C) & ADDRESS_TRANSMITTED) != ADDRESS_TRANSMITTED) {
		//ShowI2C1SR();
	}

	//Отравляем адрес регистра
	I2C->DR = reg;
	//Ждем отправки данных (проверяем с помощью маски DATA_TRANSMITTED)
	while ((I2C_SR(I2C) & DATA_TRANSMITTED) != DATA_TRANSMITTED) {

	}

	//Отправляем значение регистра
	I2C->DR = dat;
	//Ждем отправки данных (проверяем с помощью маски DATA_TRANSMITTED)
	while ((I2C_SR(I2C) & DATA_TRANSMITTED) != DATA_TRANSMITTED) {

	}

	//Ждем окончания передачи данных
	while (!(I2C->SR1 & I2C_SR1_BTF)) {

	}
	//Генерируем Stop condition
	I2C->CR1 |= I2C_CR1_STOP;
}

void ConfigAudio(CS43L22_CodecTypeDef *CS43L22_CodecStruct, uint32_t frequency,
		uint16_t bits_per_sample) {

	//Выключаем I2S для конфигурации
	CS43L22_CodecStruct->SPI->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
	//Настроим частоту для I2S
	//Настраиваем предделитель
	uint32_t prescaler = (I2S_CLK >> 8) / frequency;
	if ((((prescaler << 1) + 1) * frequency) < ((I2S_CLK >> 8) << 1)) {
		++prescaler;
	}
	//Проверяем значение предделителя на крайние значения
	if (prescaler < 4) {
		prescaler = 4;
	} else {
		if (prescaler > 255) {
			prescaler = 255;
		}
	}

	CS43L22_CodecStruct->SPI->I2SPR &= ~(SPI_I2SPR_ODD | SPI_I2SPR_I2SDIV);
	CS43L22_CodecStruct->SPI->I2SPR |= (prescaler >> 1)
			| ((prescaler << 8) & SPI_I2SPR_ODD);

	//Настраиваем размер сэмпла
	CS43L22_CodecStruct->SPI->I2SCFGR &= ~SPI_I2SCFGR_DATLEN;
	//Значение по умолчанию - 16 бит
	switch (bits_per_sample) {
	case 24:
		CS43L22_CodecStruct->SPI->I2SCFGR |= SPI_I2SCFGR_DATLEN_0;
		break;
	case 32:
		CS43L22_CodecStruct->SPI->I2SCFGR |= SPI_I2SCFGR_DATLEN_1;
		break;
	}

	//Выключаем I2S
	CS43L22_CodecStruct->SPI->I2SCFGR |= SPI_I2SCFGR_I2SE;
}

void Send_CS43L22_Sample(CS43L22_CodecTypeDef *CS43L22_CodecStruct,
		uint32_t sample, uint16_t bits_per_sample) {
	//Битовая маска для отрезания лишней части сэмпла
	uint16_t sample_mask = (uint16_t) (0xFFFF >> (bits_per_sample & 0x000F));

	while (!(SPI3->SR & SPI_SR_TXE)) {

	}

	if (bits_per_sample > 16) {
		//Если размер сэмпла больше 16 бит, отправляем его в несколько заходов
		CS43L22_CodecStruct->SPI->DR = (uint16_t)(sample >> 16);
		Send_CS43L22_Sample(CS43L22_CodecStruct, sample,
				(uint16_t) (bits_per_sample - 16));
	} else {
		CS43L22_CodecStruct->SPI->DR = (uint16_t)(sample & sample_mask);
	}

}
