#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "stm32f4xx.h"
#include "Wav.h"
#include "CS43L22.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#define FILE_OFFSET 0x800C000

void Set_PLL_HSE_Clock(void);
void Set_PLL_I2S_HSE_Clock(void);

void Set_PLL_HSE_Clock(void) {
	//Настройка генератоа частоты от внешнего кварца

	//Включили внешний источник частоты
	RCC->CR |= RCC_CR_HSEON;

	uint32_t startup_counter = 0, hse_status;
	do {
		hse_status = RCC->CR & RCC_CR_HSERDY;
		++startup_counter;
	} while ((!hse_status) && (startup_counter != HSE_STARTUP_TIMEOUT));

	if (hse_status) {

		//Изменяем латентность флеша
		FLASH->ACR &= ~FLASH_ACR_LATENCY;
		FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

		//Задаем источник для PLL
		RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;
		RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

		//Отчищаем значиния множителей из регистра PLLCFGR
		RCC->PLLCFGR &=
				~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP);
		//Выстанавливаем множители
		//С PLLP ничего не делаем - нули дают значение делителя 2
		//В PLLN записываем 336
		//В PLLM записываем 8
		RCC->PLLCFGR |= (336 << 6) | 8;

		//Отчищаем значиния множителей из регистра CFGR
		RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
		//Выставляем множители
		RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4
				| RCC_CFGR_PPRE2_DIV2;

		//Включаем PLL
		RCC->CR |= RCC_CR_PLLON;
		//Ждем готовности PLL
		while (!(RCC->CR & RCC_CR_PLLRDY)) {

		}
		//Отчищаем SW (System clock switch) в регистре CFGR
		RCC->CFGR &= ~RCC_CFGR_SW;
		//Выставляем PLL в качестве источника частоты
		RCC->CFGR |= RCC_CFGR_SW_PLL;
		//Ждем пока статус не переидет в PLL
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {

		}
	}

}

void Set_PLL_I2S_HSE_Clock(void) {
	//Выключаем тактирование I2S
	RCC->CR &= ~RCC_CR_PLLI2SON;

	//Задаем PLLI2S источником тактирования I2S
	RCC->CFGR &= ~RCC_CFGR_I2SSRC;
	RCC->PLLI2SCFGR &= ~(RCC_PLLI2SCFGR_PLLI2SN | RCC_PLLI2SCFGR_PLLI2SR);
	//Входной частотой является 8МГц, выходной ((input) * (RCC_PLLI2SCFGR_PLLI2SN / RCC_PLLCFGR_PLLM)) / RCC_PLLI2SCFGR_PLLI2SR
	//PLLI2SN = 344, PLLI2SR = 2 такие значения множителей позволят передовать аудио на частоте до 96kHz
	RCC->PLLI2SCFGR |= (344 << 6) | (2 << 28);

	//Включаем тактирование I2S
	RCC->CR |= RCC_CR_PLLI2SON;
	//Ждем включения PLL I2S
	while (!(RCC->CR & RCC_CR_PLLI2SRDY)) {

	}
}

int main(int argc, char* argv[]) {

	Set_PLL_HSE_Clock();
	Set_PLL_I2S_HSE_Clock();

	//Включаем тактирование портов A, B, C, D
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
			| RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;

	CS43L22_InitGPIO(GPIOD, GPIO_PIN_4, GPIOB, GPIO_PIN_6 | GPIO_PIN_9, GPIOA,
	GPIO_PIN_4, GPIOC, GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_12, GPIO_AF4_I2C1,
	GPIO_AF6_SPI3);

	//Диоды для discovery
	GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0
			| GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;
	GPIOD->BSRR = GPIO_BSRR_BS_12;

	WavTypeDef wav;

	if (ReadWav(&wav, FILE_OFFSET) == NO_ERROR) {
		GPIOD->BSRR = GPIO_BSRR_BS_13;

		uint32_t frequency = wav.ftm->SampleRate;
		uint16_t bits_per_sample =
				(uint16_t) (wav.ftm->BlockAlign_SignificantBitsPerSample >> 16),
				number_of_channels =
						(uint16_t) (wav.ftm->CopressionCode_NumberOfChannels
								>> 16);

		//Включаем тактирование I2C и I2S
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_SPI3EN;
		CS43L22_CodecTypeDef cs43l22;
		cs43l22.I2C = I2C1;
		cs43l22.SPI = SPI3;

		CS43L22_InitCodec(&cs43l22);
		ConfigAudio(&cs43l22, frequency, bits_per_sample);

		uint32_t i, all_channels_sample_size =
				(uint32_t) ((bits_per_sample >> 3) * number_of_channels),
				duaration = wav.data->ChunkDataSize / all_channels_sample_size,
				data_offset;

		//Адрес с которого начинаем считывать сэмплы
		data_offset = (uint32_t) (wav.data) + 8;

		for (i = 0; i < duaration; ++i) {
			uint16_t sample = *(__IO uint16_t*) (data_offset);
			Send_CS43L22_Sample(&cs43l22, sample, bits_per_sample);
			if (number_of_channels > 1) {
				sample =
						*(__IO uint16_t*) (data_offset + (bits_per_sample >> 3));
			}
			Send_CS43L22_Sample(&cs43l22, sample, bits_per_sample);
			data_offset += all_channels_sample_size;
		}

		//Закончили восроизведение - зажигаем синий диод
		//GPIOD->BSRR = GPIO_BSRR_BS_15;

	} else {
		//Ошибочный файл - зажигаем красный диод
		//GPIOD->BSRR = GPIO_BSRR_BS_14;
	}

	while (1) {

	}

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
