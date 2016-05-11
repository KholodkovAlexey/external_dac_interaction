#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "stm32f4xx.h"

#include "Clock.h"
#include "Wav.h"
#include "CS43L22.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#define FILE_OFFSET 0x800C000

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

	if (ReadWav(&wav, FILE_OFFSET) == WAV_ERROR_NO_ERROR) {
		GPIOD->BSRR = GPIO_BSRR_BS_13;

		uint32_t frequency = wav.ftm->SampleRate;
		uint16_t bits_per_sample =
				(uint16_t) (wav.ftm->BlockAlign_SignificantBitsPerSample >> 16),
				bytes_per_sample = bits_per_sample >> 3, number_of_channels =
						(uint16_t) (wav.ftm->CompressionCode_NumberOfChannels
								>> 16),
				block_align =
						(uint16_t) (wav.ftm->BlockAlign_SignificantBitsPerSample);

		//Включаем тактирование I2C и I2S
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_SPI3EN;
		CS43L22_CodecTypeDef cs43l22;
		cs43l22.I2C = I2C1;
		cs43l22.SPI = SPI3;

		CS43L22_InitCodec(&cs43l22);
		ConfigAudio(&cs43l22, frequency, bits_per_sample);

		uint32_t i, data_offset, duration = wav.data->ChunkDataSize
				/ block_align;

		//Адрес с которого начинаем считывать сэмплы
		data_offset = ((uint32_t) wav.data) + 8;

		//uint32_t data_buff[2], data_buffer_offset = 0;
		//data_buff[0] = *(__IO uint32_t*) (data_offset);

		for (i = 0; i < duration; ++i) {
			uint32_t sample = *(__IO uint32_t*) (data_offset);
			Send_CS43L22_Sample(&cs43l22, sample, bits_per_sample);
			if (number_of_channels > 1) {
				sample = *(__IO uint32_t*) (data_offset + bytes_per_sample);
			}
			Send_CS43L22_Sample(&cs43l22, sample, bits_per_sample);

			data_offset += block_align;
		}

		//Закончили воспроизведение - зажигаем синий диод
		GPIOD->BSRR = GPIO_BSRR_BS_15;

	} else {
		//Ошибочный файл - зажигаем красный диод
		GPIOD->BSRR = GPIO_BSRR_BS_14;
	}

	while (1) {

	}

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
