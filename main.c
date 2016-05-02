//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "stm32f4xx.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F4 empty sample (trace via ITM).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the ITM output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

const uint8_t cs43l22_init_data[] = {0x00, 0x99,
									0x47, 0x80,
									0x32, 0xBB,
									0x32, 0x3B,
									0x00, 0x00,
									0x05, 0x81,
									0x06, 0x07,
									0x02, 0x9E};

void Set_PLL_HSE_Clock(void);
void Set_PLL_I2S_HSE_Clock(uint32_t frequency);
void Send_CS43L22_Control(uint8_t reg, uint8_t dat);
void Init_CS43L22(void);

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

void Set_PLL_I2S_HSE_Clock(uint32_t frequency) {
	//Выключаем тактирование I2S
	RCC->CR &= ~RCC_CR_PLLI2SON;

	//Задаем PLLI2S источником тактирования I2S
	RCC->CFGR &= ~RCC_CFGR_I2SSRC;
	RCC->PLLI2SCFGR &= ~(RCC_PLLI2SCFGR_PLLI2SN | RCC_PLLI2SCFGR_PLLI2SR);
	//Входной частотой является 8МГц, выходной ((input) * (RCC_PLLI2SCFGR_PLLI2SN / RCC_PLLCFGR_PLLM)) / RCC_PLLI2SCFGR_PLLI2SR
	// (8.000.000 * (frequency / 500.000 / 8)) / 2 = frequency
	RCC->PLLI2SCFGR |= ((frequency / 500000) << 6) | RCC_PLLI2SCFGR_PLLI2SR_1;

	//Включаем тактирование I2S
	RCC->CR |= RCC_CR_PLLI2SON;
	//Ждем включения PLL I2S
	while (RCC_CR_PLLI2SRDY) {

	}
}

void Send_CS43L22_Control(uint8_t reg, uint8_t dat) {
	const uint32_t DATA_TRANSMITTED = (((uint32_t) (I2C_SR2_BUSY | I2C_SR2_MSL
			| I2C_SR2_TRA)) << 16) | I2C_SR1_TXE, ADDRESS_TRANSMITTED =
			DATA_TRANSMITTED | I2C_SR1_ADDR;

	//Ждем пока шина освободится
	while (I2C1->SR2 & I2C_SR2_BUSY) {

	}

	//Генерируем Start condition
	I2C1->CR1 |= I2C_CR1_START;
	//Ждем пока сгенерируется Start condition
	while (!(I2C1->SR1 & I2C_SR1_SB)) {

	}

	//Шлем адрес 0x94, младший бит в 0 (передаем данные)
	I2C1->DR = 0x94 & (~(0x01));
	//Ждем отправки адреса (проверяем с помощью маски ADDRESS_TRANSMITTED)
	while (((((uint32_t) (I2C1->SR2) << 16) | I2C1->SR1) & ADDRESS_TRANSMITTED)
			!= ADDRESS_TRANSMITTED) {

	}

	//Отравляем адрес регистра
	I2C1->DR = reg;
	//Ждем отправки данных (проверяем с помощью маски DATA_TRANSMITTED)
	while (((((uint32_t) (I2C1->SR2) << 16) | I2C1->SR1) & DATA_TRANSMITTED)
			!= DATA_TRANSMITTED) {

	}

	//Отправляем значение регистра
	I2C1->DR = dat;
	//Ждем отправки данных (проверяем с помощью маски DATA_TRANSMITTED)
	while (((((uint32_t) (I2C1->SR2) << 16) | I2C1->SR1) & DATA_TRANSMITTED)
			!= DATA_TRANSMITTED) {

	}

	//Ждем окончания передачи данных
	while (!(I2C1->SR1 & I2C_SR1_BTF)) {

	}
	//Генерируем Stop condition
	I2C1->CR1 |= I2C_CR1_STOP;
}

void Init_CS43L22(void) {
	uint32_t i;

	//Поднимаем reset для cs43l22
	GPIOD->BSRR = GPIO_BSRR_BS_4;

	//Заполняем регистры cs43l22 даннными
	for (i = 0; i < sizeof(cs43l22_init_data); i += 2) {
		Send_CS43L22_Control(cs43l22_init_data[i], cs43l22_init_data[i + 1]);
	}
}

int main(int argc, char* argv[]) {
	// At this stage the system clock should have already been configured
	// at high speed.

	Set_PLL_HSE_Clock();

	Set_PLL_I2S_HSE_Clock(24000);

	//Включаем тактирование портов A, B, C, D
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
			| RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;

	//Включаем тактирование I2C и I2S
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_SPI3EN;

	//Reset сигнал для CS43L22
	//Устанавливаем 4 пин для порта D
	//Режим General purpose output mode
	GPIOD->MODER &= ~GPIO_MODER_MODER4;
	GPIOD->MODER |= GPIO_MODER_MODER4_0;
	//Тип вывода push-pull
	GPIOD->OTYPER &= ~GPIO_OTYPER_OT_4;
	//Режим Pull-down
	GPIOD->PUPDR &= ~GPIO_PUPDR_PUPDR4;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPDR4_1;
	//Скорость вывода hight (50MHz)
	GPIOD->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR4;
	GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_1;

	//Пины для I2C
	//Устанавливаем 6 и 9 пины для порта B
	//Режим Alternate function
	GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER9);
	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER9_1;
	//Тип вывода open-drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_9;
	//Режим No pull-up, pull-down
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR9);
	//Скорость вывода hight (50MHz)
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR9);
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR9_1;
	//Устанавливаем alternative function AF4(I2C1..3)
	GPIOB->AFR[0] &= ~(((uint32_t) 0x0000000f) << 24);
	GPIOB->AFR[0] |= GPIO_AF4_I2C1 << 24;
	GPIOB->AFR[1] &= ~(((uint32_t) 0x0000000f) << 4);
	GPIOB->AFR[1] |= GPIO_AF4_I2C1 << 4;

	//Пины для I2S

	//Пин 4 для порта A
	//Режим Alternate function
	GPIOA->MODER &= ~GPIO_MODER_MODER4;
	GPIOA->MODER |= GPIO_MODER_MODER4_1;
	//Тип вывода push-pull
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_4;
	//Режим No pull-up, pull-down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4;
	//Скорость вывода hight (50MHz)
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR4;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_1;
	//Устанавливаем alternative function AF6(SPI3)
	GPIOA->AFR[0] &= ~(((uint32_t) 0x0000000f) << 16);
	GPIOA->AFR[0] |= GPIO_AF6_SPI3 << 16;

	//Пины 7, 10, 12 для порта C
	//Режим Alternate function
	GPIOC->MODER &= ~(GPIO_MODER_MODER7 | GPIO_MODER_MODER10
			| GPIO_MODER_MODER12);
	GPIOC->MODER |= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER10_1
			| GPIO_MODER_MODER12_1;
	//Тип вывода push-pull
	GPIOC->OTYPER &=
			~(GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_12);
	//Режим No pull-up, pull-down
	GPIOC->PUPDR &=
			~(GPIO_PUPDR_PUPDR7 | GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_12);
	//Скорость вывода hight (50MHz)
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR10
			| GPIO_OSPEEDER_OSPEEDR12);
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR10_1
			| GPIO_OSPEEDER_OSPEEDR12_1;
	//Устанавливаем alternative function AF6(SPI3)
	GPIOC->AFR[0] &= ~(((uint32_t) 0x0000000f) << 28);
	GPIOC->AFR[0] |= GPIO_AF6_SPI3 << 28;
	GPIOC->AFR[1] &= ~((((uint32_t) 0x0000000f) << 8)
			| (((uint32_t) 0x0000000f) << 16));
	GPIOC->AFR[1] |= (GPIO_AF6_SPI3 << 8) | (GPIO_AF6_SPI3 << 16);

	//Настраиваем I2C
	//Выключаем I2C
	I2C1->CR1 &= ~(I2C_CR1_PE);
	//Записываем частоту переферии APB1 (42MHz)
	I2C1->CR2 = (uint16_t) (42);
	//частота_APB1 / (частота_I2C << 1)
	I2C1->CCR = (uint16_t) (210);
	//частота_APB1 + 1
	I2C1->TRISE = (uint16_t) (43);
	//Включаем I2C, ставим бит подтверждения
	I2C1->CR1 |= I2C_CR1_PE | I2C_CR1_ACK;
	//Поднимаем 14 бит, выставляем собственный адрес 0x33
	I2C1->OAR1 |= (((uint16_t) 0x0001) << 14) | ((uint16_t) 0x0033);

	// Infinite loop
	while (1) {
		// Add your code here.

	}

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
