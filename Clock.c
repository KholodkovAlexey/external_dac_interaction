#include "Clock.h"

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
		RCC->PLLCFGR |= (PLLN << 6) | PLLM | ((PLLP - 2) << 16);

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
	//Входной частотой является 8МГц, выходной ((input) * (RCC_PLLI2SCFGR_PLLI2SN / RCC_PLLCFGR_PLLM)) / RCC_PLLI2SCFGR_PLLI2SR)
	//PLLI2SN = 344, PLLI2SR = 2 такие значения множителей позволят передовать аудио на частоте до 96kHz
	RCC->PLLI2SCFGR |= (PLLI2SN << 6) | (PLLI2SR << 28);

	//Включаем тактирование I2S
	RCC->CR |= RCC_CR_PLLI2SON;
	//Ждем включения PLL I2S
	while (!(RCC->CR & RCC_CR_PLLI2SRDY)) {

	}
}
