#include "delay.h"

void delay_init(void){
	if(!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)){
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
}

uint32_t DWT_Get(void){
	return DWT->CYCCNT;
}

__inline
uint8_t DWT_Compare(uint32_t tp){
	return ((DWT_Get() - tp) < 0);
}

void delay_us(uint32_t us){
	uint32_t end = DWT_Get() + us * (SystemCoreClock/1000000);
	while(DWT_Compare(end));
}
