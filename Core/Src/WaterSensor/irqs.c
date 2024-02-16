/*
 * irqs.c
 *
 *  Created on: Jan 11, 2023
 *      Author: fil
 */

#include "main.h"

uint16_t	tim_cntr = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	SystemVar.system_flags |= FLAGS_RX_CHAR;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if ( htim == &htim17)
		SystemVar.system_flags |= FLAGS_TIM10MSEC;
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	SystemVar.system_flags |= FLAGS_DAC_CYCLES_END;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if ( hadc == &hadc1 )
	{
		SystemVar.system_flags |= FLAGS_TEMP_COMPLETE;
	}
	if ( hadc == &hadc2 )
	{
		TIM6->CR1 &= 0xfffffffe;
		SystemVar.system_flags |= FLAGS_ADC_CONV_COMPLETE;
	}
}

