/*
 * Ws.c
 *
 *  Created on: Jan 10, 2023
 *      Author: fil
 */

#include "main.h"


SystemVar_TypeDef	SystemVar;
enum 	SENSOR_STATE	sensor_state;

__attribute__ ((aligned (64))) BoardParams_TypeDef ram_board_parameters=
{
		ADDRESS_0,			//	address
		CONDUCTANCE,		//	sensor_type
		SENSED_INPUT,		//	sensor_num
		HW_VERSION_H,		//	hw_version_high
		HW_VERSION_L,		//	hw_version_low
		SW_VERSION_H,		//	sw_version_high
		SW_VERSION_L,		//	sw_version_low
		0,
		0,
		0,
		0,
		0,
		FOOTER,				// 	footer to be aligned at 3 uint64_t
};



/*
static void go_digital(void)
{
	return;
}

static void go_analog(void)
{
	return;
}
*/
extern	const float amplitude_table[AMPLITUDE_TABLE_LEN];

void SensorInit(void)
{
	HAL_TIM_Base_Start_IT(&htim6);
	TIM6->CR1 &= 0xfffffffe;		// stop adc-dac timer
	HAL_TIM_Base_Start(&htim7);		// temperature timer
    HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
    SystemVar.wave_amplitude_index = 0;
    set_wave_amplitude(SystemVar.wave_amplitude_index);
	SystemVar.f_wave_amplitude = amplitude_table[SystemVar.wave_amplitude_index];
    HAL_OPAMP_SelfCalibrate(&hopamp1);
	HAL_Delay(10);
	HAL_OPAMP_Start(&hopamp1);
	HAL_Delay(10);
	set_opamp_gain(0);
	sensor_state = IDLE;

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)SystemVar.running_temperature, 2);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)SystemVar.sensor_dma_data, NUM_SAMPLES);
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)sine_tab, NUM_SAMPLES,DAC_ALIGN_12B_R);
	HAL_TIM_Base_Start_IT(&htim17);
	HAL_GPIO_WritePin(USART1_RE_GPIO_Port, USART1_RE_Pin, GPIO_PIN_RESET);
	HAL_UART_Receive_IT(&huart1, &SystemVar.uart_rxchar, 1 );
}

static uint8_t idle_check(void)
{
uint8_t	local_sensor_state = IDLE;

	if (( SystemVar.system_flags & FLAGS_RX_CHAR) == FLAGS_RX_CHAR)
	{
		SystemVar.system_flags &= ~FLAGS_RX_CHAR;
		uart_rxpacket_analyze();
	}
	else
	{
		HAL_GPIO_WritePin(USART1_RE_GPIO_Port, USART1_RE_Pin, GPIO_PIN_RESET);
		HAL_UART_Receive_IT(&huart1, &SystemVar.uart_rxchar, 1 );
	}

	if (( SystemVar.system_flags & FLAGS_PKT_VALID) == FLAGS_PKT_VALID)
	{
		SystemVar.system_flags &= ~FLAGS_PKT_VALID;
		HAL_GPIO_WritePin(USART1_RE_GPIO_Port, USART1_RE_Pin, GPIO_PIN_SET);
		if (( SystemVar.uart_rxbuf[1] == 'G') || ( SystemVar.uart_rxbuf[1] == 'g'))	//	go command
		{
			TIM6->CNT = 0;
			TIM6->CR1 |= 0x01;
			SystemVar.system_flags &= ~FLAGS_TEMP_COMPLETE;
			local_sensor_state = RUN;
		}
		if (( SystemVar.uart_rxbuf[1] == 'I') || ( SystemVar.uart_rxbuf[1] == 'i'))	// info command
		{
			local_sensor_state = SEND_INFO;
		}
	}
	return local_sensor_state;
}

static uint8_t run_check(void)
{
uint8_t	local_sensor_state = RUN;
	if(( SystemVar.system_flags & FLAGS_ADC_CONV_COMPLETE) == FLAGS_ADC_CONV_COMPLETE)
	{
		SystemVar.system_flags &= ~FLAGS_ADC_CONV_COMPLETE;
		local_sensor_state = GET_RESULTS;
	}
	return local_sensor_state;
}

void send_info(void)
{
	sprintf((char *)SystemVar.uart_buf,"<I %c %d.%d %d.%d %d %d>\r\n",( char )(ram_board_parameters.address),\
			(int )HW_VERSION_H,(int )HW_VERSION_L,(int )SW_VERSION_H,(int )SW_VERSION_L,\
			(int )SENSOR_TYPE,(int )SENSED_INPUT);
	HAL_Delay(10);
	tx_uart();
}

void SensorLoop(void)
{
	timers();
	switch ( sensor_state )
	{
	case	IDLE :
		sensor_state = idle_check();
    	break;
	case	RUN :
		sensor_state = run_check();
    	break;
	case	GET_RESULTS :
		get_result();
		sensor_state = TX_DATA;
    	break;
	case	TX_DATA :
		send_results();
		sensor_state = IDLE;
		break;
	case	SEND_INFO :
    	send_info();
    	sensor_state = IDLE;
	default:
		sensor_state = IDLE;
    	break;
	}
}
