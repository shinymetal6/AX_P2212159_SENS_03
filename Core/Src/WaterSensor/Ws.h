/*
 * Ws.h
 *
 *  Created on: Jan 10, 2023
 *      Author: fil
 */

#ifndef SRC_WATERSENSOR_WS_H_
#define SRC_WATERSENSOR_WS_H_

#include <string.h>
#include <stdio.h>
#include <math.h>


extern	ADC_HandleTypeDef hadc1;
extern	ADC_HandleTypeDef hadc2;
extern	DMA_HandleTypeDef hdma_adc2;
extern	DAC_HandleTypeDef hdac1;
extern	DMA_HandleTypeDef hdma_dac1_ch1;
extern	UART_HandleTypeDef huart1;
extern	OPAMP_HandleTypeDef hopamp1;
extern	TIM_HandleTypeDef htim6;
extern	TIM_HandleTypeDef htim7;
extern	TIM_HandleTypeDef htim17;

#define	NUM_SINEVAR			256
#define	NUM_SAMPLES			4096
#define	NUM_IN_SAMPLES		(NUM_SAMPLES/4)
#define	NUM_ADC_SAMPLES		NUM_SAMPLES
#define FLOOR_SAMPLES		2048
#define	SINE_OFFSET			0x080

#define CONDUCTANCE			1

#define	UART_TXBUF_LEN		128
#define	UART_RXBUF_LEN		32

#define DEFAULT_LOOP_TIME	2
#define	DEFAULT_SCALE_FACTOR	OPAMP_PGA_GAIN_2_OR_MINUS_1
#define	DEFAULT_WAVE_AMPLITUDE	0
#define	AMPLITUDE_TABLE_LEN		14

#define ADDRESS_0			0x30
#define ADDRESS_1			0x31
#define ADDRESS_2			0x32
#define ADDRESS_3			0x33
#define ADDRESS_4			0x34
#define ADDRESS_5			0x35
#define ADDRESS_6			0x36
#define ADDRESS_7			0x37

#define HW_VERSION_H		1
#define HW_VERSION_L		1
#define SW_VERSION_H		1
#define SW_VERSION_L		3
#define SENSOR_TYPE			CONDUCTANCE
#define SENSED_INPUT		1
#define	FOOTER				0xdeadbeef

enum SENSOR_STATE
{
	IDLE,
	RUN,
	GET_RESULTS,
	TX_DATA,
	SEND_INFO,
};


enum UART_STATE
{
	WAIT_FOR_START,
	WAIT_FOR_END,
};

typedef struct {
	uint8_t 		address;
	uint8_t			sensor_type;
	uint8_t			sensor_num;
	uint8_t			hw_version_high;
	uint8_t			hw_version_low;
	uint8_t			sw_version_high;
	uint8_t			sw_version_low;
	uint8_t			unused0;
	uint8_t			unused1;
	uint8_t			unused2;
	uint8_t			unused3;
	uint8_t			unused4;
	uint32_t		footer;
} BoardParams_TypeDef;

typedef struct {
	uint8_t 		system_flags;
	uint8_t 		set_flags;
	uint8_t			tim10msec_counter;
	uint8_t			tim100msec_counter;
	uint8_t			tim100msec_events;
	uint8_t			tim1Sec_counter;
	uint8_t			uart_buf[UART_TXBUF_LEN];
	uint8_t			uart_txbuf[UART_TXBUF_LEN];
	uint8_t			uart_rxbuf[UART_RXBUF_LEN];
	uint8_t			uart_rxchar;
	uint8_t			uart_buf_index;
	uint8_t			uart_buf_rxed;
	uint16_t		sensor_dma_data[NUM_SAMPLES];
	uint16_t		sample_data[NUM_IN_SAMPLES];
	int16_t			temperature[4]; // 0 is adc-temperature , 1 is adc-vddint, 2 is temperature
	int16_t			running_temperature[4]; // 0 is adc-temperature , 1 is adc-vddint, 2 is temperature
	float			f_wave_amplitude,f_floor_noise;
	uint8_t			wave_amplitude_index;
	int32_t			peak;
	uint32_t		pga_gain;
	char			pkt_indicator;
}SystemVar_TypeDef;

/* system_flags */
#define	FLAGS_TIM10MSEC			0x01
#define	FLAGS_TIM100MSEC		0x02
#define	FLAGS_TIM1SEC			0x04
#define	FLAGS_DAC_CYCLES_END	0x08
#define	FLAGS_ADC_CONV_COMPLETE	0x10
#define	FLAGS_TEMP_COMPLETE		0x20
#define	FLAGS_PKT_VALID			0x40
#define	FLAGS_RX_CHAR			0x80

#define	OPAMP_VINM_PUPD_BIT		0x80
#define	OPAMP_VINM_PORT_BIT		0xc0
#define	OPAMP_VOUTM_PORT_BIT	0x40
#define	OPAMP_VINM_OUT_PORT_BIT	0x80
#define	DOWNSCALE				0
#define	UPSCALE					1

#define FLOOR_START		256
#define FLOOR_END		768
#define	FLOOR_NUMBER	(FLOOR_END-FLOOR_START)

extern	SystemVar_TypeDef	SystemVar;
extern	const BoardParams_TypeDef board_parameters;
extern	BoardParams_TypeDef ram_board_parameters;
extern	void SensorInit(void);
extern	void SensorLoop(void);

#include "uart.h"
#include "timers.h"
#include "result.h"

#endif /* SRC_WATERSENSOR_WS_H_ */
