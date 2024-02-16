/*
 * result.h
 *
 *  Created on: Jan 10, 2023
 *      Author: fil
 */

#ifndef SRC_WATERSENSOR_RESULT_H_
#define SRC_WATERSENSOR_RESULT_H_

extern	void set_opamp_gain(uint32_t gain);
extern	void apply_scaling(uint8_t scale_direction);

extern	void get_result(void);
extern	void send_results(void);
extern	void scale_down_sine( void );
extern	void set_wave_amplitude(uint8_t amplitude_index);

extern	uint16_t	sine_tab[NUM_SAMPLES];

#endif /* SRC_WATERSENSOR_RESULT_H_ */
