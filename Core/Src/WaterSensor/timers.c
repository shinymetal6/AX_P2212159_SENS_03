/*
 * timers.c
 *
 *  Created on: Jan 10, 2023
 *      Author: fil
 */

#include "main.h"

void timers(void)
{
	if (( SystemVar.system_flags &FLAGS_TIM10MSEC) == FLAGS_TIM10MSEC)
	{
		SystemVar.system_flags &= ~FLAGS_TIM10MSEC;
		SystemVar.tim10msec_counter++;
		if ( SystemVar.tim10msec_counter > 9 )
		{
			SystemVar.tim10msec_counter = 0;
			SystemVar.system_flags |= FLAGS_TIM100MSEC;
			SystemVar.tim100msec_counter++;
			if ( SystemVar.tim100msec_counter > 9 )
			{
				SystemVar.tim100msec_counter = 0;
				SystemVar.system_flags |= FLAGS_TIM1SEC;
			}
		}
	}
}
