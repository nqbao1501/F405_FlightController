/*
 * dwt.c
 *
 *  Created on: 10 Dec 2025
 *      Author: Admin
 */
#include "timer_logging.h"

extern volatile uint32_t timer_overflow;
extern TIM_HandleTypeDef htim4;
uint32_t micros(void) {
    uint32_t overflow;
    uint32_t cnt;

    __disable_irq();
    overflow = timer_overflow;
    cnt = __HAL_TIM_GET_COUNTER(&htim4);
    // If CNT overflowed while reading, increment overflow
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE)) {
        overflow++;
        cnt = __HAL_TIM_GET_COUNTER(&htim4);
    }
    __enable_irq();

    return (overflow << 16) | cnt;
}

