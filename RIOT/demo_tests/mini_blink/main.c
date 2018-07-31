/*
 * Copyright (C) 2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test for the on-board LED macros
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdint.h>

#include "board.h"
#include "periph_conf.h"
#include "periph/gpio.h"


#ifdef CLOCK_CORECLOCK
#define DELAY_SHORT         (CLOCK_CORECLOCK / 50)
#else
#define DELAY_SHORT         (500000UL)
#endif
#define DELAY_LONG          (DELAY_SHORT * 4)

void dumb_delay(uint32_t delay)
{
    for (uint32_t i = 0; i < delay; i++) {
        __asm__("nop");
    }
}

int main(void)
{
    


    while (1) {


//(NRF_P0)->OUTCLR = (1 << 5);

gpio_init(4,GPIO_OUT);
gpio_init(5,GPIO_OUT);
gpio_init(11,GPIO_OUT);
gpio_set(11);



	puts("ledi nolla");
gpio_set(4);
        dumb_delay(DELAY_LONG);
gpio_clear(4);
        dumb_delay(DELAY_LONG);
gpio_toggle(4);
        dumb_delay(DELAY_SHORT);
gpio_toggle(4);
        dumb_delay(DELAY_SHORT);
gpio_toggle(4);
        dumb_delay(DELAY_SHORT);
gpio_toggle(4);
        dumb_delay(DELAY_LONG);

	puts("ledi yksi");
gpio_set(5);
        dumb_delay(DELAY_LONG);
gpio_clear(5);
        dumb_delay(DELAY_LONG);
gpio_toggle(5);
        dumb_delay(DELAY_SHORT);
gpio_toggle(5);
        dumb_delay(DELAY_SHORT);
gpio_toggle(5);
        dumb_delay(DELAY_SHORT);
gpio_toggle(5);
        dumb_delay(DELAY_LONG);

    }

    return 0;
}
