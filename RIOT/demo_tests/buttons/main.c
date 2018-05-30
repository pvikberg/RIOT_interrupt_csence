/*
 * Copyright (C) 2017 HAW Hamburg
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
 * @brief       Test for the on-board button macros
 *
 * @author      Sebastian Meiling <s@mlng.net>
 *
 * @}
 */

#include <stdio.h>
#include <stdint.h>

#include "board.h"
#include "periph/gpio.h"
#include "periph_conf.h"
#include "xtimer.h"
#define TEST_FLANK      GPIO_FALLING

//#ifdef BTN0_PIN /* assuming that first button is always BTN0 */

/*static void cb(void *arg)
{
    printf("Pressed BTN%d\n", (int)arg);
	LED0_TOGGLE;
}
#endif

*/



int main(void)
{
    xtimer_usleep(3000000);
  /*  int cnt = 0;
	int cnt0 = 0;
	int cnt1 = 1;
	int cnt2 = 2;
	int cnt3 = 3;
	*/
    /* get the number of available buttons and init interrupt handler */

/*
#ifdef BTN0_PIN
	
    if (gpio_init_int(BTN0_PIN, BTN0_MODE, TEST_FLANK, cb, (void *)cnt0) < 0) {
        puts("[FAILED] init BTN0!");
        return 1;
    }
	
    ++cnt;
#endif
#ifdef BTN1_PIN
    if (gpio_init_int(BTN1_PIN, BTN1_MODE, TEST_FLANK, cb, (void *)cnt1) < 0) {
        puts("[FAILED] init BTN1!");
        return 1;
    }
    ++cnt;
#endif
#ifdef BTN2_PIN
    if (gpio_init_int(BTN2_PIN, BTN2_MODE, TEST_FLANK, cb, (void *)cnt2) < 0) {
        puts("[FAILED] init BTN2!");
        return 1;
    }
    ++cnt;
#endif
#ifdef BTN3_PIN
    if (gpio_init_int(BTN3_PIN, BTN3_MODE, TEST_FLANK, cb, (void *)cnt3) < 0) {
        puts("[FAILED] init BTN3!");
        return 1;
    }
    ++cnt;
#endif

    puts("On-board button test\n");
*/

    /* cppcheck-suppress knownConditionTrueFalse
     * rationale: board-dependent ifdefs */
  /*
  if (cnt == 0) {
        puts("[FAILED] no buttons available!");
        return 2;
    }
*/
	//gpio_init_int(BTN0_PIN|BTN1_PIN, BTN0_MODE, TEST_FLANK, cb, (void *)cnt0);
	//gpio_irq_enable(BTN3_PIN);
 /*
    printf(" -- Available buttons: %i\n\n", cnt);
    puts(" -- Try pressing buttons to test.\n");
    puts("[SUCCESS]");
    return 0;
*/

while(1==1){
puts("Hello world!");
gpio_init(BTN0_PIN,BTN0_MODE);
gpio_init(BTN1_PIN,BTN1_MODE);
gpio_init(BTN2_PIN,BTN2_MODE);
gpio_init(BTN3_PIN,BTN3_MODE);
if(!gpio_read(BTN0_PIN)){puts("Nappi 0");}
if(!gpio_read(BTN1_PIN)){puts("Nappi 1");}
if(!gpio_read(BTN2_PIN)){puts("Nappi 2");}
if(!gpio_read(BTN3_PIN)){puts("Nappi 3");}
xtimer_sleep(1);}

}
