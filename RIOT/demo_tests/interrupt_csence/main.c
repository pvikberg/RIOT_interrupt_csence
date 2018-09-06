/*
 *Some experimental code for capacitive sensing for NEPPI-course summer 2018
 *
 *The idea is copied from:
 *https://playground.arduino.cc/Main/CapacitiveSensor
 *
 *TODO: should likely be changed to communicate to other threads, currently only turns a led on if finger touches metal
 *
 *-Pyry
 */

#include <stdio.h>
#include <stdint.h>

#include "board.h"
#include "periph/gpio.h"
#include "periph_conf.h"
#include "xtimer.h"

/* pins for sensing and sending, should not be changed*/
const int sendingpin = 31;
const int sensingpin = 30;

/* these can be changed as necessary */

//currently these values work for a 3.3MOhm resistor and touching the wire metal

const int touchingthreshold = 97;

//how long we should wait between sending rising and falling edges of pulse
const int touchsensingwait = 500;

//how long to wait between measurements
const int usecbetweenmeasurements = 50000;

//how many pulses
const int pulses = 1;

//smoothing in case we get induvidual errors
const int touchsmoothing = 3;


/* callback function for interrupt when the sendingpin is high */
static void capcallback_readhigh(void *arg){

	//get current time and disable further interrupts
	xtimer_ticks32_t time = xtimer_now();
	gpio_irq_disable(GPIO_PIN(0,sensingpin));
	//set pin to "low" for next measurement
	gpio_init(GPIO_PIN(0,sensingpin),GPIO_OUT);
	gpio_set(GPIO_PIN(0,sensingpin));
	//"return" the value via this
	*(int*)arg = (int)time.ticks32;
}


/* callback function for interrupt when the sendingpin is low */
static void capcallback_readlow(void *arg){
	
	//get current time and disable further interrupts
	xtimer_ticks32_t time = xtimer_now();
	gpio_irq_disable(GPIO_PIN(0,sensingpin));
	//set pin to "low" for next measurement
	gpio_init(GPIO_PIN(0,sensingpin),GPIO_OUT);
	gpio_clear(GPIO_PIN(0,sensingpin));
	//"return" the value via this
	*(int*)arg = (int)time.ticks32;
}



int main(void)
{

	//initialize some values, TODO: perhaps can be moved outside main loop...
	int measurementclock = 0;
	int measurementsum = 0;
	int touched = 0;
	
	//initialize led pin
	gpio_init(16,GPIO_OUT);


	//get ground on pin 2
	gpio_init(2,GPIO_OUT);
	gpio_clear(2);

	// Do the below part eternally, probably should be modified to only be done when needed
	while(1==1){
		//initialize pins, set both to low
		gpio_init(GPIO_PIN(0,sensingpin),GPIO_OUT);
		gpio_init(GPIO_PIN(0,sendingpin),GPIO_OUT);
		gpio_clear(GPIO_PIN(0,sensingpin));
		gpio_clear(GPIO_PIN(0,sendingpin));
		measurementsum = 0;
		
		//pulse number can changed above
		for(int i=0;i<pulses;i++){


		//send "rising" part of a pulse

			//initialize sensing pin as interrupt source
			gpio_init_int(GPIO_PIN(0, sensingpin),GPIO_IN,GPIO_RISING, capcallback_readhigh,&measurementclock);
			gpio_irq_enable(GPIO_PIN(0,sensingpin));
			
			// in case we don't recieve pulse/pulse slower than touchsensingwait
			measurementclock = 10000;
			
			//start sending the pulse, set timer to zero and wait		
			gpio_set(GPIO_PIN(0,sendingpin));
			xtimer_init();
			xtimer_usleep(touchsensingwait);

			//disable interrupts from sensing pin just in case
			gpio_irq_disable(GPIO_PIN(0,sensingpin));
			
			//keeping track how long we have waited so far for the pulses
			measurementsum += measurementclock;

		//send "falling" part of a pulse

			//initialize sensing pin as interrupt source
			gpio_init_int(GPIO_PIN(0, sensingpin),GPIO_IN,GPIO_FALLING, capcallback_readlow,&measurementclock);
			gpio_irq_enable(GPIO_PIN(0,sensingpin));
			
			// in case we don't recieve pulse/pulse slower than touchsensingwait
			measurementclock = 10000;
			
			//start sending the pulse, set timer to zero and wait		
			gpio_clear(GPIO_PIN(0,sendingpin));
			xtimer_init();
			xtimer_usleep(touchsensingwait);

			//disable interrupts from sensing pin just in case
			gpio_irq_disable(GPIO_PIN(0,sensingpin));
			
			//keeping track how long we have waited so far for the pulses
			measurementsum += measurementclock;
		}
		// all pulses sent and time waited measured at this point

		//wait until time to send next set of pulses, also print the time for debugging
		xtimer_usleep(usecbetweenmeasurements);
		printf("Sum: %d \n", measurementsum);

		//smooth the touched times in case we get a few erroneous measurements
		if((measurementsum > touchingthreshold) && (touched < touchsmoothing)){
			touched++;
		}
		else if((measurementsum < touchingthreshold) && (touched > 0)){
			touched--;
        	} 
		//led on if finger touching metal, TODO: should likely be changed to communicate to other threads
		if(touched > 0)
			gpio_set(16);//LED0_ON;
		else
			gpio_clear(16);//LED0_OFF;		
		
	}

}
