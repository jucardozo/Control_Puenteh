/*
 * main.c
 *
 *  Created on: May 1, 2015--
 *      Author: Juan Pablo VEGA - Laboratorio de Microprocesadores
 */
/*
#include "hardware.h"
#include "PORT.h"
#include "GPIO.h"
#include "FTM.h"


#define __FOREVER__ 	for(;;)



int main (void)
{


 	 	 	 	hw_Init ();
 	 	 	 	PORT_Init();
 	 	 		GPIO_Init();
 	 	 		FTM_Init();


 	 	// 		hw_DisableInterrupts();

 	 	 		__FOREVER__;

			// Enable interrupts
			//hw_EnableInterrupts();






}*/

#include "hardware.h"

void App_Init (void);
void App_Run (void);


int main (void)
{
    hw_Init();
    hw_DisableInterrupts();
    App_Init(); /* Program-specific setup */
    hw_EnableInterrupts();

    __FOREVER__
        App_Run(); /* Program-specific loop  */
}
