/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "PORT.h"
#include "GPIO.h"
#include "FTM.h"
#include "DRV_CMP.h"
#include "Timer.h"
/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static void delayLoop(uint32_t veces);

static void Fault();
/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/



//Pines Utilizados
/*CMP PTC8 INM(THRESHOLD)
 *    PTC9 INP(SEÑAL)
 *    PTB20 COUT
 *FTM PTD1 ->H1
 *    PTD0 ->L1
 *    PTD3-> H2
 *    PTD-> L2
 *    PTC12 -> FLT(INPUT FAULT)
*/
/* Función que se llama 1 vez, al comienzo del programa */
bool i=false;
void App_Init (void)
{
	 	PORT_Init();
		//GPIO_Init();
		FTM_Init();
		Fault();
		Init_CMP(CMP0);
		timerInit();
		MODO_REPOSO();
		delayLoop(9000000L);		//poner el delay necesario.
		MODO_NORMAL();

}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	if (gpioRead(PORTNUM2PIN(PA,1)))
	{
		while (gpioRead(PORTNUM2PIN(PA,1)))
		{
			//FTM_ClearFaultF();
			MODO_REPOSO();
			delayLoop(9000000L);
		}
		MODO_NORMAL();
	}
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/
static void Fault(){ //Puente el fault del timer
		gpioMode(PORTNUM2PIN(PA,1), INPUT_PULLDOWN);
		return;
}



static void delayLoop(uint32_t veces)
{
    while (veces--);
}

/*******************************************************************************
 ******************************************************************************/


