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

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static void delayLoop(uint32_t veces);
//void callback (void);

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
void App_Init (void)
{
	 	PORT_Init();
		GPIO_Init();
		FTM_Init();
		Init_CMP(CMP0);
		//delayLoop(4000000UL);
		FTM_StartClock(FTM3);


}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	//FTM_start(0);
	//void FTM_stop(uint8_t FTM);
//    delayLoop(4000000UL);
  //  FTM_StartClock(FTM3);
    //gpioToggle(PORTNUM2PIN(PD,1));
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static void delayLoop(uint32_t veces)
{
    while (veces--);
}
/*
void callback (void){
	//gpioWrite (PORTNUM2PIN(PD,1), true);
	gpioToggle(PORTNUM2PIN(PD,1));
	return;
}*/


/*******************************************************************************
 ******************************************************************************/


