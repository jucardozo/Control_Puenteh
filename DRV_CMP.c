/*
 * DRV_CMP.c
 *
 *  Created on: 27 may. 2023
 *      Author: Salta
 */
#include "DRV_CMP.h"

void Init_CMP(CMP_t cmp){

	SIM->SCGC4 |= SIM_SCGC4_CMP_MASK ;

	NVIC_EnableIRQ(CMP0_IRQn);
	NVIC_EnableIRQ(CMP1_IRQn);
	NVIC_EnableIRQ(CMP2_IRQn);


	// CONFIGURACION DE LOS PINOUTS
	//PTC 9 => input Plus
	PCRstr UserPCRIP;

	UserPCRIP.PCR=false;

	UserPCRIP.FIELD.DSE=true;
	UserPCRIP.FIELD.MUX=PORT_mAnalog;
	UserPCRIP.FIELD.IRQC=PORT_eDisabled;
	PORT_Configure2 (PORTC,9,UserPCRIP);
	//PTC 8 =>Input Minus => referencia
	PCRstr UserPCRIM;

	UserPCRIM.PCR=false;

	UserPCRIM.FIELD.DSE=true;
	UserPCRIM.FIELD.MUX=PORT_mAnalog;
	UserPCRIM.FIELD.IRQC=PORT_eDisabled;
	PORT_Configure2 (PORTC,8,UserPCRIM);

	//PTB 20 => Cout
	PCRstr UserPCRCout;
	UserPCRCout.PCR=false;

	UserPCRCout.FIELD.DSE=true;
	UserPCRCout.FIELD.MUX=PORT_mAlt6;
	UserPCRCout.FIELD.IRQC=PORT_eDisabled;
	PORT_Configure2 (PORTB,20,UserPCRCout);


	// MODO DE FUNCIONAMIENTO => Continous Mode CR0=> 0x00 , FPR=> 0x00, we=0,se=0
	//dejo COS = 0.
	cmp->CR1|=CMP_CR1_EN(1);  // dejo el se y we en cero.
	cmp->CR1|=CMP_CR1_OPE(1); // creo q habilito pa tengo un pin de salida.
	cmp->CR1|=CMP_CR1_INV(1);
	//MUX
	cmp->MUXCR|=CMP_MUXCR_PSTM(0);
	cmp->MUXCR|=CMP_MUXCR_PSEL(CMP_IN2);
	cmp->MUXCR|=CMP_MUXCR_MSEL(CMP_IN3);
	return;
}
