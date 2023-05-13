
#include "FTM.h"
#include "PORT.h"
#include "GPIO.h"
#include "FTM.h"



void PWM_Init(void);
void PWM_ISR3(void);
void PWM_ISR0(void);

uint16_t PWM_modulus = 147-1;//10000-1;
uint16_t PWM_duty    = 74;//5000-1;

/* FTM0 fault, overflow and channels interrupt handler*/
__ISR__ FTM3_IRQHandler(void)
{
	PWM_ISR3();
}
__ISR__ FTM0_IRQHandler(void)
{
	PWM_ISR0();
}

void PWM_ISR3 (void)
{
	FTM_ClearOverflowFlag (FTM3);
	FTM_StopClock(FTM3);
	FTM_SetCounter(FTM3, 0, PWM_duty);  //change DC
	FTM_SetCounter(FTM3, 1, PWM_duty);
	FTM_StartClock(FTM0);

	GPIO_Toggle(PTC, 1 << 8);			  //GPIO pin PTC8

	PWM_duty %= PWM_modulus;

}

void PWM_ISR0 (void)
{
	FTM_ClearOverflowFlag (FTM0);
	FTM_StopClock(FTM0);
	FTM_SetCounter(FTM0, 6, PWM_duty);  //change DC
	FTM_SetCounter(FTM0, 7, PWM_duty);
	FTM_StartClock(FTM3);

	GPIO_Toggle(PTC, 1 << 8);			  //GPIO pin PTC8

	PWM_duty %= PWM_modulus;
}


void FTM_Init (void)
{
	SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;
	SIM->SCGC6 |= SIM_SCGC6_FTM1_MASK;
	SIM->SCGC6 |= SIM_SCGC6_FTM2_MASK;
	SIM->SCGC3 |= SIM_SCGC3_FTM2_MASK;
	SIM->SCGC3 |= SIM_SCGC3_FTM3_MASK;

	NVIC_EnableIRQ(FTM0_IRQn);
	NVIC_EnableIRQ(FTM1_IRQn);
	NVIC_EnableIRQ(FTM2_IRQn);
	NVIC_EnableIRQ(FTM3_IRQn);

	FTM0->PWMLOAD = FTM_PWMLOAD_LDOK_MASK | 0x0F;
	FTM1->PWMLOAD = FTM_PWMLOAD_LDOK_MASK | 0x0F;
	FTM2->PWMLOAD = FTM_PWMLOAD_LDOK_MASK | 0x0F;
	FTM3->PWMLOAD = FTM_PWMLOAD_LDOK_MASK | 0x0F;

	/*
	 * TO DO
	 */

	PWM_Init();
}

/// FTM PWM Example

// To Test Connect PC9(IC)-PC8(GPIO)
// or PC9(IC)-PC1(OC)

void PWM_Init (void)
{

	// PTD 0 as PWM channel 0
		PCRstr UserPCR;

		UserPCR.PCR=false;			// Default All false, Set only those needed

		UserPCR.FIELD.DSE=true;
		UserPCR.FIELD.MUX=PORT_mAlt4;//PORT_mAlt3;
		UserPCR.FIELD.IRQC=PORT_eDisabled;

		PORT_Configure2 (PORTD,0,UserPCR);
	// PTD 1 as PWM Channel 1
		PCRstr UserPCR1;

		UserPCR1.PCR=false;			// Default All false, Set only those needed

		UserPCR1.FIELD.DSE=true;
		UserPCR1.FIELD.MUX=PORT_mAlt4;//PORT_mAlt3;
		UserPCR1.FIELD.IRQC=PORT_eDisabled;

		PORT_Configure2 (PORTD,1,UserPCR1);

	// Se inicializa los pines para las otras dos señales FTM0

		// PTA 1 as PWM Channel 6
		PCRstr UserPCR2;

		UserPCR2.PCR=false;			// Default All false, Set only those needed

		UserPCR2.FIELD.DSE=true;
		UserPCR2.FIELD.MUX=PORT_mAlt3;//PORT_mAlt3;
		UserPCR2.FIELD.IRQC=PORT_eDisabled;

		PORT_Configure2 (PORTA,1,UserPCR2);
	// PTA 2 as PWM Channel 7
		PCRstr UserPCR3;

		UserPCR3.PCR=false;			// Default All false, Set only those needed

		UserPCR3.FIELD.DSE=true;
		UserPCR3.FIELD.MUX=PORT_mAlt3;//PORT_mAlt3;
		UserPCR3.FIELD.IRQC=PORT_eDisabled;

		PORT_Configure2 (PORTA,2,UserPCR3);

	// PTC 8 as GPIO
		PCRstr UserPCRg;
		UserPCRg.PCR=false;			// Default All false, Set only those needed

		UserPCRg.FIELD.DSE=true;
		UserPCRg.FIELD.MUX=PORT_mGPIO;
		UserPCRg.FIELD.IRQC=PORT_eDisabled;

		PORT_Configure2 (PORTC,8,UserPCRg);

		GPIO_SetDirection(PTC, 8, GPIO__OUT);




	// Configuro parametros para la FTM3 ,en general

	FTM_SetPrescaler(FTM3, FTM_PSC_x4);
	FTM_SetModulus(FTM3, PWM_modulus);
	FTM_SetOverflowMode(FTM3, true);

	FTM_Sync_FTM_Counter(FTM3);
	FTM_Combine_Channels(FTM3,FTM_CH_0);   //creo q no ta funcionando. la sincronizacion

	//configuracion para los cananles especificos
	FTM_SetWorkingMode(FTM3, FTM_CH_0, FTM_mPulseWidthModulation);			// MSA  / B
	FTM_SetPulseWidthModulationLogic(FTM3, FTM_CH_0, FTM_lAssertedHigh);   // ELSA / B
	FTM_SetCounter(FTM3, FTM_CH_0, PWM_duty);

	FTM_Channel_Outinit(FTM3,FTM_CH_0); //NOANDA

	FTM_SetWorkingMode(FTM3, FTM_CH_1, FTM_mPulseWidthModulation);			// MSA  / B
	FTM_SetPulseWidthModulationLogic(FTM3, FTM_CH_1, FTM_lAssertedHigh);   // ELSA / B
	FTM_SetCounter(FTM3, FTM_CH_1, PWM_duty);



	/// Configuro parametros para la FTM0 , en general

	FTM_SetPrescaler(FTM0, FTM_PSC_x4);
	FTM_SetModulus(FTM0, PWM_modulus);
	FTM_SetOverflowMode(FTM0, true);

	FTM_Sync_FTM_Counter(FTM0);
	FTM_Combine_Channels(FTM0,3);

	//Config pa los canales
	FTM_SetWorkingMode(FTM0, 6, FTM_mPulseWidthModulation);			// MSA  / B
	FTM_SetPulseWidthModulationLogic(FTM0, 6, FTM_lAssertedHigh);   // ELSA / B
	FTM_SetCounter(FTM0, 6, PWM_duty);
;

	FTM_SetWorkingMode(FTM0, 7, FTM_mPulseWidthModulation);			// MSA  / B
	FTM_SetPulseWidthModulationLogic(FTM0, 7, FTM_lAssertedHigh);   // ELSA / B
	FTM_SetCounter(FTM0, 7, PWM_duty);

	//Se inicia La ftm
	//FTM_StartClock(FTM3);
	//FTM_StartClock(FTM0); //unico fin testearlo, no arrancarlo con el otro*/
	return;
}




// Setters
void FTM_Sync_FTM_Counter(FTM_t ftm){
	uint32_t synconf=0;
	uint32_t sync=0;
	uint32_t mode=0;

	//ftm->MODE=mode;
	ftm->SYNCONF=synconf;
	ftm->SYNC=sync;
	//MODE
	mode|=FTM_MODE_FTMEN(1);
	mode|=FTM_MODE_PWMSYNC(1);
	mode|=FTM_MODE_WPDIS(1);
	mode|=FTM_MODE_INIT(1);

	//SYNCONF
	synconf|=FTM_SYNCONF_SYNCMODE(1);//
	synconf|=FTM_SYNCONF_SWWRBUF(1);
	synconf|=FTM_SYNCONF_CNTINC(1);

	//synconf|=FTM_SYNCONF_SWRSTCNT(1); OTRA FORMA DE SINCRONIZAR

	//SYNC
	sync|=FTM_SYNC_SWSYNC(1);  //

	//sync|=FTM_SYNC_REINIT(1);	//CREO.

	ftm->MODE=mode;
	ftm->SYNCONF=synconf;
	ftm->SYNC=sync;


	return;
}


void FTM_Channel_Outinit(FTM_t ftm, uint8_t Channel){
	uint32_t outinit=0;
	uint32_t mode=0;
	mode=ftm->MODE;
	mode|=FTM_MODE_INIT(1);

	outinit|=FTM_OUTINIT_CH0OI(0);

	ftm->MODE=mode;
	ftm->OUTINIT=outinit;
	return;
}



void FTM_DeadTime(FTM_t ftm, uint8_t DeadT_value,FTM_Prescale_DT_t prescaleDT,bool on)
{
	if(on==true){
			uint32_t deadtime=0;
			deadtime=FTM_DEADTIME_DTPS(prescaleDT);
			deadtime|=FTM_DEADTIME_DTVAL(DeadT_value);
			ftm->DEADTIME=deadtime;
			ftm->COMBINE|=FTM_COMBINE_DTEN2(1);
		}
	return;
}
void FTM_Channel_Pol_ALOW(FTM_t ftm, uint8_t Channel)		//active_low
{
	uint32_t Pol_Ch=0;
	static bool lock=false;
	if (lock==false){
		Pol_Ch=0;		//comparte registro con el deadtiem , posible error
		lock=true;
		}
	switch(Channel){
		case 0:	ftm->POL=(ftm->POL&~FTM_POL_POL0_MASK)|FTM_POL_POL0(1);
			//Pol_Ch=FTM_POL_POL0(1);
				//ftm->POL|=Pol_Ch;
		break;
		case 1:Pol_Ch=FTM_POL_POL1(1);
			ftm->POL|=Pol_Ch;break;
		case 2:Pol_Ch=FTM_POL_POL2(1);
			ftm->POL|=Pol_Ch;break;
		case 3:Pol_Ch=FTM_POL_POL3(1);
			ftm->POL|=Pol_Ch;break;
		case 4:Pol_Ch=FTM_POL_POL4(1);
			ftm->POL|=Pol_Ch;break;
		case 5:Pol_Ch=FTM_POL_POL5(1);
			ftm->POL|=Pol_Ch;break;
		case 6:Pol_Ch=FTM_POL_POL6(1);
			ftm->POL|=Pol_Ch;break;
		case 7:Pol_Ch=FTM_POL_POL7(1);
			ftm->POL|=Pol_Ch;break;
		}
	return;
}

void FTM_Combine_Channels(FTM_t ftm, uint8_t pair)
{
	uint32_t combine;
	static bool lock=false;
	if (lock==false){
		combine=0;		//comparte registro con el deadtiem , posible error
		lock=true;
		}
	switch(pair){
		case 0:	combine|=FTM_COMBINE_SYNCEN0(1); //sincronizo el channel 0 con el 1.
			ftm->COMBINE|=combine;break;
		case 2:combine|=FTM_COMBINE_SYNCEN1(1); //sincronizo el channel 2 con el 3.
			ftm->COMBINE|=combine;break;
		case 4:combine|=FTM_COMBINE_SYNCEN2(1); //sincronizo el channel 4 con el 5.
			ftm->COMBINE|=combine; break;
		case 6:combine|=FTM_COMBINE_SYNCEN3(1); //sincronizo el channel 6 con el 7.
			ftm->COMBINE|=combine; break;
	}
	return;
}


void FTM_SetPrescaler (FTM_t ftm, FTM_Prescal_t data)
{
	ftm->SC = (ftm->SC & ~FTM_SC_PS_MASK) | FTM_SC_PS(data);
}

void FTM_SetModulus (FTM_t ftm, FTMData_t data)
{
	ftm->CNTIN = 0X00;
	ftm->CNT = 0X00;
	ftm->MOD = FTM_MOD_MOD(data);
}

FTMData_t FTM_GetModulus (FTM_t ftm)
{
	return ftm->MOD & FTM_MOD_MOD_MASK;
}

void FTM_StartClock (FTM_t ftm)
{
	ftm->SC |= FTM_SC_CLKS(0x01);
}

void FTM_StopClock (FTM_t ftm)
{
	ftm->SC &= ~FTM_SC_CLKS(0x01);
}

void FTM_SetOverflowMode (FTM_t ftm, bool mode)
{
	ftm->SC = (ftm->SC & ~FTM_SC_TOIE_MASK) | FTM_SC_TOIE(mode);
}

bool FTM_IsOverflowPending (FTM_t ftm)
{
	return ftm->SC & FTM_SC_TOF_MASK;
}

void FTM_ClearOverflowFlag (FTM_t ftm)
{
	ftm->SC &= ~FTM_SC_TOF_MASK;
}

void FTM_SetWorkingMode (FTM_t ftm, FTMChannel_t channel, FTMMode_t mode)
{

	ftm->CONTROLS[channel].CnSC = (ftm->CONTROLS[channel].CnSC & ~(FTM_CnSC_MSB_MASK | FTM_CnSC_MSA_MASK)) |
			                      (FTM_CnSC_MSB((mode >> 1) & 0X01) | FTM_CnSC_MSA((mode >> 0) & 0X01));
}

FTMMode_t FTM_GetWorkingMode (FTM_t ftm, FTMChannel_t channel)
{
	return (ftm->CONTROLS[channel].CnSC & (FTM_CnSC_MSB_MASK | FTM_CnSC_MSA_MASK)) >> FTM_CnSC_MSA_SHIFT;
}

void FTM_SetInputCaptureEdge (FTM_t ftm, FTMChannel_t channel, FTMEdge_t edge)
{
	ftm->CONTROLS[channel].CnSC = (ftm->CONTROLS[channel].CnSC & ~(FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK)) |
				                  (FTM_CnSC_ELSB((edge >> 1) & 0X01) | FTM_CnSC_ELSA((edge >> 0) & 0X01));
}

FTMEdge_t FTM_GetInputCaptureEdge (FTM_t ftm, FTMChannel_t channel)
{
	return (ftm->CONTROLS[channel].CnSC & (FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK)) >> FTM_CnSC_ELSA_SHIFT;
}

void FTM_SetOutputCompareEffect (FTM_t ftm, FTMChannel_t channel, FTMEffect_t effect)
{
	ftm->CONTROLS[channel].CnSC = (ftm->CONTROLS[channel].CnSC & ~(FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK)) |
				                  (FTM_CnSC_ELSB((effect >> 1) & 0X01) | FTM_CnSC_ELSA((effect >> 0) & 0X01));
}

FTMEffect_t FTM_GetOutputCompareEffect (FTM_t ftm, FTMChannel_t channel)
{
	return (ftm->CONTROLS[channel].CnSC & (FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK)) >> FTM_CnSC_ELSA_SHIFT;
}

void FTM_SetPulseWidthModulationLogic (FTM_t ftm, FTMChannel_t channel, FTMLogic_t logic)
{
	uint32_t cnsc=0;
	if(logic==FTM_lAssertedHigh){
		cnsc=ftm->CONTROLS[channel].CnSC;
		cnsc|=FTM_CnSC_ELSB(1);
	}
	else{//FTM_lAssertedHigh
		cnsc=ftm->CONTROLS[channel].CnSC;
		cnsc|=FTM_CnSC_ELSA(1);
	}
	ftm->CONTROLS[channel].CnSC=cnsc;
	return;
	/*ftm->CONTROLS[channel].CnSC = (ftm->CONTROLS[channel].CnSC & ~(FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK)) |
				                  (FTM_CnSC_ELSB((logic >> 1) & 0X01) | FTM_CnSC_ELSA((logic >> 0) & 0X01));*/
}

FTMLogic_t FTM_GetPulseWidthModulationLogic (FTM_t ftm, FTMChannel_t channel)
{
	return (ftm->CONTROLS[channel].CnSC & (FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK)) >> FTM_CnSC_ELSA_SHIFT;
}

void FTM_SetCounter (FTM_t ftm, FTMChannel_t channel, FTMData_t data)
{
	ftm->CONTROLS[channel].CnV = FTM_CnV_VAL(data);
}

FTMData_t FTM_GetCounter (FTM_t ftm, FTMChannel_t channel)
{
	return ftm->CONTROLS[channel].CnV & FTM_CnV_VAL_MASK;
}

void FTM_SetInterruptMode (FTM_t ftm, FTMChannel_t channel, bool mode)
{
	ftm->CONTROLS[channel].CnSC = (ftm->CONTROLS[channel].CnSC & ~FTM_CnSC_CHIE_MASK) | FTM_CnSC_CHIE(mode);
}

bool FTM_IsInterruptPending (FTM_t ftm, FTMChannel_t channel)
{
	return ftm->CONTROLS[channel].CnSC & FTM_CnSC_CHF_MASK;
}

void FTM_ClearInterruptFlag (FTM_t ftm, FTMChannel_t channel)
{
	ftm->CONTROLS[channel].CnSC &= ~FTM_CnSC_CHF_MASK;
}

