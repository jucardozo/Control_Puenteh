
#include "FTM.h"
#include "PORT.h"
#include "GPIO.h"
#include "Timer.h"




void PWM_Init(void);
void PWM_ISR3(void);
void PWM_ISR0(void);
void IsTime(void);
void MODO_NORMAL1();
/*FUNCIONES PRIVADAS*/
void FTM_Enable_W_Protec(FTM_t ftm);
void FTM_Disable_W_Protec(FTM_t ftm);

uint16_t PWM_modulus = 147-1;//10000-1;
uint16_t PWM_duty    = 74;//5000-1;
uint8_t DeadTime_Value=5;
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

	return;

}

void PWM_ISR0 (void)
{

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

void PWM_Init (void)
{
	//Todos las Salidas Responden al Modulo de la FTM3.
	// PTD 0 as PWM channel 0
		PCRstr UserPCR;

		UserPCR.PCR=false;			// Default All false, Set only those needed

		UserPCR.FIELD.DSE=true;
		UserPCR.FIELD.MUX=PORT_mAlt4;//PORT_mAlt3;
		UserPCR.FIELD.IRQC=PORT_eDisabled;

		PORT_Configure2 (PORTD,0,UserPCR);
	// PTD 1 as PWM Channel 1
		PCRstr UserPCR1;

		UserPCR1.PCR=false;
		UserPCR1.FIELD.DSE=true;
		UserPCR1.FIELD.MUX=PORT_mAlt4;//PORT_mAlt3;
		UserPCR1.FIELD.IRQC=PORT_eDisabled;

		PORT_Configure2 (PORTD,1,UserPCR1);

		// PTD 2 as PWM Channel 2
		PCRstr UserPCR2;

		UserPCR2.PCR=false;

		UserPCR2.FIELD.DSE=true;
		UserPCR2.FIELD.MUX=PORT_mAlt4;
		UserPCR2.FIELD.IRQC=PORT_eDisabled;

		PORT_Configure2 (PORTD,2,UserPCR2);

		// PTD 3 as PWM Channel 3
		PCRstr UserPCR3;

		UserPCR3.PCR=false;

		UserPCR3.FIELD.DSE=true;
		UserPCR3.FIELD.MUX=PORT_mAlt4;
		UserPCR3.FIELD.IRQC=PORT_eDisabled;

		PORT_Configure2 (PORTD,3,UserPCR3);


	// Configuro parametros para el modulo FTM3 ,en general
	FTM_Disable_W_Protec(FTM3);

	FTM_SetPrescaler(FTM3, FTM_PSC_x4);			//configuro prescaler
	FTM_SetModulus(FTM3, PWM_modulus);			//configuro modulo
	FTM_SetOverflowMode(FTM3, false);				//deshabilito la interrupciones

	//Configuracion Canal 0 -L1

	FTM_Combine_Channels(FTM3,FTM_CH_0); 		//channel 1 es el complemento del channel 0
	FTM_DeadTime(FTM3, FTM_CH_0 ,DeadTime_Value,FTM_Prescale_DT_1);	//habilitacion e insercion del deadtime de 100ns
	FTM_SetWorkingMode(FTM3, FTM_CH_0, FTM_mPulseWidthModulation);			// MSA  / B
	FTM_SetPulseWidthModulationLogic(FTM3, FTM_CH_0, FTM_Low_true);   // ELSA / B
	FTM_SetCounter(FTM3, FTM_CH_0, PWM_duty);
	//FTM_Channel_Pol_ALOW(FTM3,FTM_CH_0);		//ACTIVO BAJO

	//Configuracion para el canal 1 -H1

	//FTM_Channel_Outinit(FTM3,FTM_CH_1); 	//pa probar
	FTM_SetWorkingMode(FTM3, FTM_CH_1, FTM_mPulseWidthModulation);
	FTM_SetPulseWidthModulationLogic(FTM3, FTM_CH_1, FTM_High_true);
	FTM_SetCounter(FTM3, FTM_CH_1, PWM_duty);

	//configuracion para el canal 2 -L2

	FTM_Combine_Channels(FTM3,FTM_CH_2); 		//channel 3 es el complemento del channel 2
	FTM_FLT_Combine(FTM3,FTM_CH_2);
	FTM_DeadTime(FTM3, FTM_CH_2 ,DeadTime_Value,FTM_Prescale_DT_1);
	FTM_SetWorkingMode(FTM3, FTM_CH_2, FTM_mPulseWidthModulation);
	FTM_SetPulseWidthModulationLogic(FTM3, FTM_CH_2, FTM_Low_true);
	FTM_SetCounter(FTM3, FTM_CH_2, PWM_duty);



	//Configuracion para el canal 3 -H2
	//FTM_Channel_Outinit(FTM3,FTM_CH_3);
	//FTM_Channel_Pol_ALOW(FTM3,FTM_CH_3);		//ACTIVO BAJO
	FTM_SetWorkingMode(FTM3, FTM_CH_3, FTM_mPulseWidthModulation);
	FTM_SetPulseWidthModulationLogic(FTM3, FTM_CH_3, FTM_Low_true);
	FTM_SetCounter(FTM3, FTM_CH_3, PWM_duty);


	//Disable write
	FTM_Enable_W_Protec(FTM3);

	return;
}




// Setters
void MODO_REPOSO(){
	FTM3->SWOCTRL|=FTM_SWOCTRL_CH0OC(1); //inicializo las salidas.
	FTM3->SWOCTRL|=FTM_SWOCTRL_CH0OCV(1);

	FTM3->SWOCTRL|=FTM_SWOCTRL_CH1OC(1);

	FTM3->SWOCTRL|=FTM_SWOCTRL_CH2OC(1);
	FTM3->SWOCTRL|=FTM_SWOCTRL_CH2OCV(1);


	FTM3->SWOCTRL|=FTM_SWOCTRL_CH3OC(1);

	FTM_StartClock(FTM3);

	return;
	/*
	static int aux=0;
	if (aux==0){
		PCRstr UserPCRH1;
		UserPCRH1.PCR=false;
		UserPCRH1.FIELD.DSE=true;
		UserPCRH1.FIELD.MUX=PORT_mGPIO;
		UserPCRH1.FIELD.IRQC=PORT_eDisabled;
		PORT_Configure2 (PORTD,1,UserPCRH1);

		PCRstr UserPCRH2;
		UserPCRH2.PCR=false;
		UserPCRH2.FIELD.DSE=true;
		UserPCRH2.FIELD.MUX=PORT_mGPIO;
		UserPCRH2.FIELD.IRQC=PORT_eDisabled;
		PORT_Configure2 (PORTD,3,UserPCRH2);

		GPIO_SetDirection(PTD, 1, GPIO__OUT);
		GPIO_SetDirection(PTD, 3, GPIO__OUT);
		GPIO_Write(PTD, 1 << 1,0);
		GPIO_Write(PTD, 1 << 3,0);
		aux++;
	}

	PCRstr UserPCRL1;
	UserPCRL1.PCR=false;

	UserPCRL1.FIELD.DSE=true;
	UserPCRL1.FIELD.MUX=PORT_mGPIO;
	UserPCRL1.FIELD.IRQC=PORT_eDisabled;

	PORT_Configure2 (PORTD,0,UserPCRL1);

	PCRstr UserPCRL2;
	UserPCRL2.PCR=false;

	UserPCRL2.FIELD.DSE=true;
	UserPCRL2.FIELD.MUX=PORT_mGPIO;
	UserPCRL2.FIELD.IRQC=PORT_eDisabled;

	PORT_Configure2 (PORTD,2,UserPCRL2);
	GPIO_Write(PTD, 1 << 0,1);
	GPIO_Write(PTD, 1 << 2,1);

	return;*/
}

void MODO_NORMAL(){
	//FTM_StopClock(FTM3);
	FTM3->SWOCTRL=0; //Desabilito la salida forzada

	FTM_StartClock(FTM3);

	return;
}


bool FTM_IsFault (FTM_t ftm)
{
	return ftm->FMS & FTM_FMS_FAULTF_MASK ;
}

void IsTime(void){
	if (timerExpired(timerGetId())){
			FTM3->SWOCTRL=0;
			FTM_StartClock(FTM3);
	}
	return;
}

void FTM_ClearFaultF(FTM_t ftm){
	ftm->FMS &= ~FTM_FMS_FAULTF_MASK ;
	return;
}
void FTM_FaultCtrl(FTM_t ftm,bool interrup_on,uint8_t mode){
	//MODE
	ftm->MODE|=FTM_MODE_FAULTM(mode);  //SE HABILITA LA POSIBILIDAD DE TENER UNA CONDICION DE FALLA
	if(interrup_on){ftm->MODE|=FTM_MODE_FAULTIE(1);}	//SE HABILITAN INTERRUPCIONES CUANDO HAY UNA CONDICION DE FALLA
	return ;
}
void FTM_FLT_Combine(FTM_t ftm,uint8_t pair){
	switch(pair){ //habilito el par de canales
		case 0:	ftm->COMBINE|=FTM_COMBINE_FAULTEN0(1); //sincronizo el channel 0 con el 1.
				break;
		case 2:ftm->COMBINE|=FTM_COMBINE_FAULTEN1(1); //sincronizo el channel 2 con el 3.
				break;
		case 3:ftm->COMBINE|=FTM_COMBINE_FAULTEN2(1); //sincronizo el channel 4 con el 5.
				break;
		case 6:ftm->COMBINE|=FTM_COMBINE_FAULTEN3(1); //sincronizo el channel 6 con el 7.
				break;
	}
	return;
}

//no esta configurado para todas las FTM , en el caso de la 3 solo tiene dispo la FLT0. PTC12
void FTM_FLT_TimeStable(FTM_t ftm,uint8_t value){

	ftm->FLTCTRL|=FTM_FLTCTRL_FFVAL(value);		//limite del contador ffval, que garantiza tpo de establecimiento.
	ftm->FLTCTRL|=FTM_FLTCTRL_FFLTR0EN(1);
	ftm->FLTCTRL|=FTM_FLTCTRL_FAULT0EN(1);		//habilito la entrada?
	return;
}


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
	switch(Channel){
		case 0:ftm->OUTINIT|= FTM_OUTINIT_CH0OI(1);break;
		case 1:ftm->OUTINIT|= FTM_OUTINIT_CH1OI(1);break;
		case 2:ftm->OUTINIT|= FTM_OUTINIT_CH2OI(1);break;
		case 3:ftm->OUTINIT|= FTM_OUTINIT_CH3OI(1);break;
		case 4:ftm->OUTINIT|= FTM_OUTINIT_CH4OI(1);break;
		case 5:ftm->OUTINIT|= FTM_OUTINIT_CH5OI(1);break;
		case 6:ftm->OUTINIT|= FTM_OUTINIT_CH6OI(1);break;
		case 7:ftm->OUTINIT|= FTM_OUTINIT_CH7OI(1);break;
	}
	return;
}



void FTM_DeadTime(FTM_t ftm, uint8_t pair ,uint8_t DeadT_value,FTM_Prescale_DT_t prescaleDT)
{
	switch(pair){ //habilito el par de canales
		case 0:	ftm->COMBINE|=FTM_COMBINE_DTEN0(1); //sincronizo el channel 0 con el 1.
				break;
		case 2:ftm->COMBINE|=FTM_COMBINE_DTEN1(1); //sincronizo el channel 2 con el 3.
				break;
		case 3:ftm->COMBINE|=FTM_COMBINE_DTEN2(1); //sincronizo el channel 4 con el 5.
				break;
		case 6:ftm->COMBINE|=FTM_COMBINE_DTEN3(1); //sincronizo el channel 6 con el 7.
				break;
	}
	uint32_t deadtime=0;
	deadtime|=FTM_DEADTIME_DTPS(prescaleDT);
	deadtime|=FTM_DEADTIME_DTVAL(DeadT_value);
	ftm->DEADTIME=deadtime;
	return;
}
void FTM_Channel_Pol_ALOW(FTM_t ftm, uint8_t Channel)		//active_low
{
	switch(Channel){
		case 0:	ftm->POL|=(ftm->POL&~FTM_POL_POL0_MASK)|FTM_POL_POL0(1);break;
		case 1:ftm->POL|=(ftm->POL&~FTM_POL_POL1_MASK)|FTM_POL_POL1(1);break;
		case 2:ftm->POL|=(ftm->POL&~FTM_POL_POL2_MASK)|FTM_POL_POL2(1);break;
		case 3:ftm->POL|=(ftm->POL&~FTM_POL_POL3_MASK)|FTM_POL_POL3(1);break;
		case 4:ftm->POL=(ftm->POL&~FTM_POL_POL4_MASK)|FTM_POL_POL4(1);break;
		case 5:ftm->POL=(ftm->POL&~FTM_POL_POL5_MASK)|FTM_POL_POL5(1);break;
		case 6:ftm->POL=(ftm->POL&~FTM_POL_POL6_MASK)|FTM_POL_POL6(1);break;
		case 7:ftm->POL=(ftm->POL&~FTM_POL_POL7_MASK)|FTM_POL_POL7(1);break;
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
		case 0://sincronizo el channel 0 con el 1.
			ftm->COMBINE|=FTM_COMBINE_COMP0(1);break;
		case 2:combine|=FTM_COMBINE_COMP1(1); //sincronizo el channel 2 con el 3.
			ftm->COMBINE|=combine;break;
		case 4:combine|=FTM_COMBINE_COMP2(1); //sincronizo el channel 4 con el 5.
			ftm->COMBINE|=combine; break;
		case 6:combine|=FTM_COMBINE_COMP3(1); //sincronizo el channel 6 con el 7.
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
	ftm->CONTROLS[channel].CnSC = (ftm->CONTROLS[channel].CnSC & ~(FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK)) |
				                  (FTM_CnSC_ELSB((logic >> 1) & 0X01) | FTM_CnSC_ELSA((logic >> 0) & 0X01));
	return;}


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

/*FUNCIONES PRIVADAS*/
void FTM_Enable_W_Protec(FTM_t ftm){
	 ftm->MODE = (ftm->MODE & ~FTM_MODE_WPDIS_MASK) | FTM_MODE_WPDIS(0);
	 return;
}
void FTM_Disable_W_Protec(FTM_t ftm){
	ftm->MODE = FTM_MODE_WPDIS(1);
	return;
}
