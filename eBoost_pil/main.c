
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "main.h"

// linker addresses, needed to copy code from flash to ram
extern Uint16 RamfuncsRunStart, RamfuncsLoadStart, RamfuncsLoadSize;

// Select the example to compile in.  Only one example should be set as 1
// the rest should be set as 0.

// Prototype statements for functions found within this file.
__interrupt void currentcomp_isr(void);
__interrupt void adc_vp_isr(void);
__interrupt void epwm1_timer_isr(void);

// Prototype statements for functions found within this file.
void Gpio_setup1(void);  // Initialization Function
void InitExtInt(void);
void ADCSetup(void);
void DACSetup(void);
void Actuator(void);         // Functional Function
void StartUp(void);
void Controller(void);
int OverVoltage(void);
void IntiVar(void);
void delay_loop(void);
void delay_ADC(void);
void InitEPwmTimer(void);
void InitExtInt(void);


const uint16_t offtime=90;
const uint16_t delaytime=6;

const long int kp=780;
const long int ki=178;

const long int Vref1=2146;  //18  //2361*10/11=
const long int Vref2=2206;  //18.5  //2427*10/11=

const long int max_act=952320;//930<->(8.3A)
const long int min_act=0;

const long int max_control=952320;//930<->(8.3A)
const long int min_control=0;

const long int offset=94356;
const long int offset_shift=96620544;

// variable declaration
long int Vref;

long int VoltagePeak;
long int VoltageVally;

long int ipk;
long int ipkp;

long int e;
long int ep;

void SCIInit (uint32_t baudrate, uint32_t clk ){
	EALLOW;

	SysCtrlRegs.PCLKCR0.bit.SCIAENCLK=1;

    SciaRegs.SCICTL1.all = 0;
    SciaRegs.SCICCR.all = 0x07;
    SciaRegs.SCICTL1.all = 0x0013;

    int16_t brr = (Uint16)(clk /8l /baudrate) - 1;
    SciaRegs.SCIHBAUD = 0xFF & (brr>>8);
    SciaRegs.SCILBAUD = 0xFF & brr;

    // setup FIFO
    SciaRegs.SCIFFTX.all = 0xC000;
    SciaRegs.SCIFFRX.all = 0x0000;
    SciaRegs.SCIFFCT.all = 0x00;

    SciaRegs.SCICTL1.all = 0x0033;
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;

	// enable pins
	GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;
	GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;
	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;
	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;

	EDIS;
}

void SCIWriteString(const char *s){
	while((*s) != 0){
	    while(SciaRegs.SCICTL2.bit.TXRDY == 0){
			continue;
	    }
	    SciaRegs.SCITXBUF = *s;
		s++;
	}
}

void main(void)
{
	// WARNING: Always ensure you call memcpy before running any functions from RAM
	// InitSysCtrl includes a call to a RAM based function and without a call to
	// memcpy first, the processor will go "into the weeds"
	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
	InitFlash();

	// Step 1. Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the f2802x_SysCtrl.c file.
	InitSysCtrl();

	// Step 2. Initialize GPIO:
	// This example function is found in the f2802x_Gpio.c file and
	// illustrates how to set the GPIO to it's default state.
	// InitGpio(); Skipped for this example

	// Step 3. Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	DINT;

	// Initialize PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the f2802x_PieCtrl.c file.
	InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in f2802x_DefaultIsr.c.
	// This function is found in f2802x_PieVect.c.
	InitPieVectTable();

	// configure serial line
	SCIInit(BAUD_RATE, LSPCLK_HZ);
	SCIWriteString("\n\rRS-232 Initialized.\n\r");

	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.XINT1=&currentcomp_isr;
	PieVectTable.ADCINT1 = &adc_vp_isr;
	PieVectTable.EPWM1_INT = &epwm1_timer_isr;
	EDIS;


	InitAdc();
	AdcOffsetSelfCal();
	ADCSetup();
	DACSetup();

	InitEPwmTimer();
	InitExtInt();
	Gpio_setup1();

	// Step 4. Initialize all the Device Peripherals:
	// Not required for this example
	// Enable CPU INT3 which is connected to EPWM1-6 INT:
	IER |= M_INT3;
	// Enable CPU INT1 which is connected to XINT and ADCINT1
	IER |= M_INT1;
	// Enable EPWM INTn in the PIE: Group 3 interrupt 1-6
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
	// Enable XINT1 in the PIE: Group 1 interrupt 4
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
	// Enable ADCINT1 in the PIE: Group 1 interrupt 1
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
	// Enable ADCINT2 in the PIE: Group 1 interrupt 2
	// PieCtrlRegs.PIEIER1.bit.INTx2 = 1;
	// Enable the PIE Block
	PieCtrlRegs.PIECTRL.bit.ENPIE=1;
	// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM


	StartUp();
	// Step 5. User specific code:

	while(EPwm3Regs.TBCTR <= 8192)
	{
		Vref=3280;
	}

	while(1)
	{
		Vref=3000;
		GpioDataRegs.GPADAT.bit.GPIO4=0;
		GpioDataRegs.GPADAT.bit.GPIO4=1;
	}


}

void Gpio_setup1(void)
{
	// Example 1:
	// Basic Pinout.
	// This basic pinout includes:
	// PWM1-3, TZ1-TZ4, SPI-A, EQEP1, SCI-A, I2C
	// and a number of I/O pins

	// These can be combined into single statements for improved
	// code efficiency.

	// Enable PWM1-3 on GPIO0-GPIO5
	EALLOW;

	// Enable GPIO3 as pwm output, set it low
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;   // Enable pullup on GPIO3
	GpioDataRegs.GPASET.bit.GPIO3 = 0;// Load output latch
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;  // GPIO3 = GPIO3
	GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;   // GPIO3 = output

	// Enable GPIO1 as comparator1 output
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;   // Enable pullup on GPIO1
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 3;  // GPIO1 = comparator output

	//Enable PWM1A output on GPIO0
	//  GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;   // Enable pullup on GPIO0
	//  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;  // GPIO0 = PWM1A
	//Enable XINT1 input on GPIO2
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;         // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;          // input
	GpioCtrlRegs.GPAQSEL1.bit.GPIO2 = 0;        // XINT1 Synch to SYSCLKOUT only
	// Enable ADCINA1 input (Naturally) as
	// Enable Comparator1 input( also the ADCINA2 input) as ADCINA2

	// Enable GPIO 4 as a test pin
	GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;   // Enable pullup on GPIO3
	GpioDataRegs.GPASET.bit.GPIO4 = 0;// Load output latch
	GpioCtrlRegs.GPAMUX1.bit.GPIO4= 0;  // GPIO3 = GPIO3
	GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;   // GPIO3 = output
	GpioDataRegs.GPADAT.bit.GPIO4=0;   // initialize GPIO4 output to 0

	EDIS;
}

void InitExtInt()
{

	EALLOW;
	GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 2;   // XINT1 is GPIO2

	// Configure XINT
	XIntruptRegs.XINT1CR.bit.POLARITY = 1;      //  Rising edge interrupt ( Falling edge interrupt is 0)

	XIntruptRegs.XINT1CR.bit.ENABLE = 0;        // Disable XINT1

	EDIS;
}

void InitEPwmTimer()
{

	// PWM1A Setting
	EPwm1Regs.TBPRD = 600; // Period = 600 TBCLK counts
	EPwm1Regs.CMPA.half.CMPA = 350; // Compare A = 350 TBCLK counts
	//EPwm1Regs.CMPB = 100; // Compare B = 200 TBCLK counts
	EPwm1Regs.TBPHS.half.TBPHS= 0; // Set Phase register to zero
	EPwm1Regs.TBCTR = 0; // set TB counter be 0
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Phase loading disabled
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // TBCLK = SYSCLK
	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR = Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR = Zero
	EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;
	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
	EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;

	EPwm3Regs.TBPRD = 65535; // Period = 600 TBCLK counts
	//EPwm2Regs.CMPA.half.CMPA = 100; // Compare A = 350 TBCLK counts
	//EPwm1Regs.CMPB = 100; // Compare B = 200 TBCLK counts
	EPwm3Regs.TBPHS.half.TBPHS= 0; // Set Phase register to zero
	EPwm3Regs.TBCTR = 0; // set TB counter be 0
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Phase loading disabled
	EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 3 ; // DIV 8
	EPwm3Regs.TBCTL.bit.CLKDIV = 7; // DIV 128
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR = Zero
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR = Zero
	EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;
	EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;
	EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;

	// PWM1 Event Trigger Interrupt
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPA;     // Select INT on A event
	EPwm1Regs.ETSEL.bit.INTEN = 0;                 // Disable INT
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on every event

}


void ADCSetup()
{
	EALLOW;
	AdcRegs.ADCCTL1.bit.INTPULSEPOS  = 1;    //ADCINT1 trips after AdcResults latch
	AdcRegs.INTSEL1N2.bit.INT1E     = 0;     //Disabled ADCINT1
	AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;     //Disable ADCINT1 Continuous mode
	AdcRegs.INTSEL1N2.bit.INT1SEL    = 0;    //setup EOC0 to trigger ADCINT1 to fire
	AdcRegs.ADCSOC0CTL.bit.CHSEL     = 6;    //set SOC0 channel select to ADCINA6
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL   = 0;    //set SOC0 is triggered by SOFTWARE
	AdcRegs.ADCSOC0CTL.bit.ACQPS     = 6;    //set SOC0 S/H Window to 7 ADC Clock  Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCINTFLGCLR.bit.ADCINT1=1;
	PieCtrlRegs.PIEIFR1.bit.INTx1 = 0;
	EDIS;
}

void DACSetup()
{
	EALLOW;
	Comp1Regs.COMPCTL.bit.COMPDACEN = 1; //The comparator is powered up
	Comp1Regs.COMPCTL.bit.SYNCSEL = 0; //Asynchronous version
	Comp1Regs.COMPCTL.bit.CMPINV = 0; //pass
	Comp1Regs.COMPCTL.bit.COMPSOURCE = 0; //The inverting input is internal DAC
	Comp1Regs.DACCTL.bit.DACSOURCE = 0; //The DATA source is from DACVAL
	Comp1Regs.DACVAL.bit.DACVAL = 100; // (3.3/1023*xxx)V
	EDIS;
}

// 5ms delay for analog adc circuit to set up
void delay_ADC()
{
	long      i;
	for (i = 0; i < 300000; i++) {}
}

void delay_loop()
{
	long      i;
	for (i = 0; i < 1000; i++) {}
}

void IntiVar()
{
	Vref=Vref1;

	VoltagePeak=0;
	VoltageVally=0;

	e=Vref;
	ep=Vref;

	ipk=10240;
	ipkp=10240;

}

void Actuator()
{
	// actuator saturation setting
	long int action;
	if(ipk<min_act)
		action=min_act;
	else if (ipk>max_act)
		action=max_act;
	else
		action=ipk;
	// actuate
	Comp1Regs.DACVAL.bit.DACVAL = action>>10; // (3.3/1023*xxx)V
}

void Controller()
{
	e=Vref-VoltagePeak;
	ipk=ipkp+ki*e+kp*(e-ep);
	// controller saturation setting
	if (ipk>=max_control)
	{ipk=max_control;}
	else if  (ipk<=min_control)
	{ipk=min_control;}
	else {}
	ep=e;
	ipkp=ipk;
}


void StartUp()
{
	long int      i;
	for (i = 0; i < 300000; i++) {}
	// Initialize all variables
	IntiVar();
	// turn on switch ( GPIO 3)
	GpioDataRegs.GPADAT.bit.GPIO3 = 1;
	// step up command 0
	GpioDataRegs.GPADAT.bit.GPIO4 = 0;
	GpioDataRegs.GPADAT.bit.GPIO5 = 0;
	delay_ADC();
	EALLOW;
	XIntruptRegs.XINT1CR.bit.ENABLE = 1;        // Enable XINT1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	EDIS;
}

int OverVoltage()
{
	int ovflag;
	if(VoltagePeak>=3500)
		ovflag=1;
	else
		ovflag=0;
	return ovflag;
}

// Interrupt routines uses in this example:
__interrupt void currentcomp_isr(void)  // (GPIO_2,XINT_1)
{
	EALLOW;
	XIntruptRegs.XINT1CR.bit.ENABLE = 0;            // Disable_current_comp_interrupt
	GpioDataRegs.GPADAT.bit.GPIO3 = 0;              // pwm_off;
	EPwm1Regs.ETCLR.bit.INT = 1;                    // off-timer interrupt flag clear
	EPwm1Regs.TBCTR = 0;                            // ResetOffTimer;
	EPwm1Regs.ETSEL.bit.INTEN = 1;                  // Enable off_timer interrupt
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	EDIS;
}

/*__interrupt void adc_vv_isr(void)
{
    VoltageVally = AdcResult.ADCRESULT0;                       // GetValue
    GpioDataRegs.GPADAT.bit.GPIO3 = 0;                          // pwm_off;
    EPwm1Regs.TBCTR = 11;                                     // ResetOffTimer;
    EPwm1Regs.ETSEL.bit.INTEN = 1;                 //Enable_offtimer_interrupt;
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //Clear ADCINT1 flag reinitialize for next SOC
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;      // Acknowledge interrupt to PIE
    return;
}*/

__interrupt void epwm1_timer_isr(void)
{
	EALLOW;
	EPwm1Regs.ETSEL.bit.INTEN = 0;             // Disable off_timer interrupt
	AdcRegs.INTSEL1N2.bit.INT1E     = 1;       //Enabled ADCINT1
	AdcRegs.ADCSOCFRC1.bit.SOC0=1;             //Software Trigger ADC_ontime once
	//                       while(AdcRegs.ADCINTFLG.bit.ADCINT1==0);
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
	// Acknowledge interrupt to receive more interrupts from group 1\EALLOW;
	EDIS;
}

__interrupt void adc_vp_isr(void)
{
	EALLOW;
	AdcRegs.ADCINTFLGCLR.bit.ADCINT1=1;
	AdcRegs.INTSEL1N2.bit.INT1E     = 0;
	VoltagePeak = AdcResult.ADCRESULT0;
	Controller();
	Actuator();
	GpioDataRegs.GPADAT.bit.GPIO3 = 1;    // pwm_on;
	//PieCtrlRegs.PIEIFR1.bit.INTx4 = 0;   // current_comp_interrupt flag set 0
	XIntruptRegs.XINT1CR.bit.ENABLE = 1;   // enable_current_comp_interrupt
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	EDIS;
}
