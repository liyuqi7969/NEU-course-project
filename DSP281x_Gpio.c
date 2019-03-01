// TI File $Revision: /main/2 $
// Checkin $Date: April 29, 2005   11:10:14 $
//###########################################################################
//
// FILE:	DSP281x_Gpio.c
//
// TITLE:	DSP281x General Purpose I/O Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP281x C/C++ Header Files V1.20 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File

//---------------------------------------------------------------------------
// InitGpio: 
//---------------------------------------------------------------------------
// This function initializes the Gpio to a known state.
//
void InitGpio(void)
{

	Uint16 cpld_reset_delay = 0;

	EALLOW;
	/*GPIOA configuration*/
	GpioMuxRegs.GPAMUX.all = 0x073F;
	GpioMuxRegs.GPAQUAL.all = 0x0000;	// Input qualifier disabled

	GpioMuxRegs.GPADIR.bit.GPIOA6 = 1;  //Short
	GpioMuxRegs.GPADIR.bit.GPIOA7 = 1;  //Break
	//GpioMuxRegs.GPADIR.bit.GPIOA10 = 0; //No use input
	GpioMuxRegs.GPADIR.bit.GPIOA11 = 0; //No use input
	GpioMuxRegs.GPADIR.bit.GPIOA12 = 0; //No use input
	GpioMuxRegs.GPADIR.bit.GPIOA13 = 0; //No use input
	GpioMuxRegs.GPADIR.bit.GPIOA14 = 0; //No use input
	GpioMuxRegs.GPADIR.bit.GPIOA15 = 0; //No use input

	/*GPIOB configuration*/
	GpioMuxRegs.GPBMUX.all = 0x0700;
	GpioMuxRegs.GPBQUAL.all = 0x0000;	// Input qualifier disabled

	GpioMuxRegs.GPBDIR.bit.GPIOB0 = 0;  //dec(0)
	GpioMuxRegs.GPBDIR.bit.GPIOB1 = 0;  //dec(1)
	GpioMuxRegs.GPBDIR.bit.GPIOB2 = 0;  //dec(2)
	GpioMuxRegs.GPBDIR.bit.GPIOB3 = 0;  //dec(3)
	GpioMuxRegs.GPBDIR.bit.GPIOB4 = 0;  //dec(4)
	GpioMuxRegs.GPBDIR.bit.GPIOB5 = 0;  //IO0
	GpioMuxRegs.GPBDIR.bit.GPIOB6 = 0;  //IO1
	GpioMuxRegs.GPBDIR.bit.GPIOB7 = 0;  //IO2

	GpioMuxRegs.GPBDIR.bit.GPIOB11 = 0; //No use input
	GpioMuxRegs.GPBDIR.bit.GPIOB12 = 0; //No use input
	GpioMuxRegs.GPBDIR.bit.GPIOB13 = 0; //IO3
	GpioMuxRegs.GPBDIR.bit.GPIOB14 = 1; //Reset cpld
	GpioMuxRegs.GPBDIR.bit.GPIOB15 = 0; //No use input

	/*GPIOD configuration*/
	GpioMuxRegs.GPDQUAL.all = 0x0000;	// Input qualifier disabled

	GpioMuxRegs.GPDMUX.bit.T1CTRIP_PDPA_GPIOD0 = 0;
	GpioMuxRegs.GPDMUX.bit.T2CTRIP_SOCA_GPIOD1 = 0;
	GpioMuxRegs.GPDMUX.bit.T3CTRIP_PDPB_GPIOD5 = 0;
	GpioMuxRegs.GPDMUX.bit.T4CTRIP_SOCB_GPIOD6 = 0;

	GpioMuxRegs.GPDDIR.bit.GPIOD0 = 0; //No use input
	GpioMuxRegs.GPDDIR.bit.GPIOD1 = 0; //No use input
	GpioMuxRegs.GPDDIR.bit.GPIOD5 = 0; //No use input
	GpioMuxRegs.GPDDIR.bit.GPIOD6 = 0; //No use input

	/*GPIOE configurantion*/
	GpioMuxRegs.GPEQUAL.all = 0x0000;	// Input qualifier disabled

	GpioMuxRegs.GPEMUX.bit.XINT1_XBIO_GPIOE0 = 0;
	GpioMuxRegs.GPEMUX.bit.XINT2_ADCSOC_GPIOE1 = 0;
	GpioMuxRegs.GPEMUX.bit.XNMI_XINT13_GPIOE2 = 0;

	GpioMuxRegs.GPEDIR.bit.GPIOE0 = 0; //DSP ON OFF
	GpioMuxRegs.GPEDIR.bit.GPIOE1 = 0; //No use input
	GpioMuxRegs.GPEDIR.bit.GPIOE2 = 0; //No use input

	/*GPIOF configuration*/
	GpioMuxRegs.GPFMUX.bit.SPISIMOA_GPIOF0 = 1; //SPI_DAC_SIMO
	GpioMuxRegs.GPFMUX.bit.SPISOMIA_GPIOF1 = 1; //SPI_DAC_SOMI
	GpioMuxRegs.GPFMUX.bit.SPICLKA_GPIOF2 = 1;  //SPI_DAC_CLK
	GpioMuxRegs.GPFMUX.bit.SPISTEA_GPIOF3 = 1;  //SPISTEA
	GpioMuxRegs.GPFMUX.bit.SCITXDA_GPIOF4 = 1;  //SCITXDA
	GpioMuxRegs.GPFMUX.bit.SCIRXDA_GPIOF5 = 1;  //SCIRXDA
	GpioMuxRegs.GPFMUX.bit.CANTXA_GPIOF6 = 1;   //CANTXA
	GpioMuxRegs.GPFMUX.bit.CANRXA_GPIOF7 = 1;   //CANRXA
	GpioMuxRegs.GPFMUX.bit.MCLKXA_GPIOF8 = 0;
	GpioMuxRegs.GPFMUX.bit.MCLKRA_GPIOF9 = 0;
	GpioMuxRegs.GPFMUX.bit.MFSXA_GPIOF10 = 0;
	GpioMuxRegs.GPFMUX.bit.MFSRA_GPIOF11 = 0;
	GpioMuxRegs.GPFMUX.bit.MDXA_GPIOF12 = 0;
	GpioMuxRegs.GPFMUX.bit.MDRA_GPIOF13 = 0;
	GpioMuxRegs.GPFMUX.bit.XF_GPIOF14 = 0;

	GpioMuxRegs.GPFDIR.bit.GPIOF8 = 1;  //Run LED
	GpioMuxRegs.GPFDIR.bit.GPIOF9 = 0;  //No use input
	GpioMuxRegs.GPFDIR.bit.GPIOF10 = 0; //No use input
	GpioMuxRegs.GPFDIR.bit.GPIOF11 = 0; //Stop LED
	GpioMuxRegs.GPFDIR.bit.GPIOF12 = 0; //No use input
	GpioMuxRegs.GPFDIR.bit.GPIOF13 = 0; //No use input
	GpioMuxRegs.GPFDIR.bit.GPIOF14 = 0; //No use input

	/*GPIOG configuration*/
	GpioMuxRegs.GPGMUX.bit.SCITXDB_GPIOG4 = 1;
	GpioMuxRegs.GPGMUX.bit.SCIRXDB_GPIOG5 = 1;

	EDIS;

	GpioDataRegs.GPFSET.bit.GPIOF8 = 1; //program runing(high)
	GpioDataRegs.GPFCLEAR.bit.GPIOF11 = 1; //program stop(low)

	GpioDataRegs.GPASET.bit.GPIOA7 = 1; //NO Break
	GpioDataRegs.GPASET.bit.GPIOA6 = 0; //Short

	GpioDataRegs.GPBCLEAR.bit.GPIOB14 = 1;
	for( cpld_reset_delay = 0;cpld_reset_delay <= 10000;cpld_reset_delay++ )
	{ ; }
	GpioDataRegs.GPBSET.bit.GPIOB14 = 1;
	for( cpld_reset_delay = 0;cpld_reset_delay <= 10000;cpld_reset_delay++ )
	{ ; }
	GpioDataRegs.GPBCLEAR.bit.GPIOB14 = 1;

}	
	
//===========================================================================
// No more.
//===========================================================================
