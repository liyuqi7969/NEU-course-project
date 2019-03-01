// TI File $Revision: /main/2 $
// Checkin $Date: April 29, 2005   11:10:23 $
//###########################################################################
//
// FILE:	DSP281x_Ev.c
//
// TITLE:	DSP281x Event Manager Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP281x C/C++ Header Files V1.20 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File

//---------------------------------------------------------------------------
// InitEv: 
//---------------------------------------------------------------------------
// This function initializes to a known state.
//
void InitEv(void)
{
	EvaRegs.T1PR = (7500L - 1L) ;
	EvaRegs.T1CON.all = 0x080C; // Continuous-Up/-Down Count Mode x/1 timer disable

	EvaRegs.CMPR1 = 1000;
	EvaRegs.CMPR2 = 1000;
	EvaRegs.CMPR3 = 1000;
	EvaRegs.ACTRA.all = 0x0333; //PWM1,3,5 HIGH(OFF)  PWM2,4,6 LOW(ON)

	EvaRegs.COMCONA.all = 0x8A00;
	EvaRegs.GPTCONA.all = 0x0080;
	EvaRegs.DBTCONA.all = 0x0FF4;

	EvaRegs.T1CNT = 0;
	EvaRegs.EVAIMRA.bit.T1UFINT = 1;
	EvaRegs.EVAIMRA.bit.PDPINTA = 1;
	EvaRegs.EVAIFRA.all = 0xFFFF;

	EvaRegs.T2PR = 8191; //Setup period register
	EvaRegs.T2CON.all = 0x1870;
	EvaRegs.T2CNT = 0;

	EvaRegs.CAPCONA.all = 0xE000;

	EvbRegs.T3CNT = 0;
	EvbRegs.T3PR = (75000L - 1L); // Setup period register 1ms (30000) 2ms (60000) 0.1ms(3000)
	EvbRegs.T3CON.all = 0x080C; // Continuous-Up/-Down Count Mode x/1 timer disable
	EvbRegs.EVBIMRA.bit.T3UFINT = 1;
	EvbRegs.EVBIFRA.all = 0xFFFF;
}	
	
//===========================================================================
// No more.
//===========================================================================
