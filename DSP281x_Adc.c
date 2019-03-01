// TI File $Revision: /main/2 $
// Checkin $Date: April 29, 2005   11:11:45 $
//###########################################################################
//
// FILE:	DSP281x_Adc.c
//
// TITLE:	DSP281x ADC Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP281x C/C++ Header Files V1.20 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "DSP281x_Device.h"     // DSP281x Headerfile Include File
#include "DSP281x_Examples.h"   // DSP281x Examples Include File

#define ADC_usDELAY  8000L
#define ADC_usDELAY2 20L

//---------------------------------------------------------------------------
// InitAdc: 
//---------------------------------------------------------------------------
// This function initializes ADC to a known state.
//
void InitAdc(void)
{
	extern void DSP28x_usDelay(Uint32 Count);
	Uint16 i;
	
    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap and reference circuitry.
    // After a 5ms delay the rest of the ADC can be powered up. After ADC
    // powerup, another 20us delay is required before performing the first
    // ADC conversion. Please note that for the delay function below to
    // operate correctly the CPU_CLOCK_SPEED define statement in the
    // DSP28_Examples.h file must contain the correct CPU clock period in
    // nanoseconds. For example:

	AdcRegs.ADCTRL3.bit.ADCBGRFDN = 0x3;	// ADCCLK = HSPCLK/2,同步采样模式
	DELAY_US(ADC_usDELAY);                  // 延时800us等待设定成功
	AdcRegs.ADCTRL3.bit.ADCPWDN = 1;		// 模拟电路上电
	DELAY_US(ADC_usDELAY2);                 // 延时20us等待模拟电路上电

	AdcRegs.ADCTRL1.bit.RESET = 1;          //复位ADC
	for( i = 0;i < 10000;i++ )              //延时等待复位
	{
		;
	}
	AdcRegs.ADCTRL1.all = 0x1240; //仿真挂起模式，窗口大小=3*ADCCLK,不对外设时钟分频，连续转换模式，双排序模式
	AdcRegs.ADCTRL1.bit.CONT_RUN = 1;   //连续转换模式
	AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;   //双排序模式
	AdcRegs.ADCTRL2.all = 0x4100;       //复位排序器，使能EVA出发ADC转换
	AdcRegs.ADCTRL3.bit.ADCBGRFDN = 3; //间隙和参考电路上电
	for( i = 0;i < 10000;i++ )  //延时等待电路上电
	{
		;
	}
	AdcRegs.ADCTRL3.bit.ADCPWDN = 1; //模拟电路上电
	for( i = 0;i < 5000;i++ )   //延时等待电路上电
	{
		;
	}
	AdcRegs.ADCTRL3.bit.ADCCLKPS = 2; // HSPCLK/[4*(ADCTRL1[7] + 1)] = ADCCLK
	AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 1;  //最大转换通道数 = 2
	AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0xc; //1100 ADCINB4 Ia 结果放在AdcResult[0]
	AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0xd; //1101 ADCINB5 Ib 结果放在AdcResult[1]
}	
//此处使用双排序 同步采样模式，由于仅仅有2个通道需要采样，都是可以实现采样。
//===========================================================================
// No more.
//===========================================================================
