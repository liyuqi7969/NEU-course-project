/*******************************************************************************
Name: MDC_Motor_SPWM

Supported motor types:ACI

Algorithm types:
ACI: FOC

Control Method:
Fix speed

Author: Haichao Zhang
Time: 2014.11.20

Modify:
Time:
*******************************************************************************/
#include "DSP281x_Device.h"
#include "DSP281x_Examples.h" // DSP281x Examples Include File
#include <stdio.h>
#include <stdlib.h>
#define __BSD_VISIBLE
#include <math.h>
#undef __BSD_VISIBLE
#include "IQmathLib.h"
#include "util.h"
#include "svpwm.h"

/*****************************************************General variables***************************************************/
int16 K2 = 28378; //sqrt(3)/2(Q15) -->Funtion: acrloop()

int16 SINTAB[512] = {
    0, 402, 804, 1206, 1607, 2009, 2410, 2811, 3211, 3611, 4011, 4409, 4808, 5205, 5602, 5997,
    6392, 6786, 7179, 7571, 7961, 8351, 8739, 9126, 9512, 9896, 10278, 10659, 11039, 11416, 11793, 12167,
    12539, 12910, 13278, 13645, 14010, 14372, 14732, 15090, 15446, 15800, 16151, 16499, 16846, 17189, 17530, 17869,
    18204, 18537, 18868, 19195, 19519, 19841, 20159, 20475, 20787, 21097, 21403, 21706, 22005, 22301, 22594, 22884,
    23170, 23453, 23732, 24007, 24279, 24547, 24812, 25073, 25330, 25583, 25832, 26077, 26319, 26557, 26790, 27020,
    27245, 27466, 27684, 27897, 28106, 28310, 28511, 28707, 28898, 29086, 29269, 29447, 29621, 29791, 29956, 30117,
    30273, 30425, 30572, 30714, 30852, 30985, 31114, 31237, 31357, 31471, 31581, 31685, 31785, 31881, 31971, 32057,
    32138, 32214, 32285, 32351, 32413, 32469, 32521, 32568, 32610, 32647, 32679, 32706, 32728, 32745, 32758, 32765,
    32767, 32765, 32758, 32745, 32728, 32706, 32679, 32647, 32610, 32568, 32521, 32469, 32413, 32351, 32285, 32214,
    32138, 32057, 31971, 31881, 31785, 31685, 31581, 31471, 31357, 31237, 31114, 30985, 30852, 30714, 30572, 30425,
    30273, 30117, 29956, 29791, 29621, 29447, 29269, 29086, 28898, 28707, 28511, 28310, 28106, 27897, 27684, 27466,
    27245, 27020, 26790, 26557, 26319, 26077, 25832, 25583, 25330, 25073, 24812, 24547, 24279, 24007, 23732, 23453,
    23170, 22884, 22594, 22301, 22005, 21706, 21403, 21097, 20787, 20475, 20159, 19841, 19519, 19195, 18868, 18537,
    18204, 17869, 17530, 17189, 16846, 16499, 16151, 15800, 15446, 15090, 14732, 14372, 14010, 13645, 13278, 12910,
    12539, 12167, 11793, 11416, 11039, 10659, 10278, 9896, 9512, 9126, 8739, 8351, 7961, 7571, 7179, 6786,
    6392, 5997, 5602, 5205, 4808, 4409, 4011, 3611, 3211, 2811, 2410, 2009, 1607, 1206, 804, 402,
    0, -403, -805, -1207, -1608, -2010, -2411, -2812, -3212, -3612, -4012, -4410, -4809, -5206, -5603, -5998,
    -6393, -6787, -7180, -7572, -7962, -8352, -8740, -9127, -9513, -9897, -10279, -10660, -11040, -11417, -11794, -12168,
    -12540, -12911, -13279, -13646, -14011, -14373, -14733, -15091, -15447, -15801, -16152, -16500, -16847, -17190, -17531, -17870,
    -18205, -18538, -18869, -19196, -19520, -19842, -20160, -20476, -20788, -21098, -21404, -21707, -22006, -22302, -22595, -22885,
    -23171, -23454, -23733, -24008, -24280, -24548, -24813, -25073, -25331, -25584, -25833, -26078, -26320, -26558, -26791, -27021,
    -27246, -27467, -27685, -27898, -28107, -28311, -28512, -28708, -28899, -29087, -29270, -29448, -29622, -29792, -29957, -30118,
    -30274, -30426, -30573, -30715, -30853, -30986, -31115, -31238, -31358, -31472, -31582, -31686, -31786, -31882, -31972, -32058,
    -32139, -32215, -32286, -32352, -32414, -32470, -32522, -32569, -32611, -32648, -32680, -32707, -32729, -32746, -32759, -32766,
    -32768, -32766, -32759, -32746, -32729, -32707, -32680, -32648, -32611, -32569, -32522, -32470, -32414, -32352, -32286, -32215,
    -32139, -32058, -31972, -31882, -31786, -31686, -31582, -31472, -31358, -31238, -31115, -30986, -30853, -30715, -30573, -30426,
    -30274, -30118, -29957, -29792, -29622, -29448, -29270, -29087, -28899, -28708, -28512, -28311, -28107, -27898, -27685, -27467,
    -27246, -27021, -26791, -26558, -26320, -26078, -25833, -25584, -25331, -25074, -24813, -24548, -24280, -24008, -23733, -23454,
    -23171, -22885, -22595, -22302, -22006, -21707, -21404, -21098, -20788, -20476, -20160, -19842, -19520, -19196, -18869, -18538,
    -18205, -17870, -17531, -17190, -16847, -16500, -16152, -15801, -15447, -15091, -14733, -14373, -14011, -13646, -13279, -12911,
    -12540, -12168, -11794, -11417, -11040, -10660, -10279, -9897, -9513, -9127, -8740, -8352, -7962, -7572, -7180, -6787,
    -6393, -5998, -5603, -5206, -4809, -4410, -4012, -3612, -3212, -2812, -2411, -2010, -1608, -1207, -805, -403}; //Electrical angle

//Q14
int16 DEC_TAB[24] = {16384, -9459, 0, 18918, 16384, 9459, -16384, 9459, 0, 18918, -16384, -9459,
                     -16384, 9459, 0, -18918, -16384, -9459, 16384, -9459, 0, -18918, 16384, 9459}; //Function: Vector_Svpwm()

Uint16 index = 0;    //?????????
Uint16 t1per = 6000; //????????

int16 angle_cos = 0; //theta????????????
int16 angle_sin = 0;
int16 uarfa_given = 0;
int16 ubeita_given = 0; //u_out??alpha??beta??????????

int16 cmp0 = 0;
int16 cmp1 = 0;
int16 cmp2 = 0; //????

int32 long1_tmp = 0;
int32 long2_tmp = 0;
Uint32 long3_tmp = 0; //32¦Ë?§Þ????
int16 u16_tmp1 = 0;
int16 u16_tmp2 = 0; //16¦Ë?§Þ????

int16 spd_given_q12 = 0; //???????   ??0---4095??~??0---1500??
Uint16 f_given_reg1 = 0; //??¦È??????   ??0.01hz---50hz??~(1---5000)????
Uint16 f_given_reg2 = 0; //???¦È??????

/**************************************Flag Bit**************************************/

struct Flag_Bits
{                      // bits   description
  Uint16 Send : 1;     // 0
  Uint16 Test : 1;     // 1
  Uint16 Vflag : 1;    // 2
  Uint16 SendEnd : 1;  // 3
  Uint16 Sign1 : 1;    // 4    ????¦È????????
  Uint16 Sign2 : 1;    // 5    ???¦È????????
  Uint16 Openint : 1;  // 6    ????
  Uint16 Adfrist : 1;  // 7
  Uint16 Spdoff : 1;   // 8
  Uint16 Zero : 1;     // 9     ????????
  Uint16 Mode : 2;     // 10-11
  Uint16 Spdfirst : 1; // 12
  Uint16 Sign : 1;     // 13    ????????
  Uint16 Dir1 : 1;     // 14
  Uint16 Dir2 : 1;     // 15
};
union Flag_Reg {
  Uint16 all;
  struct Flag_Bits bit;
} FlagRegs;

/****************************??????????*******************************/

Uint16 Dir_Flag = 0; //dsp?§Ø???????
Uint16 dangle_e = 0; //???????????
Uint16 angle_e = 0;  //????????
int16 adl = 0;       //?????????????????
int16 angle_spd = 0; //?????????????
int16 spd_fd_q0 = 0; //??????

/*********************************ACI FOC_Variables****************************************/

int16 KT = 1896; //Q15 325 FOC Key Parameter 1624 1550  ???????
int16 KR = 238;  //5K_202  10K_101   Q16 41 FOC Key Parameter 204  194  ???????

int16 Iq_Max_FOC = 1536;  //Iq Max Value(Q12) For FOC (600mA)  Iq??????????????Q12??  375mA
int16 Iq_Min_FOC = -1536; //Iq Max Value(Q12) For FOC (-600mA)  Iq??????????§³???Q12??   -375mA

int16 Id_Max = 800; //Id Max Value(Q12)   Id??????????????Q12??   195.31mA
int16 Id_Min = 0;   //Id Min Value(Q12)     Id??????????§³???Q12??   -195.31mA

int16 Uq_Max = 2500;  //Uq Max Value(Q12) 3000    Uq?????????????Q12??    732mV
int16 Uq_Min = -2500; //Uq Min Value(Q12) 3000   Uq?????????§³???Q12??  732mV

int16 Ud_Max = 2500;  //Ud Max Value(Q12) 3000    Ud?????????????Q12??
int16 Ud_Min = -2500; //Ud Min Value(Q12) 3000   Ud?????????§³???Q12??

//??????????
int16 e_idk = 0, e2_idk = 0; //e_idk????¦Ä??????????e2_idk???¦Ä??????????

int16 e2_spd = 0;         //?????????
int16 e_spd = 0;          //?????????
int16 spd_kp = 0;         //????p????
int16 spd_ki = 0;         //????i????
int16 spd_fd = 0;         //????????Q12??
int16 spd_fd_display = 0; //use to speed display    //?????????
int16 spd_slope = 0;      //???§Ò??

Uint16 Shift_Flag = 0;      //?
Uint16 Shift_Flag_Tmp1 = 0; //New dir ?
Uint16 Shift_Flag_Tmp2 = 0; //Old dir ?

int32 id_kp = 0, id_ki = 0, id_kc = 0; //id_kc?????????
int32 iq_kp = 0, iq_ki = 0, iq_kc = 0;
int16 uout_d = 0, uout_q = 0; //???????????????
int16 e_id = 0, e_iq = 0;     //???d????????     ???q????????

Uint16 asrloop_flag = 0; //???????????¦Ë

int16 iarfa_fd = 0, ibeita_fd = 0; //????alpha??beta?????
int16 id_fd = 0, iq_fd = 0;        //d????q??????????

int16 Uarfa = 0, Ubeita = 0; //???????????????
int16 idk = 0;               //??????????
int32 UPI = 0;               //???????

int16 iq_given = 0, id_given = 0; //q??d??????????
int16 Fs = 0;                     //  ???????/???????

Uint16 index1 = 0;
Uint16 index2 = 0;               //
int16 ia_fd = 0, ib_fd = 0;      // a phase????????     b phase????????
Uint16 ia[6] = {0}, ib[6] = {0}; //?›¥ADC????????6??????

/********************************SVPWM_Variables**********************************/

int16 ind = 0;    //???????
int16 sector = 0; //??????

/************************************Use_For_Test***********************************/

//signed int a_test[512];
//signed int a_test_p=0;
//signed int z_test = 0;

#define MOTOR_PAIRS_OF_POLES_NUM 2
#define PWM_F_HZ 5000L
#define FOC_F_HZ PWM_F_HZ

typedef struct
{
  _iq22 angSpdEle;            // ??????
  _iq22 angSpdMec;            // ??§Ö?????
  _iq nMec;                   // ??§Ö???
  _iq22 deltaAngElePerPeriod; // ??????????? = angSpdEle / FOC_F_HZ
} speed_t;

typedef struct
{
  _iq Kp;
  _iq Ki;
  _iq OutMax;
  _iq OutMin;
} pid_param_t;

typedef struct
{
  _iq SetVal;
  _iq FdVal;
  _iq errLast;
  _iq Out;
} pid_state_t;

// ???
_iq uM = 0;
_iq uT = 0;
_iq uTCtrlVal = 0;

// ???
_iq22 angleEle = 0;

// ?????
speed_t spdState;             // ?????
speed_t spdSetVal = {0};      // ???????????Ú…?
speed_t spdSetValFinal = {0}; // ????Ú…???
_iq22 omegaS = 0;

// ??????
_iq22 angAccEle = 0;               // ??????
_iq22 deltaAngSpdElePerPeriod = 0; // ????????????? = angularAccEle / FOC_F_HZ

pid_param_t spdPidParam = {0};
pid_state_t spdPidState = {0};
/************************************Function***************************************/

void SetAngSpdEle(speed_t *s, _iq22 SetVal);
void SetNmec(speed_t* s, _iq nMec);
void SetAngAccEle(_iq22 SetVal);
void FOC(void);
void clk_pak(void); //park?ÈÎ????
void asrloop(void); //?????????????
void acrloop(void); //???????????????
void field(void);   //???????????????
Uint16 U_DIV_CAL(Uint32 udividend, Uint16 udivisor);
int16 DIV_CAL(int32 dividend, int16 divisor);
void Motor_Start(void);                   //????????
void Ad(void);                            //ADc????????
void Spd_Test(void);                      //?????????
void spd_reverse(void);                   //?????????
void Vector_Svpwm(void);                  //?????????
void Array_Order(Uint16 *a, Uint16 i);    //????
unsigned int fixed_sqrt(unsigned long a); //??????????

/*************************************Main Function****************************************/
//// Select the global Q value to use:
////#define GLOBAL_Q    24
//
//// Include The Following Definition Files:
long GlobalQ = GLOBAL_Q; // Used for legacy GEL & Graph Debug.
//
//// Specify the data logging size:
//#define DATA_LOG_SIZE   64
//
//// Define constants used:
//#define X_GAIN          1.0
//#define X_FREQ          1.0
//#define Y_GAIN          1.0
//#define Y_FREQ          1.0
//#define PI2             1.570796327
//#define PI              3.14159265359
//#define STEP_X_SIZE     0.314159265
//#define STEP_Y_SIZE     0.314159265
//
//// Allocate data log buffers:
//struct  DATA_LOG_C {
//    _iq   Xwaveform[DATA_LOG_SIZE];
//    _iq   Ywaveform[DATA_LOG_SIZE];
//    long  Phase[DATA_LOG_SIZE];
//    _iq   Mag[DATA_LOG_SIZE];
//    _iq   Exp[DATA_LOG_SIZE];
//    _iq   Asin[DATA_LOG_SIZE];
//} Dlog;
//
//// Define waveform global variables:
//struct  STEP {
//    _iq   Xsize;
//    _iq   Ysize;
//    _iq   Yoffset;
//    _iq   X;
//    _iq   Y;
//    _iq   GainX;
//    _iq   GainY;
//    _iq   FreqX;
//    _iq   FreqY;
//} Step;

void main(void)
{
  InitSysCtrl();
  InitEv();
  InitPieCtrl();
  InitPieVectTable();
  InitGpio();
  InitAdc();
  DINT;
  IER = 0x0000;
  IFR &= 0xFFFF; //???????

  //  EALLOW; //This is needed to write to EALLOW protected registers
  //  PieVectTable.T1UFINT = &T1UFINT_ISR; //T1 underflow interruput address  T1????????§Ø?
  //  PieVectTable.T3UFINT = &T3UFINT_ISR; //T3 underflow interrupt address  T3????????§Ø?
  //  EDIS; //This is needed to disable write to EALLOW protected registers

  PieCtrlRegs.PIEIER4.bit.INTx6 = 1; //T3 underflow interrupt enable  EVB
  PieCtrlRegs.PIEIER2.bit.INTx6 = 1; //T1 underflow interrupt enable  EVA

  IER |= M_INT2; // Enable CPU Interrupt 2 t1uf_int()
  IER |= M_INT4; // Enable CPU Interrupt 4 t3uf_int()

  //Enable global Interrupts and higher priority real-time debug events:
  EINT; // enable global interrupt
  ERTM; // Enable Global realtime interrupt DBGM ?

  //Set Speed
  SetNmec(&spdSetValFinal, _IQ(900));
  // SetAngSpdEle(&spdSetValFinal, _IQ22(2 * M_PI * 15));

  // Set Acc
  SetAngAccEle(_IQ22(10 * 2 * M_PI / 2));

  //ASR PI Parameter      ????????????
  spdPidParam.Kp = _IQ(.7*V_RMAX / (1500));
  spdPidParam.Ki = _IQ(0.002 * 30);
  spdPidParam.OutMax = _IQ(179);
  spdPidParam.OutMin = _IQ(-179);

 

  Motor_Start();

  //  unsigned int  i;
  //
  //  _iq tempX, tempY, tempP, tempM, tempMmax;
  ////  char buffer[20];
  //
  //  int *WatchdogWDCR = (void *) 0x7029;
  //
  //  // Disable the watchdog:
  //  asm(" EALLOW ");
  //  *WatchdogWDCR = 0x0068;
  //  asm(" EDIS ");*
  //
  //  Step.Xsize = _IQ(STEP_X_SIZE);
  //  Step.Ysize = _IQ(STEP_Y_SIZE);
  //  Step.Yoffset = 0;
  //  Step.X = 0;
  //  Step.Y = Step.Yoffset;
  //
  //  // Fill the buffers with some initial value
  //  for(i=0; i < DATA_LOG_SIZE; i++)
  //  {
  //      Dlog.Xwaveform[i] = _IQ(0.0);
  //      Dlog.Ywaveform[i] = _IQ(0.0);
  //      Dlog.Mag[i] = _IQ(0.0);
  //      Dlog.Phase[i] = _IQ(0.0);
  //      Dlog.Exp[i] = _IQ(0.0);
  //  }
  //
  //  Step.GainX = _IQ(X_GAIN);
  //  Step.FreqX = _IQ(X_FREQ);
  //  Step.GainY = _IQ(Y_GAIN);
  //  Step.FreqY = _IQ(Y_FREQ);
  //
  //  // Calculate maximum magnitude value:
  //  tempMmax = _IQmag(Step.GainX, Step.GainY);
  //
  //  for(i=0; i < DATA_LOG_SIZE; i++)
  //  {
  //      // Calculate waveforms:
  //      Step.X = Step.X + _IQmpy(Step.Xsize, Step.FreqX);
  //      if( Step.X > _IQ(2*PI) )
  //          Step.X -= _IQ(2*PI);
  //
  //      Step.Y = Step.Y + _IQmpy(Step.Ysize, Step.FreqY);
  //      if( Step.Y > _IQ(2*PI) )
  //          Step.Y -= _IQ(2*PI);
  //
  //      Dlog.Xwaveform[i] = tempX = _IQmpy(_IQsin(Step.X), Step.GainX);
  //      Dlog.Ywaveform[i] = tempY = _IQmpy(_IQabs(_IQsin(Step.Y)), Step.GainY);
  //
  //      // Calculate normalized magnitude:
  //      //
  //      // Mag = sqrt(X^2 + Y^2)/sqrt(GainX^2 + GainY^2);
  //      tempM = _IQmag(tempX, tempY);
  //      Dlog.Mag[i] = _IQdiv(tempM, tempMmax);
  //
  //      // Calculate normalized phase:
  //      //
  //      // Phase = (long) (atan2PU(X,Y) * 360);
  //      tempP = _IQatan2PU(tempY,tempX);
  //      Dlog.Phase[i] = _IQmpyI32int(tempP, 360);
  //
  //      // Use the exp function
  //      Dlog.Exp[i] = _IQexp(_IQmpy(_IQ(.075L),_IQ(i)));
  //
  //      // Use the asin function
  //      Dlog.Asin[i] = _IQasin(Dlog.Xwaveform[i]);
  //  }
  while (1)
  {
    ;
  } //End of main loop
} //End of main

/*********************************FOC********************************/
void SetAngSpdEle(speed_t *s, _iq22 SetVal)
{
  s->angSpdEle = SetVal;
  s->angSpdMec = _IQ22div(SetVal, _IQ22(MOTOR_PAIRS_OF_POLES_NUM));
  s->nMec = _IQdiv(_IQ22toIQ(s->angSpdMec) * 60, _IQ(2 * M_PI));
  s->deltaAngElePerPeriod = SetVal / PWM_F_HZ;
  //divide 2 times to avoid exceeding the range of IQ22 format
  //  deltaTmp = _IQ22div(SetVal, _IQ(50));
  //  s->deltaAngElePerPeriod = _IQ22div(deltaTmp, _IQ(100));
}

void SetNmec(speed_t* s, _iq nMec)
{
  _iq22 AngSpdEle = _IQtoIQ22(_IQmpy(_IQ(2*M_PI),nMec * MOTOR_PAIRS_OF_POLES_NUM / 60));
  SetAngSpdEle(s, AngSpdEle);
}

void SetAngAccEle(_iq22 SetVal)
{
  angAccEle = SetVal;
  //divide 5000
  deltaAngSpdElePerPeriod = SetVal / PWM_F_HZ;
  //divide 2 times to avoid exceeding the range of IQ22 format
  //  deltaTmp = _IQ22div(SetVal, _IQ(50));
  //  deltaAngSpdElePerPeriod = _IQ22div(deltaTmp, _IQ(100));
}

/**
 * @brief  ???§Ò??????
 * @param  None
 * @retval ??????????
 */
_iq22 AngSpdEleSlope(_iq22 finalAngSpd)
{
  static _iq22 spdSlope = 0;
  /*************?????????§Ò??**************/
  if (spdSlope < (finalAngSpd - deltaAngSpdElePerPeriod))
  {
    spdSlope += deltaAngSpdElePerPeriod;
  }
  else if (spdSlope > (finalAngSpd + deltaAngSpdElePerPeriod))
  {
    spdSlope -= deltaAngSpdElePerPeriod;
  }
  else
  {
    spdSlope = finalAngSpd;
  }
  return spdSlope;
}

_iq SpdPICtrl(pid_state_t *pidState, pid_param_t *pidParam)
{
  _iq err = pidState->SetVal - pidState->FdVal;
  pidState->Out += _IQmpy(pidParam->Kp, (err - pidState->errLast));
  pidState->errLast = err;
  err = _IQsat(err, _IQ(5), _IQ(-5));
  pidState->Out += _IQmpy(pidParam->Ki, err);
  pidState->Out = _IQsat(pidState->Out, pidParam->OutMax, pidParam->OutMin);
  return pidState->Out;
}
_iq22 deltaAngElePerPeriod_S = _IQ22(16.755/5000);

void FOC(void)
{
//  Ad();
  SetAngSpdEle(&spdSetVal, AngSpdEleSlope(spdSetValFinal.angSpdEle));
   angleEle += spdSetVal.deltaAngElePerPeriod + deltaAngElePerPeriod_S;
// angleEle += _IQ22mpy(spdState.deltaAngElePerPeriod + , _IQ22(1.05));
//  angleEle += _IQ22mpy(spdState.deltaAngElePerPeriod, _IQ22(1.05));
  utils_norm_angle_rad_Q22(&angleEle);
  uT = _IQmpy(_IQ(V_RMAX / (2 * M_PI * 50)), _IQ22toIQ(spdSetVal.angSpdEle));
  SvpwmCommutation(uM, _IQmpy(uT+ uTCtrlVal, _IQ(0.707)), angleEle);
//  SvpwmCommutation(uM, _IQmpy(uTCtrlVal, _IQ(0.707)), angleEle);
  static int i1 = 0;
  if(i1++ == 9)
  {
      i1 = 0;
      spdPidState.SetVal = spdSetVal.nMec;
      spdPidState.FdVal = spdState.nMec;
      uTCtrlVal = SpdPICtrl(&spdPidState, &spdPidParam);
  }
//  clk_pak();
//  field();
  //  afrloop();
  //  if( asrloop_flag == 0 )
  //  {
  //    asrloop_flag++;
  //  }
  //  else if( asrloop_flag == 4 )
  //  {
  //    Shift_Flag_Tmp2 = Shift_Flag_Tmp1;
  //    asrloop();
  //    asrloop_flag++;
  //  }
  //  else if( asrloop_flag >= 4 ) //2ms? 9  4--->1ms
  //  {
  //    asrloop_flag = 0;
  //  }
  //  else
  //  {
  //    asrloop_flag++;
  //  }
  //  acrloop();
  //  Vector_Svpwm(); //SVPWM output
}

//????????????
//??????????????ADC??????
void Ad(void)
{
  Uint16 Iab_i = 0; //???????

  for (Iab_i = 0; Iab_i <= 5; Iab_i++)
  {
    ia[Iab_i] = AdcRegs.ADCRESULT0 >> 4; //????????Ia
    ib[Iab_i] = AdcRegs.ADCRESULT1 >> 4; //????????Ib
    Array_Order(ia, Iab_i);
    Array_Order(ib, Iab_i); //???????6???????????????§³??????›¥?????????????????
  }
  //???????????6???????§Ö???????§³??????4???????????
  ia_fd = ia[2] + ia[3] + ia[4] + ia[5];
  ib_fd = ib[2] + ib[3] + ib[4] + ib[5];
  ia_fd = ia_fd >> 2;
  ib_fd = ib_fd >> 2; //?????????§³??????????????4?????????

  ia_fd = ia_fd - 2047; //Modify    ??????§µ???????????2047??¦Ì???????????
  ib_fd = ib_fd - 2047; //Modify     ???????????? ia_fd ?? ib_fd
}

/********************************Clarke********************************/
//clarke--park?ÈÎ

void clk_pak()
{

  //CLARKE?ÈÎ?????
  //i_alpha = (2/3)*(Ia -(1/2)*Ib - (1/2)*Ic)
  //i_beta = (2/3)*(0*Ia + (sqrt(3)/2)*Ib - (sqrt(3)/2)*Ic)
  //??Ic = -Ia - Ib????????????§µ?????????????
  //i_alpha = Ia    i_beta = ((1/sqrt(3))*Ia + (1/sqrt(3))*Ib)
  iarfa_fd = ia_fd; //iarfa=ia
  ibeita_fd = (ib_fd * 2 + ia_fd);
  ibeita_fd = ((int32)ibeita_fd * 2365) >> 12; //i_beta???????????????

  //SIN table
  index1 = index & 0x01FF;           //sin???????
  angle_sin = SINTAB[index1];        //SIN value
  index2 = (index + 0x080) & 0x01FF; //+90   cos???????
  angle_cos = SINTAB[index2];        //cos value

  //PARK?????
  //Id = angle_cos * i_alpha + angle_sin * i_beta
  //Iq = -angle_sin * i_alpha + angle_cos * i_beta
  long1_tmp = ((int32)iarfa_fd * angle_cos);
  long2_tmp = ((int32)ibeita_fd * angle_sin);
  id_fd = (long1_tmp + long2_tmp) >> 15;
  long1_tmp = ((int32)iarfa_fd * angle_sin);
  long2_tmp = ((int32)ibeita_fd * angle_cos);
  iq_fd = (long2_tmp - long1_tmp) >> 15;
}

/**********************************Speed test************************************/
//????????????EVB??T3???????????????????§Ø?????????????????????????????

//Dir_Flag
//spdState
void Spd_Test()
{
  //  static int16 MidFilter[3] = {0, 0, 0}; //???????
  //  static Uint16 zy = 0;                  //???????

  Dir_Flag = EvaRegs.GPTCONA.bit.T2STAT; //EVA??????2??¦Ë??0???B???A??1???A???B
  angle_spd = EvaRegs.T2CNT - adl;       //adl????¦Ë????-T2CNT??????????=?????????????

  //?§Ø????????§µ????????
  if (angle_spd == 0) //If feedback speed = 0,clear
  {
    spd_fd_q0 = 0; //??????????????????0  q0
    spd_fd = 0;    //Q12???
    spd_fd_display = 0;
  }
  else
  {                      //?????????Ú…??????????????????????????????????¡¤???????????????????????
    adl = EvaRegs.T2CNT; //??????????adl???????????????¦Ï?
    if (Dir_Flag == 1)   //?????????angle_spd>0??????angle_spd<0?????T2CNT?????????????  ??????????????
    {
      if (angle_spd < 0)
      {
        angle_spd = 8192 + angle_spd;
      } //??angle_spd????§µ??
    }
    else if (Dir_Flag == 0) //??????(?angle_spd<0)????angle_spd>0?????T2CNT?????????????
    {
      if (angle_spd > 0)
      {
        angle_spd = angle_spd - 8192;
      } //??angle_spd????§µ??
    }

    //??????????MidFilter[2]?§Õ›¥??3???????§Ö??§Þ?????????????????????¦Ã???????¨¢?
    // MidFilter[zy] = angle_spd; //??????????›¥
    // zy++;
    // if (zy >= 3)
    // {
    //   zy = 0;
    // }
    // if (MidFilter[0] > MidFilter[1])
    // {
    //   if (MidFilter[0] > MidFilter[2])
    //   {
    //     if (MidFilter[1] > MidFilter[2])
    //       angle_spd = MidFilter[1];
    //     else
    //       angle_spd = MidFilter[2];
    //   }
    //   else
    //   {
    //     angle_spd = MidFilter[0];
    //   }
    // }
    // else
    // {
    //   if (MidFilter[0] > MidFilter[2])
    //   {
    //     angle_spd = MidFilter[0];
    //   }
    //   else
    //   {
    //     if (MidFilter[1] > MidFilter[2])
    //       angle_spd = MidFilter[2];
    //     else
    //       angle_spd = MidFilter[1];
    //   }
    // }

    SetAngSpdEle(&spdState, _IQ22mpyI32(_IQ22((2 * MOTOR_PAIRS_OF_POLES_NUM * M_PI) / 8192 * 500),8 * angle_spd));

    //??????????????????????????????    ????????????????¦Ì?????????????????????
    spd_fd = ((int32)angle_spd * 5119) >> 9;     //Q12 (1ms)8 5119
    spd_fd_q0 = ((int32)angle_spd * 3750) >> 10; //Q0 (1ms)9 3750
  }
}

/*****************************FOC Field******************************/
//????????????
void field()
{
  Uint16 absabs = 0;
  int16 iq_fd_i = 0, idk_i = 0, iq_idk_i = 0;
  Uint16 shift_i = 0;
  // static int16 idk1 = 0;
  // long1_tmp = ((int32)KR * (id_fd - idk1)); //KR?Q16
  // u16_tmp1 = long1_tmp >> 16;               //Q0
  // idk1 = u16_tmp1 + idk1;
  //
  //
  //  if (idk1 <= 10) //????????????????????????Q12????0~4095??
  //  {
  //    idk = 10;
  //  }
  //  else if (idk1 > 4095)
  //  {
  //    idk = 4095;
  //  }
  //  else
  //  {
  //    idk = idk1;
  //  }
  static _iq idk1 = 0;
  //KR = 238
  //KR / 2^16 = 0.003631591796875
  idk1 += _IQmpyIQX(_IQ30(0.003631591796875), 30, (_IQ(id_fd) - idk1), QG);
  idk = _IQint(idk1);
  if (idk1 <= _IQ(10))
  {
    idk1 = _IQ(10);
  }
  //u16_tmp2 = iq_fd / idk /8 * KT = 237 * iq_fd  / idk
  //KT =1896   1896 / 8 = 237
  omegaS = _IQ22div(237 * iq_fd, idk1);
  //  if (iq_fd == 0)
  //  {
  //    u16_tmp2 = 0;
  //  }
  //  else
  //  {
  //    absabs = abs(iq_fd); //|iq|
  //                         //q15+12;
  //    while (absabs <= 2047)
  //    {
  //      absabs = absabs << 1;
  //      iq_fd_i++; //iq_fd_i(MAX)=11;(MIN)=0;
  //    }
  //    long2_tmp = (int32)4096 * absabs;
  //    u16_tmp1 = idk;
  //    while (u16_tmp1 <= 2047)
  //    {
  //      u16_tmp1 = u16_tmp1 << 1;
  //      idk_i++; //idk_i(MAX)=8;MIN=0;
  //    }
  //    u16_tmp1 = DIV_CAL(long2_tmp, u16_tmp1); //(iq_fd*2^12)/idk
  //    iq_idk_i = iq_fd_i - idk_i;
  //    if (iq_idk_i > 0)
  //    {
  //      shift_i = iq_idk_i;
  //      long2_tmp = ((int32)u16_tmp1) >> shift_i;
  //    }
  //    else
  //    {
  //      shift_i = -iq_idk_i;
  //      long2_tmp = ((int32)u16_tmp1) << shift_i;
  //    }
  //
  //    shift_i = 0;
  //    while (long2_tmp > 32767)
  //    {
  //      long2_tmp = long2_tmp >> 1;
  //      shift_i++;
  //    }
  //    u16_tmp1 = long2_tmp;
  //    shift_i = 15 - shift_i;
  //    long2_tmp = ((int32)u16_tmp1 * KT) >> shift_i;
  //    if (long2_tmp > 32767)
  //      u16_tmp2 = 32767;
  //    else
  //      u16_tmp2 = long2_tmp;
  //    if (iq_fd < 0)
  //      u16_tmp2 = -u16_tmp2;
  //  }

  Fs = spd_fd + u16_tmp2;

  if (Fs > 4095)
    Fs = 4095;
  else if (Fs < -4095)
    Fs = -4095;
  absabs = abs(Fs);

  dangle_e = ((int32)absabs * 655) >> 12; //5K_655  10K_328

  if (Fs >= 0)
  {
    angle_e = angle_e + dangle_e;
  }
  else
  {
    angle_e = angle_e - dangle_e;
  }
  index = (angle_e) >> 7;
}

/********************************Unsigned division***********************************/

Uint16 U_DIV_CAL(Uint32 udividend, Uint16 udivisor) //????????????
{
  Uint16 i_div = 0;
  Uint16 ures_div = 0;
  int32 tmp1 = 0;
  int32 tmp2 = 0;
  int32 tmp4 = 0;
  if (udivisor != 0)
  {
    tmp4 = udivisor;
    tmp4 = tmp4 << 15;
    tmp1 = udividend;
    for (i_div = 0; i_div <= 15; i_div++)
    {
      tmp2 = tmp1 - tmp4;
      if (tmp2 > 0)
        tmp1 = (tmp2 << 1) + 1;
      else
        tmp1 = tmp1 << 1;
    }
    ures_div = tmp1;
  }
  return (ures_div);
}

/*********************************signed division**********************************/

int16 DIV_CAL(int32 dividend, int16 divisor) //?§Ù??????????
{
  Uint16 sign_div = 0;
  int16 res_div = 0;

  if (divisor != 0)
  {
    if (((dividend < 0) && (divisor > 0)) || ((dividend > 0) && (divisor < 0)))
      sign_div = 1;
    if (dividend < 0)
      res_div = U_DIV_CAL(-dividend, abs(divisor));
    else
      res_div = U_DIV_CAL(dividend, abs(divisor));
    if (sign_div == 1)
      res_div = -res_div;
  }
  else
    res_div = 0;
  return (res_div);
}

/*************************Min data input a[0],Max data input a[1]*************/

//???????????????§³??*a,*(a+1),*(a+i).???????§³???*a?§µ??????????*??a+1??.
//????:*a = 10,*(a+1) = 16,*(a+8) = 20.
//(1) *(a+8) = 20 > *(a+1) = 16,??*??a+8?? =  16,*(a+1) = 20??
//??2??*(a+8) = 16 > *a = 10,??*??a+8?? = 16,*a = 10
//?????????????????????????§³????????§³??›¥??a[0]?§µ????????›¥??a[1]?§³?
void Array_Order(Uint16 *a, Uint16 i)
{
  Uint16 b;
  if ((i == 1) && ((*(a + 1)) < (*a)))
  { //if?????????????*??a+1??)????????*(a)????????????
    b = (*a);
    (*a) = (*(a + 1));
    (*(a + 1)) = b;
  }
  else if (i > 1)
  {
    if ((*(a + i)) > (*(a + 1)))
    {
      b = (*(a + 1));
      (*(a + 1)) = (*(a + i));
      (*(a + i)) = b;
    }
    else if ((*(a + i)) < (*a))
    {
      b = (*a);
      (*a) = (*(a + i));
      (*(a + i)) = b;
    }
  }
}

/****************************32bit SQRT******************************/

//?????32¦Ë?????????Uint32 a ------->   Uint16 b
Uint16 fixed_sqrt(Uint32 a)
{
  Uint16 i;
  Uint16 b = 0;
  Uint32 c;
  for (i = 0; i < 16; i++)
  {
    c = ((Uint32)1 << ((15 - i) << 1)) + ((Uint32)b << (16 - i));
    if (c <= a)
    {
      a -= c;
      b += (1 << (15 - i));
    }
  }
  return b;
}

/*************************Motor start***************************/
//????????
//????????§µ????????????EVA????pwm???ÈÉ??????ACTRx?????????????pwm??????§ß????Ú…EVA
//EVB??????????
void Motor_Start()
{
  EvaRegs.COMCONA.bit.FCOMPOE = 1;      //??????????
  GpioDataRegs.GPFCLEAR.bit.GPIOF8 = 1; //program runing(low)
  GpioDataRegs.GPFSET.bit.GPIOF11 = 1;  //program stop(high)

  EvaRegs.ACTRA.all = 0x0FFF;    //?????????????
  EvaRegs.ACTRA.all = 0x0999;    //pwm 1 3 5 active low;pwm2 4 6 active high
  EvaRegs.T1CON.bit.TENABLE = 1; //?????1???????
  EvbRegs.T3CON.bit.TENABLE = 1; //?????3???????
}

/******************************AD Ia Ib*****************************/

/********************************ASR PI********************************/
//????????????
void asrloop()
{
  int16 spd = 0; //????????§Þ????

  spd = spd_given_q12; //???
  if (spd >= 0)        //???????????????¦Ë?1???????0.
  {
    Shift_Flag = 1;
  }
  else
  {
    Shift_Flag = 0;
  }
  Shift_Flag_Tmp1 = Shift_Flag;

  if ((spd_slope < spd) && (spd >= 0)) //spd_slope????
  {
    if (spd_slope == 0)
      spd_slope = 2;
    else if (spd_slope < 512)
      spd_slope = spd_slope * 2;
    else
      spd_slope = spd;
  }
  if ((spd_slope >= spd) && (spd >= 0))
  {
    spd_slope = spd;
  }

  if ((spd_slope > spd) && (spd <= 0)) //<
  {
    if (spd_slope == 0)
      spd_slope = -2;
    else if (spd_slope > -512)
      spd_slope = spd_slope * 2;
    else
      spd_slope = spd;
  }
  if ((spd_slope <= spd) && (spd <= 0)) //<
  {
    spd_slope = spd;
  }

  e_spd = spd_slope - spd_fd;

  long1_tmp = (int32)spd_kp * (e_spd - e2_spd);
  long2_tmp = (int32)spd_ki * e_spd;
  UPI = ((int32)long1_tmp + (int32)long2_tmp) >> 12;
  long1_tmp = UPI + iq_given;
  e2_spd = e_spd;
  if (long1_tmp > Iq_Max_FOC)
  {
    iq_given = Iq_Max_FOC;
  }
  else if (long1_tmp < Iq_Min_FOC)
  {
    iq_given = Iq_Min_FOC;
  }
  else
    iq_given = long1_tmp;
}

/***************************AFR PI***************************/
//??????????????
void afrloop()
{
  int16 idk_given = 300;
  int16 idk_kp = 20000, idk_ki = 50; //20000 50

  e_idk = idk_given - idk;
  long1_tmp = (int32)idk_kp * (e_idk - e2_idk);
  long2_tmp = (int32)idk_ki * e_idk;
  UPI = ((int32)long1_tmp + (int32)long2_tmp) >> 12;
  long1_tmp = UPI + id_given;
  e2_idk = e_idk;
  if (long1_tmp > Id_Max)
  {
    id_given = Id_Max;
  }
  else if (long1_tmp < Id_Min)
  {
    id_given = Id_Min;
  }
  else
    id_given = long1_tmp;
}

/******************************ACR PI*******************************/
//????????????????
int16 PI_Compter(int16 Rk_1, int16 Kp, int16 Ki, int16 Kc, int16 ek, int16 umax, int16 umin)
{
  int16 Rk = 0;
  int16 U = 0;
  int16 uk = 0;

  int32 Rk_Temp = 0;
  U = ((int32)Kp * ek) >> 12;
  U = U + Rk_1;
  if (U > umax)
    uk = umax;
  else if (U < umin)
    uk = umin;
  else
    uk = U;                    //U = Rk_1 + Kp * ek (Then determin the range of U)
  Rk = ((int32)Ki * ek) >> 12; //Ki * ek (1)
  uk = uk - U;
  uk = ((int32)Kc * uk) >> 12; //Kc * (uk-U) (2)
  Rk_Temp = Rk_1 + uk + Rk;
  if (Rk_Temp > umax)
    Rk = umax;
  else if (Rk_Temp < umin)
    Rk = umin;
  else
    Rk = Rk_Temp;
  return (Rk);
}

/***************************ACR****************************/
//?????????
void acrloop()
{
  int16 b0 = 0, b1 = 0, b2 = 0;
  Uint16 p = 0;
  Uint16 abs_uout_q;
  Uint16 abs_uout_d;

  e_iq = iq_given - iq_fd;
  uout_q = PI_Compter(uout_q, iq_kp, iq_ki, iq_kc, e_iq, Uq_Max, Uq_Min);

  e_id = id_given - id_fd;
  uout_d = PI_Compter(uout_d, id_kp, id_ki, id_kc, e_id, Ud_Max, Ud_Min);

  abs_uout_q = abs(uout_q);
  abs_uout_d = abs(uout_d);
  long1_tmp = (int32)abs_uout_d * abs_uout_d;
  long2_tmp = (int32)abs_uout_q * abs_uout_q;
  long3_tmp = long1_tmp + long2_tmp;

  if (long3_tmp > 16769025)
  {
    u16_tmp1 = fixed_sqrt(long3_tmp);
    //u16_tmp1 = sqrt(long3_tmp);
    u16_tmp2 = DIV_CAL(16773119, u16_tmp1);
    uout_q = ((int32)u16_tmp2 * uout_q) >> 12;
    uout_d = ((int32)u16_tmp2 * uout_d) >> 12;
  } //??

  //SIN table
  index1 = index & 0x01ff;
  angle_sin = SINTAB[index1];        //SIN value
  index2 = (index + 0x080) & 0x01ff; //+90
  angle_cos = SINTAB[index2];        //cos value

  //PARK -1
  long1_tmp = (int32)angle_cos * uout_d; //uarfa_given=
  long2_tmp = (int32)angle_sin * uout_q; //angle_cos*uout_d-angle_sin*uout_q
  uarfa_given = (long1_tmp - long2_tmp) >> 15;
  long1_tmp = (int32)angle_cos * uout_q; //ubeita_givangSpdElelong2_tmp = (int32)angle_sin*uout_d; //angle_cos*uout_q+angle_sin*uout_d
  ubeita_given = (long1_tmp + long2_tmp) >> 15;

  //Fan
  Uarfa = ((int32)uarfa_given * K2) >> 15;
  Ubeita = ubeita_given >> 1;
  b0 = ubeita_given;
  b1 = Uarfa - Ubeita;
  b2 = -Uarfa - Ubeita;
  if (b0 > 0)
  {
    b0 = 1;
  }
  else
  {
    b0 = 0;
  }
  if (b1 > 0)
  {
    b1 = 1;
  }
  else
  {
    b1 = 0;
  }
  if (b2 > 0)
  {
    b2 = 1;
  }
  else
  {
    b2 = 0;
  }
  p = 4 * b2 + 2 * b1 + b0;
  switch (p)
  {
  case 1:
    sector = 1;
    break;
  case 2:
    sector = 5;
    break;
  case 3:
    sector = 0;
    break;
  case 4:
    sector = 3;
    break;
  case 5:
    sector = 2;
    break;
  case 6:
    sector = 4;
    break;
  default:
    break;
  }
}

/*****************************Vector******************************/

void Vector_Svpwm()
{
  Uint16 svm_cmpr1 = 0;
  Uint16 svm_cmpr2 = 0;
  Uint16 svm_cmpr3 = 0;
  ind = sector * 4;
  long1_tmp = (int32)uarfa_given * DEC_TAB[ind];      //Q12
  long2_tmp = (int32)ubeita_given * DEC_TAB[ind + 1]; //Q12
  u16_tmp1 = (long1_tmp + long2_tmp) >> 14;
  long1_tmp = (int32)uarfa_given * DEC_TAB[ind + 2];  //Q12
  long2_tmp = (int32)ubeita_given * DEC_TAB[ind + 3]; //Q12
  u16_tmp2 = (long1_tmp + long2_tmp) >> 14;
  cmp1 = ((int32)t1per * u16_tmp1) >> 12;
  cmp2 = ((int32)t1per * u16_tmp2) >> 12;
  if (cmp1 < 0)
  {
    cmp1 = 0;
  }
  if (cmp2 < 0)
  {
    cmp2 = 0;
  }
  cmp0 = (t1per - cmp1 - cmp2) >> 1;
  if (cmp0 < 0)
  {
    cmp0 = 0;
  }
  if ((cmp1 + cmp2) > t1per)
  {
    if (cmp1 >= cmp2)
    {
      if (cmp1 > t1per)
        cmp1 = t1per;
      cmp2 = t1per - cmp1;
    }
    else
    {
      if (cmp2 > t1per)
        cmp2 = t1per;
      cmp1 = t1per - cmp2;
    }
    cmp0 = 0;
  }

  switch (sector)
  {
  case 0:
    svm_cmpr1 = cmp0, svm_cmpr2 = cmp0 + cmp1, svm_cmpr3 = cmp0 + cmp1 + cmp2;
    break;
  case 1:
    svm_cmpr1 = cmp0 + cmp2, svm_cmpr2 = cmp0, svm_cmpr3 = cmp0 + cmp1 + cmp2;
    break;
  case 2:
    svm_cmpr1 = cmp0 + cmp1 + cmp2, svm_cmpr2 = cmp0, svm_cmpr3 = cmp0 + cmp1;
    break;
  case 3:
    svm_cmpr1 = cmp0 + cmp1 + cmp2, svm_cmpr2 = cmp0 + cmp2, svm_cmpr3 = cmp0;
    break;
  case 4:
    svm_cmpr1 = cmp0 + cmp1, svm_cmpr2 = cmp0 + cmp1 + cmp2, svm_cmpr3 = cmp0;
    break;
  case 5:
    svm_cmpr1 = cmp0, svm_cmpr2 = cmp0 + cmp1 + cmp2, svm_cmpr3 = cmp0 + cmp2;
    break;
  default:
    break;
  }

  EvaRegs.CMPR1 = svm_cmpr1;
  EvaRegs.CMPR2 = svm_cmpr2;
  EvaRegs.CMPR3 = svm_cmpr3;
}
