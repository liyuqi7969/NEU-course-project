#include "svpwm.h"

extern int16 SINTAB[512];
// #define ONE_BY_SQRT3    (0.57735026919f)
// #define TWO_BY_SQRT3    (2.0f * 0.57735026919f)
// #define SQRT3_BY_2      (0.86602540378f)

#define SQRT3           1.732050808

#define SQRT3_IQ       _IQ(SQRT3)  // 56755.840862416971154307361718464
//#define SQRT3_Q15       (int)(1.732050808f*(1<<15))
#define K_SQRT3_IQ         _IQ(PWM_N * SQRT3 / V_DC) //6000*SQRT3/24

// Return the sign of the argument. -1 if negative, 1 if zero or positive.
#define SIGN(x)     ((x < 0) ? -1 : 1)

// Squared
#define SQ(x)       ((x) * (x))

#define APHASE EvaRegs.CMPR1
#define BPHASE EvaRegs.CMPR2
#define CPHASE EvaRegs.CMPR3

#define DEAD_COMP 0
//vd vq 为相电压幅值
void SvpwmCommutation(_iq vd, _iq vq, _iq22 eleangle) //electrical degree
{
    _iq     v_alpha, v_beta;
    _iq     vr1, vr2, vr3;
	Uint32   flagN;
	int32     T1, T2;
	Uint32  TA, TB, TC;

#if DEAD_COMP != 0
#define PWM_PERIOD_US    200L
#define PWM_F_HZ_IQ      _IQ(1000000.0 / PWM_PERIOD_US)

	_iq idTarget = 0, iqTarget = vq;

	_iq mod_d = _IQdiv(vd , _IQ((2.0 / 3.0) * Udc));
	float mod_q = vq / ((2.0 / 3.0) * 24);
	
	const float deadTime = fabs(sqrt(SQ(iqTarget)+SQ(idTarget))) < .010f?0:.22;
	
	arm_sin_cos_f32(eleangle, &s, &c);
	float       mod_alpha = c * mod_d - s * mod_q;
	float       mod_beta  = c * mod_q + s * mod_d;
	// Deadtime compensation
	const float i_alpha_filter        = c * idTarget - s * iqTarget;
	const float i_beta_filter         = c * iqTarget + s * idTarget;
	const float ia_filter             = i_alpha_filter;
	const float ib_filter             = -0.5f * i_alpha_filter + SQRT3_BY_2 * i_beta_filter;
	const float ic_filter             = -0.5f * i_alpha_filter - SQRT3_BY_2 * i_beta_filter;
	const float mod_alpha_filter_sgn  = (2.0f / 3.0f) * SIGN(ia_filter) - (1.0f / 3.0f) * SIGN(ib_filter) - (1.0f / 3.0f) * SIGN(ic_filter);
	const float mod_beta_filter_sgn   = ONE_BY_SQRT3 * SIGN(ib_filter) - ONE_BY_SQRT3 * SIGN(ic_filter);
  const float mod_comp_fact         = deadTime * 0.000001f * PWM_F_HZ;
	const float mod_alpha_comp        = mod_alpha_filter_sgn * mod_comp_fact;
	const float mod_beta_comp         = mod_beta_filter_sgn * mod_comp_fact;

	// Apply compensation here so that 0 duty cycle has no glitches.
	v_alpha = (mod_alpha + mod_alpha_comp) * (2.0 / 3.0) * 24;
	v_beta  = (mod_beta + mod_beta_comp) * (2.0 / 3.0) * 24;

#else

	// arm_sin_cos_f32(eleangle, &sinVal, &cosVal);
  //SIN table
//  sinVal = SINTAB[eleangle & 0x01ff];        //SIN value
//  cosVal = SINTAB[(eleangle + 0x080) & 0x01ff];        //cos value
//	v_alpha >>= 15;
//	v_beta >>= 15;

  _iq sinVal, cosVal;
  sinVal = _IQsin(_IQ22toIQ(eleangle));
  cosVal = _IQcos(_IQ22toIQ(eleangle));
  v_alpha = _IQmpy(vd, cosVal) - _IQmpy(vq , sinVal);
  v_beta  = _IQmpy(vd, sinVal) + _IQmpy(vq , cosVal);

	//park逆变化
	// arm_inv_park_f32(vd, vq, &Valpha, &v_beta, arm_sin_f32(eleangle), arm_cos_f32(eleangle));

#endif

	vr1 = v_beta;
	vr2 = _IQdiv2(-v_beta + _IQmpy(SQRT3_IQ, v_alpha));
	vr3 = _IQdiv2(-v_beta - _IQmpy(SQRT3_IQ, v_alpha));

	/********************
	   判断矢量电压所在扇区
	 *********************/
	if (vr1 > 0)
		flagN = 0x01;
	else
		flagN = 0x00;
	if (vr2 > 0)
		flagN |= 0x02;
	if (vr3 > 0)
		flagN |= 0x04;

	switch (flagN)
	{
	case 3: { T1 = _IQint(_IQrmpy(vr2, K_SQRT3_IQ)); T2 = _IQint(_IQrmpy(vr1, K_SQRT3_IQ)); break; }    //u1,u2>0

	case 1: { T1 = _IQint(_IQrmpy(-vr2, K_SQRT3_IQ)); T2 = _IQint(_IQrmpy(-vr3, K_SQRT3_IQ)); break; }  //u1>0

	case 5: { T1 = _IQint(_IQrmpy(vr1, K_SQRT3_IQ)); T2 = _IQint(_IQrmpy(vr3, K_SQRT3_IQ)); break; }    //u1,u3>0

	case 4: { T1 = _IQint(_IQrmpy(-vr1, K_SQRT3_IQ)); T2 = _IQint(_IQrmpy(-vr2, K_SQRT3_IQ)); break; }  //u3>0

	case 6: { T1 = _IQint(_IQrmpy(vr3, K_SQRT3_IQ)); T2 = _IQint(_IQrmpy(vr2, K_SQRT3_IQ)); break; }    //u2,u3>0

	case 2: { T1 = _IQint(_IQrmpy(-vr3, K_SQRT3_IQ)); T2 = _IQint(_IQrmpy(-vr1, K_SQRT3_IQ)); break; }  //u2>0

	default: { T1 = 0u; T2 = 0u; break; }
	}

	if (T1 + T2 > PWM_N)
	{
	    _iq17 t1, t2;
		t1  = _IQ17(T1);
		t2  = _IQ17(T2);
		T1  = _IQ17int(_IQ17mpyI32(_IQ17div(t1, (t1 + t2)) , PWM_N));
		T2  = _IQ17int(_IQ17mpyI32(_IQ17div(t2, (t1 + t2)) , PWM_N));
	}

	TA  = (PWM_N - T1 - T2)>>1;
	TB  = (PWM_N + T1 - T2)>>1;
	TC  = (PWM_N + T1 + T2)>>1;

	switch (flagN)
	{
	case 3: { APHASE = TA; BPHASE = TB; CPHASE = TC; break; }

	case 1: { APHASE = TB; BPHASE = TA; CPHASE = TC; break; }

	case 5: { APHASE = TC; BPHASE = TA; CPHASE = TB; break; }

	case 4: { APHASE = TC; BPHASE = TB; CPHASE = TA; break; }

	case 6: { APHASE = TB; BPHASE = TC; CPHASE = TA; break; }

	case 2: { APHASE = TA; BPHASE = TC; CPHASE = TB; break; }

	default: { APHASE = PWM_N; BPHASE = PWM_N; CPHASE = PWM_N; break; }
	}
}
