/* RoKi - Robot Kinetics library.
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_dc - motor model: geared DC motor
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

#define _rkp(ms) ((rkMotorDCPrp *)((rkMotorSpec *)(ms))->prp)

#define RK_MOTOR_DC_DEFAULT_K     2.58e-2
#define RK_MOTOR_DC_DEFAULT_R     0.42373
#define RK_MOTOR_DC_DEFAULT_MAX   24.0
#define RK_MOTOR_DC_DEFAULT_MIN   -24.0
#define RK_MOTOR_DC_DEFAULT_G     120
#define RK_MOTOR_DC_DEFAULT_I     1.65e-6
#define RK_MOTOR_DC_DEFAULT_IG    5.38e-6
#define RK_MOTOR_DC_DEFAULT_COMPK 100.0
#define RK_MOTOR_DC_DEFAULT_COMPL 1.0

static void _rkMotorSpecDCInitPrp(rkMotorSpec *ms){
  _rkp(ms)->motorconst   = RK_MOTOR_DC_DEFAULT_K;
  _rkp(ms)->admittance   = RK_MOTOR_DC_DEFAULT_R;
  _rkp(ms)->vol_max      = RK_MOTOR_DC_DEFAULT_MAX;
  _rkp(ms)->vol_min      = RK_MOTOR_DC_DEFAULT_MIN;
  _rkp(ms)->decratio     = RK_MOTOR_DC_DEFAULT_G;
  _rkp(ms)->inertia      = RK_MOTOR_DC_DEFAULT_I;
  _rkp(ms)->inertia_gear = RK_MOTOR_DC_DEFAULT_IG;
  _rkp(ms)->_comp_k      = RK_MOTOR_DC_DEFAULT_COMPK;
  _rkp(ms)->_comp_l      = RK_MOTOR_DC_DEFAULT_COMPL;
}

RK_MOTOR_COM_DEF_ALLOC_COPY_FUNC( DC )

/* ZTK */

#define RK_MOTOR_SPEC_DC_DEF_ZTKFUNC( name, member ) \
  static void *_rkMotorSpecDC##name##FromZTK(void *obj, int i, void *arg, ZTK *ztk){ \
    _rkp(obj)->member = ZTKDouble(ztk); \
    return obj; \
  } \
  static bool _rkMotorSpecDC##name##FPrintZTK(FILE *fp, int i, void *obj){ \
    fprintf( fp, "%.10g\n", _rkp(obj)->member ); \
    return true; \
  }

RK_MOTOR_SPEC_DC_DEF_ZTKFUNC( MotorConstant, motorconst )
RK_MOTOR_SPEC_DC_DEF_ZTKFUNC( Admittance, admittance )
RK_MOTOR_SPEC_DC_DEF_ZTKFUNC( MaxVoltage, vol_max )
RK_MOTOR_SPEC_DC_DEF_ZTKFUNC( MinVoltage, vol_min )
RK_MOTOR_SPEC_DC_DEF_ZTKFUNC( GearRatio, decratio )
RK_MOTOR_SPEC_DC_DEF_ZTKFUNC( RotorInertia, inertia )
RK_MOTOR_SPEC_DC_DEF_ZTKFUNC( GearInertia, inertia_gear )
RK_MOTOR_SPEC_DC_DEF_ZTKFUNC( CompK, _comp_k )
RK_MOTOR_SPEC_DC_DEF_ZTKFUNC( CompL, _comp_l )

static ZTKPrp __ztk_prp_rkmotor_dc[] = {
  { ZTK_KEY_ROKI_MOTOR_MOTORCONSTANT, 1, _rkMotorSpecDCMotorConstantFromZTK, _rkMotorSpecDCMotorConstantFPrintZTK },
  { ZTK_KEY_ROKI_MOTOR_ADMITTANCE,    1, _rkMotorSpecDCAdmittanceFromZTK, _rkMotorSpecDCAdmittanceFPrintZTK },
  { ZTK_KEY_ROKI_MOTOR_MAXVOLTAGE,    1, _rkMotorSpecDCMaxVoltageFromZTK, _rkMotorSpecDCMaxVoltageFPrintZTK },
  { ZTK_KEY_ROKI_MOTOR_MINVOLTAGE,    1, _rkMotorSpecDCMinVoltageFromZTK, _rkMotorSpecDCMinVoltageFPrintZTK },
  { ZTK_KEY_ROKI_MOTOR_GEARRATIO,     1, _rkMotorSpecDCGearRatioFromZTK, _rkMotorSpecDCGearRatioFPrintZTK },
  { ZTK_KEY_ROKI_MOTOR_ROTERINERTIA,  1, _rkMotorSpecDCRotorInertiaFromZTK, _rkMotorSpecDCRotorInertiaFPrintZTK },
  { ZTK_KEY_ROKI_MOTOR_GEARINERTIA,   1, _rkMotorSpecDCGearInertiaFromZTK, _rkMotorSpecDCGearInertiaFPrintZTK },
  { ZTK_KEY_ROKI_MOTOR_COMPK,         1, _rkMotorSpecDCCompKFromZTK, _rkMotorSpecDCCompKFPrintZTK },
  { ZTK_KEY_ROKI_MOTOR_COMPL,         1, _rkMotorSpecDCCompLFromZTK, _rkMotorSpecDCCompLFPrintZTK },
};

static rkMotorSpec *_rkMotorSpecDCFromZTK(rkMotorSpec *ms, ZTK *ztk)
{
  return (rkMotorSpec *)_ZTKEvalKey( ms, NULL, ztk, __ztk_prp_rkmotor_dc );
}

static void _rkMotorSpecDCFPrintZTK(FILE *fp, rkMotorSpec *ms)
{
  _ZTKPrpKeyFPrint( fp, ms, __ztk_prp_rkmotor_dc );
}

#undef _rkp

/* methods for motor instances */

#define _rkp(m) ((rkMotorDCPrp *)((rkMotor *)(m))->spec->prp)
#define _rks(m) ((rkMotorDCState *)((rkMotor *)(m))->state)

static void _rkMotorDCInitState(rkMotor *motor){
  _rks(motor)->e  = 0.0;
  _rks(motor)->tf = 0.0;
}

static void _rkMotorDCSetInput(rkMotor *motor, double *val){
  _rks(motor)->e = zLimit( *val, _rkp(motor)->vol_min, _rkp(motor)->vol_max );
}

static void _rkMotorDCInertia(rkMotor *motor, double *val){
  *val = zSqr( _rkp(motor)->decratio ) * ( _rkp(motor)->inertia + _rkp(motor)->inertia_gear );
}

static void _rkMotorDCInputTrq(rkMotor *motor, double *val){
  *val = _rkp(motor)->admittance * _rkp(motor)->decratio * _rkp(motor)->motorconst * _rks(motor)->e;
}

static void _rkMotorDCRegistance(rkMotor *motor, double *dis, double *vel, double *val){
  *val = _rkp(motor)->admittance * zSqr( _rkp(motor)->decratio * _rkp(motor)->motorconst ) * (*vel);
}

static void _rkMotorDCDrivingTrq(rkMotor *motor, double *dis, double *vel, double *acc, double *val){
  double temp;
  _rkMotorDCInputTrq( motor, val );
  _rkMotorDCRegistance( motor, dis, vel, &temp );
  *val -= temp;
  _rkMotorDCInertia( motor, &temp );
  *val -= temp * (*acc);
}

rkMotorCom rk_motor_dc = {
  "dc",
  1,
  _rkMotorSpecDCInitPrp,
  _rkMotorSpecDCAllocPrp,
  _rkMotorSpecDCCopyPrp,
  _rkMotorSpecDCFromZTK,
  _rkMotorSpecDCFPrintZTK,
  _rkMotorDCInitState,
  _rkMotorDCAllocState,
  _rkMotorDCCopyState,
  _rkMotorDCSetInput,
  _rkMotorDCInertia,
  _rkMotorDCInputTrq,
  _rkMotorDCRegistance,
  _rkMotorDCDrivingTrq,
};

#undef _rkp
#undef _rks
