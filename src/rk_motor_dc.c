/* RoKi - Robot Kinetics library.
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_dc - motor model: DC motor
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

#define _rkc(p) ((rkMotorDCPrp *)p)

#define RK_MOTOR_DC_DEFAULT_K     2.58e-2
#define RK_MOTOR_DC_DEFAULT_R     0.42373
#define RK_MOTOR_DC_DEFAULT_MAX   24.0
#define RK_MOTOR_DC_DEFAULT_MIN   -24.0
#define RK_MOTOR_DC_DEFAULT_G     120
#define RK_MOTOR_DC_DEFAULT_I     1.65e-6
#define RK_MOTOR_DC_DEFAULT_IG    5.38e-6
#define RK_MOTOR_DC_DEFAULT_COMPK 100.0
#define RK_MOTOR_DC_DEFAULT_COMPL 1.0

static void _rkMotorDCInit(void *prp)
{
  _rkc(prp)->k            = RK_MOTOR_DC_DEFAULT_K;
  _rkc(prp)->admit        = RK_MOTOR_DC_DEFAULT_R;
  _rkc(prp)->maxvol       = RK_MOTOR_DC_DEFAULT_MAX;
  _rkc(prp)->minvol       = RK_MOTOR_DC_DEFAULT_MIN;
  _rkc(prp)->decratio     = RK_MOTOR_DC_DEFAULT_G;
  _rkc(prp)->inertia      = RK_MOTOR_DC_DEFAULT_I;
  _rkc(prp)->inertia_gear = RK_MOTOR_DC_DEFAULT_IG;
  _rkc(prp)->_comp_k = RK_MOTOR_DC_DEFAULT_COMPK;
  _rkc(prp)->_comp_l = RK_MOTOR_DC_DEFAULT_COMPL;
  _rkc(prp)->e  = 0.0;
  _rkc(prp)->tf = 0.0;
}

static void *_rkMotorDCAlloc(void){ return zAlloc( rkMotorDCPrp, 1 ); }

static void _rkMotorDCCopy(void *src, void *dst){
  memcpy( dst, src, sizeof(rkMotorDCPrp) );
}

static void _rkMotorDCSetInput(void *prp, double *val){
  _rkc(prp)->e = zLimit( *val, _rkc(prp)->minvol, _rkc(prp)->maxvol );
}

static void _rkMotorDCInertia(void *prp, double *val){
  *val = zSqr( _rkc(prp)->decratio ) * ( _rkc(prp)->inertia + _rkc(prp)->inertia_gear );
}

static void _rkMotorDCInputTrq(void *prp, double *val){
  *val = _rkc(prp)->admit * _rkc(prp)->decratio * _rkc(prp)->k * _rkc(prp)->e;
}

static void _rkMotorDCRegistance(void *prp, double *dis, double *vel, double *val){
  *val = _rkc(prp)->admit * zSqr( _rkc(prp)->decratio * _rkc(prp)->k ) * (*vel);
}

static void _rkMotorDCDrivingTrq(void *prp, double *dis, double *vel, double *acc, double *val){
  double temp;
  _rkMotorDCInputTrq( prp, val );
  _rkMotorDCRegistance( prp, dis, vel, &temp );
  *val -= temp;
  _rkMotorDCInertia( prp, &temp );
  *val -= temp * (*acc);
}

/* ZTK */

static void *_rkMotorDCMotorConstantFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->k = ZTKDouble(ztk);
  return prp;
}
static void *_rkMotorDCAdmittanceFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->admit = ZTKDouble(ztk);
  return prp;
}
static void *_rkMotorDCMaxVoltageFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->maxvol = ZTKDouble(ztk);
  return prp;
}
static void *_rkMotorDCMinVoltageFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->minvol = ZTKDouble(ztk);
  return prp;
}
static void *_rkMotorDCGearRatioFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->decratio = ZTKDouble(ztk);
  return prp;
}
static void *_rkMotorDCRotorInertiaFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->inertia = ZTKDouble(ztk);
  return prp;
}
static void *_rkMotorDCGearInertiaFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->inertia_gear = ZTKDouble(ztk);
  return prp;
}
static void *_rkMotorDCCompKFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->_comp_k = ZTKDouble(ztk);
  return prp;
}
static void *_rkMotorDCCompLFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->_comp_l = ZTKDouble(ztk);
  return prp;
}

static void _rkMotorDCMotorConstantFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->k );
}
static void _rkMotorDCAdmittanceFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->admit );
}
static void _rkMotorDCMaxVoltageFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->maxvol );
}
static void _rkMotorDCMinVoltageFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->minvol );
}
static void _rkMotorDCGearRatioFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->decratio );
}
static void _rkMotorDCRotorInertiaFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->inertia );
}
static void _rkMotorDCGearInertiaFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->inertia_gear );
}
static void _rkMotorDCCompKFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->_comp_k );
}
static void _rkMotorDCCompLFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->_comp_l );
}

static ZTKPrp __ztk_prp_rkmotor_dc[] = {
  { "motorconstant", 1, _rkMotorDCMotorConstantFromZTK, _rkMotorDCMotorConstantFPrintZTK },
  { "admittance", 1, _rkMotorDCAdmittanceFromZTK, _rkMotorDCAdmittanceFPrintZTK },
  { "maxvoltage", 1, _rkMotorDCMaxVoltageFromZTK, _rkMotorDCMaxVoltageFPrintZTK },
  { "minvoltage", 1, _rkMotorDCMinVoltageFromZTK, _rkMotorDCMinVoltageFPrintZTK },
  { "gearratio", 1, _rkMotorDCGearRatioFromZTK, _rkMotorDCGearRatioFPrintZTK },
  { "rotorinertia", 1, _rkMotorDCRotorInertiaFromZTK, _rkMotorDCRotorInertiaFPrintZTK },
  { "gearinertia", 1, _rkMotorDCGearInertiaFromZTK, _rkMotorDCGearInertiaFPrintZTK },
  { "compk", 1, _rkMotorDCCompKFromZTK, _rkMotorDCCompKFPrintZTK },
  { "compl", 1, _rkMotorDCCompLFromZTK, _rkMotorDCCompLFPrintZTK },
};

static bool _rkMotorDCRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkmotor_dc );
}

static void *_rkMotorDCFromZTK(void *prp, ZTK *ztk)
{
  return ZTKEvalKey( prp, NULL, ztk, __ztk_prp_rkmotor_dc );
}

static void _rkMotorDCFPrintZTK(FILE *fp, void *prp)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkmotor_dc );
}

rkMotorCom rk_motor_dc = {
  "dc",
  1,
  _rkMotorDCInit,
  _rkMotorDCAlloc,
  _rkMotorDCCopy,
  _rkMotorDCSetInput,
  _rkMotorDCInertia,
  _rkMotorDCInputTrq,
  _rkMotorDCRegistance,
  _rkMotorDCDrivingTrq,
  _rkMotorDCRegZTK,
  _rkMotorDCFromZTK,
  _rkMotorDCFPrintZTK,
};

#undef _rkc
