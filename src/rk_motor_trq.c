/* RoKi - Robot Kinetics library.
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_trq - motor model: torque motor
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

#define _rkc(p) ((rkMotorTrqPrp *)p)

static void _rkMotorTrqInit(void *prp)
{
  _rkc(prp)->input = 0.0;
  _rkc(prp)->min = -HUGE_VAL;
  _rkc(prp)->max = HUGE_VAL;
}

static void *_rkMotorTrqAlloc(void){ return zAlloc( rkMotorTrqPrp, 1 ); }

static void _rkMotorTrqCopy(void *src, void *dst){
  memcpy( dst, src, sizeof(rkMotorTrqPrp) );
}

static void _rkMotorTrqSetInput(void *prp, double *val){
  _rkc(prp)->input = zLimit( *val, _rkc(prp)->min, _rkc(prp)->max );
}

static void _rkMotorTrqInertia(void *prp, double *val){
  *val = 0.0;
}

static void _rkMotorTrqInputTrq(void *prp, double *val){
  *val = _rkc(prp)->input;
}

static void _rkMotorTrqRegistance(void *prp, double *dis, double *vel, double *val){
  *val = 0.0;
}

static void _rkMotorTrqDrivingTrq(void *prp, double *dis, double *vel, double *acc, double *val){
  _rkMotorTrqInputTrq( prp, val );
}

/* ZTK */

static void *_rkMotorTrqMaxFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->max = ZTKDouble(ztk);
  return prp;
}
static void *_rkMotorTrqMinFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->min = ZTKDouble(ztk);
  return prp;
}

static void _rkMotorTrqMaxFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->max );
}
static void _rkMotorTrqMinFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->min );
}

static ZTKPrp __ztk_prp_rkmotor_trq[] = {
  { "max", 1, _rkMotorTrqMaxFromZTK, _rkMotorTrqMaxFPrintZTK },
  { "min", 1, _rkMotorTrqMinFromZTK, _rkMotorTrqMinFPrintZTK },
};

static bool _rkMotorTrqRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkmotor_trq );
}

static void *_rkMotorTrqFromZTK(void *prp, ZTK *ztk)
{
  return ZTKEvalKey( prp, NULL, ztk, __ztk_prp_rkmotor_trq );
}

static void _rkMotorTrqFPrintZTK(FILE *fp, void *prp)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkmotor_trq );
}

rkMotorCom rk_motor_trq = {
  "trq",
  1,
  _rkMotorTrqInit,
  _rkMotorTrqAlloc,
  _rkMotorTrqCopy,
  _rkMotorTrqSetInput,
  _rkMotorTrqInertia,
  _rkMotorTrqInputTrq,
  _rkMotorTrqRegistance,
  _rkMotorTrqDrivingTrq,
  _rkMotorTrqRegZTK,
  _rkMotorTrqFromZTK,
  _rkMotorTrqFPrintZTK,
};

#undef _rkc
