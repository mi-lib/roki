/* RoKi - Robot Kinetics library.
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_trq - motor model: torque motor
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

#define _rkc(p) ((rkMotorPrpTrq *)p)

static void _rkMotorInitTrq(void *prp)
{
  _rkc(prp)->input = 0.0;
  _rkc(prp)->min = -HUGE_VAL;
  _rkc(prp)->max = HUGE_VAL;
}

static void *_rkMotorAllocTrq(void){ return zAlloc( rkMotorPrpTrq, 1 ); }

static void _rkMotorCopyTrq(void *src, void *dst){
  memcpy( dst, src, sizeof(rkMotorPrpTrq) );
}

static void _rkMotorSetInputTrq(void *prp, double *val){
  _rkc(prp)->input = zLimit( *val, _rkc(prp)->min, _rkc(prp)->max );
}

static void _rkMotorInertiaTrq(void *prp, double *val){
  *val = 0.0;
}

static void _rkMotorInputTrqTrq(void *prp, double *val){
  *val = _rkc(prp)->input;
}

static void _rkMotorRegistanceTrq(void *prp, double *dis, double *vel, double *val){
  *val = 0.0;
}

static void _rkMotorDrivingTrqTrq(void *prp, double *dis, double *vel, double *acc, double *val){
  _rkMotorInputTrqTrq( prp, val );
}

static bool _rkMotorQueryFScanTrq(FILE *fp, char *key, void *prp)
{
  if( strcmp( key, "max" ) == 0 )
    _rkc(prp)->max = zFDouble( fp );
  else if( strcmp( key, "min" ) == 0 )
    _rkc(prp)->min = zFDouble( fp );
  else
    return false;
  return true;
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

static void _rkMotorTrqMaxFPrint(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->max );
}
static void _rkMotorTrqMinFPrint(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->min );
}

static ZTKPrp __ztk_prp_rkmotor_trq[] = {
  { "max", 1, _rkMotorTrqMaxFromZTK, _rkMotorTrqMaxFPrint },
  { "min", 1, _rkMotorTrqMinFromZTK, _rkMotorTrqMinFPrint },
};

static void *_rkMotorFromZTKTrq(void *prp, ZTK *ztk)
{
  return ZTKEncodeKey( prp, NULL, ztk, __ztk_prp_rkmotor_trq );
}

#if 0
static void _rkMotorFPrintTrq(FILE *fp, void *prp)
{
  fprintf( fp, "\n" );
}
#else
static void _rkMotorFPrintTrq(FILE *fp, void *prp)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkmotor_trq );
}
#endif

rkMotorCom rk_motor_trq = {
  "trq",
  1,
  _rkMotorInitTrq,
  _rkMotorAllocTrq,
  _rkMotorCopyTrq,
  _rkMotorSetInputTrq,
  _rkMotorInertiaTrq,
  _rkMotorInputTrqTrq,
  _rkMotorRegistanceTrq,
  _rkMotorDrivingTrqTrq,
  _rkMotorQueryFScanTrq,
  _rkMotorFromZTKTrq,
  _rkMotorFPrintTrq,
};

bool rkMotorRegZTKTrq(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkmotor_trq );
}

#undef _rkc
