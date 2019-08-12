/* RoKi - Robot Kinetics library.
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_dc - motor model: DC motor
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

#define _rkc(p) ((rkMotorPrpDC *)p)

#define RK_MOTOR_DC_DEFAULT_K     2.58e-2
#define RK_MOTOR_DC_DEFAULT_R     0.42373
#define RK_MOTOR_DC_DEFAULT_MAX   24.0
#define RK_MOTOR_DC_DEFAULT_MIN   -24.0
#define RK_MOTOR_DC_DEFAULT_G     120
#define RK_MOTOR_DC_DEFAULT_I     1.65e-6
#define RK_MOTOR_DC_DEFAULT_IG    5.38e-6
#define RK_MOTOR_DC_DEFAULT_COMPK 100.0
#define RK_MOTOR_DC_DEFAULT_COMPL 1.0

static void _rkMotorInitDC(void *prp)
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

static void *_rkMotorAllocDC(void){ return zAlloc( rkMotorPrpDC, 1 ); }

static void _rkMotorCopyDC(void *src, void *dst){
  memcpy( dst, src, sizeof(rkMotorPrpDC) );
}

static void _rkMotorSetInputDC(void *prp, double *val){
  _rkc(prp)->e = zLimit( *val, _rkc(prp)->minvol, _rkc(prp)->maxvol );
}

static void _rkMotorInertiaDC(void *prp, double *val){
  *val = zSqr( _rkc(prp)->decratio ) * ( _rkc(prp)->inertia + _rkc(prp)->inertia_gear );
}

static void _rkMotorInputTrqDC(void *prp, double *val){
  *val = _rkc(prp)->admit * _rkc(prp)->decratio * _rkc(prp)->k * _rkc(prp)->e;
}

static void _rkMotorRegistanceDC(void *prp, double *dis, double *vel, double *val){
  *val = _rkc(prp)->admit * zSqr( _rkc(prp)->decratio * _rkc(prp)->k ) * (*vel);
}

static void _rkMotorDrivingTrqDC(void *prp, double *dis, double *vel, double *acc, double *val){
  double temp;
  _rkMotorInputTrqDC( prp, val );
  _rkMotorRegistanceDC( prp, dis, vel, &temp );
  *val -= temp;
  _rkMotorInertiaDC( prp, &temp );
  *val -= temp * (*acc);
}

static bool _rkMotorQueryFScanDC(FILE *fp, char *key, void *prp)
{
  if( strcmp( key, "motorconstant" ) == 0 )
    _rkc(prp)->k = zFDouble( fp );
  else if( strcmp( key, "admitance" ) == 0 )
    _rkc(prp)->admit = zFDouble( fp );
  else if( strcmp( key, "maxvoltage" ) == 0 )
    _rkc(prp)->maxvol = zFDouble( fp );
  else if( strcmp( key, "minvoltage" ) == 0 )
    _rkc(prp)->minvol = zFDouble( fp );
  else if( strcmp( key, "ratio" ) == 0 )
    _rkc(prp)->decratio = zFDouble( fp );
  else if( strcmp( key, "inertia" ) == 0 )
    _rkc(prp)->inertia = zFDouble( fp );
  else if( strcmp( key, "gearinertia" ) == 0 )
    _rkc(prp)->inertia_gear = zFDouble( fp );
  else if( strcmp( key, "compk" ) == 0 )
    _rkc(prp)->_comp_k = zFDouble( fp );
  else if( strcmp( key, "compl" ) == 0 )
    _rkc(prp)->_comp_l = zFDouble( fp );
  else
    return false;
  return true;
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

static void _rkMotorDCMotorConstantFPrint(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->k );
}
static void _rkMotorDCAdmittanceFPrint(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->admit );
}
static void _rkMotorDCMaxVoltageFPrint(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->maxvol );
}
static void _rkMotorDCMinVoltageFPrint(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->minvol );
}
static void _rkMotorDCGearRatioFPrint(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->decratio );
}
static void _rkMotorDCRotorInertiaFPrint(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->inertia );
}
static void _rkMotorDCGearInertiaFPrint(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->inertia_gear );
}
static void _rkMotorDCCompKFPrint(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->_comp_k );
}
static void _rkMotorDCCompLFPrint(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->_comp_l );
}

static ZTKPrp __ztk_prp_rkmotor_dc[] = {
  { "motorconstant", 1, _rkMotorDCMotorConstantFromZTK, _rkMotorDCMotorConstantFPrint },
  { "admittance", 1, _rkMotorDCAdmittanceFromZTK, _rkMotorDCAdmittanceFPrint },
  { "maxvoltage", 1, _rkMotorDCMaxVoltageFromZTK, _rkMotorDCMaxVoltageFPrint },
  { "minvoltage", 1, _rkMotorDCMinVoltageFromZTK, _rkMotorDCMinVoltageFPrint },
  { "gearratio", 1, _rkMotorDCGearRatioFromZTK, _rkMotorDCGearRatioFPrint },
  { "rotorinertia", 1, _rkMotorDCRotorInertiaFromZTK, _rkMotorDCRotorInertiaFPrint },
  { "gearinertia", 1, _rkMotorDCGearInertiaFromZTK, _rkMotorDCGearInertiaFPrint },
  { "compk", 1, _rkMotorDCCompKFromZTK, _rkMotorDCCompKFPrint },
  { "compl", 1, _rkMotorDCCompLFromZTK, _rkMotorDCCompLFPrint },
};

static void *_rkMotorFromZTKDC(void *prp, ZTK *ztk)
{
  return ZTKEncodeKey( prp, NULL, ztk, __ztk_prp_rkmotor_dc );
}

#if 0
static void _rkMotorFPrintDC(FILE *fp, void *prp)
{
  fprintf( fp, "motorconstant: %.10g\n", _rkc(prp)->k            );
  fprintf( fp, "admitance: %.10g\n"    , _rkc(prp)->admit        );
  fprintf( fp, "maxvoltage: %.10g\n"   , _rkc(prp)->maxvol       );
  fprintf( fp, "minvoltage: %.10g\n"   , _rkc(prp)->minvol       );
  fprintf( fp, "ratio: %.10g\n"        , _rkc(prp)->decratio     );
  fprintf( fp, "inertia: %.10g\n"      , _rkc(prp)->inertia      );
  fprintf( fp, "gearinertia: %.10g\n"  , _rkc(prp)->inertia_gear );
  fprintf( fp, "compk: %.10g\n"        , _rkc(prp)->_comp_k      );
  fprintf( fp, "compl: %.10g\n"        , _rkc(prp)->_comp_l      );
  fprintf( fp, "\n" );
}
#else
static void _rkMotorFPrintDC(FILE *fp, void *prp)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkmotor_dc );
}
#endif

rkMotorCom rk_motor_dc = {
  "dc",
  1,
  _rkMotorInitDC,
  _rkMotorAllocDC,
  _rkMotorCopyDC,
  _rkMotorSetInputDC,
  _rkMotorInertiaDC,
  _rkMotorInputTrqDC,
  _rkMotorRegistanceDC,
  _rkMotorDrivingTrqDC,
  _rkMotorQueryFScanDC,
  _rkMotorFromZTKDC,
  _rkMotorFPrintDC,
};

bool rkMotorRegZTKDC(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkmotor_dc );
}

#undef _rkc
