/* RoKi - Robot Kinetics library.
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_dc - motor model: DC motor
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

static void _rkMotorSetInputDC(void *prp, double *val);

static void _rkMotorInertiaDC(void *prp, double *val);
static void _rkMotorInputTrqDC(void *prp, double *val);
static void _rkMotorRegistanceDC(void *prp, double *dis, double *vel, double *val);
static void _rkMotorDrivingTrqDC(void *prp, double *dis, double *vel, double *acc, double *val);

static void _rkMotorStateCopyDC(void *src, void *dst);

static bool _rkMotorQueryFReadDC(FILE *fp, char *key, void *prp);
static void _rkMotorFWriteDC(FILE *fp, void *prp);

#define _rkc(p) ((rkMotorPrpDC *)p)

void _rkMotorSetInputDC(void *prp, double *val){
  _rkc(prp)->e = zLimit( *val, _rkc(prp)->minvol, _rkc(prp)->maxvol );
}

void _rkMotorInertiaDC(void *prp, double *val){
  *val = zSqr( _rkc(prp)->decratio ) * ( _rkc(prp)->inertia + _rkc(prp)->inertia_gear );
}

void _rkMotorInputTrqDC(void *prp, double *val){
  *val = _rkc(prp)->admit * _rkc(prp)->decratio * _rkc(prp)->k * _rkc(prp)->e;
}

void _rkMotorRegistanceDC(void *prp, double *dis, double *vel, double *val){
  *val = _rkc(prp)->admit * zSqr( _rkc(prp)->decratio * _rkc(prp)->k ) * (*vel);
}
void _rkMotorDrivingTrqDC(void *prp, double *dis, double *vel, double *acc, double *val){
  double temp;
  _rkMotorInputTrqDC( prp, val );
  _rkMotorRegistanceDC( prp, dis, vel, &temp );
  *val -= temp;
  _rkMotorInertiaDC( prp, &temp );
  *val -= temp * (*acc);
}

void _rkMotorStateCopyDC(void *src, void *dst){
  memcpy(dst, src, sizeof(rkMotorPrpDC));
}

bool _rkMotorQueryFReadDC(FILE *fp, char *key, void *prp)
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

void _rkMotorFWriteDC(FILE *fp, void *prp)
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

static rkMotorCom rk_motor_dc = {
  1,
  _rkMotorSetInputDC,
  _rkMotorInertiaDC,
  _rkMotorInputTrqDC,
  _rkMotorRegistanceDC,
  _rkMotorDrivingTrqDC,
  _rkMotorStateCopyDC,
  _rkMotorQueryFReadDC,
  _rkMotorFWriteDC,
};

#define RK_MOTOR_DC_DEFAULT_K     2.58e-2
#define RK_MOTOR_DC_DEFAULT_R     0.42373
#define RK_MOTOR_DC_DEFAULT_MAX   24.0
#define RK_MOTOR_DC_DEFAULT_MIN   -24.0
#define RK_MOTOR_DC_DEFAULT_G     120
#define RK_MOTOR_DC_DEFAULT_I     1.65e-6
#define RK_MOTOR_DC_DEFAULT_IG    5.38e-6
#define RK_MOTOR_DC_DEFAULT_COMPK 100.0
#define RK_MOTOR_DC_DEFAULT_COMPL 1.0

void _rkMotorInitPrpDC(void *prp)
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

rkMotor *rkMotorCreateDC(rkMotor *m)
{
  if( !( m->prp = zAlloc( rkMotorPrpDC, 1 ) ) )
    return NULL;
  _rkMotorInitPrpDC( m->prp );
  m->com = &rk_motor_dc;
  return m;
}

#undef _rkc
