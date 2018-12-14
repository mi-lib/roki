/* RoKi - Robot Kinetics library.
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_trq - motor model: torque motor
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

static void _rkMotorSetInputTRQ(void *prp, double *val);

static void _rkMotorInertiaTRQ(void *prp, double *val);
static void _rkMotorInputTrqTRQ(void *prp, double *val);
static void _rkMotorRegistanceTRQ(void *prp, double *dis, double *vel, double *val);
static void _rkMotorDrivingTrqTRQ(void *prp, double *dis, double *vel, double *acc, double *val);

static void _rkMotorStateCopyTRQ(void *src, void *dst);

static bool _rkMotorQueryFReadTRQ(FILE *fp, char *key, void *prp);
static void _rkMotorFWriteTRQ(FILE *fp, void *prp);

#define _rkc(p) ((rkMotorPrpTRQ *)p)

void _rkMotorSetInputTRQ(void *prp, double *val){
  _rkc(prp)->input = zLimit( *val, _rkc(prp)->min, _rkc(prp)->max );
}

void _rkMotorInertiaTRQ(void *prp, double *val){
  *val = 0.0;
}

void _rkMotorInputTrqTRQ(void *prp, double *val){
  *val = _rkc(prp)->input;
}

void _rkMotorRegistanceTRQ(void *prp, double *dis, double *vel, double *val){
  *val = 0.0;
}
void _rkMotorDrivingTrqTRQ(void *prp, double *dis, double *vel, double *acc, double *val){
  _rkMotorInputTrqTRQ( prp, val );
}

void _rkMotorStateCopyTRQ(void *src, void *dst){
  memcpy(dst, src, sizeof(rkMotorPrpTRQ));
}

bool _rkMotorQueryFReadTRQ(FILE *fp, char *key, void *prp)
{
  if( strcmp( key, "max" ) == 0 )
    _rkc(prp)->max = zFDouble( fp );
  else if( strcmp( key, "min" ) == 0 )
    _rkc(prp)->min = zFDouble( fp );
  else
    return false;
  return true;
}

void _rkMotorFWriteTRQ(FILE *fp, void *prp)
{
  fprintf( fp, "\n" );
}

static rkMotorCom rk_motor_trq = {
  1,
  _rkMotorSetInputTRQ,
  _rkMotorInertiaTRQ,
  _rkMotorInputTrqTRQ,
  _rkMotorRegistanceTRQ,
  _rkMotorDrivingTrqTRQ,
  _rkMotorStateCopyTRQ,
  _rkMotorQueryFReadTRQ,
  _rkMotorFWriteTRQ,
};

void _rkMotorInitPrpTRQ(void *prp)
{
  _rkc(prp)->input = 0.0;
  _rkc(prp)->min = -HUGE_VAL;
  _rkc(prp)->max = HUGE_VAL;
}

rkMotor *rkMotorCreateTRQ(rkMotor *m)
{
  if( !( m->prp = zAlloc( rkMotorPrpTRQ, 1 ) ) )
    return NULL;
  _rkMotorInitPrpTRQ( m->prp );
  m->com = &rk_motor_trq;
  return m;
}

#undef _rkc
