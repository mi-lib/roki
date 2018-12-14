/* RoKi - Robot Kinetics library.
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_none - motor model: unactuation
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

static void _rkMotorSetInputNone(void *prp, double *val);

static void _rkMotorInertiaNone(void *prp, double *val);
static void _rkMotorInputTrqNone(void *prp, double *val);
static void _rkMotorRegistanceNone(void *prp, double *dis, double *vel, double *val);
static void _rkMotorDrivingTrqNone(void *prp, double *dis, double *vel, double *acc, double *val);

static void _rkMotorStateCopyNone(void *src, void *dst);

static bool _rkMotorQueryFReadNone(FILE *fp, char *key, void *prp);
static void _rkMotorFWriteNone(FILE *fp, void *prp);


void _rkMotorSetInputNone(void *prp, double *val){}

void _rkMotorInertiaNone(void *prp, double *val){}
void _rkMotorInputTrqNone(void *prp, double *val){}
void _rkMotorRegistanceNone(void *prp, double *dis, double *vel, double *val){}
void _rkMotorDrivingTrqNone(void *prp, double *dis, double *vel, double *acc, double *val){}

void _rkMotorStateCopyNone(void *src, void *dst){}

bool _rkMotorQueryFReadNone(FILE *fp, char *key, void *prp){
  return false;
}

void _rkMotorFWriteNone(FILE *fp, void *prp){}

static rkMotorCom rk_motor_none = {
  0,
  _rkMotorSetInputNone,
  _rkMotorInputTrqNone,
  _rkMotorInertiaNone,
  _rkMotorRegistanceNone,
  _rkMotorDrivingTrqNone,
  _rkMotorStateCopyNone,
  _rkMotorQueryFReadNone,
  _rkMotorFWriteNone,
};

rkMotor *rkMotorCreateNone(rkMotor *m){
  m->com = &rk_motor_none;
  return m;
}
