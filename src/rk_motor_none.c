/* RoKi - Robot Kinetics library.
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_none - motor model: unactuation
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

static void _rkMotorInitNone(void *prp){}
static void *_rkMotorAllocNone(void){ return NULL; }
static void _rkMotorCopyNone(void *src, void *dst){}

static void _rkMotorSetInputNone(void *prp, double *val){}

static void _rkMotorInertiaNone(void *prp, double *val){}
static void _rkMotorInputTrqNone(void *prp, double *val){}
static void _rkMotorRegistanceNone(void *prp, double *dis, double *vel, double *val){}
static void _rkMotorDrivingTrqNone(void *prp, double *dis, double *vel, double *acc, double *val){}

static bool _rkMotorQueryFScanNone(FILE *fp, char *key, void *prp){
  return false;
}

static void *_rkMotorFromZTKNone(void *prp, ZTK *ztk){ return prp; }

static void _rkMotorFPrintNone(FILE *fp, void *prp){}

rkMotorCom rk_motor_none = {
  "none",
  0,
  _rkMotorInitNone,
  _rkMotorAllocNone,
  _rkMotorCopyNone,
  _rkMotorSetInputNone,
  _rkMotorInputTrqNone,
  _rkMotorInertiaNone,
  _rkMotorRegistanceNone,
  _rkMotorDrivingTrqNone,
  _rkMotorQueryFScanNone,
  _rkMotorFromZTKNone,
  _rkMotorFPrintNone,
};
