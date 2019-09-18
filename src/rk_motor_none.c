/* RoKi - Robot Kinetics library.
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_none - motor model: unactuation
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

static void _rkMotorNoneInit(void *prp){}
static void *_rkMotorNoneAlloc(void){ return NULL; }
static void _rkMotorNoneCopy(void *src, void *dst){}

static void _rkMotorNoneSetInput(void *prp, double *val){}

static void _rkMotorNoneInertia(void *prp, double *val){}
static void _rkMotorNoneInputTrq(void *prp, double *val){}
static void _rkMotorNoneRegistance(void *prp, double *dis, double *vel, double *val){}
static void _rkMotorNoneDrivingTrq(void *prp, double *dis, double *vel, double *acc, double *val){}

static bool _rkMotorNoneQueryFScan(FILE *fp, char *key, void *prp){
  return false;
}

static bool _rkMotorNoneRegZTK(ZTK *ztk, char *tag){ return true; }

static void *_rkMotorNoneFromZTK(void *prp, ZTK *ztk){ return prp; }

static void _rkMotorNoneFPrint(FILE *fp, void *prp){}

rkMotorCom rk_motor_none = {
  "none",
  0,
  _rkMotorNoneInit,
  _rkMotorNoneAlloc,
  _rkMotorNoneCopy,
  _rkMotorNoneSetInput,
  _rkMotorNoneInputTrq,
  _rkMotorNoneInertia,
  _rkMotorNoneRegistance,
  _rkMotorNoneDrivingTrq,
  _rkMotorNoneQueryFScan,
  _rkMotorNoneRegZTK,
  _rkMotorNoneFromZTK,
  _rkMotorNoneFPrint,
};
