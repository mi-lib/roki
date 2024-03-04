/* RoKi - Robot Kinetics library.
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_none - motor model: unactuation
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

static void _rkMotorSpecNoneInitPrp(rkMotorSpec *ms){}
static void *_rkMotorSpecNoneAllocPrp(void){ return NULL; }
static void _rkMotorSpecNoneCopyPrp(rkMotorSpec *src, rkMotorSpec *dst){}

static rkMotorSpec *_rkMotorSpecNoneFromZTK(rkMotorSpec *ms, ZTK *ztk){ return ms; }
static void _rkMotorSpecNoneFPrintZTK(FILE *fp, rkMotorSpec *ms){}

static void _rkMotorNoneInitState(rkMotor *motor){}
static void *_rkMotorNoneAllocState(void){ return NULL; }
static void _rkMotorNoneCopyState(rkMotor *src, rkMotor *dst){}

static void _rkMotorNoneSetInput(rkMotor *motor, double *val){}

static void _rkMotorNoneInertia(rkMotor *motor, double *val){}
static void _rkMotorNoneInputTrq(rkMotor *motor, double *val){}
static void _rkMotorNoneRegistance(rkMotor *motor, double *dis, double *vel, double *val){}
static void _rkMotorNoneDrivingTrq(rkMotor *motor, double *dis, double *vel, double *acc, double *val){}

rkMotorCom rk_motor_none = {
  "none",
  0,
  _rkMotorSpecNoneInitPrp,
  _rkMotorSpecNoneAllocPrp,
  _rkMotorSpecNoneCopyPrp,
  _rkMotorSpecNoneFromZTK,
  _rkMotorSpecNoneFPrintZTK,

  _rkMotorNoneInitState,
  _rkMotorNoneAllocState,
  _rkMotorNoneCopyState,
  _rkMotorNoneSetInput,
  _rkMotorNoneInputTrq,
  _rkMotorNoneInertia,
  _rkMotorNoneRegistance,
  _rkMotorNoneDrivingTrq,
};
