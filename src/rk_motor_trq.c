/* RoKi - Robot Kinetics library.
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_trq - motor model: torque motor
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

#define _rkp(ms) ((rkMotorTrqPrp *)((rkMotorSpec *)(ms))->prp)

static void _rkMotorSpecTrqInitPrp(rkMotorSpec *ms){
  _rkp(ms)->min = -HUGE_VAL;
  _rkp(ms)->max =  HUGE_VAL;
}

RK_MOTOR_COM_DEF_ALLOC_COPY_FUNC( Trq )

/* ZTK */

static void *_rkMotorSpecTrqMaxFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  _rkp(obj)->max = ZTKDouble(ztk);
  return obj;
}
static void *_rkMotorSpecTrqMinFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  _rkp(obj)->min = ZTKDouble(ztk);
  return obj;
}

static void _rkMotorSpecTrqMaxFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", _rkp(obj)->max );
}
static void _rkMotorSpecTrqMinFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", _rkp(obj)->min );
}

static ZTKPrp __ztk_prp_rkmotor_trq[] = {
  { "max", 1, _rkMotorSpecTrqMaxFromZTK, _rkMotorSpecTrqMaxFPrintZTK },
  { "min", 1, _rkMotorSpecTrqMinFromZTK, _rkMotorSpecTrqMinFPrintZTK },
};

static rkMotorSpec *_rkMotorSpecTrqFromZTK(rkMotorSpec *ms, ZTK *ztk)
{
  return (rkMotorSpec *)ZTKEvalKey( ms, NULL, ztk, __ztk_prp_rkmotor_trq );
}

static void _rkMotorSpecTrqFPrintZTK(FILE *fp, rkMotorSpec *ms)
{
  ZTKPrpKeyFPrint( fp, ms, __ztk_prp_rkmotor_trq );
}

#undef _rkp

/* methods for motor instances */

#define _rkp(m) ((rkMotorTrqPrp *)((rkMotor *)(m))->spec->prp)
#define _rks(m) ((rkMotorTrqState *)((rkMotor *)(m))->state)

static void _rkMotorTrqInitState(rkMotor *motor){
  _rks(motor)->input = 0.0;
}

static void _rkMotorTrqSetInput(rkMotor *motor, double *val){
  _rks(motor)->input = zLimit( *val, _rkp(motor)->min, _rkp(motor)->max );
}

static void _rkMotorTrqInertia(rkMotor *motor, double *val){
  *val = 0.0;
}

static void _rkMotorTrqInputTrq(rkMotor *motor, double *val){
  *val = _rks(motor)->input;
}

static void _rkMotorTrqRegistance(rkMotor *motor, double *dis, double *vel, double *val){
  *val = 0.0;
}

static void _rkMotorTrqDrivingTrq(rkMotor *motor, double *dis, double *vel, double *acc, double *val){
  _rkMotorTrqInputTrq( motor, val );
}

rkMotorCom rk_motor_trq = {
  "trq",
  1,
  _rkMotorSpecTrqInitPrp,
  _rkMotorSpecTrqAllocPrp,
  _rkMotorSpecTrqCopyPrp,
  _rkMotorSpecTrqFromZTK,
  _rkMotorSpecTrqFPrintZTK,
  _rkMotorTrqInitState,
  _rkMotorTrqAllocState,
  _rkMotorTrqCopyState,
  _rkMotorTrqSetInput,
  _rkMotorTrqInertia,
  _rkMotorTrqInputTrq,
  _rkMotorTrqRegistance,
  _rkMotorTrqDrivingTrq,
};

#undef _rkp
#undef _rks
