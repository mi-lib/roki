/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_fixed - joint structure: fixed joint
 */

#include <roki/rk_joint.h>

static void _rkJointFixedInit(void *prp){}

static void *_rkJointFixedAlloc(void){ return NULL; }

/* limit joint displacement (dummy) */
static void _rkJointFixedLimVal(void *prp, double *testval, double *limval){}

/* set/get joint displacement, velocity, acceleration and torque (dummy) */
static void _rkJointFixedVal(void *prp, double *val){}

/* continuously update joint displacement */
static void _rkJointFixedCatDis(void *prp, double *dis, double k, double *val){}
static void _rkJointFixedSubDis(void *prp, double *dis, double *sdis){}
static void _rkJointFixedSetDisCNT(void *prp, double *val, double dt){}

/* joint frame transformation */
static zFrame3D *_rkJointFixedXform(void *prp, zFrame3D *fo, zFrame3D *f){
  /* suppose f is the same with fo */
  zFrame3DCopy( fo, f );
  return f;
}

/* joint motion rate transformation */
static void _rkJointFixedIncVel(void *prp, zVec6D *vel){}
static void _rkJointFixedIncAccOnVel(void *prp, zVec3D *w, zVec6D *acc){}
static void _rkJointFixedIncAcc(void *prp, zVec6D *acc){}

/* joint torque transformation */
static void _rkJointFixedCalcTrq(void *prp, zVec6D *f){}

/* inverse computation of joint torsion and displacement */
static void _rkJointFixedTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  zVec6D to;
  zFrame3DToVec6DAA( dev, &to );
  zMulMat3DTVec6D( zFrame3DAtt(dev), &to, t );
}

/* joint axes */
static zVec3D* (*_rk_joint_fixed_axis_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};
static zVec3D* (*_rk_joint_fixed_axis_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};

/* pivot for static friction computation */
static void _rkJointFixedFrictionPivot(void *prp, rkJointFrictionPivot *fp){}

/* motor */
static rkMotor *_rkJointFixedGetMotor(void *prp){ return NULL; }

static void _rkJointFixedMotorSetInput(void *prp, double *val){}
static void _rkJointFixedMotorInertia(void *prp, double *val){}
static void _rkJointFixedMotorInputTrq(void *prp, double *val){}
static void _rkJointFixedMotorResistance(void *prp, double *val){}
static void _rkJointFixedMotorDrivingTrq(void *prp, double *val){}

/* ABI */
static void _rkJointFixedABIAxisInertia(void *prp, zMat6D *m, zMat h, zMat ih){}

static void _rkJointFixedABIAddABI(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  zMat6D tmpm;
  rkJointXformMat6D( f, m, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}

static void _rkJointFixedABIAddBias(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  zVec6D tmpv;
  zMulMat3DVec6D( zFrame3DAtt(f), b, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}

static void _rkJointFixedABIDrivingTorque(void *prp){}
static void _rkJointFixedABIQAcc(void *prp, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  zVec6DCopy( jac, acc );
}

static bool _rkJointFixedRegZTK(ZTK *ztk, char *tag){ return true; }

static void *_rkJointFixedFromZTK(void *prp, rkMotorArray *motorarray, ZTK *ztk){
  return prp;
}

static void _rkJointFixedFPrintZTK(FILE *fp, void *prp, char *name){}

rkJointCom rk_joint_fixed = {
  "fix",
  0,
  _rkJointFixedInit,
  _rkJointFixedAlloc,
  _rkJointFixedLimVal,
  _rkJointFixedVal,
  _rkJointFixedVal,
  _rkJointFixedVal,
  _rkJointFixedVal,
  _rkJointFixedVal,
  _rkJointFixedVal,
  _rkJointFixedVal,
  _rkJointFixedVal,
  _rkJointFixedCatDis,
  _rkJointFixedSubDis,
  _rkJointFixedSetDisCNT,
  _rkJointFixedXform,
  _rkJointFixedIncVel,
  _rkJointFixedIncAccOnVel,
  _rkJointFixedIncAcc,
  _rkJointFixedCalcTrq,
  _rkJointFixedTorsion,
  _rk_joint_fixed_axis_ang,
  _rk_joint_fixed_axis_lin,

  _rkJointFixedFrictionPivot,
  _rkJointFixedFrictionPivot,
  _rkJointFixedVal,
  _rkJointFixedVal,
  _rkJointFixedVal,
  _rkJointFixedVal,

  _rkJointFixedGetMotor,
  _rkJointFixedMotorSetInput,
  _rkJointFixedMotorInertia,
  _rkJointFixedMotorInputTrq,
  _rkJointFixedMotorResistance,
  _rkJointFixedMotorDrivingTrq,

  _rkJointFixedABIAxisInertia,
  _rkJointFixedABIAddABI,
  _rkJointFixedABIAddBias,
  _rkJointFixedABIDrivingTorque,
  _rkJointFixedABIQAcc,
  _rkJointUpdateWrench,

  _rkJointFixedRegZTK,
  NULL,
  _rkJointFixedFromZTK,
  NULL,
  _rkJointFixedFPrintZTK,
};
