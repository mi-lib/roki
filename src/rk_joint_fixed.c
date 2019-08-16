/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_fixed - joint structure: fixed joint
 */

#include <roki/rk_joint.h>

static void _rkJointInitFixed(void *prp){}

static void *_rkJointAllocFixed(void){ return NULL; }

/* limit joint displacement (dummy) */
static void _rkJointLimValFixed(void *prp, double *testval, double *limval){}

/* set/get joint displacement, velocity, acceleration and torque (dummy) */
static void _rkJointValFixed(void *prp, double *val){}

/* continuously update joint displacement */
static void _rkJointCatDisFixed(void *prp, double *dis, double k, double *val){}
static void _rkJointSubDisFixed(void *prp, double *dis, double *sdis){}
static void _rkJointSetDisCNTFixed(void *prp, double *val, double dt){}

/* joint frame transformation */
static zFrame3D *_rkJointXformFixed(void *prp, zFrame3D *fo, zFrame3D *f){
  /* suppose f is the same with fo */
  zFrame3DCopy( fo, f );
  return f;
}

/* joint motion rate transformation */
static void _rkJointIncVelFixed(void *prp, zVec6D *vel){}
static void _rkJointIncAccOnVelFixed(void *prp, zVec3D *w, zVec6D *acc){}
static void _rkJointIncAccFixed(void *prp, zVec6D *acc){}

/* joint torque transformation */
static void _rkJointCalcTrqFixed(void *prp, zVec6D *f){}

/* inverse computation of joint torsion and displacement */
static void _rkJointTorsionFixed(zFrame3D *dev, zVec6D *t, double dis[]){
  zVec6D to;
  zFrame3DToVec6DAA( dev, &to );
  zMulMat3DTVec6D( zFrame3DAtt(dev), &to, t );
}

/* joint axes */
static zVec3D* (*_rk_joint_axis_fixed_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};
static zVec3D* (*_rk_joint_axis_fixed_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};

/* pivot for static friction computation */
static void _rkJointFrictionPivotFixed(void *prp, rkJointFrictionPivot *fp){}

/* motor */
static rkMotor *_rkJointGetMotorFixed(void *prp){ return NULL; }

static void _rkJointMotorSetInputFixed(void *prp, double *val){}
static void _rkJointMotorInertiaFixed(void *prp, double *val){}
static void _rkJointMotorInputTrqFixed(void *prp, double *val){}
static void _rkJointMotorResistanceFixed(void *prp, double *val){}
static void _rkJointMotorDrivingTrqFixed(void *prp, double *val){}

/* ABI */
static void _rkJointABIAxisInertiaFixed(void *prp, zMat6D *m, zMat h, zMat ih){}

static void _rkJointABIAddAbiFixed(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  zMat6D tmpm;
  rkJointXformMat6D( f, m, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}

static void _rkJointABIAddBiasFixed(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  zVec6D tmpv;
  zMulMat3DVec6D( zFrame3DAtt(f), b, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}

static void _rkJointABIDrivingTorqueFixed(void *prp){}
static void _rkJointABIQAccFixed(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  zVec6DCopy( jac, acc );
}

/* query joint properties */
static bool _rkJointQueryFScanFixed(FILE *fp, char *key, void *prp, rkMotor *marray, int nm){
  return false;
}

static void *_rkJointFromZTKFixed(void *prp, rkMotorArray *motorarray, ZTK *ztk){
  return prp;
}

static void _rkJointFPrintFixed(FILE *fp, void *prp, char *name){}

rkJointCom rk_joint_fixed = {
  "fix",
  0,
  _rkJointInitFixed,
  _rkJointAllocFixed,
  _rkJointLimValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointCatDisFixed,
  _rkJointSubDisFixed,
  _rkJointSetDisCNTFixed,
  _rkJointXformFixed,
  _rkJointIncVelFixed,
  _rkJointIncAccOnVelFixed,
  _rkJointIncAccFixed,
  _rkJointCalcTrqFixed,
  _rkJointTorsionFixed,
  _rk_joint_axis_fixed_ang,
  _rk_joint_axis_fixed_lin,

  _rkJointFrictionPivotFixed,
  _rkJointFrictionPivotFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,

  _rkJointGetMotorFixed,
  _rkJointMotorSetInputFixed,
  _rkJointMotorInertiaFixed,
  _rkJointMotorInputTrqFixed,
  _rkJointMotorResistanceFixed,
  _rkJointMotorDrivingTrqFixed,

  _rkJointABIAxisInertiaFixed,
  _rkJointABIAddAbiFixed,
  _rkJointABIAddBiasFixed,
  _rkJointABIDrivingTorqueFixed,
  _rkJointABIQAccFixed,
  _rkJointUpdateWrench,

  _rkJointQueryFScanFixed,
  NULL,
  _rkJointFromZTKFixed,
  NULL,
  _rkJointFPrintFixed,
};
