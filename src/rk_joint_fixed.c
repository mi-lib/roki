/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_fixed - joint structure: fixed joint
 */

#include <roki/rk_joint.h>

static void _rkJointFixedInit(rkJoint *joint){}
static void *_rkJointFixedAllocDummy(void){ return NULL; }
static void _rkJoinFixedCopyDummy(rkJoint *src, rkJoint *dst){}

/* limit joint displacement (dummy) */

static void _rkJointFixedLimVal(rkJoint *joint, double *testval, double *limval){}

/* set/get joint displacement, velocity, acceleration and torque (dummy) */

static void _rkJointFixedVal(rkJoint *joint, double *val){}

/* continuously update joint displacement */

static void _rkJointFixedCatDis(rkJoint *joint, double *dis, double k, double *val){}
static void _rkJointFixedSubDis(rkJoint *joint, double *dis, double *sdis){}
static void _rkJointFixedSetDisCNT(rkJoint *joint, double *val, double dt){}

/* joint frame transformation */

static zFrame3D *_rkJointFixedXform(rkJoint *joint, zFrame3D *fo, zFrame3D *f){
  /* suppose f is the same with fo */
  zFrame3DCopy( fo, f );
  return f;
}

/* joint motion rate transformation */

static void _rkJointFixedIncVel(rkJoint *joint, zVec6D *vel){}
static void _rkJointFixedIncAccOnVel(rkJoint *joint, zVec3D *w, zVec6D *acc){}
static void _rkJointFixedIncAcc(rkJoint *joint, zVec6D *acc){}

/* joint torque transformation */

static void _rkJointFixedCalcTrq(rkJoint *joint, zVec6D *f){}

/* inverse computation of joint torsion and displacement */

static void _rkJointFixedTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  zVec6D to;
  zFrame3DToVec6DAA( dev, &to );
  zMulMat3DTVec6D( zFrame3DAtt(dev), &to, t );
}

/* joint axes */

static zVec3D* (*_rk_joint_fixed_axis_ang[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};
static zVec3D* (*_rk_joint_fixed_axis_lin[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};

/* CRB method */

static void _rkJointFixedCRBWrench(rkJoint *joint, rkMP *crb, zVec6D wi[]){}
static void _rkJointFixedCRBXform(rkJoint *joint, zFrame3D *f, zVec6D si[]){}

/* pivot for static friction computation */

static void _rkJointFixedFrictionPivot(rkJoint *joint, rkJointFrictionPivot *fp){}

/* ABI */

static void _rkJointFixedABIAxisInertia(rkJoint *joint, zMat6D *m, zMat h, zMat ih){}

static void _rkJointFixedABIAddABI(rkJoint *joint, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  zMat6D tmpm;
  rkJointXformMat6D( f, m, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}

static void _rkJointFixedABIAddBias(rkJoint *joint, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  zVec6D tmpv;
  zMulMat3DVec6D( zFrame3DAtt(f), b, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}

static void _rkJointFixedABIDrivingTorque(rkJoint *joint){}
static void _rkJointFixedABIQAcc(rkJoint *joint, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  zVec6DCopy( jac, acc );
}

/* ZTK */

static rkJoint *_rkJointFixedFromZTK(rkJoint *joint, rkMotorSpecArray *motorarray, ZTK *ztk){
  return joint;
}

static void _rkJointFixedFPrintZTK(FILE *fp, rkJoint *joint, char *name){}

rkJointCom rk_joint_fixed = {
  "fixed",
  0,
  _rkJointFixedInit,
  _rkJointFixedAllocDummy,
  _rkJointFixedAllocDummy,
  _rkJoinFixedCopyDummy,
  _rkJoinFixedCopyDummy,
  _rkJointFixedLimVal,
  _rkJointFixedVal,
  _rkJointFixedVal,
  _rkJointFixedVal,
  _rkJointFixedVal,
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

  _rkJointFixedCRBWrench,
  _rkJointFixedCRBXform,

  _rkJointFixedFrictionPivot,
  _rkJointFixedFrictionPivot,
  _rkJointFixedVal,
  _rkJointFixedVal,
  _rkJointFixedVal,
  _rkJointFixedVal,

  rkJointMotorSetValDummy,
  rkJointMotorGetValDummy,
  rkJointMotorGetValDummy,
  rkJointMotorGetValDummy,
  rkJointMotorGetValDummy,

  _rkJointFixedABIAxisInertia,
  _rkJointFixedABIAddABI,
  _rkJointFixedABIAddBias,
  _rkJointFixedABIDrivingTorque,
  _rkJointFixedABIQAcc,
  _rkJointUpdateWrench,

  NULL,
  _rkJointFixedFromZTK,
  NULL,
  _rkJointFixedFPrintZTK,
};
