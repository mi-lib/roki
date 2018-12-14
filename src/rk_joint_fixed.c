/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_fixed - joint structure: fixed joint
 */

#include <roki/rk_joint.h>

static void _rkJointLimValFixed(void *prp, double *testval, double *limval);
static void _rkJointValFixed(void *prp, double *val);
static void _rkJointCatDisFixed(void *prp, double *dis, double k, double *val);
static void _rkJointSubDisFixed(void *prp, double *dis, double *sdis);
static void _rkJointSetDisCNTFixed(void *prp, double *val, double dt);
static void _rkJointGetMotorFixed(void *prp, rkMotor **m);
static zFrame3D *_rkJointXferFixed(void *prp, zFrame3D *fo, zFrame3D *f);
static void _rkJointIncVelFixed(void *prp, zVec6D *vel);
static void _rkJointIncAccOnVelFixed(void *prp, zVec3D *w, zVec6D *acc);
static void _rkJointIncAccFixed(void *prp, zVec6D *acc);
static void _rkJointCalcTrqFixed(void *prp, zVec6D *f);
static void _rkJointTorsionFixed(zFrame3D *dev, zVec6D *t, double dis[]);
static void _rkJointRefFixed(void *prp, rkJointRef *ref);
static bool _rkJointQueryFReadFixed(FILE *fp, char *key, void *prp, rkMotor *marray, int nm);
static void _rkJointFWriteFixed(FILE *fp, void *prp, char *name);

/* limit joint displacement (dummy) */
void _rkJointLimValFixed(void *prp, double *testval, double *limval){}

/* set/get joint displacement, velocity, acceleration and torque (dummy) */
void _rkJointValFixed(void *prp, double *val){}

/* continuously update joint displacement */
void _rkJointCatDisFixed(void *prp, double *dis, double k, double *val){}
void _rkJointSubDisFixed(void *prp, double *dis, double *sdis){}
void _rkJointSetDisCNTFixed(void *prp, double *val, double dt){}

/* get motor */
void _rkJointGetMotorFixed(void *prp, rkMotor **m){
  *m = NULL;
}

/* joint frame transfer function */
zFrame3D *_rkJointXferFixed(void *prp, zFrame3D *fo, zFrame3D *f)
{ /* suppose f is the same with fo */
  zFrame3DCopy( fo, f );
  return f;
}

/* joint motion rate transfer function */
void _rkJointIncVelFixed(void *prp, zVec6D *vel){}
void _rkJointIncAccOnVelFixed(void *prp, zVec3D *w, zVec6D *acc){}
void _rkJointIncAccFixed(void *prp, zVec6D *acc){}

/* joint torque transfer function */
void _rkJointCalcTrqFixed(void *prp, zVec6D *f){}

/* inverse computation of joint torsion and displacement */
void _rkJointTorsionFixed(zFrame3D *dev, zVec6D *t, double dis[])
{
  zVec6D to;

  zFrame3DToVec6DAA( dev, &to );
  zMulMatTVec6D( zFrame3DAtt(dev), &to, t );
}

/* referential displacement for static friction computation */
void _rkJointRefFixed(void *prp, rkJointRef *ref){}

/* query joint properties */
bool _rkJointQueryFReadFixed(FILE *fp, char *key, void *prp, rkMotor *marray, int nm){
  return false;
}

void _rkJointFWriteFixed(FILE *fp, void *prp, char *name){}

static zVec3D* (*_rk_joint_axis_fixed_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};
static zVec3D* (*_rk_joint_axis_fixed_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};
static rkJointCom rk_joint_fixed = {
  0,
  _rkJointLimValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointGetMotorFixed,
  _rkJointCatDisFixed,
  _rkJointSubDisFixed,
  _rkJointSetDisCNTFixed,
  _rkJointXferFixed,
  _rkJointIncVelFixed,
  _rkJointIncAccOnVelFixed,
  _rkJointIncAccFixed,
  _rkJointCalcTrqFixed,
  _rkJointTorsionFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointValFixed,
  _rkJointRefFixed,
  _rkJointRefFixed,
  _rk_joint_axis_fixed_ang,
  _rk_joint_axis_fixed_lin,
  _rkJointQueryFReadFixed,
  _rkJointFWriteFixed,
};

/* motor */
static byte _rkJointMotorFixed(void *prp);
static void _rkJointMotorSetInputFixed(void *prp, double *val);
static void _rkJointMotorInertiaFixed(void *prp, double *val);
static void _rkJointMotorInputTrqFixed(void *prp, double *val);
static void _rkJointMotorResistanceFixed(void *prp, double *val);
static void _rkJointMotorDrivingTrqFixed(void *prp, double *val);

byte _rkJointMotorFixed(void *prp){return RK_MOTOR_INVALID;}
void _rkJointMotorSetInputFixed(void *prp, double *val){}
void _rkJointMotorInertiaFixed(void *prp, double *val){}
void _rkJointMotorInputTrqFixed(void *prp, double *val){}
void _rkJointMotorResistanceFixed(void *prp, double *val){}
void _rkJointMotorDrivingTrqFixed(void *prp, double *val){}

static rkJointMotorCom rk_joint_motor_fixed = {
  _rkJointMotorFixed,
  _rkJointMotorSetInputFixed,
  _rkJointMotorInertiaFixed,
  _rkJointMotorInputTrqFixed,
  _rkJointMotorResistanceFixed,
  _rkJointMotorDrivingTrqFixed,
};

/* ABI */
static void _rkJointABIAxisInertiaFixed(void *prp, zMat6D *m, zMat h, zMat ih);
static void _rkJointABIAddAbiFixed(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm);
static void _rkJointABIAddBiasFixed(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb);
static void _rkJointABIDrivingTorqueFixed(void *prp);
static void _rkJointABIQAccFixed(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc);

void _rkJointABIAxisInertiaFixed(void *prp, zMat6D *m, zMat h, zMat ih){}
void _rkJointABIAddAbiFixed(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm)
{
  zMat6D tmpm;

  rkJointXferMat6D( f, m, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}

void _rkJointABIAddBiasFixed(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb)
{
  zVec6D tmpv;

  zMulMatVec6D( zFrame3DAtt(f), b, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}

void _rkJointABIDrivingTorqueFixed(void *prp){}
void _rkJointABIQAccFixed(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
{
  zVec6DCopy( jac, acc );
}

static rkJointABICom rk_joint_abi_fixed = {
  _rkJointABIAxisInertiaFixed,
  _rkJointABIAddAbiFixed,
  _rkJointABIAddBiasFixed,
  _rkJointABIDrivingTorqueFixed,
  _rkJointABIQAccFixed,
  _rkJointUpdateWrench,
};

/* rkJointCreateFixed
 * - create fixed joint instance.
 */
rkJoint *rkJointCreateFixed(rkJoint *j)
{
  j->com = &rk_joint_fixed;
  j->mcom = &rk_joint_motor_fixed;
  j->acom = &rk_joint_abi_fixed;
  return j;
}
