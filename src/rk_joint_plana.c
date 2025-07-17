/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_plana - joint structure: planar joint
 */

#include <roki/rk_joint.h>

#define _rks(joint) ((rkJointPlanaState *)((rkJoint *)(joint))->state)
#define _rkp(joint) ((rkJointPlanaPrp   *)((rkJoint *)(joint))->prp)

static void _rkJointPlanaInit(rkJoint *joint){}

static void *_rkJointPlanaAllocPrp(void){ return NULL; }
static void _rkJointPlanaCopyPrp(rkJoint *src, rkJoint *dst){}

RK_JOINT_COM_DEF_STATE_FUNC( Plana )

/* test joint displacement */
static void _rkJointPlanaTestDis(rkJoint *joint, double *testval, double *val){
  zCoord2DCopy( (zCoord2D *)testval, (zCoord2D *)val );
}

/* set joint displacement */
static void _rkJointPlanaSetDis(rkJoint *joint, double *val){
  _rkJointPlanaTestDis( joint, val, (double *)&_rks(joint)->dis );
  zSinCos( zCoord2DAngle(&_rks(joint)->dis), &_rks(joint)->_s, &_rks(joint)->_c );
}

static void _rkJointPlanaSetVel(rkJoint *joint, double *val){
  zCoord2DCopy( val, &_rks(joint)->vel );
}

static void _rkJointPlanaSetAcc(rkJoint *joint, double *val){
  zCoord2DCopy( val, &_rks(joint)->acc );
}

static void _rkJointPlanaSetTrq(rkJoint *joint, double *val){
  zCoord2DCopy( val, &_rks(joint)->trq );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointPlanaGetDis(rkJoint *joint, double *val){
  zCoord2DCopy( &_rks(joint)->dis, val );
}

static void _rkJointPlanaGetMin(rkJoint *joint, double *val){
  val[0] = val[1] = val[2] = -HUGE_VAL;
}

static void _rkJointPlanaGetMax(rkJoint *joint, double *val){
  val[0] = val[1] = val[2] = HUGE_VAL;
}

static void _rkJointPlanaGetVel(rkJoint *joint, double *val){
  zCoord2DCopy( &_rks(joint)->vel, val );
}

static void _rkJointPlanaGetAcc(rkJoint *joint, double *val){
  zCoord2DCopy( &_rks(joint)->acc, val );
}

static void _rkJointPlanaGetTrq(rkJoint *joint, double *val){
  zCoord2DCopy( &_rks(joint)->trq, val );
}

static void _rkJointPlanaCatDis(rkJoint *joint, double *dis, double k, double *val){
  zCoord2DCatDRC( (zCoord2D*)dis, k, (zCoord2D*)val );
}

static void _rkJointPlanaSubDis(rkJoint *joint, double *dis, double *sdis){
  zCoord2DSubDRC( (zCoord2D*)dis, (zCoord2D*)sdis );
}

/* continuously update joint displacement over delta time */
static void _rkJointPlanaSetDisCNT(rkJoint *joint, double *val, double dt){
  zCoord2D olddis, oldvel;

  _rkJointPlanaGetDis( joint, (double*)&olddis );
  _rkJointPlanaGetVel( joint, (double*)&oldvel );
  _rkJointPlanaSetDis( joint, val );
  zCoord2DSub( &_rks(joint)->dis, &olddis, &_rks(joint)->vel );
  zCoord2DDivDRC( &_rks(joint)->vel, dt );
  zCoord2DSub( &_rks(joint)->vel, &oldvel, &_rks(joint)->acc );
  zCoord2DDivDRC( &_rks(joint)->acc, dt );
}

/* joint frame transformation */
static zFrame3D *_rkJointPlanaXform(rkJoint *joint, zFrame3D *fo, zFrame3D *f){
  /* slide */
  zVec3DCat(   zFrame3DPos(fo), zCoord2DX(&_rks(joint)->dis), zFrame3DVec(fo,zX), zFrame3DPos(f) );
  zVec3DCatDRC( zFrame3DPos(f), zCoord2DY(&_rks(joint)->dis), zFrame3DVec(fo,zY) );
  /* rotation */
  _rkJointRotateZ( joint, fo, f );
  return f;
}

/* joint velocity transformation */
static void _rkJointPlanaIncVel(rkJoint *joint, zVec6D *vel){
  vel->e[zX ] += zCoord2DX(&_rks(joint)->vel);
  vel->e[zY ] += zCoord2DY(&_rks(joint)->vel);
  vel->e[zZA] += zCoord2DAngle(&_rks(joint)->vel);
}

static void _rkJointPlanaIncAccOnVel(rkJoint *joint, zVec3D *w, zVec6D *acc){
  acc->e[zX ] -= 2 * zCoord2DY(&_rks(joint)->vel) * w->e[zZ];
  acc->e[zY ] += 2 * zCoord2DX(&_rks(joint)->vel) * w->e[zZ];
  acc->e[zZ ] += 2 * ( zCoord2DY(&_rks(joint)->vel) * w->e[zX] - zCoord2DX(&_rks(joint)->vel) * w->e[zY] );
  acc->e[zXA] += zCoord2DAngle(&_rks(joint)->vel) * w->e[zY];
  acc->e[zYA] -= zCoord2DAngle(&_rks(joint)->vel) * w->e[zX];
}

/* joint acceleration transformation */
static void _rkJointPlanaIncAcc(rkJoint *joint, zVec6D *acc){
  acc->e[zX ] += zCoord2DX(&_rks(joint)->acc);
  acc->e[zY ] += zCoord2DY(&_rks(joint)->acc);
  acc->e[zZA] += zCoord2DAngle(&_rks(joint)->acc);
}

/* joint torque transformation */
static void _rkJointPlanaCalcTrq(rkJoint *joint, zVec6D *f){
  zCoord2DCreate( &_rks(joint)->trq, f->e[zX], f->e[zY], f->e[zZA] );
}

/* inverse computation of joint torsion and displacement */
static void _rkJointPlanaTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  zVec3D aa;
  zMat3D rj0;
  dis[2] = rkJointRevolTorsionDis( dev, t );
  zMulMat3DVec3D( zFrame3DAtt(dev), zVec6DAng(t), &aa );
  zMat3DFromAA( &rj0, &aa );
  zMulMat3DTVec3D( &rj0, zFrame3DPos(dev), zVec6DLin(t) );
  dis[0] = t->e[zX];
  dis[1] = t->e[zY];
  t->e[zX] = 0;
  t->e[zY] = 0;
}

/* joint axes */
static zVec3D* (*_rk_joint_plana_axis_ang[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisZ,
};
static zVec3D* (*_rk_joint_plana_axis_lin[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisX,
  _rkJointAxisY,
  _rkJointAxisNull,
};

/* CRB method */

static void _rkJointPlanaCRBWrench(rkJoint *joint, rkMP *crb, zVec6D wi[]){
  _rkJointCRBWrenchLinX( crb, &wi[0] );
  _rkJointCRBWrenchLinY( crb, &wi[1] );
  _rkJointCRBWrenchAngZ( crb, &wi[2] );
}
static void _rkJointPlanaCRBXform(rkJoint *joint, zFrame3D *f, zVec6D si[]){
  _rkJointCRBXformLin( f, zX, &si[0] );
  _rkJointCRBXformLin( f, zY, &si[1] );
  _rkJointCRBXformAng( f, zZ, &si[2] );
}

/* motor */

static void _rkJointPlanaMotorInertia(rkJoint *joint, double *val){ zMat3DZero( (zMat3D *)val ); }
static void _rkJointPlanaMotorInputTrq(rkJoint *joint, double *val){ val[0] = val[1] = val[2] = 0; }
static void _rkJointPlanaMotorResistance(rkJoint *joint, double *val){ val[0] = val[1] = val[2] = 0; }
static void _rkJointPlanaMotorDrivingTrq(rkJoint *joint, double *val){ val[0] = val[1] = val[2] = 0; }

/* ABI */

static void _rkJointPlanaABIAxisInertia(rkJoint *joint, zMat6D *m, zMat h, zMat ih){
  eprintf( "ABI method for planar joint not implemented yet.\n" );
}

static void _rkJointPlanaABIAddABI(rkJoint *joint, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  eprintf( "ABI method for planar joint not implemented yet.\n" );
}

static void _rkJointPlanaABIAddBias(rkJoint *joint, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  eprintf( "ABI method for planar joint not implemented yet.\n" );
}

static void _rkJointPlanaABIDrivingTorque(rkJoint *joint){
  eprintf( "ABI method for planar joint not implemented yet.\n" );
}

static void _rkJointPlanaABIQAcc(rkJoint *joint, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  eprintf( "ABI method for planar joint not implemented yet.\n" );
}

/* ZTK */

static void *_rkJointPlanaDisFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  double dis[3];
  dis[0] = ZTKDouble(ztk);
  dis[1] = ZTKDouble(ztk);
  dis[2] = zDeg2Rad(ZTKDouble(ztk));
  _rkJointPlanaSetDis( (rkJoint *)joint, dis );
  return joint;
}

static bool _rkJointPlanaDisFPrintZTK(FILE *fp, int i, void *joint){
  if( zCoord2DIsTiny( &_rks(joint)->dis ) ) return false;
  fprintf( fp, "%.10g %.10g %.10g\n",
    zCoord2DX(&_rks(joint)->dis),
    zCoord2DY(&_rks(joint)->dis),
    zRad2Deg( zCoord2DAngle(&_rks(joint)->dis) ) );
  return true;
}

static const ZTKPrp __ztk_prp_rkjoint_plana[] = {
  { ZTK_KEY_ROKI_JOINT_DIS, 1, _rkJointPlanaDisFromZTK, _rkJointPlanaDisFPrintZTK },
};

static rkJoint *_rkJointPlanaFromZTK(rkJoint *joint, rkMotorSpecArray *motorspecarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorspecarray, ztk, __ztk_prp_rkjoint_plana );
}

static void _rkJointPlanaFPrintZTK(FILE *fp, rkJoint *joint, char *name)
{
  _ZTKPrpKeyFPrint( fp, joint, __ztk_prp_rkjoint_plana );
}

rkJointCom rk_joint_plana = {
  "planar",
  3,
  _rkJointPlanaInit,
  _rkJointPlanaAllocPrp,
  _rkJointPlanaAllocState,
  _rkJointPlanaCopyPrp,
  _rkJointPlanaCopyState,
  _rkJointPlanaTestDis,
  _rkJointPlanaSetDis,
  _rkJointDummyVal,
  _rkJointDummyVal,
  _rkJointPlanaSetVel,
  _rkJointPlanaSetAcc,
  _rkJointPlanaSetTrq,
  _rkJointPlanaGetDis,
  _rkJointPlanaGetMin,
  _rkJointPlanaGetMax,
  _rkJointPlanaGetVel,
  _rkJointPlanaGetAcc,
  _rkJointPlanaGetTrq,
  _rkJointPlanaCatDis,
  _rkJointPlanaSubDis,
  _rkJointPlanaSetDisCNT,
  _rkJointPlanaXform,
  _rkJointPlanaIncVel,
  _rkJointPlanaIncAccOnVel,
  _rkJointPlanaIncAcc,
  _rkJointPlanaCalcTrq,
  _rkJointPlanaTorsion,
  _rk_joint_plana_axis_ang,
  _rk_joint_plana_axis_lin,

  _rkJointPlanaCRBWrench,
  _rkJointPlanaCRBXform,

  _rkJointDummyFrictionPivot,
  _rkJointDummyFrictionPivot,
  _rkJointDummyVal,
  _rkJointDummyVal,
  _rkJointDummyVal,
  _rkJointDummyVal,

  rkJointMotorSetValDummy,
  _rkJointPlanaMotorInertia,
  _rkJointPlanaMotorInputTrq,
  _rkJointPlanaMotorResistance,
  _rkJointPlanaMotorDrivingTrq,

  _rkJointPlanaABIAxisInertia,
  _rkJointPlanaABIAddABI,
  _rkJointPlanaABIAddBias,
  _rkJointPlanaABIDrivingTorque,
  _rkJointPlanaABIQAcc,
  _rkJointUpdateWrench,

  _rkJointPlanaDisFromZTK,
  _rkJointPlanaFromZTK,
  _rkJointPlanaDisFPrintZTK,
  _rkJointPlanaFPrintZTK,
};

#undef _rks
#undef _rkp
