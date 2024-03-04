/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_revol - joint structure: revolutional joint
 */

#include <roki/rk_joint.h>

#define _rkp(joint) ((rkJointRevolPrp   *)((rkJoint*)(joint))->prp)
#define _rks(joint) ((rkJointRevolState *)((rkJoint*)(joint))->state)

static void _rkJointRevolInit(rkJoint *joint){
  _rkp(joint)->max = HUGE_VAL;
  _rkp(joint)->min =-HUGE_VAL;
}

static void *_rkJointRevolAllocState(void){ return zAlloc( rkJointRevolState, 1 ); }
static void *_rkJointRevolAllocPrp(void){ return zAlloc( rkJointRevolPrp, 1 ); }

static void _rkJointRevolCopyPrp(rkJoint *src, rkJoint *dst){
  _rkp(dst)->min = _rkp(src)->min;
  _rkp(dst)->max = _rkp(src)->max;
  _rkp(dst)->stiffness = _rkp(src)->stiffness;
  _rkp(dst)->viscosity = _rkp(src)->viscosity;
  _rkp(dst)->coulomb = _rkp(src)->coulomb;
  _rkp(dst)->sf = _rkp(src)->sf;
}

/* set joint displacement, velocity, acceleration and torque */

static void _rkJointRevolLimDis(rkJoint *joint, double *testval, double *limval){
  double angle;
  angle = zPhaseNormalize( *testval );
  *limval = zLimit( angle, _rkp(joint)->min, _rkp(joint)->max );
}

static void _rkJointRevolSetDis(rkJoint *joint, double *val){
  _rkJointRevolLimDis( joint, val, &_rks(joint)->dis );
  zSinCos( _rks(joint)->dis, &_rks(joint)->_s, &_rks(joint)->_c );
}

static void _rkJointRevolSetMin(rkJoint *joint, double *val){
  _rkp(joint)->min = *val;
}

static void _rkJointRevolSetMax(rkJoint *joint, double *val){
  _rkp(joint)->max = *val;
}

static void _rkJointRevolSetVel(rkJoint *joint, double *val){
  _rks(joint)->vel = *val;
}

static void _rkJointRevolSetAcc(rkJoint *joint, double *val){
  _rks(joint)->acc = *val;
}

static void _rkJointRevolSetTrq(rkJoint *joint, double *val){
  _rks(joint)->trq = *val;
}

/* get joint displacement, velocity, acceleration and torque */

static void _rkJointRevolGetDis(rkJoint *joint, double *val){
  *val = _rks(joint)->dis;
}

static void _rkJointRevolGetMin(rkJoint *joint, double *val){
  *val = _rkp(joint)->min;
}

static void _rkJointRevolGetMax(rkJoint *joint, double *val){
  *val = _rkp(joint)->max;
}

static void _rkJointRevolGetVel(rkJoint *joint, double *val){
  *val = _rks(joint)->vel;
}

static void _rkJointRevolGetAcc(rkJoint *joint, double *val){
  *val = _rks(joint)->acc;
}

static void _rkJointRevolGetTrq(rkJoint *joint, double *val){
  *val = _rks(joint)->trq;
}

static void _rkJointRevolCatDis(rkJoint *joint, double *dis, double k, double *val){
  *dis += k * *val;
}

static void _rkJointRevolSubDis(rkJoint *joint, double *dis, double *sdis){
  *dis -= *sdis;
}

/* continuously update joint displacement */
static void _rkJointRevolSetDisCNT(rkJoint *joint, double *val, double dt)
{
  double olddis, oldvel;

  _rkJointRevolGetDis( joint, &olddis );
  _rkJointRevolGetVel( joint, &oldvel );
  _rkJointRevolSetDis( joint, val );
  _rks(joint)->vel = ( *val - olddis ) / dt;
  _rks(joint)->acc = ( _rks(joint)->vel - oldvel ) / dt;
}

/* joint frame transformation */
static zFrame3D *_rkJointRevolXform(rkJoint *joint, zFrame3D *fo, zFrame3D *f)
{
  /* position */
  zVec3DCopy( zFrame3DPos(fo), zFrame3DPos(f) );
  /* attitude */
  zVec3DMul( &zFrame3DAtt(fo)->v[0], _rks(joint)->_c, &zFrame3DAtt(f)->v[0] );
  zVec3DCatDRC( &zFrame3DAtt(f)->v[0], _rks(joint)->_s, &zFrame3DAtt(fo)->v[1] );
  zVec3DMul( &zFrame3DAtt(fo)->v[0],-_rks(joint)->_s, &zFrame3DAtt(f)->v[1] );
  zVec3DCatDRC( &zFrame3DAtt(f)->v[1], _rks(joint)->_c, &zFrame3DAtt(fo)->v[1] );
  zVec3DCopy( &zFrame3DAtt(fo)->v[2], &zFrame3DAtt(f)->v[2] );
  return f;
}

/* joint velocity transformation */
static void _rkJointRevolIncVel(rkJoint *joint, zVec6D *vel){
  vel->e[zZA] += _rks(joint)->vel;
}

static void _rkJointRevolIncAccOnVel(rkJoint *joint, zVec3D *w, zVec6D *acc){
  acc->e[zXA] += _rks(joint)->vel * w->e[zY];
  acc->e[zYA] -= _rks(joint)->vel * w->e[zX];
}

/* joint acceleration transformation */
static void _rkJointRevolIncAcc(rkJoint *joint, zVec6D *acc){
  acc->e[zZA] += _rks(joint)->acc;
}

/* joint torque transformation */
static void _rkJointRevolCalcTrq(rkJoint *joint, zVec6D *f){
  _rks(joint)->trq = f->e[zZA];
}

/* inverse computation of joint torsion and displacement */
static void _rkJointRevolTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  zMulMat3DTVec3D( zFrame3DAtt(dev), zFrame3DPos(dev), zVec6DLin(t) );
  dis[0] = rkJointRevolTorsionDis( dev, t );
}

static zVec3D* (*_rk_joint_revol_axis_ang[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisZ,
};

static zVec3D* (*_rk_joint_revol_axis_lin[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};

/* CRB method */

static void _rkJointRevolCRBWrench(rkJoint *joint, rkMP *crb, zVec6D wi[]){
  _rkJointCRBWrenchAngZ( crb, &wi[0] );
}
static void _rkJointRevolCRBXform(rkJoint *joint, zFrame3D *f, zVec6D si[]){
  _rkJointCRBXformAng( f, zZ, &si[0] );
}

static void _rkJointRevolSetFrictionPivot(rkJoint *joint, rkJointFrictionPivot *fp){
  _rks(joint)->_fp = *fp;
}

static void _rkJointRevolGetFrictionPivot(rkJoint *joint, rkJointFrictionPivot *fp){
  *fp = _rks(joint)->_fp;
}

static void _rkJointRevolSetFriction(rkJoint *joint, double *val){
  _rkp(joint)->tf = *val;
}

static void _rkJointRevolGetFriction(rkJoint *joint, double *val){
  *val = _rkp(joint)->tf;
}

static void _rkJointRevolGetSFriction(rkJoint *joint, double *val){
  *val = _rkp(joint)->sf;
}

static void _rkJointRevolGetKFriction(rkJoint *joint, double *val){
  *val = _rkJointRestTrq( _rkp(joint)->stiffness, _rkp(joint)->viscosity, _rkp(joint)->coulomb, _rks(joint)->dis, _rks(joint)->vel );
}

/* motor */

static void _rkJointRevolMotorSetInput(rkJoint *joint, double *val){
  rkMotorSetInput( rkJointMotor(joint), val );
}

static void _rkJointRevolMotorInertia(rkJoint *joint, double *val){
  *val = 0.0;
  rkMotorInertia( rkJointMotor(joint), val );
}

static void _rkJointRevolMotorInputTrq(rkJoint *joint, double *val){
  *val = 0.0;
  rkMotorInputTrq( rkJointMotor(joint), val );
}

static void _rkJointRevolMotorResistance(rkJoint *joint, double *val){
  *val = 0.0;
  rkMotorRegistance( rkJointMotor(joint), &_rks(joint)->dis, &_rks(joint)->vel, val );
}

static void _rkJointRevolMotorDrivingTrq(rkJoint *joint, double *val){
  *val = 0.0;
  rkMotorDrivingTrq( rkJointMotor(joint), &_rks(joint)->dis, &_rks(joint)->vel, &_rks(joint)->acc, val );
}

/* ABI */

static void _rkJointRevolABIAxisInertia(rkJoint *joint, zMat6D *m, zMat h, zMat ih)
{
  _rkJointRevolMotorInertia( joint, zMatBufNC(h) );
  zMatElemNC(h,0,0) += m->e[1][1].e[2][2];
  if( !zIsTiny( zMatElemNC(h,0,0) ) )
    zMatElemNC(ih,0,0) = 1.0 / zMatElemNC(h,0,0);
  else
    zMatElemNC(ih,0,0) = 0.0;
}

static void _rkJointRevolABIAddABI(rkJoint *joint, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm)
{
  zVec6D tmpv, tmpv2;
  zMat6D tmpm;

  zMat6DCol( m, zZA, &tmpv );
  zMat6DRow( m, zZA, &tmpv2 );
  _zVec6DMulDRC( &tmpv, -zMatElemNC(h,0,0) );
  _zMat6DDyad( &tmpm, &tmpv, &tmpv2 );
  _zMat6DAddDRC( &tmpm, m );
  rkJointXformMat6D( f, &tmpm, &tmpm );
  _zMat6DAddDRC( pm, &tmpm );
}

static void _rkJointRevolABIAddBias(rkJoint *joint, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb)
{
  zVec6D tmpv, tmpv2;

  zMat6DCol( m, zZA, &tmpv );
  _zVec6DMulDRC( &tmpv, -zMatElemNC(h,0,0) );
  zVec6DMulDRC( &tmpv, _rks(joint)->_u - b->e[zZA] );
  _zVec6DSub( b, &tmpv, &tmpv2 );

  zMulMat3DVec6D( zFrame3DAtt(f), &tmpv2, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  _zVec6DAddDRC( pb, &tmpv );
}

static void _rkJointRevolABIDrivingTorque(rkJoint *joint)
{
  double val;

  _rkJointRevolMotorInputTrq( joint, &_rks(joint)->_u );
  _rkJointRevolMotorResistance( joint, &val );
  _rks(joint)->_u -= val;
  _rks(joint)->_u += _rkp(joint)->tf;
}

static void _rkJointRevolABIQAcc(rkJoint *joint, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
{
  zVec6D tmpv;

  zMat6DRow( m, zZA, &tmpv );
  /* q */
  _rks(joint)->acc = zMatElemNC(h,0,0)*( _rks(joint)->_u - zVec6DInnerProd( &tmpv, jac ) - b->e[zZA] );
  /* acc */
  zVec6DCopy( jac, acc );
  acc->e[zZA] += _rks(joint)->acc;
}

/* ZTK */

static void *_rkJointRevolDisFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  double val;
  val = zDeg2Rad( ZTKDouble(ztk) );
  _rkJointRevolSetDis( joint, &val );
  return joint;
}
static void *_rkJointRevolMinFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->min = zDeg2Rad(ZTKDouble(ztk));
  return joint;
}
static void *_rkJointRevolMaxFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->max = zDeg2Rad(ZTKDouble(ztk));
  return joint;
}
static void *_rkJointRevolStiffnessFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->stiffness = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointRevolViscosityFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->viscosity = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointRevolCoulombFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->coulomb = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointRevolStaticFrictionFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->sf = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointRevolMotorFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  return rkJointMotorQuery( joint, arg, ZTKVal(ztk) );
}

static void _rkJointRevolDisFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", zRad2Deg(_rks(joint)->dis) );
}
static void _rkJointRevolMinFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", zRad2Deg(_rkp(joint)->min) );
}
static void _rkJointRevolMaxFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", zRad2Deg(_rkp(joint)->max) );
}
static void _rkJointRevolStiffnessFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkp(joint)->stiffness );
}
static void _rkJointRevolViscosityFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkp(joint)->viscosity );
}
static void _rkJointRevolCoulombFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkp(joint)->coulomb );
}
static void _rkJointRevolStaticFrictionFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkp(joint)->sf );
}

static ZTKPrp __ztk_prp_rkjoint_revol[] = {
  { "dis", 1, _rkJointRevolDisFromZTK, _rkJointRevolDisFPrintZTK },
  { "min", 1, _rkJointRevolMinFromZTK, _rkJointRevolMinFPrintZTK },
  { "max", 1, _rkJointRevolMaxFromZTK, _rkJointRevolMaxFPrintZTK },
  { "stiffness", 1, _rkJointRevolStiffnessFromZTK, _rkJointRevolStiffnessFPrintZTK },
  { "viscosity", 1, _rkJointRevolViscosityFromZTK, _rkJointRevolViscosityFPrintZTK },
  { "coulomb", 1, _rkJointRevolCoulombFromZTK, _rkJointRevolCoulombFPrintZTK },
  { "staticfriction", 1, _rkJointRevolStaticFrictionFromZTK, _rkJointRevolStaticFrictionFPrintZTK },
  { "motor", 1, _rkJointRevolMotorFromZTK, NULL },
};

static rkJoint *_rkJointRevolFromZTK(rkJoint *joint, rkMotorSpecArray *motorspecarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorspecarray, ztk, __ztk_prp_rkjoint_revol );
}

static void _rkJointRevolFPrintZTK(FILE *fp, rkJoint *joint, char *name)
{
  ZTKPrpKeyFPrint( fp, joint, __ztk_prp_rkjoint_revol );
  if( rkJointMotor( joint ) )
    fprintf( fp, "motor: %s\n", rkMotorName( rkJointMotor(joint) ) );
}

rkJointCom rk_joint_revol = {
  "revolute",
  1,
  _rkJointRevolInit,
  _rkJointRevolAllocState,
  _rkJointRevolAllocPrp,
  _rkJointRevolCopyPrp,
  _rkJointRevolLimDis,
  _rkJointRevolSetDis,
  _rkJointRevolSetMin,
  _rkJointRevolSetMax,
  _rkJointRevolSetVel,
  _rkJointRevolSetAcc,
  _rkJointRevolSetTrq,
  _rkJointRevolGetDis,
  _rkJointRevolGetMin,
  _rkJointRevolGetMax,
  _rkJointRevolGetVel,
  _rkJointRevolGetAcc,
  _rkJointRevolGetTrq,
  _rkJointRevolCatDis,
  _rkJointRevolSubDis,
  _rkJointRevolSetDisCNT,
  _rkJointRevolXform,
  _rkJointRevolIncVel,
  _rkJointRevolIncAccOnVel,
  _rkJointRevolIncAcc,
  _rkJointRevolCalcTrq,
  _rkJointRevolTorsion,
  _rk_joint_revol_axis_ang,
  _rk_joint_revol_axis_lin,

  _rkJointRevolCRBWrench,
  _rkJointRevolCRBXform,

  _rkJointRevolSetFrictionPivot,
  _rkJointRevolGetFrictionPivot,
  _rkJointRevolSetFriction,
  _rkJointRevolGetFriction,
  _rkJointRevolGetSFriction,
  _rkJointRevolGetKFriction,

  _rkJointRevolMotorSetInput,
  _rkJointRevolMotorInertia,
  _rkJointRevolMotorInputTrq,
  _rkJointRevolMotorResistance,
  _rkJointRevolMotorDrivingTrq,

  _rkJointRevolABIAxisInertia,
  _rkJointRevolABIAddABI,
  _rkJointRevolABIAddBias,
  _rkJointRevolABIDrivingTorque,
  _rkJointRevolABIQAcc,
  _rkJointUpdateWrench,

  _rkJointRevolDisFromZTK,
  _rkJointRevolFromZTK,
  _rkJointRevolDisFPrintZTK,
  _rkJointRevolFPrintZTK,
};

#undef _rks
#undef _rkp
