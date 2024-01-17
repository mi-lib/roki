/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_revol - joint structure: revolutional joint
 */

#include <roki/rk_joint.h>

#define _rkc(joint) ((rkJointRevolPrp *)((rkJoint *)(joint))->prp)

static void _rkJointRevolInit(rkJoint *joint){
  _rkc(joint)->max = HUGE_VAL;
  _rkc(joint)->min =-HUGE_VAL;
  rkMotorAssign( &_rkc(joint)->m, &rk_motor_none );
}

static void *_rkJointRevolAllocPrp(void){ return zAlloc( rkJointRevolPrp, 1 ); }

static void _rkJointRevolCopyPrp(rkJoint *src, rkJoint *dst){
  _rkc(dst)->min = _rkc(src)->min;
  _rkc(dst)->max = _rkc(src)->max;
  _rkc(dst)->stiffness = _rkc(src)->stiffness;
  _rkc(dst)->viscosity = _rkc(src)->viscosity;
  _rkc(dst)->coulomb = _rkc(src)->coulomb;
  _rkc(dst)->sf = _rkc(src)->sf;
}

/* set joint displacement, velocity, acceleration and torque */

static void _rkJointRevolLimDis(rkJoint *joint, double *testval, double *limval){
  double angle;
  angle = zPhaseNormalize( *testval );
  *limval = zLimit( angle, _rkc(joint)->min, _rkc(joint)->max );
}

static void _rkJointRevolSetDis(rkJoint *joint, double *val){
  _rkJointRevolLimDis( joint, val, &_rkc(joint)->dis );
  zSinCos( _rkc(joint)->dis, &_rkc(joint)->_s, &_rkc(joint)->_c );
}

static void _rkJointRevolSetMin(rkJoint *joint, double *val){
  _rkc(joint)->min = *val;
}

static void _rkJointRevolSetMax(rkJoint *joint, double *val){
  _rkc(joint)->max = *val;
}

static void _rkJointRevolSetVel(rkJoint *joint, double *val){
  _rkc(joint)->vel = *val;
}

static void _rkJointRevolSetAcc(rkJoint *joint, double *val){
  _rkc(joint)->acc = *val;
}

static void _rkJointRevolSetTrq(rkJoint *joint, double *val){
  _rkc(joint)->trq = *val;
}

/* get joint displacement, velocity, acceleration and torque */

static void _rkJointRevolGetDis(rkJoint *joint, double *val){
  *val = _rkc(joint)->dis;
}

static void _rkJointRevolGetMin(rkJoint *joint, double *val){
  *val = _rkc(joint)->min;
}

static void _rkJointRevolGetMax(rkJoint *joint, double *val){
  *val = _rkc(joint)->max;
}

static void _rkJointRevolGetVel(rkJoint *joint, double *val){
  *val = _rkc(joint)->vel;
}

static void _rkJointRevolGetAcc(rkJoint *joint, double *val){
  *val = _rkc(joint)->acc;
}

static void _rkJointRevolGetTrq(rkJoint *joint, double *val){
  *val = _rkc(joint)->trq;
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
  _rkc(joint)->vel = ( *val - olddis ) / dt;
  _rkc(joint)->acc = ( _rkc(joint)->vel - oldvel ) / dt;
}

/* joint frame transformation */
static zFrame3D *_rkJointRevolXform(rkJoint *joint, zFrame3D *fo, zFrame3D *f)
{
  /* position */
  zVec3DCopy( zFrame3DPos(fo), zFrame3DPos(f) );
  /* attitude */
  zVec3DMul( &zFrame3DAtt(fo)->v[0], _rkc(joint)->_c, &zFrame3DAtt(f)->v[0] );
  zVec3DCatDRC( &zFrame3DAtt(f)->v[0], _rkc(joint)->_s, &zFrame3DAtt(fo)->v[1] );
  zVec3DMul( &zFrame3DAtt(fo)->v[0],-_rkc(joint)->_s, &zFrame3DAtt(f)->v[1] );
  zVec3DCatDRC( &zFrame3DAtt(f)->v[1], _rkc(joint)->_c, &zFrame3DAtt(fo)->v[1] );
  zVec3DCopy( &zFrame3DAtt(fo)->v[2], &zFrame3DAtt(f)->v[2] );
  return f;
}

/* joint velocity transformation */
static void _rkJointRevolIncVel(rkJoint *joint, zVec6D *vel){
  vel->e[zZA] += _rkc(joint)->vel;
}

static void _rkJointRevolIncAccOnVel(rkJoint *joint, zVec3D *w, zVec6D *acc){
  acc->e[zXA] += _rkc(joint)->vel * w->e[zY];
  acc->e[zYA] -= _rkc(joint)->vel * w->e[zX];
}

/* joint acceleration transformation */
static void _rkJointRevolIncAcc(rkJoint *joint, zVec6D *acc){
  acc->e[zZA] += _rkc(joint)->acc;
}

/* joint torque transformation */
static void _rkJointRevolCalcTrq(rkJoint *joint, zVec6D *f){
  _rkc(joint)->trq = f->e[zZA];
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
  _rkc(joint)->_fp = *fp;
}

static void _rkJointRevolGetFrictionPivot(rkJoint *joint, rkJointFrictionPivot *fp){
  *fp = _rkc(joint)->_fp;
}

static void _rkJointRevolSetFriction(rkJoint *joint, double *val){
  _rkc(joint)->tf = *val;
}

static void _rkJointRevolGetFriction(rkJoint *joint, double *val){
  *val = _rkc(joint)->tf;
}

static void _rkJointRevolGetSFriction(rkJoint *joint, double *val){
  *val = _rkc(joint)->sf;
}

static void _rkJointRevolGetKFriction(rkJoint *joint, double *val){
  *val = _rkJointRestTrq( _rkc(joint)->stiffness, _rkc(joint)->viscosity, _rkc(joint)->coulomb, _rkc(joint)->dis, _rkc(joint)->vel );
}

/* motor */

static rkMotor *_rkJointRevolGetMotor(rkJoint *joint){ return &_rkc(joint)->m; }

static void _rkJointRevolMotorSetInput(rkJoint *joint, double *val){
  rkMotorSetInput( &_rkc(joint)->m, val );
}

static void _rkJointRevolMotorInertia(rkJoint *joint, double *val){
  *val = 0.0;
  rkMotorInertia( &_rkc(joint)->m, val );
}

static void _rkJointRevolMotorInputTrq(rkJoint *joint, double *val){
  *val = 0.0;
  rkMotorInputTrq( &_rkc(joint)->m, val );
}

static void _rkJointRevolMotorResistance(rkJoint *joint, double *val){
  *val = 0.0;
  rkMotorRegistance( &_rkc(joint)->m, &_rkc(joint)->dis, &_rkc(joint)->vel, val );
}

static void _rkJointRevolMotorDrivingTrq(rkJoint *joint, double *val){
  *val = 0.0;
  rkMotorDrivingTrq( &_rkc(joint)->m, &_rkc(joint)->dis, &_rkc(joint)->vel, &_rkc(joint)->acc, val );
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
  zVec6DMulDRC( &tmpv, -zMatElemNC(h,0,0) );
  zMat6DDyad( &tmpm, &tmpv, &tmpv2 );
  zMat6DAddDRC( &tmpm, m );
  rkJointXformMat6D( f, &tmpm, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}

static void _rkJointRevolABIAddBias(rkJoint *joint, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb)
{
  zVec6D tmpv, tmpv2;

  zMat6DCol( m, zZA, &tmpv );
  zVec6DMulDRC( &tmpv, -zMatElemNC(h,0,0) );
  zVec6DMulDRC( &tmpv, _rkc(joint)->_u - b->e[zZA] );
  zVec6DSub( b, &tmpv, &tmpv2 );

  zMulMat3DVec6D( zFrame3DAtt(f), &tmpv2, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}

static void _rkJointRevolABIDrivingTorque(rkJoint *joint)
{
  double val;

  _rkJointRevolMotorInputTrq( joint, &_rkc(joint)->_u );
  _rkJointRevolMotorResistance( joint, &val );
  _rkc(joint)->_u -= val;
  _rkc(joint)->_u += _rkc(joint)->tf;
}

static void _rkJointRevolABIQAcc(rkJoint *joint, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
{
  zVec6D tmpv;

  zMat6DRow( m, zZA, &tmpv );
  /* q */
  _rkc(joint)->acc = zMatElemNC(h,0,0)*( _rkc(joint)->_u - zVec6DInnerProd( &tmpv, jac ) - b->e[zZA] );
  /* acc */
  zVec6DCopy( jac, acc );
  acc->e[zZA] += _rkc(joint)->acc;
}

/* ZTK */

static void *_rkJointRevolDisFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  double val;
  val = zDeg2Rad( ZTKDouble(ztk) );
  _rkJointRevolSetDis( joint, &val );
  return joint;
}
static void *_rkJointRevolMinFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->min = zDeg2Rad(ZTKDouble(ztk));
  return joint;
}
static void *_rkJointRevolMaxFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->max = zDeg2Rad(ZTKDouble(ztk));
  return joint;
}
static void *_rkJointRevolStiffnessFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->stiffness = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointRevolViscosityFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->viscosity = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointRevolCoulombFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->coulomb = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointRevolStaticFrictionFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->sf = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointRevolMotorFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( (rkMotorArray *)arg, ZTKVal(ztk) ) ) ) return NULL;
  return rkMotorClone( mp, &_rkc(joint)->m ) ? joint : NULL;
}

static void _rkJointRevolDisFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", zRad2Deg(_rkc(joint)->dis) );
}
static void _rkJointRevolMinFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", zRad2Deg(_rkc(joint)->min) );
}
static void _rkJointRevolMaxFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", zRad2Deg(_rkc(joint)->max) );
}
static void _rkJointRevolStiffnessFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkc(joint)->stiffness );
}
static void _rkJointRevolViscosityFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkc(joint)->viscosity );
}
static void _rkJointRevolCoulombFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkc(joint)->coulomb );
}
static void _rkJointRevolStaticFrictionFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkc(joint)->sf );
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

static rkJoint *_rkJointRevolFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorarray, ztk, __ztk_prp_rkjoint_revol );
}

static void _rkJointRevolFPrintZTK(FILE *fp, rkJoint *joint, char *name)
{
  ZTKPrpKeyFPrint( fp, joint, __ztk_prp_rkjoint_revol );
  if( rkMotorIsAssigned( &_rkc(joint)->m ) )
    fprintf( fp, "motor: %s\n", zName(&_rkc(joint)->m) );
}

rkJointCom rk_joint_revol = {
  "revolute",
  1,
  _rkJointRevolInit,
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

  _rkJointRevolGetMotor,
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

#undef _rkc
