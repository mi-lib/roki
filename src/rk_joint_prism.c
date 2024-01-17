/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_prism - joint structure: prismatic joint
 */

#include <roki/rk_joint.h>

#define _rks(joint) ((rkJointPrismState *)((rkJoint *)(joint))->state)
#define _rkp(joint) ((rkJointPrismPrp   *)((rkJoint *)(joint))->prp)

static void _rkJointPrismInit(rkJoint *joint){
  _rkp(joint)->max = HUGE_VAL;
  _rkp(joint)->min =-HUGE_VAL;
  rkMotorAssign( &_rkp(joint)->m, &rk_motor_none );
}

static void *_rkJointPrismAllocState(void){ return zAlloc( rkJointPrismState, 1 ); }
static void *_rkJointPrismAllocPrp(void){ return zAlloc( rkJointPrismPrp, 1 ); }

static void _rkJointPrismCopyPrp(rkJoint *src, rkJoint *dst){
  _rkp(dst)->min = _rkp(src)->min;
  _rkp(dst)->max = _rkp(src)->max;
  _rkp(dst)->stiffness = _rkp(src)->stiffness;
  _rkp(dst)->viscosity = _rkp(src)->viscosity;
  _rkp(dst)->coulomb = _rkp(src)->coulomb;
  _rkp(dst)->sf = _rkp(src)->sf;
}

/* limit joint displacement */
static void _rkJointPrismLimDis(rkJoint *joint, double *testval, double *limval){
  *limval = zLimit( *testval, _rkp(joint)->min, _rkp(joint)->max );
}

/* set joint displacement */
static void _rkJointPrismSetDis(rkJoint *joint, double *val){
  _rkJointPrismLimDis( joint, val, &_rks(joint)->dis );
}

static void _rkJointPrismSetMin(rkJoint *joint, double *val){
  _rkp(joint)->min = *val;
}

static void _rkJointPrismSetMax(rkJoint *joint, double *val){
  _rkp(joint)->max = *val;
}

static void _rkJointPrismSetVel(rkJoint *joint, double *val){
  _rks(joint)->vel = *val;
}

static void _rkJointPrismSetAcc(rkJoint *joint, double *val){
  _rks(joint)->acc = *val;
}

static void _rkJointPrismSetTrq(rkJoint *joint, double *val){
  _rks(joint)->trq = *val;
}

/* get joint displacement, velocity, acceleration and torque */

static void _rkJointPrismGetDis(rkJoint *joint, double *val){
  *val = _rks(joint)->dis;
}

static void _rkJointPrismGetMin(rkJoint *joint, double *val){
  *val = _rkp(joint)->min;
}

static void _rkJointPrismGetMax(rkJoint *joint, double *val){
  *val = _rkp(joint)->max;
}

static void _rkJointPrismGetVel(rkJoint *joint, double *val){
  *val = _rks(joint)->vel;
}

static void _rkJointPrismGetAcc(rkJoint *joint, double *val){
  *val = _rks(joint)->acc;
}

static void _rkJointPrismGetTrq(rkJoint *joint, double *val){
  *val = _rks(joint)->trq;
}

static void _rkJointPrismCatDis(rkJoint *joint, double *dis, double k, double *val){
  *dis += k * *val;
}

static void _rkJointPrismSubDis(rkJoint *joint, double *dis, double *sdis){
  *dis -= *sdis;
}

/* continuously update joint displacement */
static void _rkJointPrismSetDisCNT(rkJoint *joint, double *val, double dt){
  double olddis, oldvel;

  olddis = _rks(joint)->dis;
  oldvel = _rks(joint)->vel;
  _rkJointPrismSetDis( joint, val );
  _rks(joint)->vel = ( *val - olddis ) / dt;
  _rks(joint)->acc = ( _rks(joint)->vel - oldvel ) / dt;
}

/* joint frame transformation */
static zFrame3D *_rkJointPrismXform(rkJoint *joint, zFrame3D *fo, zFrame3D *f);

static zFrame3D *_rkJointPrismXform(rkJoint *joint, zFrame3D *fo, zFrame3D *f){
  zVec3DCat( zFrame3DPos(fo),
    _rks(joint)->dis, &zFrame3DAtt(fo)->v[2], zFrame3DPos(f) );
  zMat3DCopy( zFrame3DAtt(fo), zFrame3DAtt(f) );
  return f;
}

/* joint velocity transformation */
static void _rkJointPrismIncVel(rkJoint *joint, zVec6D *vel){
  vel->e[zZ] += _rks(joint)->vel;
}

static void _rkJointPrismIncAccOnVel(rkJoint *joint, zVec3D *w, zVec6D *acc){
  acc->e[zX] += 2 * _rks(joint)->vel * w->e[zY];
  acc->e[zY] -= 2 * _rks(joint)->vel * w->e[zX];
}

/* joint acceleration transformation */
static void _rkJointPrismIncAcc(rkJoint *joint, zVec6D *acc){
  acc->e[zZ] += _rks(joint)->acc;
}

/* joint torque transformation */
static void _rkJointPrismCalcTrq(rkJoint *joint, zVec6D *f){
  _rks(joint)->trq = f->e[zZ];
}

/* inverse computation of joint torsion and displacement */
static void _rkJointPrismTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  zVec3D aa;
  zMat3DToAA( zFrame3DAtt(dev), &aa );
  zMulMat3DTVec3D( zFrame3DAtt(dev), &aa, zVec6DAng(t) );
  dis[0] = rkJointPrismTorsionDis( dev, t );
}

static zVec3D* (*_rk_joint_prism_axis_ang[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};
static zVec3D* (*_rk_joint_prism_axis_lin[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisZ,
};

/* CRB method */

static void _rkJointPrismCRBWrench(rkJoint *joint, rkMP *crb, zVec6D wi[]){
  _rkJointCRBWrenchLinZ( crb, &wi[0] );
}
static void _rkJointPrismCRBXform(rkJoint *joint, zFrame3D *f, zVec6D si[]){
  _rkJointCRBXformLin( f, zZ, &si[0] );
}

static void _rkJointPrismSetFrictionPivot(rkJoint *joint, rkJointFrictionPivot *fp){
  *fp = _rks(joint)->_fp;
}

static void _rkJointPrismGetFrictionPivot(rkJoint *joint, rkJointFrictionPivot *fp){
  _rks(joint)->_fp = *fp;
}

static void _rkJointPrismSetFriction(rkJoint *joint, double *val){
  _rkp(joint)->tf = *val;
}

static void _rkJointPrismGetFriction(rkJoint *joint, double *val){
  *val = _rkp(joint)->tf;
}

static void _rkJointPrismGetSFriction(rkJoint *joint, double *val){
  *val = _rkp(joint)->sf;
}

static void _rkJointPrismGetKFriction(rkJoint *joint, double *val){
  *val = _rkJointRestTrq( _rkp(joint)->stiffness, _rkp(joint)->viscosity, _rkp(joint)->coulomb, _rks(joint)->dis, _rks(joint)->vel );
}

/* motor */

static rkMotor *_rkJointPrismGetMotor(rkJoint *joint){ return &_rkp(joint)->m; }

static void _rkJointPrismMotorSetInput(rkJoint *joint, double *val){
  rkMotorSetInput( &_rkp(joint)->m, val );
}

static void _rkJointPrismMotorInertia(rkJoint *joint, double *val){
  *val = 0.0;
  rkMotorInertia( &_rkp(joint)->m, val );
}

static void _rkJointPrismMotorInputTrq(rkJoint *joint, double *val){
  *val = 0.0;
  rkMotorInputTrq( &_rkp(joint)->m, val );
}

static void _rkJointPrismMotorResistance(rkJoint *joint, double *val){
  *val = 0.0;
  rkMotorRegistance( &_rkp(joint)->m, &_rks(joint)->dis, &_rks(joint)->vel, val );
}

static void _rkJointPrismMotorDrivingTrq(rkJoint *joint, double *val){
  *val = 0.0;
  rkMotorDrivingTrq( &_rkp(joint)->m, &_rks(joint)->dis, &_rks(joint)->vel, &_rks(joint)->acc, val );
}

/* ABI */

static void _rkJointPrismABIAxisInertia(rkJoint *joint, zMat6D *m, zMat h, zMat ih)
{
  _rkJointPrismMotorInertia( joint, zMatBufNC(h) );
  zMatElemNC(h,0,0) += m->e[0][0].e[2][2];
  if( !zIsTiny( zMatElemNC(h,0,0) ) )
    zMatElemNC(ih,0,0) = 1.0 / zMatElemNC(h,0,0);
  else
    zMatElemNC(ih,0,0) = 0.0;
}

static void _rkJointPrismABIAddABI(rkJoint *joint, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm)
{
  zVec6D tmpv, tmpv2;
  zMat6D tmpm;

  zMat6DCol( m, zZ, &tmpv );
  zMat6DRow( m, zZ, &tmpv2 );
  zVec6DMulDRC( &tmpv, -zMatElemNC(h,0,0) );
  zMat6DDyad( &tmpm, &tmpv, &tmpv2 );
  zMat6DAddDRC( &tmpm, m );
  rkJointXformMat6D( f, &tmpm, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}

static void _rkJointPrismABIAddBias(rkJoint *joint, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb)
{
  zVec6D tmpv, tmpv2;

  zMat6DCol( m, zZ, &tmpv );
  zVec6DMulDRC( &tmpv, _rks(joint)->_u - b->e[zZ] );
  zVec6DSub( b, &tmpv, &tmpv2 );

  zMulMat3DVec6D( zFrame3DAtt(f), &tmpv2, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}

static void _rkJointPrismABIDrivingTorque(rkJoint *joint)
{
  double val;

  _rkJointPrismMotorInputTrq( joint, &_rks(joint)->_u );
  _rkJointPrismMotorResistance( joint, &val );
  _rks(joint)->_u -= val;
  _rks(joint)->_u += _rkp(joint)->tf;
}

static void _rkJointPrismABIQAcc(rkJoint *joint, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
{
  zVec6D tmpv;

  zMat6DRow( m, zZ, &tmpv );
  /* q */
  _rks(joint)->acc = zMatElemNC(h,0,0)*( _rks(joint)->_u - zVec6DInnerProd( &tmpv, jac ) - b->e[zZ] );
  /* acc */
  zVec6DCopy( jac, acc );
  acc->e[zZ] += _rks(joint)->acc;
}

/* ZTK */

static void *_rkJointPrismDisFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  double val;
  val = ZTKDouble(ztk);
  _rkJointPrismSetDis( joint, &val );
  return joint;
}
static void *_rkJointPrismMinFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->min = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointPrismMaxFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->max = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointPrismStiffnessFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->stiffness = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointPrismViscosityFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->viscosity = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointPrismCoulombFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->coulomb = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointPrismStaticFrictionFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->sf = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointPrismMotorFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( (rkMotorArray *)arg, ZTKVal(ztk) ) ) ) return NULL;
  return rkMotorClone( mp, &_rkp(joint)->m ) ? joint : NULL;
}

static void _rkJointPrismDisFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rks(joint)->dis );
}
static void _rkJointPrismMinFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkp(joint)->min );
}
static void _rkJointPrismMaxFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkp(joint)->max );
}
static void _rkJointPrismStiffnessFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkp(joint)->stiffness );
}
static void _rkJointPrismViscosityFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkp(joint)->viscosity );
}
static void _rkJointPrismCoulombFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkp(joint)->coulomb );
}
static void _rkJointPrismStaticFrictionFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkp(joint)->sf );
}

static ZTKPrp __ztk_prp_rkjoint_prism[] = {
  { "dis", 1, _rkJointPrismDisFromZTK, _rkJointPrismDisFPrintZTK },
  { "min", 1, _rkJointPrismMinFromZTK, _rkJointPrismMinFPrintZTK },
  { "max", 1, _rkJointPrismMaxFromZTK, _rkJointPrismMaxFPrintZTK },
  { "stiffness", 1, _rkJointPrismStiffnessFromZTK, _rkJointPrismStiffnessFPrintZTK },
  { "viscosity", 1, _rkJointPrismViscosityFromZTK, _rkJointPrismViscosityFPrintZTK },
  { "coulomb", 1, _rkJointPrismCoulombFromZTK, _rkJointPrismCoulombFPrintZTK },
  { "staticfriction", 1, _rkJointPrismStaticFrictionFromZTK, _rkJointPrismStaticFrictionFPrintZTK },
  { "motor", 1, _rkJointPrismMotorFromZTK, NULL },
};

static rkJoint *_rkJointPrismFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorarray, ztk, __ztk_prp_rkjoint_prism );
}

static void _rkJointPrismFPrintZTK(FILE *fp, rkJoint *joint, char *name)
{
  ZTKPrpKeyFPrint( fp, joint, __ztk_prp_rkjoint_prism );
  if( rkMotorIsAssigned( &_rkp(joint)->m ) )
    fprintf( fp, "motor: %s\n", zName(&_rkp(joint)->m) );
}

rkJointCom rk_joint_prism = {
  "prismatic",
  1,
  _rkJointPrismInit,
  _rkJointPrismAllocState,
  _rkJointPrismAllocPrp,
  _rkJointPrismCopyPrp,
  _rkJointPrismLimDis,
  _rkJointPrismSetDis,
  _rkJointPrismSetMin,
  _rkJointPrismSetMax,
  _rkJointPrismSetVel,
  _rkJointPrismSetAcc,
  _rkJointPrismSetTrq,
  _rkJointPrismGetDis,
  _rkJointPrismGetMin,
  _rkJointPrismGetMax,
  _rkJointPrismGetVel,
  _rkJointPrismGetAcc,
  _rkJointPrismGetTrq,
  _rkJointPrismCatDis,
  _rkJointPrismSubDis,
  _rkJointPrismSetDisCNT,
  _rkJointPrismXform,
  _rkJointPrismIncVel,
  _rkJointPrismIncAccOnVel,
  _rkJointPrismIncAcc,
  _rkJointPrismCalcTrq,
  _rkJointPrismTorsion,
  _rk_joint_prism_axis_ang,
  _rk_joint_prism_axis_lin,

  _rkJointPrismCRBWrench,
  _rkJointPrismCRBXform,

  _rkJointPrismSetFrictionPivot,
  _rkJointPrismGetFrictionPivot,
  _rkJointPrismSetFriction,
  _rkJointPrismGetFriction,
  _rkJointPrismGetSFriction,
  _rkJointPrismGetKFriction,

  _rkJointPrismGetMotor,
  _rkJointPrismMotorSetInput,
  _rkJointPrismMotorInertia,
  _rkJointPrismMotorInputTrq,
  _rkJointPrismMotorResistance,
  _rkJointPrismMotorDrivingTrq,

  _rkJointPrismABIAxisInertia,
  _rkJointPrismABIAddABI,
  _rkJointPrismABIAddBias,
  _rkJointPrismABIDrivingTorque,
  _rkJointPrismABIQAcc,
  _rkJointUpdateWrench,

  _rkJointPrismDisFromZTK,
  _rkJointPrismFromZTK,
  _rkJointPrismDisFPrintZTK,
  _rkJointPrismFPrintZTK,
};

#undef _rks
#undef _rkp
