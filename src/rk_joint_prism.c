/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_prism - joint structure: prismatic joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointPrismPrp *)p)

static void _rkJointPrismInit(void *prp){
  _rkc(prp)->max = HUGE_VAL;
  _rkc(prp)->min =-HUGE_VAL;
  rkMotorAssign( &_rkc(prp)->m, &rk_motor_none );
}

static void *_rkJointPrismAlloc(void){ return zAlloc( rkJointPrismPrp, 1 ); }

/* limit joint displacement */
static void _rkJointPrismLimDis(void *prp, double *testval, double *limval){
  *limval = zLimit( *testval, _rkc(prp)->min, _rkc(prp)->max );
}

/* set joint displacement */
static void _rkJointPrismSetDis(void *prp, double *val){
  _rkJointPrismLimDis( prp, val, &_rkc(prp)->dis );
}

static void _rkJointPrismSetVel(void *prp, double *val){
  _rkc(prp)->vel = *val;
}

static void _rkJointPrismSetAcc(void *prp, double *val){
  _rkc(prp)->acc = *val;
}

static void _rkJointPrismSetTrq(void *prp, double *val){
  _rkc(prp)->trq = *val;
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointPrismGetDis(void *prp, double *val){
  *val = _rkc(prp)->dis;
}

static void _rkJointPrismGetVel(void *prp, double *val){
  *val = _rkc(prp)->vel;
}

static void _rkJointPrismGetAcc(void *prp, double *val){
  *val = _rkc(prp)->acc;
}

static void _rkJointPrismGetTrq(void *prp, double *val){
  *val = _rkc(prp)->trq;
}

static void _rkJointPrismCatDis(void *prp, double *dis, double k, double *val){
  *dis += k * *val;
}

static void _rkJointPrismSubDis(void *prp, double *dis, double *sdis){
  *dis -= *sdis;
}

/* continuously update joint displacement */
static void _rkJointPrismSetDisCNT(void *prp, double *val, double dt){
  double olddis, oldvel;

  olddis = _rkc(prp)->dis;
  oldvel = _rkc(prp)->vel;
  _rkJointPrismSetDis( prp, val );
  _rkc(prp)->vel = ( *val - olddis ) / dt;
  _rkc(prp)->acc = ( _rkc(prp)->vel - oldvel ) / dt;
}

/* joint frame transformation */
static zFrame3D *_rkJointPrismXform(void *prp, zFrame3D *fo, zFrame3D *f);

static zFrame3D *_rkJointPrismXform(void *prp, zFrame3D *fo, zFrame3D *f){
  zVec3DCat( zFrame3DPos(fo),
    _rkc(prp)->dis, &zFrame3DAtt(fo)->v[2], zFrame3DPos(f) );
  zMat3DCopy( zFrame3DAtt(fo), zFrame3DAtt(f) );
  return f;
}

/* joint velocity transformation */
static void _rkJointPrismIncVel(void *prp, zVec6D *vel){
  vel->e[zZ] += _rkc(prp)->vel;
}

static void _rkJointPrismIncAccOnVel(void *prp, zVec3D *w, zVec6D *acc){
  acc->e[zX] += 2 * _rkc(prp)->vel * w->e[zY];
  acc->e[zY] -= 2 * _rkc(prp)->vel * w->e[zX];
}

/* joint acceleration transformation */
static void _rkJointPrismIncAcc(void *prp, zVec6D *acc){
  acc->e[zZ] += _rkc(prp)->acc;
}

/* joint torque transformation */
static void _rkJointPrismCalcTrq(void *prp, zVec6D *f){
  _rkc(prp)->trq = f->e[zZ];
}

/* inverse computation of joint torsion and displacement */
static void _rkJointPrismTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  zVec3D aa;
  zMat3DToAA( zFrame3DAtt(dev), &aa );
  zMulMat3DTVec3D( zFrame3DAtt(dev), &aa, zVec6DAng(t) );
  dis[0] = rkJointPrismTorsionDis( dev, t );
}

static zVec3D* (*_rk_joint_prism_axis_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};
static zVec3D* (*_rk_joint_prism_axis_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisZ,
};

static void _rkJointPrismSetFrictionPivot(void *prp, rkJointFrictionPivot *fp){
  *fp = _rkc(prp)->_fp;
}

static void _rkJointPrismGetFrictionPivot(void *prp, rkJointFrictionPivot *fp){
  _rkc(prp)->_fp = *fp;
}

static void _rkJointPrismSetFric(void *prp, double *val){
  _rkc(prp)->tf = *val;
}

static void _rkJointPrismGetFric(void *prp, double *val){
  *val = _rkc(prp)->tf;
}

static void _rkJointPrismGetSFric(void *prp, double *val){
  *val = _rkc(prp)->sf;
}

static void _rkJointPrismGetKFric(void *prp, double *val){
  *val = _rkJointRestTrq( _rkc(prp)->stiffness, _rkc(prp)->viscosity, _rkc(prp)->coulomb, _rkc(prp)->dis, _rkc(prp)->vel );
}

/* motor */
static rkMotor *_rkJointPrismGetMotor(void *prp){ return &_rkc(prp)->m; }

static void _rkJointPrismMotorSetInput(void *prp, double *val){
  rkMotorSetInput( &_rkc(prp)->m, val );
}

static void _rkJointPrismMotorInertia(void *prp, double *val){
  *val = 0.0;
  rkMotorInertia( &_rkc(prp)->m, val );
}

static void _rkJointPrismMotorInputTrq(void *prp, double *val){
  *val = 0.0;
  rkMotorInputTrq( &_rkc(prp)->m, val );
}

static void _rkJointPrismMotorResistance(void *prp, double *val){
  *val = 0.0;
  rkMotorRegistance( &_rkc(prp)->m, &_rkc(prp)->dis, &_rkc(prp)->vel, val );
}

static void _rkJointPrismMotorDrivingTrq(void *prp, double *val){
  *val = 0.0;
  rkMotorDrivingTrq( &_rkc(prp)->m, &_rkc(prp)->dis, &_rkc(prp)->vel, &_rkc(prp)->acc, val );
}

/* ABI */
static void _rkJointPrismABIAxisInertia(void *prp, zMat6D *m, zMat h, zMat ih)
{
  _rkJointPrismMotorInertia( prp, zMatBuf(h) );
  zMatElemNC(h,0,0) += m->e[0][0].e[2][2];
  if( !zIsTiny( zMatElemNC(h,0,0) ) )
    zMatElemNC(ih,0,0) = 1.0 / zMatElemNC(h,0,0);
  else
    zMatElemNC(ih,0,0) = 0.0;
}

static void _rkJointPrismABIAddABI(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm)
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

static void _rkJointPrismABIAddBias(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb)
{
  zVec6D tmpv, tmpv2;

  zMat6DCol( m, zZ, &tmpv );
  zVec6DMulDRC( &tmpv, _rkc(prp)->_u - b->e[zZ] );
  zVec6DSub( b, &tmpv, &tmpv2 );

  zMulMat3DVec6D( zFrame3DAtt(f), &tmpv2, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}

static void _rkJointPrismABIDrivingTorque(void *prp)
{
  double val;

  _rkJointPrismMotorInputTrq( prp, &_rkc(prp)->_u );
  _rkJointPrismMotorResistance( prp, &val );
  _rkc(prp)->_u -= val;
  _rkc(prp)->_u += _rkc(prp)->tf;
}

static void _rkJointPrismABIQAcc(void *prp, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
{
  zVec6D tmpv;

  zMat6DRow( m, zZ, &tmpv );
  /* q */
  _rkc(prp)->acc = zMatElemNC(h,0,0)*( _rkc(prp)->_u - zVec6DInnerProd( &tmpv, jac ) - b->e[zZ] );
  /* acc */
  zVec6DCopy( jac, acc );
  acc->e[zZ] += _rkc(prp)->acc;
}

static void *_rkJointPrismDisFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  double val;
  val = zDeg2Rad( ZTKDouble(ztk) );
  _rkJointPrismSetDis( _rkc(prp), &val );
  return prp;
}
static void *_rkJointPrismMinFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->min = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointPrismMaxFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->max = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointPrismStiffnessFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->stiffness = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointPrismViscosityFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->viscosity = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointPrismCoulombFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->coulomb = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointPrismStaticFricFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->sf = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointPrismMotorFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk) ) ) ) return NULL;
  return rkMotorClone( mp, &_rkc(prp)->m ) ? prp : NULL;
}

static void _rkJointPrismDisFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->dis );
}
static void _rkJointPrismMinFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->min );
}
static void _rkJointPrismMaxFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->max );
}
static void _rkJointPrismStiffnessFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->stiffness );
}
static void _rkJointPrismViscosityFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->viscosity );
}
static void _rkJointPrismCoulombFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->coulomb );
}
static void _rkJointPrismStaticFricFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->sf );
}
static void _rkJointPrismMotorFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%s\n", zName(&_rkc(prp)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_prism[] = {
  { "dis", 1, _rkJointPrismDisFromZTK, _rkJointPrismDisFPrintZTK },
  { "min", 1, _rkJointPrismMinFromZTK, _rkJointPrismMinFPrintZTK },
  { "max", 1, _rkJointPrismMaxFromZTK, _rkJointPrismMaxFPrintZTK },
  { "stiffness", 1, _rkJointPrismStiffnessFromZTK, _rkJointPrismStiffnessFPrintZTK },
  { "viscosity", 1, _rkJointPrismViscosityFromZTK, _rkJointPrismViscosityFPrintZTK },
  { "coulomb", 1, _rkJointPrismCoulombFromZTK, _rkJointPrismCoulombFPrintZTK },
  { "staticfriction", 1, _rkJointPrismStaticFricFromZTK, _rkJointPrismStaticFricFPrintZTK },
  { "motor", 1, _rkJointPrismMotorFromZTK, _rkJointPrismMotorFPrintZTK },
};

static bool _rkJointPrismRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_prism ) ? true : false;
}

static void *_rkJointPrismFromZTK(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_prism );
}

static void _rkJointPrismFPrintZTK(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_prism );
}

rkJointCom rk_joint_prism = {
  "prismatic",
  1,
  _rkJointPrismInit,
  _rkJointPrismAlloc,
  _rkJointPrismLimDis,
  _rkJointPrismSetDis,
  _rkJointPrismSetVel,
  _rkJointPrismSetAcc,
  _rkJointPrismSetTrq,
  _rkJointPrismGetDis,
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

  _rkJointPrismSetFrictionPivot,
  _rkJointPrismGetFrictionPivot,
  _rkJointPrismSetFric,
  _rkJointPrismGetFric,
  _rkJointPrismGetSFric,
  _rkJointPrismGetKFric,

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

  _rkJointPrismRegZTK,
  _rkJointPrismDisFromZTK,
  _rkJointPrismFromZTK,
  _rkJointPrismDisFPrintZTK,
  _rkJointPrismFPrintZTK,
};

#undef _rkc
