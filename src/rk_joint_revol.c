/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_revol - joint structure: revolutional joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointRevolPrp *)p)

static void _rkJointRevolInit(void *prp){
  _rkc(prp)->max = HUGE_VAL;
  _rkc(prp)->min =-HUGE_VAL;
  rkMotorAssign( &_rkc(prp)->m, &rk_motor_none );
}

static void *_rkJointRevolAlloc(void){ return zAlloc( rkJointRevolPrp, 1 ); }

/* limit joint displacement */
static void _rkJointRevolLimDis(void *prp, double *testval, double *limval){
  double angle;
  angle = zPhaseNormalize( *testval );
  *limval = zLimit( angle, _rkc(prp)->min, _rkc(prp)->max );
}

/* set joint displacement, velocity, acceleration and torque */
static void _rkJointRevolSetDis(void *prp, double *val){
  _rkJointRevolLimDis( prp, val, &_rkc(prp)->dis );
  zSinCos( _rkc(prp)->dis, &_rkc(prp)->_s, &_rkc(prp)->_c );
}

static void _rkJointRevolSetVel(void *prp, double *val){
  _rkc(prp)->vel = *val;
}

static void _rkJointRevolSetAcc(void *prp, double *val){
  _rkc(prp)->acc = *val;
}

static void _rkJointRevolSetTrq(void *prp, double *val){
  _rkc(prp)->trq = *val;
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointRevolGetDis(void *prp, double *val){
  *val = _rkc(prp)->dis;
}

static void _rkJointRevolGetVel(void *prp, double *val){
  *val = _rkc(prp)->vel;
}

static void _rkJointRevolGetAcc(void *prp, double *val){
  *val = _rkc(prp)->acc;
}

static void _rkJointRevolGetTrq(void *prp, double *val){
  *val = _rkc(prp)->trq;
}

static void _rkJointRevolCatDis(void *prp, double *dis, double k, double *val){
  *dis += k * *val;
}

static void _rkJointRevolSubDis(void *prp, double *dis, double *sdis){
  *dis -= *sdis;
}

/* continuously update joint displacement */
static void _rkJointRevolSetDisCNT(void *prp, double *val, double dt)
{
  double olddis, oldvel;

  _rkJointRevolGetDis( prp, &olddis );
  _rkJointRevolGetVel( prp, &oldvel );
  _rkJointRevolSetDis( prp, val );
  _rkc(prp)->vel = ( *val - olddis ) / dt;
  _rkc(prp)->acc = ( _rkc(prp)->vel - oldvel ) / dt;
}

/* joint frame transformation */
static zFrame3D *_rkJointRevolXform(void *prp, zFrame3D *fo, zFrame3D *f)
{
  /* position */
  zVec3DCopy( zFrame3DPos(fo), zFrame3DPos(f) );
  /* attitude */
  zVec3DMul( &zFrame3DAtt(fo)->v[0], _rkc(prp)->_c, &zFrame3DAtt(f)->v[0] );
  zVec3DCatDRC( &zFrame3DAtt(f)->v[0], _rkc(prp)->_s, &zFrame3DAtt(fo)->v[1] );
  zVec3DMul( &zFrame3DAtt(fo)->v[0],-_rkc(prp)->_s, &zFrame3DAtt(f)->v[1] );
  zVec3DCatDRC( &zFrame3DAtt(f)->v[1], _rkc(prp)->_c, &zFrame3DAtt(fo)->v[1] );
  zVec3DCopy( &zFrame3DAtt(fo)->v[2], &zFrame3DAtt(f)->v[2] );
  return f;
}

/* joint velocity transformation */
static void _rkJointRevolIncVel(void *prp, zVec6D *vel){
  vel->e[zZA] += _rkc(prp)->vel;
}

static void _rkJointRevolIncAccOnVel(void *prp, zVec3D *w, zVec6D *acc){
  acc->e[zXA] += _rkc(prp)->vel * w->e[zY];
  acc->e[zYA] -= _rkc(prp)->vel * w->e[zX];
}

/* joint acceleration transformation */
static void _rkJointRevolIncAcc(void *prp, zVec6D *acc){
  acc->e[zZA] += _rkc(prp)->acc;
}

/* joint torque transformation */
static void _rkJointRevolCalcTrq(void *prp, zVec6D *f){
  _rkc(prp)->trq = f->e[zZA];
}

/* inverse computation of joint torsion and displacement */
static void _rkJointRevolTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  zMulMat3DTVec3D( zFrame3DAtt(dev), zFrame3DPos(dev), zVec6DLin(t) );
  dis[0] = rkJointRevolTorsionDis( dev, t );
}

static zVec3D* (*_rk_joint_revol_axis_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisZ,
};

static zVec3D* (*_rk_joint_revol_axis_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};

static void _rkJointRevolSetFrictionPivot(void *prp, rkJointFrictionPivot *fp){
  _rkc(prp)->_fp = *fp;
}

static void _rkJointRevolGetFrictionPivot(void *prp, rkJointFrictionPivot *fp){
  *fp = _rkc(prp)->_fp;
}

static void _rkJointRevolSetFriction(void *prp, double *val){
  _rkc(prp)->tf = *val;
}

static void _rkJointRevolGetFriction(void *prp, double *val){
  *val = _rkc(prp)->tf;
}

static void _rkJointRevolGetSFriction(void *prp, double *val){
  *val = _rkc(prp)->sf;
}

static void _rkJointRevolGetKFriction(void *prp, double *val){
  *val = _rkJointRestTrq( _rkc(prp)->stiffness, _rkc(prp)->viscosity, _rkc(prp)->coulomb, _rkc(prp)->dis, _rkc(prp)->vel );
}

/* motor */
static rkMotor *_rkJointRevolGetMotor(void *prp){ return &_rkc(prp)->m; }

static void _rkJointRevolMotorSetInput(void *prp, double *val){
  rkMotorSetInput( &_rkc(prp)->m, val );
}

static void _rkJointRevolMotorInertia(void *prp, double *val){
  *val = 0.0;
  rkMotorInertia( &_rkc(prp)->m, val );
}

static void _rkJointRevolMotorInputTrq(void *prp, double *val){
  *val = 0.0;
  rkMotorInputTrq( &_rkc(prp)->m, val );
}

static void _rkJointRevolMotorResistance(void *prp, double *val){
  *val = 0.0;
  rkMotorRegistance( &_rkc(prp)->m, &_rkc(prp)->dis, &_rkc(prp)->vel, val );
}

static void _rkJointRevolMotorDrivingTrq(void *prp, double *val){
  *val = 0.0;
  rkMotorDrivingTrq( &_rkc(prp)->m, &_rkc(prp)->dis, &_rkc(prp)->vel, &_rkc(prp)->acc, val );
}

/* ABI */
static void _rkJointRevolABIAxisInertia(void *prp, zMat6D *m, zMat h, zMat ih)
{
  _rkJointRevolMotorInertia( prp, zMatBuf(h) );
  zMatElemNC(h,0,0) += m->e[1][1].e[2][2];
  if( !zIsTiny( zMatElemNC(h,0,0) ) )
    zMatElemNC(ih,0,0) = 1.0 / zMatElemNC(h,0,0);
  else
    zMatElemNC(ih,0,0) = 0.0;
}

static void _rkJointRevolABIAddABI(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm)
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

static void _rkJointRevolABIAddBias(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb)
{
  zVec6D tmpv, tmpv2;

  zMat6DCol( m, zZA, &tmpv );
  zVec6DMulDRC( &tmpv, -zMatElemNC(h,0,0) );
  zVec6DMulDRC( &tmpv, _rkc(prp)->_u - b->e[zZA] );
  zVec6DSub( b, &tmpv, &tmpv2 );

  zMulMat3DVec6D( zFrame3DAtt(f), &tmpv2, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}

static void _rkJointRevolABIDrivingTorque(void *prp)
{
  double val;

  _rkJointRevolMotorInputTrq( prp, &_rkc(prp)->_u );
  _rkJointRevolMotorResistance( prp, &val );
  _rkc(prp)->_u -= val;
  _rkc(prp)->_u += _rkc(prp)->tf;
}

static void _rkJointRevolABIQAcc(void *prp, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
{
  zVec6D tmpv;

  zMat6DRow( m, zZA, &tmpv );
  /* q */
  _rkc(prp)->acc = zMatElemNC(h,0,0)*( _rkc(prp)->_u - zVec6DInnerProd( &tmpv, jac ) - b->e[zZA] );
  /* acc */
  zVec6DCopy( jac, acc );
  acc->e[zZA] += _rkc(prp)->acc;
}

static void *_rkJointRevolDisFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  double val;
  val = zDeg2Rad( ZTKDouble(ztk) );
  _rkJointRevolSetDis( prp, &val );
  return prp;
}
static void *_rkJointRevolMinFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->min = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointRevolMaxFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->max = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointRevolStiffnessFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->stiffness = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointRevolViscosityFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->viscosity = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointRevolCoulombFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->coulomb = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointRevolStaticFrictionFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->sf = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointRevolMotorFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk) ) ) ) return NULL;
  return rkMotorClone( mp, &_rkc(prp)->m ) ? prp : NULL;
}

static void _rkJointRevolDisFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", zRad2Deg(_rkc(prp)->dis) );
}
static void _rkJointRevolMinFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", zRad2Deg(_rkc(prp)->min) );
}
static void _rkJointRevolMaxFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", zRad2Deg(_rkc(prp)->max) );
}
static void _rkJointRevolStiffnessFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->stiffness );
}
static void _rkJointRevolViscosityFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->viscosity );
}
static void _rkJointRevolCoulombFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->coulomb );
}
static void _rkJointRevolStaticFrictionFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->sf );
}
static void _rkJointRevolMotorFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%s\n", zName(&_rkc(prp)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_revol[] = {
  { "dis", 1, _rkJointRevolDisFromZTK, _rkJointRevolDisFPrintZTK },
  { "min", 1, _rkJointRevolMinFromZTK, _rkJointRevolMinFPrintZTK },
  { "max", 1, _rkJointRevolMaxFromZTK, _rkJointRevolMaxFPrintZTK },
  { "stiffness", 1, _rkJointRevolStiffnessFromZTK, _rkJointRevolStiffnessFPrintZTK },
  { "viscosity", 1, _rkJointRevolViscosityFromZTK, _rkJointRevolViscosityFPrintZTK },
  { "coulomb", 1, _rkJointRevolCoulombFromZTK, _rkJointRevolCoulombFPrintZTK },
  { "staticfriction", 1, _rkJointRevolStaticFrictionFromZTK, _rkJointRevolStaticFrictionFPrintZTK },
  { "motor", 1, _rkJointRevolMotorFromZTK, _rkJointRevolMotorFPrintZTK },
};

static bool _rkJointRevolRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_revol ) ? true : false;
}

static void *_rkJointRevolFromZTK(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_revol );
}

static void _rkJointRevolFPrintZTK(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_revol );
}

rkJointCom rk_joint_revol = {
  "revolute",
  1,
  _rkJointRevolInit,
  _rkJointRevolAlloc,
  _rkJointRevolLimDis,
  _rkJointRevolSetDis,
  _rkJointRevolSetVel,
  _rkJointRevolSetAcc,
  _rkJointRevolSetTrq,
  _rkJointRevolGetDis,
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

  _rkJointRevolRegZTK,
  _rkJointRevolDisFromZTK,
  _rkJointRevolFromZTK,
  _rkJointRevolDisFPrintZTK,
  _rkJointRevolFPrintZTK,
};

#undef _rkc
