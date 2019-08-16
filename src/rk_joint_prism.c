/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_prism - joint structure: prismatic joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointPrpPrism *)p)

static void _rkJointInitPrism(void *prp){
  _rkc(prp)->max = HUGE_VAL;
  _rkc(prp)->min =-HUGE_VAL;
  rkMotorAssign( &_rkc(prp)->m, &rk_motor_none );
}

static void *_rkJointAllocPrism(void){ return zAlloc( rkJointPrpPrism, 1 ); }

/* limit joint displacement */
static void _rkJointLimDisPrism(void *prp, double *testval, double *limval){
  *limval = zLimit( *testval, _rkc(prp)->min, _rkc(prp)->max );
}

/* set joint displacement */
static void _rkJointSetDisPrism(void *prp, double *val){
  _rkJointLimDisPrism( prp, val, &_rkc(prp)->dis );
}

static void _rkJointSetVelPrism(void *prp, double *val){
  _rkc(prp)->vel = *val;
}

static void _rkJointSetAccPrism(void *prp, double *val){
  _rkc(prp)->acc = *val;
}

static void _rkJointSetTrqPrism(void *prp, double *val){
  _rkc(prp)->trq = *val;
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointGetDisPrism(void *prp, double *val){
  *val = _rkc(prp)->dis;
}

static void _rkJointGetVelPrism(void *prp, double *val){
  *val = _rkc(prp)->vel;
}

static void _rkJointGetAccPrism(void *prp, double *val){
  *val = _rkc(prp)->acc;
}

static void _rkJointGetTrqPrism(void *prp, double *val){
  *val = _rkc(prp)->trq;
}

static void _rkJointCatDisPrism(void *prp, double *dis, double k, double *val){
  *dis += k * *val;
}

static void _rkJointSubDisPrism(void *prp, double *dis, double *sdis){
  *dis -= *sdis;
}

/* continuously update joint displacement */
static void _rkJointSetDisCNTPrism(void *prp, double *val, double dt){
  double olddis, oldvel;

  olddis = _rkc(prp)->dis;
  oldvel = _rkc(prp)->vel;
  _rkJointSetDisPrism( prp, val );
  _rkc(prp)->vel = ( *val - olddis ) / dt;
  _rkc(prp)->acc = ( _rkc(prp)->vel - oldvel ) / dt;
}

/* joint frame transformation */
static zFrame3D *_rkJointXformPrism(void *prp, zFrame3D *fo, zFrame3D *f);

static zFrame3D *_rkJointXformPrism(void *prp, zFrame3D *fo, zFrame3D *f){
  zVec3DCat( zFrame3DPos(fo),
    _rkc(prp)->dis, &zFrame3DAtt(fo)->v[2], zFrame3DPos(f) );
  zMat3DCopy( zFrame3DAtt(fo), zFrame3DAtt(f) );
  return f;
}

/* joint velocity transformation */
static void _rkJointIncVelPrism(void *prp, zVec6D *vel){
  vel->e[zZ] += _rkc(prp)->vel;
}

static void _rkJointIncAccOnVelPrism(void *prp, zVec3D *w, zVec6D *acc){
  acc->e[zX] += 2 * _rkc(prp)->vel * w->e[zY];
  acc->e[zY] -= 2 * _rkc(prp)->vel * w->e[zX];
}

/* joint acceleration transformation */
static void _rkJointIncAccPrism(void *prp, zVec6D *acc){
  acc->e[zZ] += _rkc(prp)->acc;
}

/* joint torque transformation */
static void _rkJointCalcTrqPrism(void *prp, zVec6D *f){
  _rkc(prp)->trq = f->e[zZ];
}

/* inverse computation of joint torsion and displacement */
static void _rkJointTorsionPrism(zFrame3D *dev, zVec6D *t, double dis[]){
  zVec3D aa;
  zMat3DToAA( zFrame3DAtt(dev), &aa );
  zMulMat3DTVec3D( zFrame3DAtt(dev), &aa, zVec6DAng(t) );
  dis[0] = rkJointTorsionDisPrism( dev, t );
}

static zVec3D* (*_rk_joint_axis_prism_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};
static zVec3D* (*_rk_joint_axis_prism_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisZ,
};

static void _rkJointSetFrictionPivotPrism(void *prp, rkJointFrictionPivot *fp){
  *fp = _rkc(prp)->_fp;
}

static void _rkJointGetFrictionPivotPrism(void *prp, rkJointFrictionPivot *fp){
  _rkc(prp)->_fp = *fp;
}

static void _rkJointSetFricPrism(void *prp, double *val){
  _rkc(prp)->tf = *val;
}

static void _rkJointGetFricPrism(void *prp, double *val){
  *val = _rkc(prp)->tf;
}

static void _rkJointGetSFricPrism(void *prp, double *val){
  *val = _rkc(prp)->sf;
}

static void _rkJointGetKFricPrism(void *prp, double *val){
  *val = _rkJointRestTrq( _rkc(prp)->stiffness, _rkc(prp)->viscosity, _rkc(prp)->coulomb, _rkc(prp)->dis, _rkc(prp)->vel );
}

/* motor */
static rkMotor *_rkJointGetMotorPrism(void *prp){ return &_rkc(prp)->m; }

static void _rkJointMotorSetInputPrism(void *prp, double *val){
  rkMotorSetInput( &_rkc(prp)->m, val );
}

static void _rkJointMotorInertiaPrism(void *prp, double *val){
  *val = 0.0;
  rkMotorInertia( &_rkc(prp)->m, val );
}

static void _rkJointMotorInputTrqPrism(void *prp, double *val){
  *val = 0.0;
  rkMotorInputTrq( &_rkc(prp)->m, val );
}

static void _rkJointMotorResistancePrism(void *prp, double *val){
  *val = 0.0;
  rkMotorRegistance( &_rkc(prp)->m, &_rkc(prp)->dis, &_rkc(prp)->vel, val );
}

static void _rkJointMotorDrivingTrqPrism(void *prp, double *val){
  *val = 0.0;
  rkMotorDrivingTrq( &_rkc(prp)->m, &_rkc(prp)->dis, &_rkc(prp)->vel, &_rkc(prp)->acc, val );
}

/* ABI */
static void _rkJointABIAxisInertiaPrism(void *prp, zMat6D *m, zMat h, zMat ih)
{
  _rkJointMotorInertiaPrism( prp, zMatBuf(h) );
  zMatElemNC(h,0,0) += m->e[0][0].e[2][2];
  if( !zIsTiny( zMatElemNC(h,0,0) ) )
    zMatElemNC(ih,0,0) = 1.0 / zMatElemNC(h,0,0);
  else
    zMatElemNC(ih,0,0) = 0.0;
}

static void _rkJointABIAddAbiPrism(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm)
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

static void _rkJointABIAddBiasPrism(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb)
{
  zVec6D tmpv, tmpv2;

  zMat6DCol( m, zZ, &tmpv );
  zVec6DMulDRC( &tmpv, _rkc(prp)->_u - b->e[zZ] );
  zVec6DSub( b, &tmpv, &tmpv2 );

  zMulMat3DVec6D( zFrame3DAtt(f), &tmpv2, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}

static void _rkJointABIDrivingTorquePrism(void *prp)
{
  double val;

  _rkJointMotorInputTrqPrism( prp, &_rkc(prp)->_u );
  _rkJointMotorResistancePrism( prp, &val );
  _rkc(prp)->_u -= val;
  _rkc(prp)->_u += _rkc(prp)->tf;
}

static void _rkJointABIQAccPrism(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
{
  zVec6D tmpv;

  zMat6DRow( m, zZ, &tmpv );
  /* q */
  _rkc(prp)->acc = zMatElemNC(h,0,0)*( _rkc(prp)->_u - zVec6DInnerProd( &tmpv, jac ) - b->e[zZ] );
  /* acc */
  zVec6DCopy( jac, acc );
  acc->e[zZ] += _rkc(prp)->acc;
}

/* query joint properties */
static bool _rkJointQueryFScanPrism(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm)
{
  double val;
  rkMotor *mp;

  if( strcmp( buf, "dis" ) == 0 ){
    val = zFDouble(fp);
    _rkJointSetDisPrism( prp, &val );
  } else
  if( strcmp( buf, "min" ) == 0 )
    _rkc(prp)->min = zFDouble(fp);
  else
  if( strcmp( buf, "max" ) == 0 )
    _rkc(prp)->max = zFDouble(fp);
  else
  if( strcmp( buf, "stiffness" ) == 0 )
    _rkc(prp)->stiffness = zFDouble(fp);
  else
  if( strcmp( buf, "viscosity" ) == 0 )
    _rkc(prp)->viscosity = zFDouble(fp);
  else
  if( strcmp( buf, "coulomb" ) == 0 )
    _rkc(prp)->coulomb = zFDouble(fp);
  else
  if( strcmp( buf, "staticfriction" ) == 0 )
    _rkc(prp)->sf = zFDouble(fp);
  else
  if( strcmp( buf, "motor" ) == 0 ){
    zFToken( fp, buf, BUFSIZ );
    zNameFind( marray, nm, buf, mp );
    if( !mp ){
      ZRUNERROR( RK_ERR_MOTOR_UNKNOWN, buf );
      return true;
    }
    if( rkMotorSize(mp) != 1 ){
      ZRUNERROR( RK_ERR_JOINT_SIZMISMATCH );
      return true;
    }
    rkMotorClone( mp, &_rkc(prp)->m );
  } else
  if( !rkMotorQueryFScan( fp, buf, &_rkc(prp)->m ) )
    return false;
  return true;
}

static void *_rkJointDisFromZTKPrism(void *prp, int i, void *arg, ZTK *ztk){
  double val;
  val = zDeg2Rad( ZTKDouble(ztk) );
  _rkJointSetDisPrism( _rkc(prp), &val );
  return prp;
}
static void *_rkJointMinFromZTKPrism(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->min = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointMaxFromZTKPrism(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->max = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointStiffnessFromZTKPrism(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->stiffness = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointViscosityFromZTKPrism(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->viscosity = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointCoulombFromZTKPrism(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->coulomb = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointStaticFricFromZTKPrism(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->sf = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointMotorFromZTKPrism(void *prp, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk) ) ) ) return NULL;
  return rkMotorClone( mp, &_rkc(prp)->m ) ? prp : NULL;
}

static void _rkJointDisFPrintPrism(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->dis );
}
static void _rkJointMinFPrintPrism(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->min );
}
static void _rkJointMaxFPrintPrism(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->max );
}
static void _rkJointStiffnessFPrintPrism(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->stiffness );
}
static void _rkJointViscosityFPrintPrism(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->viscosity );
}
static void _rkJointCoulombFPrintPrism(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->coulomb );
}
static void _rkJointStaticFricFPrintPrism(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->sf );
}
static void _rkJointMotorFPrintPrism(FILE *fp, int i, void *prp){
  fprintf( fp, "%s\n", zName(&_rkc(prp)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_prism[] = {
  { "dis", 1, _rkJointDisFromZTKPrism, _rkJointDisFPrintPrism },
  { "min", 1, _rkJointMinFromZTKPrism, _rkJointMinFPrintPrism },
  { "max", 1, _rkJointMaxFromZTKPrism, _rkJointMaxFPrintPrism },
  { "stiffness", 1, _rkJointStiffnessFromZTKPrism, _rkJointStiffnessFPrintPrism },
  { "viscosity", 1, _rkJointViscosityFromZTKPrism, _rkJointViscosityFPrintPrism },
  { "coulomb", 1, _rkJointCoulombFromZTKPrism, _rkJointCoulombFPrintPrism },
  { "staticfriction", 1, _rkJointStaticFricFromZTKPrism, _rkJointStaticFricFPrintPrism },
  { "motor", 1, _rkJointMotorFromZTKPrism, _rkJointMotorFPrintPrism },
};

static void *_rkJointFromZTKPrism(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_prism );
}

static void _rkJointFPrintPrism(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_prism );
}

rkJointCom rk_joint_prism = {
  "prismatic",
  1,
  _rkJointInitPrism,
  _rkJointAllocPrism,
  _rkJointLimDisPrism,
  _rkJointSetDisPrism,
  _rkJointSetVelPrism,
  _rkJointSetAccPrism,
  _rkJointSetTrqPrism,
  _rkJointGetDisPrism,
  _rkJointGetVelPrism,
  _rkJointGetAccPrism,
  _rkJointGetTrqPrism,
  _rkJointCatDisPrism,
  _rkJointSubDisPrism,
  _rkJointSetDisCNTPrism,
  _rkJointXformPrism,
  _rkJointIncVelPrism,
  _rkJointIncAccOnVelPrism,
  _rkJointIncAccPrism,
  _rkJointCalcTrqPrism,
  _rkJointTorsionPrism,
  _rk_joint_axis_prism_ang,
  _rk_joint_axis_prism_lin,

  _rkJointSetFrictionPivotPrism,
  _rkJointGetFrictionPivotPrism,
  _rkJointSetFricPrism,
  _rkJointGetFricPrism,
  _rkJointGetSFricPrism,
  _rkJointGetKFricPrism,

  _rkJointGetMotorPrism,
  _rkJointMotorSetInputPrism,
  _rkJointMotorInertiaPrism,
  _rkJointMotorInputTrqPrism,
  _rkJointMotorResistancePrism,
  _rkJointMotorDrivingTrqPrism,

  _rkJointABIAxisInertiaPrism,
  _rkJointABIAddAbiPrism,
  _rkJointABIAddBiasPrism,
  _rkJointABIDrivingTorquePrism,
  _rkJointABIQAccPrism,
  _rkJointUpdateWrench,

  _rkJointQueryFScanPrism,
  _rkJointDisFromZTKPrism,
  _rkJointFromZTKPrism,
  _rkJointDisFPrintPrism,
  _rkJointFPrintPrism,
};

bool rkJointRegZTKPrism(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_prism ) ? true : false;
}

#undef _rkc
