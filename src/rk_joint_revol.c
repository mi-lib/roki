/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_revol - joint structure: revolutional joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointPrpRevol *)p)

static void _rkJointInitRevol(void *prp){
  _rkc(prp)->max = HUGE_VAL;
  _rkc(prp)->min =-HUGE_VAL;
  rkMotorAssign( &_rkc(prp)->m, &rk_motor_none );
}

static void *_rkJointAllocRevol(void){ return zAlloc( rkJointPrpRevol, 1 ); }

/* limit joint displacement */
static void _rkJointLimDisRevol(void *prp, double *testval, double *limval){
  double angle;
  angle = zPhaseNormalize( *testval );
  *limval = zLimit( angle, _rkc(prp)->min, _rkc(prp)->max );
}

/* set joint displacement, velocity, acceleration and torque */
static void _rkJointSetDisRevol(void *prp, double *val){
  _rkJointLimDisRevol( prp, val, &_rkc(prp)->dis );
  zSinCos( _rkc(prp)->dis, &_rkc(prp)->_s, &_rkc(prp)->_c );
}

static void _rkJointSetVelRevol(void *prp, double *val){
  _rkc(prp)->vel = *val;
}

static void _rkJointSetAccRevol(void *prp, double *val){
  _rkc(prp)->acc = *val;
}

static void _rkJointSetTrqRevol(void *prp, double *val){
  _rkc(prp)->trq = *val;
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointGetDisRevol(void *prp, double *val){
  *val = _rkc(prp)->dis;
}

static void _rkJointGetVelRevol(void *prp, double *val){
  *val = _rkc(prp)->vel;
}

static void _rkJointGetAccRevol(void *prp, double *val){
  *val = _rkc(prp)->acc;
}

static void _rkJointGetTrqRevol(void *prp, double *val){
  *val = _rkc(prp)->trq;
}

static void _rkJointCatDisRevol(void *prp, double *dis, double k, double *val){
  *dis += k * *val;
}

static void _rkJointSubDisRevol(void *prp, double *dis, double *sdis){
  *dis -= *sdis;
}

/* continuously update joint displacement */
static void _rkJointSetDisCNTRevol(void *prp, double *val, double dt)
{
  double olddis, oldvel;

  _rkJointGetDisRevol( prp, &olddis );
  _rkJointGetVelRevol( prp, &oldvel );
  _rkJointSetDisRevol( prp, val );
  _rkc(prp)->vel = ( *val - olddis ) / dt;
  _rkc(prp)->acc = ( _rkc(prp)->vel - oldvel ) / dt;
}

/* joint frame transformation */
static zFrame3D *_rkJointXformRevol(void *prp, zFrame3D *fo, zFrame3D *f)
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
static void _rkJointIncVelRevol(void *prp, zVec6D *vel){
  vel->e[zZA] += _rkc(prp)->vel;
}

static void _rkJointIncAccOnVelRevol(void *prp, zVec3D *w, zVec6D *acc){
  acc->e[zXA] += _rkc(prp)->vel * w->e[zY];
  acc->e[zYA] -= _rkc(prp)->vel * w->e[zX];
}

/* joint acceleration transformation */
static void _rkJointIncAccRevol(void *prp, zVec6D *acc){
  acc->e[zZA] += _rkc(prp)->acc;
}

/* joint torque transformation */
static void _rkJointCalcTrqRevol(void *prp, zVec6D *f){
  _rkc(prp)->trq = f->e[zZA];
}

/* inverse computation of joint torsion and displacement */
static void _rkJointTorsionRevol(zFrame3D *dev, zVec6D *t, double dis[]){
  zMulMat3DTVec3D( zFrame3DAtt(dev), zFrame3DPos(dev), zVec6DLin(t) );
  dis[0] = rkJointTorsionDisRevol( dev, t );
}

static zVec3D* (*_rk_joint_axis_revol_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisZ,
};

static zVec3D* (*_rk_joint_axis_revol_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};

static void _rkJointSetFrictionPivotRevol(void *prp, rkJointFrictionPivot *fp){
  _rkc(prp)->_fp = *fp;
}

static void _rkJointGetFrictionPivotRevol(void *prp, rkJointFrictionPivot *fp){
  *fp = _rkc(prp)->_fp;
}

static void _rkJointSetFricRevol(void *prp, double *val){
  _rkc(prp)->tf = *val;
}

static void _rkJointGetFricRevol(void *prp, double *val){
  *val = _rkc(prp)->tf;
}

static void _rkJointGetSFricRevol(void *prp, double *val){
  *val = _rkc(prp)->sf;
}

static void _rkJointGetKFricRevol(void *prp, double *val){
  *val = _rkJointRestTrq( _rkc(prp)->stiffness, _rkc(prp)->viscosity, _rkc(prp)->coulomb, _rkc(prp)->dis, _rkc(prp)->vel );
}

/* motor */
static rkMotor *_rkJointGetMotorRevol(void *prp){ return &_rkc(prp)->m; }

static void _rkJointMotorSetInputRevol(void *prp, double *val){
  rkMotorSetInput( &_rkc(prp)->m, val );
}

static void _rkJointMotorInertiaRevol(void *prp, double *val){
  *val = 0.0;
  rkMotorInertia( &_rkc(prp)->m, val );
}

static void _rkJointMotorInputTrqRevol(void *prp, double *val){
  *val = 0.0;
  rkMotorInputTrq( &_rkc(prp)->m, val );
}

static void _rkJointMotorResistanceRevol(void *prp, double *val){
  *val = 0.0;
  rkMotorRegistance( &_rkc(prp)->m, &_rkc(prp)->dis, &_rkc(prp)->vel, val );
}

static void _rkJointMotorDrivingTrqRevol(void *prp, double *val){
  *val = 0.0;
  rkMotorDrivingTrq( &_rkc(prp)->m, &_rkc(prp)->dis, &_rkc(prp)->vel, &_rkc(prp)->acc, val );
}

/* ABI */
static void _rkJointABIAxisInertiaRevol(void *prp, zMat6D *m, zMat h, zMat ih)
{
  _rkJointMotorInertiaRevol( prp, zMatBuf(h) );
  zMatElemNC(h,0,0) += m->e[1][1].e[2][2];
  if( !zIsTiny( zMatElemNC(h,0,0) ) )
    zMatElemNC(ih,0,0) = 1.0 / zMatElemNC(h,0,0);
  else
    zMatElemNC(ih,0,0) = 0.0;
}

static void _rkJointABIAddAbiRevol(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm)
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

static void _rkJointABIAddBiasRevol(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb)
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

static void _rkJointABIDrivingTorqueRevol(void *prp)
{
  double val;

  _rkJointMotorInputTrqRevol( prp, &_rkc(prp)->_u );
  _rkJointMotorResistanceRevol( prp, &val );
  _rkc(prp)->_u -= val;
  _rkc(prp)->_u += _rkc(prp)->tf;
}

static void _rkJointABIQAccRevol(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
{
  zVec6D tmpv;

  zMat6DRow( m, zZA, &tmpv );
  /* q */
  _rkc(prp)->acc = zMatElemNC(h,0,0)*( _rkc(prp)->_u - zVec6DInnerProd( &tmpv, jac ) - b->e[zZA] );
  /* acc */
  zVec6DCopy( jac, acc );
  acc->e[zZA] += _rkc(prp)->acc;
}

/* query joint properties */
static bool _rkJointQueryFScanRevol(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm)
{
  double val;
  rkMotor *mp;

  if( strcmp( buf, "dis" ) == 0 ){
    val = zDeg2Rad( zFDouble(fp) );
    _rkJointSetDisRevol( prp, &val );
  } else
  if( strcmp( buf, "min" ) == 0 )
    _rkc(prp)->min = zDeg2Rad(zFDouble(fp));
  else
  if( strcmp( buf, "max" ) == 0 )
    _rkc(prp)->max = zDeg2Rad(zFDouble(fp));
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

static void *_rkJointDisFromZTKRevol(void *prp, int i, void *arg, ZTK *ztk){
  double val;
  val = zDeg2Rad( ZTKDouble(ztk) );
  _rkJointSetDisRevol( prp, &val );
  return prp;
}
static void *_rkJointMinFromZTKRevol(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->min = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointMaxFromZTKRevol(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->max = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointStiffnessFromZTKRevol(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->stiffness = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointViscosityFromZTKRevol(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->viscosity = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointCoulombFromZTKRevol(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->coulomb = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointStaticFricFromZTKRevol(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->sf = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointMotorFromZTKRevol(void *prp, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk) ) ) ) return NULL;
  return rkMotorClone( mp, &_rkc(prp)->m ) ? prp : NULL;
}

static void _rkJointDisFPrintRevol(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", zRad2Deg(_rkc(prp)->dis) );
}
static void _rkJointMinFPrintRevol(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", zRad2Deg(_rkc(prp)->min) );
}
static void _rkJointMaxFPrintRevol(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", zRad2Deg(_rkc(prp)->max) );
}
static void _rkJointStiffnessFPrintRevol(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->stiffness );
}
static void _rkJointViscosityFPrintRevol(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->viscosity );
}
static void _rkJointCoulombFPrintRevol(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->coulomb );
}
static void _rkJointStaticFricFPrintRevol(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->sf );
}
static void _rkJointMotorFPrintRevol(FILE *fp, int i, void *prp){
  fprintf( fp, "%s\n", zName(&_rkc(prp)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_revol[] = {
  { "dis", 1, _rkJointDisFromZTKRevol, _rkJointDisFPrintRevol },
  { "min", 1, _rkJointMinFromZTKRevol, _rkJointMinFPrintRevol },
  { "max", 1, _rkJointMaxFromZTKRevol, _rkJointMaxFPrintRevol },
  { "stiffness", 1, _rkJointStiffnessFromZTKRevol, _rkJointStiffnessFPrintRevol },
  { "viscosity", 1, _rkJointViscosityFromZTKRevol, _rkJointViscosityFPrintRevol },
  { "coulomb", 1, _rkJointCoulombFromZTKRevol, _rkJointCoulombFPrintRevol },
  { "staticfriction", 1, _rkJointStaticFricFromZTKRevol, _rkJointStaticFricFPrintRevol },
  { "motor", 1, _rkJointMotorFromZTKRevol, _rkJointMotorFPrintRevol },
};

static void *_rkJointFromZTKRevol(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_revol );
}

static void _rkJointFPrintRevol(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_revol );
}

rkJointCom rk_joint_revol = {
  "revolute",
  1,
  _rkJointInitRevol,
  _rkJointAllocRevol,
  _rkJointLimDisRevol,
  _rkJointSetDisRevol,
  _rkJointSetVelRevol,
  _rkJointSetAccRevol,
  _rkJointSetTrqRevol,
  _rkJointGetDisRevol,
  _rkJointGetVelRevol,
  _rkJointGetAccRevol,
  _rkJointGetTrqRevol,
  _rkJointCatDisRevol,
  _rkJointSubDisRevol,
  _rkJointSetDisCNTRevol,
  _rkJointXformRevol,
  _rkJointIncVelRevol,
  _rkJointIncAccOnVelRevol,
  _rkJointIncAccRevol,
  _rkJointCalcTrqRevol,
  _rkJointTorsionRevol,
  _rk_joint_axis_revol_ang,
  _rk_joint_axis_revol_lin,

  _rkJointSetFrictionPivotRevol,
  _rkJointGetFrictionPivotRevol,
  _rkJointSetFricRevol,
  _rkJointGetFricRevol,
  _rkJointGetSFricRevol,
  _rkJointGetKFricRevol,

  _rkJointGetMotorRevol,
  _rkJointMotorSetInputRevol,
  _rkJointMotorInertiaRevol,
  _rkJointMotorInputTrqRevol,
  _rkJointMotorResistanceRevol,
  _rkJointMotorDrivingTrqRevol,

  _rkJointABIAxisInertiaRevol,
  _rkJointABIAddAbiRevol,
  _rkJointABIAddBiasRevol,
  _rkJointABIDrivingTorqueRevol,
  _rkJointABIQAccRevol,
  _rkJointUpdateWrench,

  _rkJointQueryFScanRevol,
  _rkJointDisFromZTKRevol,
  _rkJointFromZTKRevol,
  _rkJointDisFPrintRevol,
  _rkJointFPrintRevol,
};

bool rkJointRegZTKRevol(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_revol ) ? true : false;
}

#undef _rkc
