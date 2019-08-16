/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_hooke - joint structure: universal joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointPrpHooke *)p)

static void _rkJointInitHooke(void *prp){
  _rkc(prp)->max[0] = HUGE_VAL;
  _rkc(prp)->min[0] =-HUGE_VAL;
  _rkc(prp)->max[1] = HUGE_VAL;
  _rkc(prp)->min[1] =-HUGE_VAL;
  rkMotorAssign( &_rkc(prp)->m, &rk_motor_none );
}

static void *_rkJointAllocHooke(void){ return zAlloc( rkJointPrpHooke, 1 ); }

/* limit joint displacement */
static double _rkJointLimDis1Hooke(void *prp, int i, double testval){
  testval = zPhaseNormalize( testval );
  return zLimit( testval, _rkc(prp)->min[i], _rkc(prp)->max[i] );
}

static void _rkJointLimDisHooke(void *prp, double *testval, double *limval){
  limval[0] = _rkJointLimDis1Hooke( prp, 0, testval[0] );
  limval[1] = _rkJointLimDis1Hooke( prp, 1, testval[1] );
}

/* set joint displacement */
static void _rkJointSetDis1Hooke(void *prp, int i, double val){
  _rkc(prp)->dis[i] = _rkJointLimDis1Hooke( prp, i, val );
  zSinCos( _rkc(prp)->dis[i], &_rkc(prp)->_s[i], &_rkc(prp)->_c[i] );
}

static void _rkJointSetDisHooke(void *prp, double *val){
  _rkJointSetDis1Hooke( prp, 0, val[0] );
  _rkJointSetDis1Hooke( prp, 1, val[1] );
}

static void _rkJointSetVelHooke(void *prp, double *val){
  memcpy( _rkc(prp)->vel, val, sizeof(double)*2 );
}

static void _rkJointSetAccHooke(void *prp, double *val){
  memcpy( _rkc(prp)->acc, val, sizeof(double)*2 );
}

static void _rkJointSetTrqHooke(void *prp, double *val){
  memcpy( _rkc(prp)->trq, val, sizeof(double)*2 );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointGetDisHooke(void *prp, double *val){
  memcpy( val, _rkc(prp)->dis, sizeof(double)*2 );
}

static void _rkJointGetVelHooke(void *prp, double *val){
  memcpy( val, _rkc(prp)->vel, sizeof(double)*2 );
}

static void _rkJointGetAccHooke(void *prp, double *val){
  memcpy( val, _rkc(prp)->acc, sizeof(double)*2 );
}

static void _rkJointGetTrqHooke(void *prp, double *val){
  memcpy( val, _rkc(prp)->trq, sizeof(double)*2 );
}

static void _rkJointCatDisHooke(void *prp, double *dis, double k, double *val){
  dis[0] += val[0] * k;
  dis[1] += val[1] * k;
}

static void _rkJointSubDisHooke(void *prp, double *dis, double *sdis){
  dis[0] -= sdis[0];
  dis[1] -= sdis[1];
}

/* continuously update joint displacement */
static void _rkJointSetDisCNTHooke(void *prp, double *val, double dt){
  double olddis[2], oldvel[2];
  _rkJointGetDisHooke( prp, olddis );
  _rkJointGetVelHooke( prp, oldvel );
  _rkJointSetDisHooke( prp, val );
  _rkc(prp)->vel[0] = ( val[0] - olddis[0] ) / dt;
  _rkc(prp)->vel[1] = ( val[1] - olddis[1] ) / dt;
  _rkc(prp)->acc[0] = ( _rkc(prp)->vel[0] - oldvel[0] ) / dt;
  _rkc(prp)->acc[1] = ( _rkc(prp)->vel[1] - oldvel[1] ) / dt;
}

/* joint frame transformation */
static zFrame3D *_rkJointXformHooke(void *prp, zFrame3D *fo, zFrame3D *f){
  zMat3D m;
  /* position */
  zVec3DCopy( zFrame3DPos(fo), zFrame3DPos(f) );
  /* joint displacements correspond to the rotation angle about
   * z-axis and y-axis, respectively */
  zMat3DFromZYXSC( &m, _rkc(prp)->_s[0], _rkc(prp)->_c[0], _rkc(prp)->_s[1], _rkc(prp)->_c[1], 0, 1 );
  zMulMat3DMat3D( zFrame3DAtt(fo), &m, zFrame3DAtt(f) );
  return f;
}

/* joint motion rate transformation */
static void _rkJointIncVelHooke(void *prp, zVec6D *vel){
  zVec3D v;
  zVec3DCreate( &v, -_rkc(prp)->_s[1]*_rkc(prp)->vel[0], _rkc(prp)->vel[1], _rkc(prp)->_c[1]*_rkc(prp)->vel[0] );
  zVec3DAddDRC( zVec6DAng(vel), &v );
}

static void _rkJointIncAccOnVelHooke(void *prp, zVec3D *w, zVec6D *acc){
  zVec3D v;
  double dq2;
  zVec3DCreate( &v, -_rkc(prp)->_s[1]*_rkc(prp)->vel[0], _rkc(prp)->vel[1], _rkc(prp)->_c[1]*_rkc(prp)->vel[0] );
  zVec3DOuterProd( w, &v, &v );
  zVec3DAddDRC( zVec6DAng(acc), &v );
  dq2 = _rkc(prp)->vel[0] * _rkc(prp)->vel[1];
  zVec3DCreate( &v, -_rkc(prp)->_c[1]*dq2, 0, -_rkc(prp)->_s[1]*dq2 );
  zVec3DAddDRC( zVec6DAng(acc), &v );
}

static void _rkJointIncAccHooke(void *prp, zVec6D *acc){
  zVec3D v1;
  rkJointPrpHooke *p;
  p = prp;
  zVec3DCreate( &v1, -p->_s[1]*p->acc[0], p->acc[1], p->_c[1]*p->acc[0] );
  zVec3DAddDRC( zVec6DAng(acc), &v1 );
}

/* joint torque transformation */
static void _rkJointCalcTrqHooke(void *prp, zVec6D *f){
  rkJointPrpHooke *p;
  p = prp;
  p->trq[0] =-p->_s[1]*f->e[zXA]+p->_c[1]*f->e[zZA];
  p->trq[1] = f->e[zYA];
}

/* inverse computation of joint torsion and displacement */
static void _rkJointTorsionHooke(zFrame3D *dev, zVec6D *t, double dis[]){
  zMat3D *r;
  double qx, s, c;
  r = zFrame3DAtt(dev);
  zMulMat3DTVec3D( r, zFrame3DPos(dev), zVec6DLin(t) );
  qx = atan2( r->e[1][2], r->e[1][1] );
  zMat3DRow( r, 0, zVec6DAng(t) );
  zVec3DMulDRC( zVec6DAng(t), qx );
  zSinCos( qx, &s, &c );
  dis[0] = atan2( -r->e[1][0], c*r->e[1][1]+s*r->e[1][2] );
  c = cos(dis[0]);
  dis[1] = atan2( c*r->e[2][0], c*r->e[0][0] );
}

/* joint axes */
static zVec3D *_rkJointAngAxisHooke1(void *prp, zFrame3D *f, zVec3D *a){
  zVec3DMul( &zFrame3DAtt(f)->v[zX],-_rkc(prp)->_s[1], a );
  return zVec3DCatDRC( a, _rkc(prp)->_c[1], &zFrame3DAtt(f)->v[2] );
}

static zVec3D *_rkJointAngAxisHooke2(void *prp, zFrame3D *f, zVec3D *a){
  zVec3DCopy( &zFrame3DAtt(f)->v[1], a );
  return a;
}

static zVec3D* (*_rk_joint_axis_hooke_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAngAxisHooke1,
  _rkJointAngAxisHooke2,
};
static zVec3D* (*_rk_joint_axis_hooke_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisNull,
};

static void _rkJointSetFrictionPivotHooke(void *prp, rkJointFrictionPivot *fp){
  fp[0] = _rkc(prp)->_fp[0];
  fp[1] = _rkc(prp)->_fp[1];
}

static void _rkJointGetFrictionPivotHooke(void *prp, rkJointFrictionPivot *fp){
  _rkc(prp)->_fp[0] = fp[0];
  _rkc(prp)->_fp[1] = fp[1];
}

static void _rkJointSetFricHooke(void *prp, double *val){
  _rkc(prp)->tf[0] = val[0];
  _rkc(prp)->tf[1] = val[1];
}

static void _rkJointGetFricHooke(void *prp, double *val){
  val[0] = _rkc(prp)->tf[0];
  val[1] = _rkc(prp)->tf[1];
}

static void _rkJointGetSFricHooke(void *prp, double *val){
  val[0] = _rkc(prp)->sf[0];
  val[1] = _rkc(prp)->sf[1];
}

static void _rkJointGetKFricHooke(void *prp, double *val){
  val[0] = _rkJointRestTrq( _rkc(prp)->stiffness[0], _rkc(prp)->viscosity[0], _rkc(prp)->coulomb[0], _rkc(prp)->dis[0], _rkc(prp)->vel[0] );
  val[1] = _rkJointRestTrq( _rkc(prp)->stiffness[1], _rkc(prp)->viscosity[1], _rkc(prp)->coulomb[1], _rkc(prp)->dis[1], _rkc(prp)->vel[1] );
}

/* motor */
static rkMotor *_rkJointGetMotorHooke(void *prp){ return &_rkc(prp)->m; }

static void _rkJointMotorSetInputHooke(void *prp, double *val){
  rkMotorSetInput( &_rkc(prp)->m, val );
}

static void _rkJointMotorInertiaHooke(void *prp, double *val){
  zRawVecZero( val, 4 );
  rkMotorInertia( &_rkc(prp)->m, val );
}

static void _rkJointMotorInputTrqHooke(void *prp, double *val){
  zRawVecZero( val, 2 );
  rkMotorInputTrq( &_rkc(prp)->m, val );
}

static void _rkJointMotorResistanceHooke(void *prp, double *val){
  zRawVecZero( val, 2 );
  rkMotorRegistance( &_rkc(prp)->m, _rkc(prp)->dis, _rkc(prp)->vel, val );
}

static void _rkJointMotorDrivingTrqHooke(void *prp, double *val){
  zRawVecZero( val, 2 );
  rkMotorDrivingTrq( &_rkc(prp)->m, _rkc(prp)->dis, _rkc(prp)->vel, _rkc(prp)->acc, val );
}

/* ABI */
static void _rkJointABIAxisInertiaHooke(void *prp, zMat6D *m, zMat h, zMat ih){
  rkJointPrpHooke *p;
  zMat3D *m22;

  _rkJointMotorInertiaHooke( prp, zMatBuf(h) );
  p = prp;
  m22 = &m->e[1][1];
  zMatElemNC(h,0,0) += m22->e[0][0]*p->_s[1]*p->_s[1] - (m22->e[2][0] + m22->e[0][2])*p->_s[1]*p->_c[1] + m22->e[2][2]*p->_c[1]*p->_c[1];
  zMatElemNC(h,1,0) += m22->e[2][1]*p->_c[1] - m22->e[0][1]*p->_s[1];
  zMatElemNC(h,0,1) += m22->e[1][2]*p->_c[1] - m22->e[1][0]*p->_s[1];
  zMatElemNC(h,1,1) += m22->e[1][1];
  zMatInv( h, ih );
}

static void _rkJointABIAddAbiHooke(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  eprintf("under construction error: abi update for hooke joint\n");
}

static void _rkJointABIAddBiasHooke(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  eprintf("under construction error: abi update for hooke joint\n");
}

static void _rkJointABIDrivingTorqueHooke(void *prp){
  eprintf("under construction error: abi update for hooke joint\n");
}

static void _rkJointABIQAccHooke(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){}

/* query joint properties */
static bool _rkJointQueryFScanHooke(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm)
{
  rkMotor *mp;

  if( strcmp( buf, "dis" ) == 0 ){
    _rkJointSetDis1Hooke( prp, 0, zDeg2Rad(zFDouble(fp)) );
    _rkJointSetDis1Hooke( prp, 1, zDeg2Rad(zFDouble(fp)) );
  } else
  if( strcmp( buf, "min" ) == 0 ){
    _rkc(prp)->min[0] = zDeg2Rad(zFDouble(fp));
    _rkc(prp)->min[1] = zDeg2Rad(zFDouble(fp));
  } else
  if( strcmp( buf, "max" ) == 0 ){
    _rkc(prp)->max[0] = zDeg2Rad(zFDouble(fp));
    _rkc(prp)->max[1] = zDeg2Rad(zFDouble(fp));
  } else
  if( strcmp( buf, "stiffness" ) == 0 ){
    _rkc(prp)->stiffness[0] = zFDouble(fp);
    _rkc(prp)->stiffness[1] = zFDouble(fp);
  } else
  if( strcmp( buf, "viscosity" ) == 0 ){
    _rkc(prp)->viscosity[0] = zFDouble(fp);
    _rkc(prp)->viscosity[1] = zFDouble(fp);
  } else
  if( strcmp( buf, "coulomb" ) == 0 ){
    _rkc(prp)->coulomb[0] = zFDouble(fp);
    _rkc(prp)->coulomb[1] = zFDouble(fp);
  } else
  if( strcmp( buf, "staticfriction" ) == 0 ){
    _rkc(prp)->sf[0] = zFDouble(fp);
    _rkc(prp)->sf[1] = zFDouble(fp);
  }
  if( strcmp( buf, "motor" ) == 0 ){
    zFToken( fp, buf, BUFSIZ );
    zNameFind( marray, nm, buf, mp );
    if( !mp ){
      ZRUNERROR( RK_ERR_MOTOR_UNKNOWN, buf );
      return true;
    }
    if( rkMotorSize(mp) != 2 ){
      ZRUNERROR( RK_ERR_JOINT_SIZMISMATCH );
      return true;
    }
    rkMotorClone( mp, &_rkc(prp)->m );
  } else
  if( !rkMotorQueryFScan( fp, buf, &_rkc(prp)->m ) )
    return false;
  return true;
}

static void *_rkJointDisFromZTKHooke(void *prp, int i, void *arg, ZTK *ztk){
  _rkJointSetDis1Hooke( prp, 0, zDeg2Rad(ZTKDouble(ztk)) );
  _rkJointSetDis1Hooke( prp, 1, zDeg2Rad(ZTKDouble(ztk)) );
  return prp;
}
static void *_rkJointMinFromZTKHooke(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->min[0] = zDeg2Rad(ZTKDouble(ztk));
  _rkc(prp)->min[1] = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointMaxFromZTKHooke(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->max[0] = zDeg2Rad(ZTKDouble(ztk));
  _rkc(prp)->max[1] = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointStiffnessFromZTKHooke(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->stiffness[0] = ZTKDouble(ztk);
  _rkc(prp)->stiffness[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointViscosityFromZTKHooke(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->viscosity[0] = ZTKDouble(ztk);
  _rkc(prp)->viscosity[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointCoulombFromZTKHooke(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->coulomb[0] = ZTKDouble(ztk);
  _rkc(prp)->coulomb[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointStaticFrictionFromZTKHooke(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->sf[0] = ZTKDouble(ztk);
  _rkc(prp)->sf[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointMotorFromZTKHooke(void *prp, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk) ) ) ) return NULL;
  return rkMotorClone( mp, &_rkc(prp)->m ) ? prp : NULL;
}

static void _rkJointDisFPrintHooke(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", zRad2Deg(_rkc(prp)->dis[0]), zRad2Deg(_rkc(prp)->dis[1]) );
}
static void _rkJointMinFPrintHooke(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", zRad2Deg(_rkc(prp)->min[0]), zRad2Deg(_rkc(prp)->min[1]) );
}
static void _rkJointMaxFPrintHooke(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", zRad2Deg(_rkc(prp)->max[0]), zRad2Deg(_rkc(prp)->max[1]) );
}
static void _rkJointStiffnessFPrintHooke(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", _rkc(prp)->stiffness[0], _rkc(prp)->stiffness[1] );
}
static void _rkJointViscosityFPrintHooke(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", _rkc(prp)->viscosity[0], _rkc(prp)->viscosity[1] );
}
static void _rkJointCoulombFPrintHooke(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", _rkc(prp)->coulomb[0], _rkc(prp)->coulomb[1] );
}
static void _rkJointStaticFrictionFPrintHooke(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", _rkc(prp)->sf[0], _rkc(prp)->sf[1] );
}
static void _rkJointMotorFPrintHooke(FILE *fp, int i, void *prp){
  fprintf( fp, "%s\n", zName(&_rkc(prp)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_hooke[] = {
  { "dis", 1, _rkJointDisFromZTKHooke, _rkJointDisFPrintHooke },
  { "min", 1, _rkJointMinFromZTKHooke, _rkJointMinFPrintHooke },
  { "max", 1, _rkJointMaxFromZTKHooke, _rkJointMaxFPrintHooke },
  { "stiffness", 1, _rkJointStiffnessFromZTKHooke, _rkJointStiffnessFPrintHooke },
  { "viscosity", 1, _rkJointViscosityFromZTKHooke, _rkJointViscosityFPrintHooke },
  { "coulomb", 1, _rkJointCoulombFromZTKHooke, _rkJointCoulombFPrintHooke },
  { "staticfriction", 1, _rkJointStaticFrictionFromZTKHooke, _rkJointStaticFrictionFPrintHooke },
  { "motor", 1, _rkJointMotorFromZTKHooke, _rkJointMotorFPrintHooke },
};

static void *_rkJointFromZTKHooke(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_hooke );
}

static void _rkJointFPrintHooke(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_hooke );
}

rkJointCom rk_joint_hooke = {
  "hooke",
  2,
  _rkJointInitHooke,
  _rkJointAllocHooke,
  _rkJointLimDisHooke,
  _rkJointSetDisHooke,
  _rkJointSetVelHooke,
  _rkJointSetAccHooke,
  _rkJointSetTrqHooke,
  _rkJointGetDisHooke,
  _rkJointGetVelHooke,
  _rkJointGetAccHooke,
  _rkJointGetTrqHooke,
  _rkJointCatDisHooke,
  _rkJointSubDisHooke,
  _rkJointSetDisCNTHooke,
  _rkJointXformHooke,
  _rkJointIncVelHooke,
  _rkJointIncAccOnVelHooke,
  _rkJointIncAccHooke,
  _rkJointCalcTrqHooke,
  _rkJointTorsionHooke,
  _rk_joint_axis_hooke_ang,
  _rk_joint_axis_hooke_lin,

  _rkJointSetFrictionPivotHooke,
  _rkJointGetFrictionPivotHooke,
  _rkJointSetFricHooke,
  _rkJointGetFricHooke,
  _rkJointGetSFricHooke,
  _rkJointGetKFricHooke,

  _rkJointGetMotorHooke,
  _rkJointMotorSetInputHooke,
  _rkJointMotorInertiaHooke,
  _rkJointMotorInputTrqHooke,
  _rkJointMotorResistanceHooke,
  _rkJointMotorDrivingTrqHooke,

  _rkJointABIAxisInertiaHooke,
  _rkJointABIAddAbiHooke,
  _rkJointABIAddBiasHooke,
  _rkJointABIDrivingTorqueHooke,
  _rkJointABIQAccHooke,
  _rkJointUpdateWrench,

  _rkJointQueryFScanHooke,
  _rkJointDisFromZTKHooke,
  _rkJointFromZTKHooke,
  _rkJointDisFPrintHooke,
  _rkJointFPrintHooke,
};

bool rkJointRegZTKHooke(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_hooke ) ? true : false;
}

#undef _rkc
