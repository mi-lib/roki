/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_cylin - joint structure: cylindrical joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointPrpCylin *)p)

static void _rkJointInitCylin(void *prp){
  _rkc(prp)->max[0] = HUGE_VAL;
  _rkc(prp)->min[0] =-HUGE_VAL;
  _rkc(prp)->max[1] = HUGE_VAL;
  _rkc(prp)->min[1] =-HUGE_VAL;
  rkMotorAssign( &_rkc(prp)->m, &rk_motor_none );
}

static void *_rkJointAllocCylin(void){ return zAlloc( rkJointPrpCylin, 1 ); }

/* limit joint displacement */
static void _rkJointLimDisCylin(void *prp, double *testval, double *limval){
  double angle;
  /* 0: prismatic */
  limval[0] = zLimit( testval[0], _rkc(prp)->min[0], _rkc(prp)->max[0] );
  /* 1: revolutional */
  angle = zPhaseNormalize( testval[1] );
  limval[1] = zLimit( angle, _rkc(prp)->min[1], _rkc(prp)->max[1] );
}

/* joint displacement set function */
static void _rkJointSetDisCylin(void *prp, double *val){
  _rkJointLimDisCylin( prp, val, _rkc(prp)->dis );
  zSinCos( _rkc(prp)->dis[1], &_rkc(prp)->_s, &_rkc(prp)->_c );
}

static void _rkJointSetVelCylin(void *prp, double *val){
  memcpy( _rkc(prp)->vel, val, sizeof(double)*2 );
}

static void _rkJointSetAccCylin(void *prp, double *val){
  memcpy( _rkc(prp)->acc, val, sizeof(double)*2 );
}

static void _rkJointSetTrqCylin(void *prp, double *val){
  memcpy( _rkc(prp)->trq, val, sizeof(double)*2 );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointGetDisCylin(void *prp, double *val){
  memcpy( val, _rkc(prp)->dis, sizeof(double)*2 );
}

static void _rkJointGetVelCylin(void *prp, double *val){
  memcpy( val, _rkc(prp)->vel, sizeof(double)*2 );
}

static void _rkJointGetAccCylin(void *prp, double *val){
  memcpy( val, _rkc(prp)->acc, sizeof(double)*2 );
}

static void _rkJointGetTrqCylin(void *prp, double *val){
  memcpy( val, _rkc(prp)->trq, sizeof(double)*2 );
}

static void _rkJointCatDisCylin(void *prp, double *dis, double k, double *val){
  dis[0] += val[0] * k;
  dis[1] += val[1] * k;
}

static void _rkJointSubDisCylin(void *prp, double *dis, double *sdis){
  dis[0] -= sdis[0];
  dis[1] -= sdis[1];
}

/* continuously update joint displacement over delta time */
static void _rkJointSetDisCNTCylin(void *prp, double *val, double dt){
  double olddis[2], oldvel[2];

  _rkJointGetDisCylin( prp, olddis );
  _rkJointGetVelCylin( prp, oldvel );
  _rkJointSetDisCylin( prp, val );
  _rkc(prp)->vel[0] = ( val[0] - olddis[0] ) / dt;
  _rkc(prp)->vel[1] = ( val[1] - olddis[1] ) / dt;
  _rkc(prp)->acc[0] = ( _rkc(prp)->vel[0] - oldvel[0] ) / dt;
  _rkc(prp)->acc[1] = ( _rkc(prp)->vel[1] - oldvel[1] ) / dt;
}

/* joint frame transformation */
static zFrame3D *_rkJointXformCylin(void *prp, zFrame3D *fo, zFrame3D *f){
  /* rotation */
  zVec3DMul( &zFrame3DAtt(fo)->v[0], _rkc(prp)->_c, &zFrame3DAtt(f)->v[0] );
  zVec3DCatDRC( &zFrame3DAtt(f)->v[0], _rkc(prp)->_s, &zFrame3DAtt(fo)->v[1] );
  zVec3DMul( &zFrame3DAtt(fo)->v[0],-_rkc(prp)->_s, &zFrame3DAtt(f)->v[1] );
  zVec3DCatDRC( &zFrame3DAtt(f)->v[1], _rkc(prp)->_c, &zFrame3DAtt(fo)->v[1] );
  zVec3DCopy( &zFrame3DAtt(fo)->v[2], &zFrame3DAtt(f)->v[2] );
  /* slide */
  zVec3DCat( zFrame3DPos(fo),
    _rkc(prp)->dis[0], &zFrame3DAtt(fo)->v[2], zFrame3DPos(f) );
  return f;
}

/* joint velocity transformation */
static void _rkJointIncVelCylin(void *prp, zVec6D *vel){
  vel->e[zZ ] += _rkc(prp)->vel[0];
  vel->e[zZA] += _rkc(prp)->vel[1];
}

static void _rkJointIncAccOnVelCylin(void *prp, zVec3D *w, zVec6D *acc){
  acc->e[zX ] += 2 * _rkc(prp)->vel[0] * w->e[zY];
  acc->e[zY ] -= 2 * _rkc(prp)->vel[0] * w->e[zX];
  acc->e[zXA] += _rkc(prp)->vel[1] * w->e[zY];
  acc->e[zYA] -= _rkc(prp)->vel[1] * w->e[zX];
}

/* joint acceleration transformation */
static void _rkJointIncAccCylin(void *prp, zVec6D *acc){
  acc->e[zZ ] += _rkc(prp)->acc[0];
  acc->e[zZA] += _rkc(prp)->acc[1];
}

/* joint torque transformation */
static void _rkJointCalcTrqCylin(void *prp, zVec6D *f){
  _rkc(prp)->trq[0] = f->e[zZ];
  _rkc(prp)->trq[1] = f->e[zZA];
}

/* inverse computation of joint torsion and displacement */
static void _rkJointTorsionCylin(zFrame3D *dev, zVec6D *t, double dis[]){
  dis[0] = rkJointTorsionDisPrism( dev, t );
  dis[1] = rkJointTorsionDisRevol( dev, t );
}

/* joint axes */
static zVec3D* (*_rk_joint_axis_cylin_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisZ,
};
static zVec3D* (*_rk_joint_axis_cylin_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisZ,
  _rkJointAxisNull,
};

static void _rkJointSetFrictionPivotCylin(void *prp, rkJointFrictionPivot *fp){
  fp[0] = _rkc(prp)->_fp[0];
  fp[1] = _rkc(prp)->_fp[1];
}

static void _rkJointGetFrictionPivotCylin(void *prp, rkJointFrictionPivot *fp){
  _rkc(prp)->_fp[0] = fp[0];
  _rkc(prp)->_fp[1] = fp[1];
}

static void _rkJointSetFricCylin(void *prp, double *val)
{
  _rkc(prp)->tf[0] = val[0];
  _rkc(prp)->tf[1] = val[1];
}
static void _rkJointGetFricCylin(void *prp, double *val){
  val[0] = _rkc(prp)->tf[0];
  val[1] = _rkc(prp)->tf[1];
}
static void _rkJointGetSFricCylin(void *prp, double *val){
  val[0] = _rkc(prp)->sf[0];
  val[1] = _rkc(prp)->sf[1];
}
static void _rkJointGetKFricCylin(void *prp, double *val){
  val[0] = _rkJointRestTrq( _rkc(prp)->stiffness[0], _rkc(prp)->viscosity[0], _rkc(prp)->coulomb[0], _rkc(prp)->dis[0], _rkc(prp)->vel[0] );
  val[1] = _rkJointRestTrq( _rkc(prp)->stiffness[1], _rkc(prp)->viscosity[1], _rkc(prp)->coulomb[1], _rkc(prp)->dis[1], _rkc(prp)->vel[1] );
}

/* motor */
static rkMotor *_rkJointGetMotorCylin(void *prp){ return &_rkc(prp)->m; }
static void _rkJointMotorSetInputCylin(void *prp, double *val){
  rkMotorSetInput( &_rkc(prp)->m, val );
}
static void _rkJointMotorInertiaCylin(void *prp, double *val){
  zRawVecZero( val, 4 );
  rkMotorInertia( &_rkc(prp)->m, val );
}
static void _rkJointMotorInputTrqCylin(void *prp, double *val){
  zRawVecZero( val, 2 );
  rkMotorInputTrq( &_rkc(prp)->m, val );
}
static void _rkJointMotorResistanceCylin(void *prp, double *val){
  zRawVecZero( val, 2 );
  rkMotorRegistance( &_rkc(prp)->m, _rkc(prp)->dis, _rkc(prp)->vel, val );
}
static void _rkJointMotorDrivingTrqCylin(void *prp, double *val){
  zRawVecZero( val, 2 );
  rkMotorDrivingTrq( &_rkc(prp)->m, _rkc(prp)->dis, _rkc(prp)->vel, _rkc(prp)->acc, val );
}

/* ABI */
static void _rkJointABIAxisInertiaCylin(void *prp, zMat6D *m, zMat h, zMat ih){
  _rkJointMotorInertiaCylin( prp, zMatBuf(h) );
  zMatElemNC(h,0,0) += m->e[0][0].e[2][2];
  zMatElemNC(h,1,0) += m->e[0][1].e[2][2];
  zMatElemNC(h,0,1) += m->e[1][0].e[2][2];
  zMatElemNC(h,1,1) += m->e[1][1].e[2][2];
  zMatInv( h, ih );
}

static void _rkJointABIAddAbiCylin(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  zVec6D v13, v31, v16, v61, tmpv;
  zMat6D tmpm, tmpm2;

  zMat6DCol( m, zZ,  &v13 );
  zMat6DCol( m, zZA, &v16 );
  zMat6DRow( m, zZ,  &v31 );
  zMat6DRow( m, zZA, &v61 );

  zVec6DMul( &v13, zMatElemNC(h,0,0), &tmpv );
  zMat6DDyad( &tmpm, &tmpv, &v31 );
  zVec6DMul( &v13, zMatElemNC(h,0,1), &tmpv );
  zMat6DDyad( &tmpm2, &tmpv, &v61 );
  zMat6DAddDRC( &tmpm, &tmpm2 );
  zVec6DMul( &v16, zMatElemNC(h,1,0), &tmpv );
  zMat6DDyad( &tmpm2, &tmpv, &v31 );
  zMat6DAddDRC( &tmpm, &tmpm2 );
  zVec6DMul( &v16, zMatElemNC(h,1,1), &tmpv );
  zMat6DDyad( &tmpm2, &tmpv, &v61 );
  zMat6DAddDRC( &tmpm, &tmpm2 );

  rkJointXformMat6D( f, &tmpm, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}

static void _rkJointABIAddBiasCylin(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  zVec6D v13, v16, tmpv;
  zVec3D tmp3v;

  zMat6DCol( m, zZ,  &v13 );
  zMat6DCol( m, zZA, &v16 );
  zVec6DCat(b,        (_rkc(prp)->_u[0] - b->e[zZ])*zMatElemNC(h,0,0) + (_rkc(prp)->_u[1] - b->e[zZA])*zMatElemNC(h,0,1), &v13, &tmpv );
  zVec6DCatDRC(&tmpv, (_rkc(prp)->_u[0] - b->e[zZ])*zMatElemNC(h,1,0) + (_rkc(prp)->_u[1] - b->e[zZA])*zMatElemNC(h,1,1), &v16 );

  zMulMat3DVec6D( zFrame3DAtt(f), &tmpv, &v13 );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec3DAddDRC( zVec6DAng(&v13), zVec3DOuterProd( zFrame3DPos(f), zVec6DLin(&v13), &tmp3v ) );
  zVec6DAddDRC( pb, &v13 );
}

static void _rkJointABIDrivingTorqueCylin(void *prp){
  double val[2];
  _rkJointMotorInputTrqCylin( prp, _rkc(prp)->_u );
  _rkJointMotorResistanceCylin( prp, val );
  _rkc(prp)->_u[0] -= val[0];
  _rkc(prp)->_u[1] -= val[1];
  _rkc(prp)->_u[0] += _rkc(prp)->tf[0];
  _rkc(prp)->_u[1] += _rkc(prp)->tf[1];
}

static void _rkJointABIQAccCylin(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  double u[2];
  zVec6D v31, v61;

  zMat6DRow( m, zZ, &v31 );
  zMat6DRow( m, zZA, &v61 );
  u[0] = _rkc(prp)->_u[0] - zVec6DInnerProd( &v31, jac ) + b->e[zZ];
  u[1] = _rkc(prp)->_u[1] - zVec6DInnerProd( &v61, jac ) + b->e[zZA];
  /* q */
  _rkc(prp)->acc[0] = u[0]*zMatElemNC(h,0,0) + u[1]*zMatElemNC(h,0,1);
  _rkc(prp)->acc[1] = u[0]*zMatElemNC(h,1,0) + u[1]*zMatElemNC(h,1,1);
  /* acc */
  zVec6DCopy( jac, acc );
  acc->e[zZ ] += _rkc(prp)->acc[0];
  acc->e[zZA] += _rkc(prp)->acc[1];
}

/* query joint properties */
static bool _rkJointQueryFScanCylin(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm){
  rkMotor *mp;
  double dis[2];

  if( strcmp( buf, "dis" ) == 0 ){
    dis[0] = zFDouble(fp);
    dis[1] = zDeg2Rad(zFDouble(fp));
    _rkJointSetDisCylin( prp, dis );
  } else
  if( strcmp( buf, "min" ) == 0 ){
    _rkc(prp)->min[0] = zFDouble(fp);
    _rkc(prp)->min[1] = zDeg2Rad(zFDouble(fp));
  } else
  if( strcmp( buf, "max" ) == 0 ){
    _rkc(prp)->max[0] = zFDouble(fp);
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

static void *_rkJointDisFromZTKCylin(void *prp, int i, void *arg, ZTK *ztk){
  double dis[2];
  dis[0] = ZTKDouble(ztk);
  dis[1] = zDeg2Rad(ZTKDouble(ztk));
  _rkJointSetDisCylin( prp, dis );
  return prp;
}
static void *_rkJointMinFromZTKCylin(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->min[0] = ZTKDouble(ztk);
  _rkc(prp)->min[1] = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointMaxFromZTKCylin(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->max[0] = ZTKDouble(ztk);
  _rkc(prp)->max[1] = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointStiffnessFromZTKCylin(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->stiffness[0] = ZTKDouble(ztk);
  _rkc(prp)->stiffness[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointViscosityFromZTKCylin(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->viscosity[0] = ZTKDouble(ztk);
  _rkc(prp)->viscosity[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointCoulombFromZTKCylin(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->coulomb[0] = ZTKDouble(ztk);
  _rkc(prp)->coulomb[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointStaticFrictionFromZTKCylin(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->sf[0] = ZTKDouble(ztk);
  _rkc(prp)->sf[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointMotorFromZTKCylin(void *prp, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk) ) ) ) return NULL;
  return rkMotorClone( mp, &_rkc(prp)->m ) ? prp : NULL;
}

static void _rkJointDisFPrintCylin(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->dis[0],
    zRad2Deg(_rkc(prp)->dis[1]) );
}
static void _rkJointMinFPrintCylin(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->min[0],
    zRad2Deg(_rkc(prp)->min[1]) );
}
static void _rkJointMaxFPrintCylin(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->max[0],
    zRad2Deg(_rkc(prp)->max[1]) );
}
static void _rkJointStiffnessFPrintCylin(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->stiffness[0],
    _rkc(prp)->stiffness[1] );
}
static void _rkJointViscosityFPrintCylin(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->viscosity[0],
    _rkc(prp)->viscosity[1] );
}
static void _rkJointCoulombFPrintCylin(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->coulomb[0],
    _rkc(prp)->coulomb[1] );
}
static void _rkJointStaticFrictionFPrintCylin(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->sf[0],
    _rkc(prp)->sf[1] );
}
static void _rkJointMotorFPrintCylin(FILE *fp, int i, void *prp){
  fprintf( fp, "%s\n", zName(&_rkc(prp)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_cylin[] = {
  { "dis", 1, _rkJointDisFromZTKCylin, _rkJointDisFPrintCylin },
  { "min", 1, _rkJointMinFromZTKCylin, _rkJointMinFPrintCylin },
  { "max", 1, _rkJointMaxFromZTKCylin, _rkJointMaxFPrintCylin },
  { "stiffness", 1, _rkJointStiffnessFromZTKCylin, _rkJointStiffnessFPrintCylin },
  { "viscosity", 1, _rkJointViscosityFromZTKCylin, _rkJointViscosityFPrintCylin },
  { "coulomb", 1, _rkJointCoulombFromZTKCylin, _rkJointCoulombFPrintCylin },
  { "staticfriction", 1, _rkJointStaticFrictionFromZTKCylin, _rkJointStaticFrictionFPrintCylin },
  { "motor", 1, _rkJointMotorFromZTKCylin, _rkJointMotorFPrintCylin },
};

static void *_rkJointFromZTKCylin(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_cylin );
}

static void _rkJointFPrintCylin(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_cylin );
}

rkJointCom rk_joint_cylin = {
  "cylindrical",
  2,
  _rkJointInitCylin,
  _rkJointAllocCylin,
  _rkJointLimDisCylin,
  _rkJointSetDisCylin,
  _rkJointSetVelCylin,
  _rkJointSetAccCylin,
  _rkJointSetTrqCylin,
  _rkJointGetDisCylin,
  _rkJointGetVelCylin,
  _rkJointGetAccCylin,
  _rkJointGetTrqCylin,
  _rkJointCatDisCylin,
  _rkJointSubDisCylin,
  _rkJointSetDisCNTCylin,
  _rkJointXformCylin,
  _rkJointIncVelCylin,
  _rkJointIncAccOnVelCylin,
  _rkJointIncAccCylin,
  _rkJointCalcTrqCylin,
  _rkJointTorsionCylin,
  _rk_joint_axis_cylin_ang,
  _rk_joint_axis_cylin_lin,

  _rkJointSetFrictionPivotCylin,
  _rkJointGetFrictionPivotCylin,
  _rkJointSetFricCylin,
  _rkJointGetFricCylin,
  _rkJointGetSFricCylin,
  _rkJointGetKFricCylin,

  _rkJointGetMotorCylin,
  _rkJointMotorSetInputCylin,
  _rkJointMotorInertiaCylin,
  _rkJointMotorInputTrqCylin,
  _rkJointMotorResistanceCylin,
  _rkJointMotorDrivingTrqCylin,

  _rkJointABIAxisInertiaCylin,
  _rkJointABIAddAbiCylin,
  _rkJointABIAddBiasCylin,
  _rkJointABIDrivingTorqueCylin,
  _rkJointABIQAccCylin,
  _rkJointUpdateWrench,

  _rkJointQueryFScanCylin,
  _rkJointDisFromZTKCylin,
  _rkJointFromZTKCylin,
  _rkJointDisFPrintCylin,
  _rkJointFPrintCylin,
};

bool rkJointRegZTKCylin(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_cylin ) ? true : false;
}

#undef _rkc
