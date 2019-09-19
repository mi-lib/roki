/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_cylin - joint structure: cylindrical joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointCylinPrp *)p)

static void _rkJointCylinInit(void *prp){
  _rkc(prp)->max[0] = HUGE_VAL;
  _rkc(prp)->min[0] =-HUGE_VAL;
  _rkc(prp)->max[1] = HUGE_VAL;
  _rkc(prp)->min[1] =-HUGE_VAL;
  rkMotorAssign( &_rkc(prp)->m, &rk_motor_none );
}

static void *_rkJointCylinAlloc(void){ return zAlloc( rkJointCylinPrp, 1 ); }

/* limit joint displacement */
static void _rkJointCylinLimDis(void *prp, double *testval, double *limval){
  double angle;
  /* 0: prismatic */
  limval[0] = zLimit( testval[0], _rkc(prp)->min[0], _rkc(prp)->max[0] );
  /* 1: revolutional */
  angle = zPhaseNormalize( testval[1] );
  limval[1] = zLimit( angle, _rkc(prp)->min[1], _rkc(prp)->max[1] );
}

/* joint displacement set function */
static void _rkJointCylinSetDis(void *prp, double *val){
  _rkJointCylinLimDis( prp, val, _rkc(prp)->dis );
  zSinCos( _rkc(prp)->dis[1], &_rkc(prp)->_s, &_rkc(prp)->_c );
}

static void _rkJointCylinSetVel(void *prp, double *val){
  memcpy( _rkc(prp)->vel, val, sizeof(double)*2 );
}

static void _rkJointCylinSetAcc(void *prp, double *val){
  memcpy( _rkc(prp)->acc, val, sizeof(double)*2 );
}

static void _rkJointCylinSetTrq(void *prp, double *val){
  memcpy( _rkc(prp)->trq, val, sizeof(double)*2 );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointCylinGetDis(void *prp, double *val){
  memcpy( val, _rkc(prp)->dis, sizeof(double)*2 );
}

static void _rkJointCylinGetVel(void *prp, double *val){
  memcpy( val, _rkc(prp)->vel, sizeof(double)*2 );
}

static void _rkJointCylinGetAcc(void *prp, double *val){
  memcpy( val, _rkc(prp)->acc, sizeof(double)*2 );
}

static void _rkJointCylinGetTrq(void *prp, double *val){
  memcpy( val, _rkc(prp)->trq, sizeof(double)*2 );
}

static void _rkJointCylinCatDis(void *prp, double *dis, double k, double *val){
  dis[0] += val[0] * k;
  dis[1] += val[1] * k;
}

static void _rkJointCylinSubDis(void *prp, double *dis, double *sdis){
  dis[0] -= sdis[0];
  dis[1] -= sdis[1];
}

/* continuously update joint displacement over delta time */
static void _rkJointCylinSetDisCNT(void *prp, double *val, double dt){
  double olddis[2], oldvel[2];

  _rkJointCylinGetDis( prp, olddis );
  _rkJointCylinGetVel( prp, oldvel );
  _rkJointCylinSetDis( prp, val );
  _rkc(prp)->vel[0] = ( val[0] - olddis[0] ) / dt;
  _rkc(prp)->vel[1] = ( val[1] - olddis[1] ) / dt;
  _rkc(prp)->acc[0] = ( _rkc(prp)->vel[0] - oldvel[0] ) / dt;
  _rkc(prp)->acc[1] = ( _rkc(prp)->vel[1] - oldvel[1] ) / dt;
}

/* joint frame transformation */
static zFrame3D *_rkJointCylinXform(void *prp, zFrame3D *fo, zFrame3D *f){
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
static void _rkJointCylinIncVel(void *prp, zVec6D *vel){
  vel->e[zZ ] += _rkc(prp)->vel[0];
  vel->e[zZA] += _rkc(prp)->vel[1];
}

static void _rkJointCylinIncAccOnVel(void *prp, zVec3D *w, zVec6D *acc){
  acc->e[zX ] += 2 * _rkc(prp)->vel[0] * w->e[zY];
  acc->e[zY ] -= 2 * _rkc(prp)->vel[0] * w->e[zX];
  acc->e[zXA] += _rkc(prp)->vel[1] * w->e[zY];
  acc->e[zYA] -= _rkc(prp)->vel[1] * w->e[zX];
}

/* joint acceleration transformation */
static void _rkJointCylinIncAcc(void *prp, zVec6D *acc){
  acc->e[zZ ] += _rkc(prp)->acc[0];
  acc->e[zZA] += _rkc(prp)->acc[1];
}

/* joint torque transformation */
static void _rkJointCylinCalcTrq(void *prp, zVec6D *f){
  _rkc(prp)->trq[0] = f->e[zZ];
  _rkc(prp)->trq[1] = f->e[zZA];
}

/* inverse computation of joint torsion and displacement */
static void _rkJointCylinTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  dis[0] = rkJointPrismTorsionDis( dev, t );
  dis[1] = rkJointRevolTorsionDis( dev, t );
}

/* joint axes */
static zVec3D* (*_rk_joint_cylin_axis_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisZ,
};
static zVec3D* (*_rk_joint_cylin_axis_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisZ,
  _rkJointAxisNull,
};

static void _rkJointCylinSetFrictionPivot(void *prp, rkJointFrictionPivot *fp){
  fp[0] = _rkc(prp)->_fp[0];
  fp[1] = _rkc(prp)->_fp[1];
}

static void _rkJointCylinGetFrictionPivot(void *prp, rkJointFrictionPivot *fp){
  _rkc(prp)->_fp[0] = fp[0];
  _rkc(prp)->_fp[1] = fp[1];
}

static void _rkJointCylinSetFric(void *prp, double *val)
{
  _rkc(prp)->tf[0] = val[0];
  _rkc(prp)->tf[1] = val[1];
}
static void _rkJointCylinGetFric(void *prp, double *val){
  val[0] = _rkc(prp)->tf[0];
  val[1] = _rkc(prp)->tf[1];
}
static void _rkJointCylinGetSFric(void *prp, double *val){
  val[0] = _rkc(prp)->sf[0];
  val[1] = _rkc(prp)->sf[1];
}
static void _rkJointCylinGetKFric(void *prp, double *val){
  val[0] = _rkJointRestTrq( _rkc(prp)->stiffness[0], _rkc(prp)->viscosity[0], _rkc(prp)->coulomb[0], _rkc(prp)->dis[0], _rkc(prp)->vel[0] );
  val[1] = _rkJointRestTrq( _rkc(prp)->stiffness[1], _rkc(prp)->viscosity[1], _rkc(prp)->coulomb[1], _rkc(prp)->dis[1], _rkc(prp)->vel[1] );
}

/* motor */
static rkMotor *_rkJointCylinGetMotor(void *prp){ return &_rkc(prp)->m; }
static void _rkJointCylinMotorSetInput(void *prp, double *val){
  rkMotorSetInput( &_rkc(prp)->m, val );
}
static void _rkJointCylinMotorInertia(void *prp, double *val){
  zRawVecZero( val, 4 );
  rkMotorInertia( &_rkc(prp)->m, val );
}
static void _rkJointCylinMotorInputTrq(void *prp, double *val){
  zRawVecZero( val, 2 );
  rkMotorInputTrq( &_rkc(prp)->m, val );
}
static void _rkJointCylinMotorResistance(void *prp, double *val){
  zRawVecZero( val, 2 );
  rkMotorRegistance( &_rkc(prp)->m, _rkc(prp)->dis, _rkc(prp)->vel, val );
}
static void _rkJointCylinMotorDrivingTrq(void *prp, double *val){
  zRawVecZero( val, 2 );
  rkMotorDrivingTrq( &_rkc(prp)->m, _rkc(prp)->dis, _rkc(prp)->vel, _rkc(prp)->acc, val );
}

/* ABI */
static void _rkJointCylinABIAxisInertia(void *prp, zMat6D *m, zMat h, zMat ih){
  _rkJointCylinMotorInertia( prp, zMatBuf(h) );
  zMatElemNC(h,0,0) += m->e[0][0].e[2][2];
  zMatElemNC(h,1,0) += m->e[0][1].e[2][2];
  zMatElemNC(h,0,1) += m->e[1][0].e[2][2];
  zMatElemNC(h,1,1) += m->e[1][1].e[2][2];
  zMatInv( h, ih );
}

static void _rkJointCylinABIAddABI(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
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

static void _rkJointCylinABIAddBias(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
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

static void _rkJointCylinABIDrivingTorque(void *prp){
  double val[2];
  _rkJointCylinMotorInputTrq( prp, _rkc(prp)->_u );
  _rkJointCylinMotorResistance( prp, val );
  _rkc(prp)->_u[0] -= val[0];
  _rkc(prp)->_u[1] -= val[1];
  _rkc(prp)->_u[0] += _rkc(prp)->tf[0];
  _rkc(prp)->_u[1] += _rkc(prp)->tf[1];
}

static void _rkJointCylinABIQAcc(void *prp, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
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

static void *_rkJointCylinDisFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  double dis[2];
  dis[0] = ZTKDouble(ztk);
  dis[1] = zDeg2Rad(ZTKDouble(ztk));
  _rkJointCylinSetDis( prp, dis );
  return prp;
}
static void *_rkJointCylinMinFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->min[0] = ZTKDouble(ztk);
  _rkc(prp)->min[1] = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointCylinMaxFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->max[0] = ZTKDouble(ztk);
  _rkc(prp)->max[1] = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointCylinStiffnessFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->stiffness[0] = ZTKDouble(ztk);
  _rkc(prp)->stiffness[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointCylinViscosityFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->viscosity[0] = ZTKDouble(ztk);
  _rkc(prp)->viscosity[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointCylinCoulombFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->coulomb[0] = ZTKDouble(ztk);
  _rkc(prp)->coulomb[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointCylinStaticFrictionFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->sf[0] = ZTKDouble(ztk);
  _rkc(prp)->sf[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointCylinMotorFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk) ) ) ) return NULL;
  return rkMotorClone( mp, &_rkc(prp)->m ) ? prp : NULL;
}

static void _rkJointCylinDisFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->dis[0],
    zRad2Deg(_rkc(prp)->dis[1]) );
}
static void _rkJointCylinMinFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->min[0],
    zRad2Deg(_rkc(prp)->min[1]) );
}
static void _rkJointCylinMaxFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->max[0],
    zRad2Deg(_rkc(prp)->max[1]) );
}
static void _rkJointCylinStiffnessFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->stiffness[0],
    _rkc(prp)->stiffness[1] );
}
static void _rkJointCylinViscosityFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->viscosity[0],
    _rkc(prp)->viscosity[1] );
}
static void _rkJointCylinCoulombFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->coulomb[0],
    _rkc(prp)->coulomb[1] );
}
static void _rkJointCylinStaticFrictionFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n",
    _rkc(prp)->sf[0],
    _rkc(prp)->sf[1] );
}
static void _rkJointCylinMotorFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%s\n", zName(&_rkc(prp)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_cylin[] = {
  { "dis", 1, _rkJointCylinDisFromZTK, _rkJointCylinDisFPrintZTK },
  { "min", 1, _rkJointCylinMinFromZTK, _rkJointCylinMinFPrintZTK },
  { "max", 1, _rkJointCylinMaxFromZTK, _rkJointCylinMaxFPrintZTK },
  { "stiffness", 1, _rkJointCylinStiffnessFromZTK, _rkJointCylinStiffnessFPrintZTK },
  { "viscosity", 1, _rkJointCylinViscosityFromZTK, _rkJointCylinViscosityFPrintZTK },
  { "coulomb", 1, _rkJointCylinCoulombFromZTK, _rkJointCylinCoulombFPrintZTK },
  { "staticfriction", 1, _rkJointCylinStaticFrictionFromZTK, _rkJointCylinStaticFrictionFPrintZTK },
  { "motor", 1, _rkJointCylinMotorFromZTK, _rkJointCylinMotorFPrintZTK },
};

static bool _rkJointCylinRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_cylin ) ? true : false;
}

static void *_rkJointCylinFromZTK(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_cylin );
}

static void _rkJointCylinFPrintZTK(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_cylin );
}

rkJointCom rk_joint_cylin = {
  "cylindrical",
  2,
  _rkJointCylinInit,
  _rkJointCylinAlloc,
  _rkJointCylinLimDis,
  _rkJointCylinSetDis,
  _rkJointCylinSetVel,
  _rkJointCylinSetAcc,
  _rkJointCylinSetTrq,
  _rkJointCylinGetDis,
  _rkJointCylinGetVel,
  _rkJointCylinGetAcc,
  _rkJointCylinGetTrq,
  _rkJointCylinCatDis,
  _rkJointCylinSubDis,
  _rkJointCylinSetDisCNT,
  _rkJointCylinXform,
  _rkJointCylinIncVel,
  _rkJointCylinIncAccOnVel,
  _rkJointCylinIncAcc,
  _rkJointCylinCalcTrq,
  _rkJointCylinTorsion,
  _rk_joint_cylin_axis_ang,
  _rk_joint_cylin_axis_lin,

  _rkJointCylinSetFrictionPivot,
  _rkJointCylinGetFrictionPivot,
  _rkJointCylinSetFric,
  _rkJointCylinGetFric,
  _rkJointCylinGetSFric,
  _rkJointCylinGetKFric,

  _rkJointCylinGetMotor,
  _rkJointCylinMotorSetInput,
  _rkJointCylinMotorInertia,
  _rkJointCylinMotorInputTrq,
  _rkJointCylinMotorResistance,
  _rkJointCylinMotorDrivingTrq,

  _rkJointCylinABIAxisInertia,
  _rkJointCylinABIAddABI,
  _rkJointCylinABIAddBias,
  _rkJointCylinABIDrivingTorque,
  _rkJointCylinABIQAcc,
  _rkJointUpdateWrench,

  _rkJointCylinRegZTK,
  _rkJointCylinDisFromZTK,
  _rkJointCylinFromZTK,
  _rkJointCylinDisFPrintZTK,
  _rkJointCylinFPrintZTK,
};

#undef _rkc
