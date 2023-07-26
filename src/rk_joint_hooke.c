/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_hooke - joint structure: universal joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointHookePrp *)p)

static void _rkJointHookeInit(void *prp){
  _rkc(prp)->max[0] = HUGE_VAL;
  _rkc(prp)->min[0] =-HUGE_VAL;
  _rkc(prp)->max[1] = HUGE_VAL;
  _rkc(prp)->min[1] =-HUGE_VAL;
  rkMotorAssign( &_rkc(prp)->m, &rk_motor_none );
}

static void *_rkJointHookeAlloc(void){ return zAlloc( rkJointHookePrp, 1 ); }

/* limit joint displacement */
static double _rkJointHookeLimDis1(void *prp, int i, double testval){
  testval = zPhaseNormalize( testval );
  return zLimit( testval, _rkc(prp)->min[i], _rkc(prp)->max[i] );
}

static void _rkJointHookeLimDis(void *prp, double *testval, double *limval){
  limval[0] = _rkJointHookeLimDis1( prp, 0, testval[0] );
  limval[1] = _rkJointHookeLimDis1( prp, 1, testval[1] );
}

/* set joint displacement */
static void _rkJointHookeSetDis1(void *prp, int i, double val){
  _rkc(prp)->dis[i] = _rkJointHookeLimDis1( prp, i, val );
  zSinCos( _rkc(prp)->dis[i], &_rkc(prp)->_s[i], &_rkc(prp)->_c[i] );
}

static void _rkJointHookeSetDis(void *prp, double *val){
  _rkJointHookeSetDis1( prp, 0, val[0] );
  _rkJointHookeSetDis1( prp, 1, val[1] );
}

static void _rkJointHookeSetMin(void *prp, double *val){
  memcpy( _rkc(prp)->min, val, sizeof(double)*2 );
}

static void _rkJointHookeSetMax(void *prp, double *val){
  memcpy( _rkc(prp)->max, val, sizeof(double)*2 );
}

static void _rkJointHookeSetVel(void *prp, double *val){
  memcpy( _rkc(prp)->vel, val, sizeof(double)*2 );
}

static void _rkJointHookeSetAcc(void *prp, double *val){
  memcpy( _rkc(prp)->acc, val, sizeof(double)*2 );
}

static void _rkJointHookeSetTrq(void *prp, double *val){
  memcpy( _rkc(prp)->trq, val, sizeof(double)*2 );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointHookeGetDis(void *prp, double *val){
  memcpy( val, _rkc(prp)->dis, sizeof(double)*2 );
}

static void _rkJointHookeGetMin(void *prp, double *val){
  memcpy( val, _rkc(prp)->min, sizeof(double)*2 );
}

static void _rkJointHookeGetMax(void *prp, double *val){
  memcpy( val, _rkc(prp)->max, sizeof(double)*2 );
}

static void _rkJointHookeGetVel(void *prp, double *val){
  memcpy( val, _rkc(prp)->vel, sizeof(double)*2 );
}

static void _rkJointHookeGetAcc(void *prp, double *val){
  memcpy( val, _rkc(prp)->acc, sizeof(double)*2 );
}

static void _rkJointHookeGetTrq(void *prp, double *val){
  memcpy( val, _rkc(prp)->trq, sizeof(double)*2 );
}

static void _rkJointHookeCatDis(void *prp, double *dis, double k, double *val){
  dis[0] += val[0] * k;
  dis[1] += val[1] * k;
}

static void _rkJointHookeSubDis(void *prp, double *dis, double *sdis){
  dis[0] -= sdis[0];
  dis[1] -= sdis[1];
}

/* continuously update joint displacement */
static void _rkJointHookeSetDisCNT(void *prp, double *val, double dt){
  double olddis[2], oldvel[2];
  _rkJointHookeGetDis( prp, olddis );
  _rkJointHookeGetVel( prp, oldvel );
  _rkJointHookeSetDis( prp, val );
  _rkc(prp)->vel[0] = ( val[0] - olddis[0] ) / dt;
  _rkc(prp)->vel[1] = ( val[1] - olddis[1] ) / dt;
  _rkc(prp)->acc[0] = ( _rkc(prp)->vel[0] - oldvel[0] ) / dt;
  _rkc(prp)->acc[1] = ( _rkc(prp)->vel[1] - oldvel[1] ) / dt;
}

/* joint frame transformation */
static zFrame3D *_rkJointHookeXform(void *prp, zFrame3D *fo, zFrame3D *f){
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
static void _rkJointHookeIncVel(void *prp, zVec6D *vel){
  zVec3D v;
  zVec3DCreate( &v, -_rkc(prp)->_s[1]*_rkc(prp)->vel[0], _rkc(prp)->vel[1], _rkc(prp)->_c[1]*_rkc(prp)->vel[0] );
  zVec3DAddDRC( zVec6DAng(vel), &v );
}

static void _rkJointHookeIncAccOnVel(void *prp, zVec3D *w, zVec6D *acc){
  zVec3D v;
  double dq2;
  zVec3DCreate( &v, -_rkc(prp)->_s[1]*_rkc(prp)->vel[0], _rkc(prp)->vel[1], _rkc(prp)->_c[1]*_rkc(prp)->vel[0] );
  zVec3DOuterProd( w, &v, &v );
  zVec3DAddDRC( zVec6DAng(acc), &v );
  dq2 = _rkc(prp)->vel[0] * _rkc(prp)->vel[1];
  zVec3DCreate( &v, -_rkc(prp)->_c[1]*dq2, 0, -_rkc(prp)->_s[1]*dq2 );
  zVec3DAddDRC( zVec6DAng(acc), &v );
}

static void _rkJointHookeIncAcc(void *prp, zVec6D *acc){
  zVec3D v1;
  zVec3DCreate( &v1, -_rkc(prp)->_s[1]*_rkc(prp)->acc[0], _rkc(prp)->acc[1], _rkc(prp)->_c[1]*_rkc(prp)->acc[0] );
  zVec3DAddDRC( zVec6DAng(acc), &v1 );
}

/* joint torque transformation */
static void _rkJointHookeCalcTrq(void *prp, zVec6D *f){
  _rkc(prp)->trq[0] =-_rkc(prp)->_s[1]*f->e[zXA]+_rkc(prp)->_c[1]*f->e[zZA];
  _rkc(prp)->trq[1] = f->e[zYA];
}

/* inverse computation of joint torsion and displacement */
static void _rkJointHookeTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
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
static zVec3D *_rkJointHookeAngAxis1(void *prp, zFrame3D *f, zVec3D *a){
  zVec3DMul( &zFrame3DAtt(f)->v[zX],-_rkc(prp)->_s[1], a );
  return zVec3DCatDRC( a, _rkc(prp)->_c[1], &zFrame3DAtt(f)->v[2] );
}

static zVec3D *_rkJointHookeAngAxis2(void *prp, zFrame3D *f, zVec3D *a){
  zVec3DCopy( &zFrame3DAtt(f)->v[1], a );
  return a;
}

static zVec3D* (*_rk_joint_hooke_axis_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointHookeAngAxis1,
  _rkJointHookeAngAxis2,
};
static zVec3D* (*_rk_joint_hooke_axis_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisNull,
};

/* CRB method */
static void _rkJointHookeCRBWrench(void *prp, rkMP *crb, zVec6D wi[]){
  zVec6D wx, wz;

  _rkJointCRBWrenchAngX( crb, &wx );
  _rkJointCRBWrenchAngZ( crb, &wz );
  _zVec6DMul( &wx, -((rkJointHookePrp *)prp)->_s[1], &wi[0] );
  _zVec6DCatDRC( &wi[0], ((rkJointHookePrp *)prp)->_c[1], &wz );

  _rkJointCRBWrenchAngY( crb, &wi[1] );
}
static void _rkJointHookeCRBXform(void *prp, zFrame3D *f, zVec6D si[]){
  zVec6D sx, sz;

  _rkJointCRBXformAng( f, zX, &sx );
  _rkJointCRBXformAng( f, zZ, &sz );
  _zVec6DMul( &sx, -((rkJointHookePrp *)prp)->_s[1], &si[0] );
  _zVec6DCatDRC( &si[0], ((rkJointHookePrp *)prp)->_c[1], &sz );

  _rkJointCRBXformAng( f, zY, &si[1] );
}


static void _rkJointHookeSetFrictionPivot(void *prp, rkJointFrictionPivot *fp){
  fp[0] = _rkc(prp)->_fp[0];
  fp[1] = _rkc(prp)->_fp[1];
}

static void _rkJointHookeGetFrictionPivot(void *prp, rkJointFrictionPivot *fp){
  _rkc(prp)->_fp[0] = fp[0];
  _rkc(prp)->_fp[1] = fp[1];
}

static void _rkJointHookeSetFriction(void *prp, double *val){
  _rkc(prp)->tf[0] = val[0];
  _rkc(prp)->tf[1] = val[1];
}

static void _rkJointHookeGetFriction(void *prp, double *val){
  val[0] = _rkc(prp)->tf[0];
  val[1] = _rkc(prp)->tf[1];
}

static void _rkJointHookeGetSFriction(void *prp, double *val){
  val[0] = _rkc(prp)->sf[0];
  val[1] = _rkc(prp)->sf[1];
}

static void _rkJointHookeGetKFriction(void *prp, double *val){
  val[0] = _rkJointRestTrq( _rkc(prp)->stiffness[0], _rkc(prp)->viscosity[0], _rkc(prp)->coulomb[0], _rkc(prp)->dis[0], _rkc(prp)->vel[0] );
  val[1] = _rkJointRestTrq( _rkc(prp)->stiffness[1], _rkc(prp)->viscosity[1], _rkc(prp)->coulomb[1], _rkc(prp)->dis[1], _rkc(prp)->vel[1] );
}

/* motor */
static rkMotor *_rkJointHookeGetMotor(void *prp){ return &_rkc(prp)->m; }

static void _rkJointHookeMotorSetInput(void *prp, double *val){
  rkMotorSetInput( &_rkc(prp)->m, val );
}

static void _rkJointHookeMotorInertia(void *prp, double *val){
  zRawVecZero( val, 4 );
  rkMotorInertia( &_rkc(prp)->m, val );
}

static void _rkJointHookeMotorInputTrq(void *prp, double *val){
  zRawVecZero( val, 2 );
  rkMotorInputTrq( &_rkc(prp)->m, val );
}

static void _rkJointHookeMotorResistance(void *prp, double *val){
  zRawVecZero( val, 2 );
  rkMotorRegistance( &_rkc(prp)->m, _rkc(prp)->dis, _rkc(prp)->vel, val );
}

static void _rkJointHookeMotorDrivingTrq(void *prp, double *val){
  zRawVecZero( val, 2 );
  rkMotorDrivingTrq( &_rkc(prp)->m, _rkc(prp)->dis, _rkc(prp)->vel, _rkc(prp)->acc, val );
}

/* ABI */
static void _rkJointHookeABIAxisInertia(void *prp, zMat6D *m, zMat h, zMat ih){
  zMat3D *m22;

  _rkJointHookeMotorInertia( prp, zMatBufNC(h) );
  m22 = &m->e[1][1];
  zMatElemNC(h,0,0) += m22->e[0][0]*_rkc(prp)->_s[1]*_rkc(prp)->_s[1] - (m22->e[2][0] + m22->e[0][2])*_rkc(prp)->_s[1]*_rkc(prp)->_c[1] + m22->e[2][2]*_rkc(prp)->_c[1]*_rkc(prp)->_c[1];
  zMatElemNC(h,1,0) += m22->e[2][1]*_rkc(prp)->_c[1] - m22->e[0][1]*_rkc(prp)->_s[1];
  zMatElemNC(h,0,1) += m22->e[1][2]*_rkc(prp)->_c[1] - m22->e[1][0]*_rkc(prp)->_s[1];
  zMatElemNC(h,1,1) += m22->e[1][1];
  zMatInv( h, ih );
}

static void _rkJointHookeABIAddABI(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  eprintf("under construction error: abi update for hooke joint\n");
}

static void _rkJointHookeABIAddBias(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  eprintf("under construction error: abi update for hooke joint\n");
}

static void _rkJointHookeABIDrivingTorque(void *prp){
  eprintf("under construction error: abi update for hooke joint\n");
}

static void _rkJointHookeABIQAcc(void *prp, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){}

static void *_rkJointHookeDisFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkJointHookeSetDis1( prp, 0, zDeg2Rad(ZTKDouble(ztk)) );
  _rkJointHookeSetDis1( prp, 1, zDeg2Rad(ZTKDouble(ztk)) );
  return prp;
}
static void *_rkJointHookeMinFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->min[0] = zDeg2Rad(ZTKDouble(ztk));
  _rkc(prp)->min[1] = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointHookeMaxFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->max[0] = zDeg2Rad(ZTKDouble(ztk));
  _rkc(prp)->max[1] = zDeg2Rad(ZTKDouble(ztk));
  return prp;
}
static void *_rkJointHookeStiffnessFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->stiffness[0] = ZTKDouble(ztk);
  _rkc(prp)->stiffness[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointHookeViscosityFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->viscosity[0] = ZTKDouble(ztk);
  _rkc(prp)->viscosity[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointHookeCoulombFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->coulomb[0] = ZTKDouble(ztk);
  _rkc(prp)->coulomb[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointHookeStaticFrictionFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->sf[0] = ZTKDouble(ztk);
  _rkc(prp)->sf[1] = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointHookeMotorFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( (rkMotorArray *)arg, ZTKVal(ztk) ) ) ) return NULL;
  return rkMotorClone( mp, &_rkc(prp)->m ) ? prp : NULL;
}

static void _rkJointHookeDisFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", zRad2Deg(_rkc(prp)->dis[0]), zRad2Deg(_rkc(prp)->dis[1]) );
}
static void _rkJointHookeMinFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", zRad2Deg(_rkc(prp)->min[0]), zRad2Deg(_rkc(prp)->min[1]) );
}
static void _rkJointHookeMaxFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", zRad2Deg(_rkc(prp)->max[0]), zRad2Deg(_rkc(prp)->max[1]) );
}
static void _rkJointHookeStiffnessFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", _rkc(prp)->stiffness[0], _rkc(prp)->stiffness[1] );
}
static void _rkJointHookeViscosityFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", _rkc(prp)->viscosity[0], _rkc(prp)->viscosity[1] );
}
static void _rkJointHookeCoulombFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", _rkc(prp)->coulomb[0], _rkc(prp)->coulomb[1] );
}
static void _rkJointHookeStaticFrictionFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g %.10g\n", _rkc(prp)->sf[0], _rkc(prp)->sf[1] );
}

static ZTKPrp __ztk_prp_rkjoint_hooke[] = {
  { "dis", 1, _rkJointHookeDisFromZTK, _rkJointHookeDisFPrintZTK },
  { "min", 1, _rkJointHookeMinFromZTK, _rkJointHookeMinFPrintZTK },
  { "max", 1, _rkJointHookeMaxFromZTK, _rkJointHookeMaxFPrintZTK },
  { "stiffness", 1, _rkJointHookeStiffnessFromZTK, _rkJointHookeStiffnessFPrintZTK },
  { "viscosity", 1, _rkJointHookeViscosityFromZTK, _rkJointHookeViscosityFPrintZTK },
  { "coulomb", 1, _rkJointHookeCoulombFromZTK, _rkJointHookeCoulombFPrintZTK },
  { "staticfriction", 1, _rkJointHookeStaticFrictionFromZTK, _rkJointHookeStaticFrictionFPrintZTK },
  { "motor", 1, _rkJointHookeMotorFromZTK, NULL },
};

static void *_rkJointHookeFromZTK(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_hooke );
}

static void _rkJointHookeFPrintZTK(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_hooke );
  if( rkMotorIsAssigned( &_rkc(prp)->m ) )
    fprintf( fp, "motor: %s\n", zName(&_rkc(prp)->m) );
}

rkJointCom rk_joint_hooke = {
  "hooke",
  2,
  _rkJointHookeInit,
  _rkJointHookeAlloc,
  _rkJointHookeLimDis,
  _rkJointHookeSetDis,
  _rkJointHookeSetMin,
  _rkJointHookeSetMax,
  _rkJointHookeSetVel,
  _rkJointHookeSetAcc,
  _rkJointHookeSetTrq,
  _rkJointHookeGetDis,
  _rkJointHookeGetMin,
  _rkJointHookeGetMax,
  _rkJointHookeGetVel,
  _rkJointHookeGetAcc,
  _rkJointHookeGetTrq,
  _rkJointHookeCatDis,
  _rkJointHookeSubDis,
  _rkJointHookeSetDisCNT,
  _rkJointHookeXform,
  _rkJointHookeIncVel,
  _rkJointHookeIncAccOnVel,
  _rkJointHookeIncAcc,
  _rkJointHookeCalcTrq,
  _rkJointHookeTorsion,
  _rk_joint_hooke_axis_ang,
  _rk_joint_hooke_axis_lin,

  _rkJointHookeCRBWrench,
  _rkJointHookeCRBXform,

  _rkJointHookeSetFrictionPivot,
  _rkJointHookeGetFrictionPivot,
  _rkJointHookeSetFriction,
  _rkJointHookeGetFriction,
  _rkJointHookeGetSFriction,
  _rkJointHookeGetKFriction,

  _rkJointHookeGetMotor,
  _rkJointHookeMotorSetInput,
  _rkJointHookeMotorInertia,
  _rkJointHookeMotorInputTrq,
  _rkJointHookeMotorResistance,
  _rkJointHookeMotorDrivingTrq,

  _rkJointHookeABIAxisInertia,
  _rkJointHookeABIAddABI,
  _rkJointHookeABIAddBias,
  _rkJointHookeABIDrivingTorque,
  _rkJointHookeABIQAcc,
  _rkJointUpdateWrench,

  _rkJointHookeDisFromZTK,
  _rkJointHookeFromZTK,
  _rkJointHookeDisFPrintZTK,
  _rkJointHookeFPrintZTK,
};

#undef _rkc
