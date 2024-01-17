/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_hooke - joint structure: universal joint
 */

#include <roki/rk_joint.h>

#define _rkc(joint) ((rkJointHookePrp *)((rkJoint *)(joint))->prp)

static void _rkJointHookeInit(rkJoint *joint){
  _rkc(joint)->max[0] = HUGE_VAL;
  _rkc(joint)->min[0] =-HUGE_VAL;
  _rkc(joint)->max[1] = HUGE_VAL;
  _rkc(joint)->min[1] =-HUGE_VAL;
  rkMotorAssign( &_rkc(joint)->m, &rk_motor_none );
}

static void *_rkJointHookeAllocPrp(void){ return zAlloc( rkJointHookePrp, 1 ); }

static void _rkJointHookeCopyPrp(rkJoint *src, rkJoint *dst){
  _rkc(dst)->min[0] = _rkc(src)->min[0];
  _rkc(dst)->min[1] = _rkc(src)->min[1];
  _rkc(dst)->max[0] = _rkc(src)->max[0];
  _rkc(dst)->max[1] = _rkc(src)->max[1];
  _rkc(dst)->stiffness[0] = _rkc(src)->stiffness[0];
  _rkc(dst)->stiffness[1] = _rkc(src)->stiffness[1];
  _rkc(dst)->viscosity[0] = _rkc(src)->viscosity[0];
  _rkc(dst)->viscosity[1] = _rkc(src)->viscosity[1];
  _rkc(dst)->coulomb[0] = _rkc(src)->coulomb[0];
  _rkc(dst)->coulomb[1] = _rkc(src)->coulomb[1];
  _rkc(dst)->sf[0] = _rkc(src)->sf[0];
  _rkc(dst)->sf[1] = _rkc(src)->sf[1];
}

/* limit joint displacement */
static double _rkJointHookeLimDis1(rkJoint *joint, int i, double testval){
  testval = zPhaseNormalize( testval );
  return zLimit( testval, _rkc(joint)->min[i], _rkc(joint)->max[i] );
}

static void _rkJointHookeLimDis(rkJoint *joint, double *testval, double *limval){
  limval[0] = _rkJointHookeLimDis1( joint, 0, testval[0] );
  limval[1] = _rkJointHookeLimDis1( joint, 1, testval[1] );
}

/* set joint displacement */
static void _rkJointHookeSetDis1(rkJoint *joint, int i, double val){
  _rkc(joint)->dis[i] = _rkJointHookeLimDis1( joint, i, val );
  zSinCos( _rkc(joint)->dis[i], &_rkc(joint)->_s[i], &_rkc(joint)->_c[i] );
}

static void _rkJointHookeSetDis(rkJoint *joint, double *val){
  _rkJointHookeSetDis1( joint, 0, val[0] );
  _rkJointHookeSetDis1( joint, 1, val[1] );
}

static void _rkJointHookeSetMin(rkJoint *joint, double *val){
  memcpy( _rkc(joint)->min, val, sizeof(double)*2 );
}

static void _rkJointHookeSetMax(rkJoint *joint, double *val){
  memcpy( _rkc(joint)->max, val, sizeof(double)*2 );
}

static void _rkJointHookeSetVel(rkJoint *joint, double *val){
  memcpy( _rkc(joint)->vel, val, sizeof(double)*2 );
}

static void _rkJointHookeSetAcc(rkJoint *joint, double *val){
  memcpy( _rkc(joint)->acc, val, sizeof(double)*2 );
}

static void _rkJointHookeSetTrq(rkJoint *joint, double *val){
  memcpy( _rkc(joint)->trq, val, sizeof(double)*2 );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointHookeGetDis(rkJoint *joint, double *val){
  memcpy( val, _rkc(joint)->dis, sizeof(double)*2 );
}

static void _rkJointHookeGetMin(rkJoint *joint, double *val){
  memcpy( val, _rkc(joint)->min, sizeof(double)*2 );
}

static void _rkJointHookeGetMax(rkJoint *joint, double *val){
  memcpy( val, _rkc(joint)->max, sizeof(double)*2 );
}

static void _rkJointHookeGetVel(rkJoint *joint, double *val){
  memcpy( val, _rkc(joint)->vel, sizeof(double)*2 );
}

static void _rkJointHookeGetAcc(rkJoint *joint, double *val){
  memcpy( val, _rkc(joint)->acc, sizeof(double)*2 );
}

static void _rkJointHookeGetTrq(rkJoint *joint, double *val){
  memcpy( val, _rkc(joint)->trq, sizeof(double)*2 );
}

static void _rkJointHookeCatDis(rkJoint *joint, double *dis, double k, double *val){
  dis[0] += val[0] * k;
  dis[1] += val[1] * k;
}

static void _rkJointHookeSubDis(rkJoint *joint, double *dis, double *sdis){
  dis[0] -= sdis[0];
  dis[1] -= sdis[1];
}

/* continuously update joint displacement */
static void _rkJointHookeSetDisCNT(rkJoint *joint, double *val, double dt){
  double olddis[2], oldvel[2];
  _rkJointHookeGetDis( joint, olddis );
  _rkJointHookeGetVel( joint, oldvel );
  _rkJointHookeSetDis( joint, val );
  _rkc(joint)->vel[0] = ( val[0] - olddis[0] ) / dt;
  _rkc(joint)->vel[1] = ( val[1] - olddis[1] ) / dt;
  _rkc(joint)->acc[0] = ( _rkc(joint)->vel[0] - oldvel[0] ) / dt;
  _rkc(joint)->acc[1] = ( _rkc(joint)->vel[1] - oldvel[1] ) / dt;
}

/* joint frame transformation */
static zFrame3D *_rkJointHookeXform(rkJoint *joint, zFrame3D *fo, zFrame3D *f){
  zMat3D m;
  /* position */
  zVec3DCopy( zFrame3DPos(fo), zFrame3DPos(f) );
  /* joint displacements correspond to the rotation angle about
   * z-axis and y-axis, respectively */
  zMat3DFromZYXSC( &m, _rkc(joint)->_s[0], _rkc(joint)->_c[0], _rkc(joint)->_s[1], _rkc(joint)->_c[1], 0, 1 );
  zMulMat3DMat3D( zFrame3DAtt(fo), &m, zFrame3DAtt(f) );
  return f;
}

/* joint motion rate transformation */
static void _rkJointHookeIncVel(rkJoint *joint, zVec6D *vel){
  zVec3D v;
  zVec3DCreate( &v, -_rkc(joint)->_s[1]*_rkc(joint)->vel[0], _rkc(joint)->vel[1], _rkc(joint)->_c[1]*_rkc(joint)->vel[0] );
  zVec3DAddDRC( zVec6DAng(vel), &v );
}

static void _rkJointHookeIncAccOnVel(rkJoint *joint, zVec3D *w, zVec6D *acc){
  zVec3D v;
  double dq2;
  zVec3DCreate( &v, -_rkc(joint)->_s[1]*_rkc(joint)->vel[0], _rkc(joint)->vel[1], _rkc(joint)->_c[1]*_rkc(joint)->vel[0] );
  zVec3DOuterProd( w, &v, &v );
  zVec3DAddDRC( zVec6DAng(acc), &v );
  dq2 = _rkc(joint)->vel[0] * _rkc(joint)->vel[1];
  zVec3DCreate( &v, -_rkc(joint)->_c[1]*dq2, 0, -_rkc(joint)->_s[1]*dq2 );
  zVec3DAddDRC( zVec6DAng(acc), &v );
}

static void _rkJointHookeIncAcc(rkJoint *joint, zVec6D *acc){
  zVec3D v1;
  zVec3DCreate( &v1, -_rkc(joint)->_s[1]*_rkc(joint)->acc[0], _rkc(joint)->acc[1], _rkc(joint)->_c[1]*_rkc(joint)->acc[0] );
  zVec3DAddDRC( zVec6DAng(acc), &v1 );
}

/* joint torque transformation */
static void _rkJointHookeCalcTrq(rkJoint *joint, zVec6D *f){
  _rkc(joint)->trq[0] =-_rkc(joint)->_s[1]*f->e[zXA]+_rkc(joint)->_c[1]*f->e[zZA];
  _rkc(joint)->trq[1] = f->e[zYA];
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

static zVec3D *_rkJointHookeAngAxis1(rkJoint *joint, zFrame3D *f, zVec3D *a){
  zVec3DMul( &zFrame3DAtt(f)->v[zX],-_rkc(joint)->_s[1], a );
  return zVec3DCatDRC( a, _rkc(joint)->_c[1], &zFrame3DAtt(f)->v[2] );
}

static zVec3D *_rkJointHookeAngAxis2(rkJoint *joint, zFrame3D *f, zVec3D *a){
  zVec3DCopy( &zFrame3DAtt(f)->v[1], a );
  return a;
}

static zVec3D* (*_rk_joint_hooke_axis_ang[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointHookeAngAxis1,
  _rkJointHookeAngAxis2,
};
static zVec3D* (*_rk_joint_hooke_axis_lin[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisNull,
};

/* CRB method */

static void _rkJointHookeCRBWrench(rkJoint *joint, rkMP *crb, zVec6D wi[]){
  zVec6D wx, wz;

  _rkJointCRBWrenchAngX( crb, &wx );
  _rkJointCRBWrenchAngZ( crb, &wz );
  _zVec6DMul( &wx, -_rkc(joint)->_s[1], &wi[0] );
  _zVec6DCatDRC( &wi[0], _rkc(joint)->_c[1], &wz );

  _rkJointCRBWrenchAngY( crb, &wi[1] );
}
static void _rkJointHookeCRBXform(rkJoint *joint, zFrame3D *f, zVec6D si[]){
  zVec6D sx, sz;

  _rkJointCRBXformAng( f, zX, &sx );
  _rkJointCRBXformAng( f, zZ, &sz );
  _zVec6DMul( &sx, -_rkc(joint)->_s[1], &si[0] );
  _zVec6DCatDRC( &si[0], _rkc(joint)->_c[1], &sz );

  _rkJointCRBXformAng( f, zY, &si[1] );
}


static void _rkJointHookeSetFrictionPivot(rkJoint *joint, rkJointFrictionPivot *fp){
  fp[0] = _rkc(joint)->_fp[0];
  fp[1] = _rkc(joint)->_fp[1];
}

static void _rkJointHookeGetFrictionPivot(rkJoint *joint, rkJointFrictionPivot *fp){
  _rkc(joint)->_fp[0] = fp[0];
  _rkc(joint)->_fp[1] = fp[1];
}

static void _rkJointHookeSetFriction(rkJoint *joint, double *val){
  _rkc(joint)->tf[0] = val[0];
  _rkc(joint)->tf[1] = val[1];
}

static void _rkJointHookeGetFriction(rkJoint *joint, double *val){
  val[0] = _rkc(joint)->tf[0];
  val[1] = _rkc(joint)->tf[1];
}

static void _rkJointHookeGetSFriction(rkJoint *joint, double *val){
  val[0] = _rkc(joint)->sf[0];
  val[1] = _rkc(joint)->sf[1];
}

static void _rkJointHookeGetKFriction(rkJoint *joint, double *val){
  val[0] = _rkJointRestTrq( _rkc(joint)->stiffness[0], _rkc(joint)->viscosity[0], _rkc(joint)->coulomb[0], _rkc(joint)->dis[0], _rkc(joint)->vel[0] );
  val[1] = _rkJointRestTrq( _rkc(joint)->stiffness[1], _rkc(joint)->viscosity[1], _rkc(joint)->coulomb[1], _rkc(joint)->dis[1], _rkc(joint)->vel[1] );
}

/* motor */

static rkMotor *_rkJointHookeGetMotor(rkJoint *joint){ return &_rkc(joint)->m; }

static void _rkJointHookeMotorSetInput(rkJoint *joint, double *val){
  rkMotorSetInput( &_rkc(joint)->m, val );
}

static void _rkJointHookeMotorInertia(rkJoint *joint, double *val){
  zRawVecZero( val, 4 );
  rkMotorInertia( &_rkc(joint)->m, val );
}

static void _rkJointHookeMotorInputTrq(rkJoint *joint, double *val){
  zRawVecZero( val, 2 );
  rkMotorInputTrq( &_rkc(joint)->m, val );
}

static void _rkJointHookeMotorResistance(rkJoint *joint, double *val){
  zRawVecZero( val, 2 );
  rkMotorRegistance( &_rkc(joint)->m, _rkc(joint)->dis, _rkc(joint)->vel, val );
}

static void _rkJointHookeMotorDrivingTrq(rkJoint *joint, double *val){
  zRawVecZero( val, 2 );
  rkMotorDrivingTrq( &_rkc(joint)->m, _rkc(joint)->dis, _rkc(joint)->vel, _rkc(joint)->acc, val );
}

/* ABI */

static void _rkJointHookeABIAxisInertia(rkJoint *joint, zMat6D *m, zMat h, zMat ih){
  zMat3D *m22;

  _rkJointHookeMotorInertia( joint, zMatBufNC(h) );
  m22 = &m->e[1][1];
  zMatElemNC(h,0,0) += m22->e[0][0]*_rkc(joint)->_s[1]*_rkc(joint)->_s[1] - (m22->e[2][0] + m22->e[0][2])*_rkc(joint)->_s[1]*_rkc(joint)->_c[1] + m22->e[2][2]*_rkc(joint)->_c[1]*_rkc(joint)->_c[1];
  zMatElemNC(h,1,0) += m22->e[2][1]*_rkc(joint)->_c[1] - m22->e[0][1]*_rkc(joint)->_s[1];
  zMatElemNC(h,0,1) += m22->e[1][2]*_rkc(joint)->_c[1] - m22->e[1][0]*_rkc(joint)->_s[1];
  zMatElemNC(h,1,1) += m22->e[1][1];
  zMatInv( h, ih );
}

static void _rkJointHookeABIAddABI(rkJoint *joint, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  eprintf("under construction error: abi update for hooke joint\n");
}

static void _rkJointHookeABIAddBias(rkJoint *joint, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  eprintf("under construction error: abi update for hooke joint\n");
}

static void _rkJointHookeABIDrivingTorque(rkJoint *joint){
  eprintf("under construction error: abi update for hooke joint\n");
}

static void _rkJointHookeABIQAcc(rkJoint *joint, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){}

/* ZTK */

static void *_rkJointHookeDisFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkJointHookeSetDis1( joint, 0, zDeg2Rad(ZTKDouble(ztk)) );
  _rkJointHookeSetDis1( joint, 1, zDeg2Rad(ZTKDouble(ztk)) );
  return joint;
}
static void *_rkJointHookeMinFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->min[0] = zDeg2Rad(ZTKDouble(ztk));
  _rkc(joint)->min[1] = zDeg2Rad(ZTKDouble(ztk));
  return joint;
}
static void *_rkJointHookeMaxFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->max[0] = zDeg2Rad(ZTKDouble(ztk));
  _rkc(joint)->max[1] = zDeg2Rad(ZTKDouble(ztk));
  return joint;
}
static void *_rkJointHookeStiffnessFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->stiffness[0] = ZTKDouble(ztk);
  _rkc(joint)->stiffness[1] = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointHookeViscosityFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->viscosity[0] = ZTKDouble(ztk);
  _rkc(joint)->viscosity[1] = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointHookeCoulombFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->coulomb[0] = ZTKDouble(ztk);
  _rkc(joint)->coulomb[1] = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointHookeStaticFrictionFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->sf[0] = ZTKDouble(ztk);
  _rkc(joint)->sf[1] = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointHookeMotorFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( (rkMotorArray *)arg, ZTKVal(ztk) ) ) ) return NULL;
  return rkMotorClone( mp, &_rkc(joint)->m ) ? joint : NULL;
}

static void _rkJointHookeDisFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g %.10g\n", zRad2Deg(_rkc(joint)->dis[0]), zRad2Deg(_rkc(joint)->dis[1]) );
}
static void _rkJointHookeMinFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g %.10g\n", zRad2Deg(_rkc(joint)->min[0]), zRad2Deg(_rkc(joint)->min[1]) );
}
static void _rkJointHookeMaxFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g %.10g\n", zRad2Deg(_rkc(joint)->max[0]), zRad2Deg(_rkc(joint)->max[1]) );
}
static void _rkJointHookeStiffnessFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g %.10g\n", _rkc(joint)->stiffness[0], _rkc(joint)->stiffness[1] );
}
static void _rkJointHookeViscosityFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g %.10g\n", _rkc(joint)->viscosity[0], _rkc(joint)->viscosity[1] );
}
static void _rkJointHookeCoulombFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g %.10g\n", _rkc(joint)->coulomb[0], _rkc(joint)->coulomb[1] );
}
static void _rkJointHookeStaticFrictionFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g %.10g\n", _rkc(joint)->sf[0], _rkc(joint)->sf[1] );
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

static rkJoint *_rkJointHookeFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorarray, ztk, __ztk_prp_rkjoint_hooke );
}

static void _rkJointHookeFPrintZTK(FILE *fp, rkJoint *joint, char *name)
{
  ZTKPrpKeyFPrint( fp, joint, __ztk_prp_rkjoint_hooke );
  if( rkMotorIsAssigned( &_rkc(joint)->m ) )
    fprintf( fp, "motor: %s\n", zName(&_rkc(joint)->m) );
}

rkJointCom rk_joint_hooke = {
  "hooke",
  2,
  _rkJointHookeInit,
  _rkJointHookeAllocPrp,
  _rkJointHookeCopyPrp,
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
