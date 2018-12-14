/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_cylin - joint structure: cylindrical joint
 */

#include <roki/rk_joint.h>

static void _rkJointLimDisCylin(void *prp, double *testval, double *limval);
static void _rkJointSetDisCylin(void *prp, double *val);
static void _rkJointSetVelCylin(void *prp, double *val);
static void _rkJointSetAccCylin(void *prp, double *val);
static void _rkJointSetTrqCylin(void *prp, double *val);
static void _rkJointGetDisCylin(void *prp, double *val);
static void _rkJointGetVelCylin(void *prp, double *val);
static void _rkJointGetAccCylin(void *prp, double *val);
static void _rkJointGetTrqCylin(void *prp, double *val);
static void _rkJointGetMotorCylin(void *prp, rkMotor **m);
static void _rkJointCatDisCylin(void *prp, double *dis, double k, double *val);
static void _rkJointSubDisCylin(void *prp, double *dis, double *sdis);
static void _rkJointSetDisCNTCylin(void *prp, double *val, double dt);

static zFrame3D *_rkJointXferCylin(void *prp, zFrame3D *fo, zFrame3D *f);
static void _rkJointIncVelCylin(void *prp, zVec6D *vel);
static void _rkJointIncAccOnVelCylin(void *prp, zVec3D *w, zVec6D *acc);
static void _rkJointIncAccCylin(void *prp, zVec6D *acc);

static void _rkJointCalcTrqCylin(void *prp, zVec6D *f);
static void _rkJointTorsionCylin(zFrame3D *dev, zVec6D *t, double dis[]);

static void _rkJointSetFricCylin(void *prp, double *val);
static void _rkJointGetFricCylin(void *prp, double *val);
static void _rkJointGetSFricCylin(void *prp, double *val);
static void _rkJointGetKFricCylin(void *prp, double *val);

static void _rkJointSetRefCylin(void *prp, rkJointRef *ref);
static void _rkJointGetRefCylin(void *prp, rkJointRef *ref);

static bool _rkJointQueryFReadCylin(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm);
static void _rkJointFWriteCylin(FILE *fp, void *prp, char *name);

#define _rkc(p) ((rkJointPrpCylin *)p)

/* limit joint displacement */
void _rkJointLimDisCylin(void *prp, double *testval, double *limval){
  double angle;

  /* 0: prismatic */
  limval[0] = zLimit( testval[0], _rkc(prp)->min[0], _rkc(prp)->max[0] );
  /* 1: revolutional */
  angle = zPhaseNormalize( testval[1] );
  limval[1] = zLimit( angle, _rkc(prp)->min[1], _rkc(prp)->max[1] );
}

/* joint displacement set function */
void _rkJointSetDisCylin(void *prp, double *val){
  _rkJointLimDisCylin( prp, val, _rkc(prp)->dis );
  zSinCos( _rkc(prp)->dis[1], &_rkc(prp)->_s, &_rkc(prp)->_c );
}

void _rkJointSetVelCylin(void *prp, double *val){
  memcpy( _rkc(prp)->vel, val, sizeof(double)*2 );
}

void _rkJointSetAccCylin(void *prp, double *val){
  memcpy( _rkc(prp)->acc, val, sizeof(double)*2 );
}

void _rkJointSetTrqCylin(void *prp, double *val){
  memcpy( _rkc(prp)->trq, val, sizeof(double)*2 );
}

/* get joint displacement, velocity, acceleration and torque */
void _rkJointGetDisCylin(void *prp, double *val){
  memcpy( val, _rkc(prp)->dis, sizeof(double)*2 );
}

void _rkJointGetVelCylin(void *prp, double *val){
  memcpy( val, _rkc(prp)->vel, sizeof(double)*2 );
}

void _rkJointGetAccCylin(void *prp, double *val){
  memcpy( val, _rkc(prp)->acc, sizeof(double)*2 );
}

void _rkJointGetTrqCylin(void *prp, double *val){
  memcpy( val, _rkc(prp)->trq, sizeof(double)*2 );
}

/* motor */
void _rkJointGetMotorCylin(void *prp, rkMotor **m){
  *m = &_rkc(prp)->m;
}

void _rkJointCatDisCylin(void *prp, double *dis, double k, double *val){
  dis[0] += val[0] * k;
  dis[1] += val[1] * k;
}

void _rkJointSubDisCylin(void *prp, double *dis, double *sdis){
  dis[0] -= sdis[0];
  dis[1] -= sdis[1];
}

/* continuously update joint displacement */
void _rkJointSetDisCNTCylin(void *prp, double *val, double dt)
{
  double olddis[2], oldvel[2];

  _rkJointGetDisCylin( prp, olddis );
  _rkJointGetVelCylin( prp, oldvel );
  _rkJointSetDisCylin( prp, val );
  _rkc(prp)->vel[0] = ( val[0] - olddis[0] ) / dt;
  _rkc(prp)->vel[1] = ( val[1] - olddis[1] ) / dt;
  _rkc(prp)->acc[0] = ( _rkc(prp)->vel[0] - oldvel[0] ) / dt;
  _rkc(prp)->acc[1] = ( _rkc(prp)->vel[1] - oldvel[1] ) / dt;
}

/* joint frame transfer function */
zFrame3D *_rkJointXferCylin(void *prp, zFrame3D *fo, zFrame3D *f)
{
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

/* joint velocity transfer function */
void _rkJointIncVelCylin(void *prp, zVec6D *vel)
{
  vel->e[zZ ] += _rkc(prp)->vel[0];
  vel->e[zZA] += _rkc(prp)->vel[1];
}

void _rkJointIncAccOnVelCylin(void *prp, zVec3D *w, zVec6D *acc)
{
  acc->e[zX ] += 2 * _rkc(prp)->vel[0] * w->e[zY];
  acc->e[zY ] -= 2 * _rkc(prp)->vel[0] * w->e[zX];
  acc->e[zXA] += _rkc(prp)->vel[1] * w->e[zY];
  acc->e[zYA] -= _rkc(prp)->vel[1] * w->e[zX];
}

/* joint acceleration transfer function */
void _rkJointIncAccCylin(void *prp, zVec6D *acc)
{
  acc->e[zZ ] += _rkc(prp)->acc[0];
  acc->e[zZA] += _rkc(prp)->acc[1];
}

/* joint torque transfer function */
void _rkJointCalcTrqCylin(void *prp, zVec6D *f)
{
  _rkc(prp)->trq[0] = f->e[zZ];
  _rkc(prp)->trq[1] = f->e[zZA];
}

/* inverse computation of joint torsion and displacement */
void _rkJointTorsionCylin(zFrame3D *dev, zVec6D *t, double dis[])
{
  dis[0] = rkJointTorsionDisPrism( dev, t );
  dis[1] = rkJointTorsionDisRevol( dev, t );
}

void _rkJointSetFricCylin(void *prp, double *val)
{
  _rkc(prp)->tf[0] = val[0];
  _rkc(prp)->tf[1] = val[1];
}
void _rkJointGetFricCylin(void *prp, double *val){
  val[0] = _rkc(prp)->tf[0];
  val[1] = _rkc(prp)->tf[1];
}

void _rkJointGetSFricCylin(void *prp, double *val)
{
  val[0] = _rkc(prp)->sf[0];
  val[1] = _rkc(prp)->sf[1];
}

void _rkJointGetKFricCylin(void *prp, double *val)
{
  val[0] = _rkJointRestTrq( _rkc(prp)->stiff[0], _rkc(prp)->viscos[0], _rkc(prp)->coulomb[0], _rkc(prp)->dis[0], _rkc(prp)->vel[0] );
  val[1] = _rkJointRestTrq( _rkc(prp)->stiff[1], _rkc(prp)->viscos[1], _rkc(prp)->coulomb[1], _rkc(prp)->dis[1], _rkc(prp)->vel[1] );
}

void _rkJointSetRefCylin(void *prp, rkJointRef *ref){
  ref[0] = _rkc(prp)->_ref[0];
  ref[1] = _rkc(prp)->_ref[1];
}
void _rkJointGetRefCylin(void *prp, rkJointRef *ref){
  _rkc(prp)->_ref[0] = ref[0];
  _rkc(prp)->_ref[1] = ref[1];
}

/* query joint properties */
bool _rkJointQueryFReadCylin(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm)
{
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
  if( strcmp( buf, "stiff" ) == 0 ){
    _rkc(prp)->stiff[0] = zFDouble(fp);
    _rkc(prp)->stiff[1] = zFDouble(fp);
  } else
  if( strcmp( buf, "viscos" ) == 0 ){
    _rkc(prp)->viscos[0] = zFDouble(fp);
    _rkc(prp)->viscos[1] = zFDouble(fp);
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
      ZRUNERROR( "invalid motor name %s detected", buf );
      return true;
    }
    if( rkMotorSize(mp) != 2 ){
      ZRUNERROR( "unmatched motor size" );
      return true;
    }
    rkMotorClone( mp, &_rkc(prp)->m );
  } else
  if( !rkMotorQueryFRead( fp, buf, &_rkc(prp)->m ) )
    return false;
  return true;
}

void _rkJointFWriteCylin(FILE *fp, void *prp, char *name)
{
  rkJointPrpCylin *v;

  v = prp;
  if( !zIsTiny( v->dis[0] ) || !zIsTiny( v->dis[1] ) )
    fprintf( fp, "%s: %.10f %.10f\n", name, v->dis[0], zRad2Deg(v->dis[1]) );
  fprintf( fp, "min: %.10f %.10f\n", v->min[0], zRad2Deg(v->min[1]) );
  fprintf( fp, "max: %.10f %.10f\n", v->max[0], zRad2Deg(v->max[1]) );
  fprintf( fp, "stiff: %.10f %.10f\n", v->stiff[0], zDeg2Rad(v->stiff[1]) );
  fprintf( fp, "viscos: %.10f %.10f\n", v->viscos[0], zDeg2Rad(v->viscos[1]) );
  fprintf( fp, "coulomb: %.10f %.10f\n", v->coulomb[0], zDeg2Rad(v->coulomb[1]) );
}

static zVec3D* (*_rk_joint_axis_cylin_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisZ,
};
static zVec3D* (*_rk_joint_axis_cylin_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisZ,
  _rkJointAxisNull,
};
static rkJointCom rk_joint_cylin = {
  2,
  _rkJointLimDisCylin,
  _rkJointSetDisCylin,
  _rkJointSetVelCylin,
  _rkJointSetAccCylin,
  _rkJointSetTrqCylin,
  _rkJointGetDisCylin,
  _rkJointGetVelCylin,
  _rkJointGetAccCylin,
  _rkJointGetTrqCylin,
  _rkJointGetMotorCylin,
  _rkJointCatDisCylin,
  _rkJointSubDisCylin,
  _rkJointSetDisCNTCylin,
  _rkJointXferCylin,
  _rkJointIncVelCylin,
  _rkJointIncAccOnVelCylin,
  _rkJointIncAccCylin,
  _rkJointCalcTrqCylin,
  _rkJointTorsionCylin,
  _rkJointSetFricCylin,
  _rkJointGetFricCylin,
  _rkJointGetSFricCylin,
  _rkJointGetKFricCylin,
  _rkJointSetRefCylin,
  _rkJointGetRefCylin,
  _rk_joint_axis_cylin_ang,
  _rk_joint_axis_cylin_lin,
  _rkJointQueryFReadCylin,
  _rkJointFWriteCylin,
};

/* motor */
static byte _rkJointMotorCylin(void *prp);
static void _rkJointMotorSetInputCylin(void *prp, double *val);
static void _rkJointMotorInertiaCylin(void *prp, double *val);
static void _rkJointMotorInputTrqCylin(void *prp, double *val);
static void _rkJointMotorResistanceCylin(void *prp, double *val);
static void _rkJointMotorDrivingTrqCylin(void *prp, double *val);

byte _rkJointMotorCylin(void *prp){
  return rkMotorType( &_rkc(prp)->m );
}
void _rkJointMotorSetInputCylin(void *prp, double *val){
  rkMotorSetInput( &_rkc(prp)->m, val );
}
void _rkJointMotorInertiaCylin(void *prp, double *val){
  zRawVecClear( val, 4 );
  rkMotorInertia( &_rkc(prp)->m, val );
}
void _rkJointMotorInputTrqCylin(void *prp, double *val){
  zRawVecClear( val, 2 );
  rkMotorInputTrq( &_rkc(prp)->m, val );
}
void _rkJointMotorResistanceCylin(void *prp, double *val){
  zRawVecClear( val, 2 );
  rkMotorRegistance( &_rkc(prp)->m, _rkc(prp)->dis, _rkc(prp)->vel, val );
}
void _rkJointMotorDrivingTrqCylin(void *prp, double *val){
  zRawVecClear( val, 2 );
  rkMotorDrivingTrq( &_rkc(prp)->m, _rkc(prp)->dis, _rkc(prp)->vel, _rkc(prp)->acc, val );
}

static rkJointMotorCom rk_joint_motor_cylin = {
  _rkJointMotorCylin,
  _rkJointMotorSetInputCylin,
  _rkJointMotorInertiaCylin,
  _rkJointMotorInputTrqCylin,
  _rkJointMotorResistanceCylin,
  _rkJointMotorDrivingTrqCylin,
};

/* ABI */
static void _rkJointABIAxisInertiaCylin(void *prp, zMat6D *m, zMat h, zMat ih);
static void _rkJointABIAddAbiCylin(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm);
static void _rkJointABIAddBiasCylin(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb);
static void _rkJointABIDrivingTorqueCylin(void *prp);
static void _rkJointABIQAccCylin(void *prp, zMat3D *R, zMat6D *I, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc);

void _rkJointABIAxisInertiaCylin(void *prp, zMat6D *m, zMat h, zMat ih)
{
  _rkJointMotorInertiaCylin( prp, zMatBuf(h) );
  zMatElem(h,0,0) += zMat6DMat3D(m,0,0)->e[2][2];
  zMatElem(h,1,0) += zMat6DMat3D(m,1,0)->e[2][2];
  zMatElem(h,0,1) += zMat6DMat3D(m,0,1)->e[2][2];
  zMatElem(h,1,1) += zMat6DMat3D(m,1,1)->e[2][2];
  zMatInv( h, ih );
}

void _rkJointABIAddAbiCylin(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm)
{
  zVec6D v13, v31, v16, v61, tmpv;
  zMat6D tmpm, tmpm2;

  zMat6DCol( m, zZ,  &v13 );
  zMat6DCol( m, zZA, &v16 );
  zMat6DRow( m, zZ,  &v31 );
  zMat6DRow( m, zZA, &v61 );

  zVec6DMul( &v13, zMatElem(h,0,0), &tmpv );
  zMat6DDyad( &tmpm, &tmpv, &v31 );
  zVec6DMul( &v13, zMatElem(h,0,1), &tmpv );
  zMat6DDyad( &tmpm2, &tmpv, &v61 );
  zMat6DAddDRC( &tmpm, &tmpm2 );
  zVec6DMul( &v16, zMatElem(h,1,0), &tmpv );
  zMat6DDyad( &tmpm2, &tmpv, &v31 );
  zMat6DAddDRC( &tmpm, &tmpm2 );
  zVec6DMul( &v16, zMatElem(h,1,1), &tmpv );
  zMat6DDyad( &tmpm2, &tmpv, &v61 );
  zMat6DAddDRC( &tmpm, &tmpm2 );

  rkJointXferMat6D( f, &tmpm, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}

void _rkJointABIAddBiasCylin(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb)
{
  zVec6D v13, v16, tmpv;
  zVec3D tmp3v;

  zMat6DCol( m, zZ,  &v13 );
  zMat6DCol( m, zZA, &v16 );
  zVec6DCat(b,        (_rkc(prp)->_u[0] - b->e[zZ])*zMatElem(h,0,0) + (_rkc(prp)->_u[1] - b->e[zZA])*zMatElem(h,0,1), &v13, &tmpv );
  zVec6DCatDRC(&tmpv, (_rkc(prp)->_u[0] - b->e[zZ])*zMatElem(h,1,0) + (_rkc(prp)->_u[1] - b->e[zZA])*zMatElem(h,1,1), &v16 );

  zMulMatVec6D( zFrame3DAtt( f ), &tmpv, &v13 );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec3DAddDRC( zVec6DAng( &v13 ), zVec3DOuterProd( zFrame3DPos( f ), zVec6DLin( &v13 ), &tmp3v ) );
  zVec6DAddDRC( pb, &v13 );
}

void _rkJointABIDrivingTorqueCylin(void *prp)
{
  double val[2];
  _rkJointMotorInputTrqCylin( prp, _rkc(prp)->_u );
  _rkJointMotorResistanceCylin( prp, val );
  _rkc(prp)->_u[0] -= val[0];
  _rkc(prp)->_u[1] -= val[1];
  _rkc(prp)->_u[0] += _rkc(prp)->tf[0];
  _rkc(prp)->_u[1] += _rkc(prp)->tf[1];
}

void _rkJointABIQAccCylin(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
{
  double u[2];
  zVec6D v31, v61;

  zMat6DRow( m, zZ, &v31 );
  zMat6DRow( m, zZA, &v61 );
  u[0] = _rkc(prp)->_u[0] - zVec6DInnerProd( &v31, jac ) + b->e[zZ];
  u[1] = _rkc(prp)->_u[1] - zVec6DInnerProd( &v61, jac ) + b->e[zZA];

  /* q */
  _rkc(prp)->acc[0] = u[0]*zMatElem(h,0,0) + u[1]*zMatElem(h,0,1);
  _rkc(prp)->acc[1] = u[0]*zMatElem(h,1,0) + u[1]*zMatElem(h,1,1);
  /* acc */
  zVec6DCopy( jac, acc );
  acc->e[zZ ] += _rkc(prp)->acc[0];
  acc->e[zZA] += _rkc(prp)->acc[1];
}

static rkJointABICom rk_joint_abi_cylin = {
  _rkJointABIAxisInertiaCylin,
  _rkJointABIAddAbiCylin,
  _rkJointABIAddBiasCylin,
  _rkJointABIDrivingTorqueCylin,
  _rkJointABIQAccCylin,
  _rkJointUpdateWrench,
};

/* rkJointCreateCylin
 * - create cylindrical joint instance.
 */
rkJoint *rkJointCreateCylin(rkJoint *j)
{
  if( !( j->prp = zAlloc( rkJointPrpCylin, 1 ) ) )
    return NULL;
  _rkc(j->prp)->max[0] = HUGE_VAL;
  _rkc(j->prp)->min[0] =-HUGE_VAL;
  _rkc(j->prp)->max[1] = zPI;
  _rkc(j->prp)->min[1] =-zPI;
  rkMotorCreate( &_rkc(j->prp)->m, RK_MOTOR_NONE );
  j->com = &rk_joint_cylin;
  j->mcom = &rk_joint_motor_cylin;
  j->acom = &rk_joint_abi_cylin;
  return j;
}

#undef _rkc
