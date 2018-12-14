/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_prism - joint structure: prismatic joint
 */

#include <roki/rk_joint.h>

static void _rkJointLimDisPrism(void *prp, double *testval, double *limval);
static void _rkJointSetDisPrism(void *prp, double *val);
static void _rkJointSetVelPrism(void *prp, double *val);
static void _rkJointSetAccPrism(void *prp, double *val);
static void _rkJointSetTrqPrism(void *prp, double *val);
static void _rkJointGetDisPrism(void *prp, double *val);
static void _rkJointGetVelPrism(void *prp, double *val);
static void _rkJointGetAccPrism(void *prp, double *val);
static void _rkJointGetTrqPrism(void *prp, double *val);
static void _rkJointGetMotorPrism(void *prp, rkMotor **m);
static void _rkJointCatDisPrism(void *prp, double *dis, double k, double *val);
static void _rkJointSubDisPrism(void *prp, double *dis, double *sdis);
static void _rkJointSetDisCNTPrism(void *prp, double *val, double dt);

static zFrame3D *_rkJointXferPrism(void *prp, zFrame3D *fo, zFrame3D *f);
static void _rkJointIncVelPrism(void *prp, zVec6D *vel);
static void _rkJointIncAccOnVelPrism(void *prp, zVec3D *w, zVec6D *acc);
static void _rkJointIncAccPrism(void *prp, zVec6D *acc);

static void _rkJointCalcTrqPrism(void *prp, zVec6D *f);
static void _rkJointTorsionPrism(zFrame3D *dev, zVec6D *t, double dis[]);

static void _rkJointSetFricPrism(void *prp, double *val);
static void _rkJointGetFricPrism(void *prp, double *val);
static void _rkJointGetSFricPrism(void *prp, double *val);
static void _rkJointGetKFricPrism(void *prp, double *val);

static void _rkJointSetRefPrism(void *prp, rkJointRef *ref);
static void _rkJointGetRefPrism(void *prp, rkJointRef *ref);

static bool _rkJointQueryFReadPrism(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm);
static void _rkJointFWritePrism(FILE *fp, void *prp, char *name);

#define _rkc(p) ((rkJointPrpPrism *)p)

/* limit joint displacement */
void _rkJointLimDisPrism(void *prp, double *testval, double *limval){
  *limval = zLimit( *testval, _rkc(prp)->min, _rkc(prp)->max );
}

/* joint displacement set function */
void _rkJointSetDisPrism(void *prp, double *val){
  _rkJointLimDisPrism( prp, val, &_rkc(prp)->dis );
}

void _rkJointSetVelPrism(void *prp, double *val){
  _rkc(prp)->vel = *val;
}

void _rkJointSetAccPrism(void *prp, double *val){
  _rkc(prp)->acc = *val;
}

void _rkJointSetTrqPrism(void *prp, double *val){
  _rkc(prp)->trq = *val;
}

/* get joint displacement, velocity, acceleration and torque */
void _rkJointGetDisPrism(void *prp, double *val){
  *val = _rkc(prp)->dis;
}

void _rkJointGetVelPrism(void *prp, double *val){
  *val = _rkc(prp)->vel;
}

void _rkJointGetAccPrism(void *prp, double *val){
  *val = _rkc(prp)->acc;
}

void _rkJointGetTrqPrism(void *prp, double *val){
  *val = _rkc(prp)->trq;
}

/* motor */
void _rkJointGetMotorPrism(void *prp, rkMotor **m){
  *m = &_rkc(prp)->m;
}

void _rkJointCatDisPrism(void *prp, double *dis, double k, double *val)
{
  *dis += k * *val;
}

void _rkJointSubDisPrism(void *prp, double *dis, double *sdis)
{
  *dis -= *sdis;
}

/* continuously update joint displacement */
void _rkJointSetDisCNTPrism(void *prp, double *val, double dt)
{
  double olddis, oldvel;

  olddis = _rkc(prp)->dis;
  oldvel = _rkc(prp)->vel;
  _rkJointSetDisPrism( prp, val );
  _rkc(prp)->vel = ( *val - olddis ) / dt;
  _rkc(prp)->acc = ( _rkc(prp)->vel - oldvel ) / dt;
}

/* joint frame transfer function */
zFrame3D *_rkJointXferPrism(void *prp, zFrame3D *fo, zFrame3D *f)
{
  zVec3DCat( zFrame3DPos(fo),
    _rkc(prp)->dis, &zFrame3DAtt(fo)->v[2], zFrame3DPos(f) );
  zMat3DCopy( zFrame3DAtt(fo), zFrame3DAtt(f) );
  return f;
}

/* joint velocity transfer function */
void _rkJointIncVelPrism(void *prp, zVec6D *vel)
{
  vel->e[zZ] += _rkc(prp)->vel;
}

void _rkJointIncAccOnVelPrism(void *prp, zVec3D *w, zVec6D *acc)
{
  acc->e[zX] += 2 * _rkc(prp)->vel * w->e[zY];
  acc->e[zY] -= 2 * _rkc(prp)->vel * w->e[zX];
}

/* joint acceleration transfer function */
void _rkJointIncAccPrism(void *prp, zVec6D *acc)
{
  acc->e[zZ] += _rkc(prp)->acc;
}

/* joint torque transfer function */
void _rkJointCalcTrqPrism(void *prp, zVec6D *f)
{
  _rkc(prp)->trq = f->e[zZ];
}

/* inverse computation of joint torsion and displacement */
void _rkJointTorsionPrism(zFrame3D *dev, zVec6D *t, double dis[])
{
  zVec3D aa;

  zMat3DToAA( zFrame3DAtt(dev), &aa );
  zMulMatTVec3D( zFrame3DAtt(dev), &aa, zVec6DAng(t) );
  dis[0] = rkJointTorsionDisPrism( dev, t );
}

void _rkJointSetFricPrism(void *prp, double *val){
  _rkc(prp)->tf = *val;
}
void _rkJointGetFricPrism(void *prp, double *val){
  *val = _rkc(prp)->tf;
}
void _rkJointGetSFricPrism(void *prp, double *val){
  *val = _rkc(prp)->sf;
}
void _rkJointGetKFricPrism(void *prp, double *val){
  *val = _rkJointRestTrq( _rkc(prp)->stiff, _rkc(prp)->viscos, _rkc(prp)->coulomb, _rkc(prp)->dis, _rkc(prp)->vel );
}

void _rkJointSetRefPrism(void *prp, rkJointRef *ref){
  *ref = _rkc(prp)->_ref;
}
void _rkJointGetRefPrism(void *prp, rkJointRef *ref){
  _rkc(prp)->_ref = *ref;
}

/* query joint properties */
bool _rkJointQueryFReadPrism(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm)
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
  if( strcmp( buf, "stiff" ) == 0 )
    _rkc(prp)->stiff = zFDouble(fp);
  else
  if( strcmp( buf, "viscos" ) == 0 )
    _rkc(prp)->viscos = zFDouble(fp);
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
      ZRUNERROR( "invalid motor name %s detected", buf );
      return true;
    }
    if( rkMotorSize(mp) != 1 ){
      ZRUNERROR( "unmatched motor size" );
      return true;
    }
    rkMotorClone( mp, &_rkc(prp)->m );
  } else
  if( !rkMotorQueryFRead( fp, buf, &_rkc(prp)->m ) )
    return false;
  return true;
}

void _rkJointFWritePrism(FILE *fp, void *prp, char *name)
{
  rkJointPrpPrism *v;

  v = prp;
  if( !zIsTiny( v->dis ) )
    fprintf( fp, "%s: %.10f\n", name, v->dis );
  fprintf( fp, "min: %.10f\n", v->min );
  fprintf( fp, "max: %.10f\n", v->max );
  fprintf( fp, "stiff: %.10f\n", v->stiff );
  fprintf( fp, "viscos: %.10f\n", v->viscos );
  fprintf( fp, "coulomb: %.10f\n", v->coulomb );
}

static zVec3D* (*_rk_joint_axis_prism_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
};
static zVec3D* (*_rk_joint_axis_prism_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisZ,
};
static rkJointCom rk_joint_prism = {
  1,
  _rkJointLimDisPrism,
  _rkJointSetDisPrism,
  _rkJointSetVelPrism,
  _rkJointSetAccPrism,
  _rkJointSetTrqPrism,
  _rkJointGetDisPrism,
  _rkJointGetVelPrism,
  _rkJointGetAccPrism,
  _rkJointGetTrqPrism,
  _rkJointGetMotorPrism,
  _rkJointCatDisPrism,
  _rkJointSubDisPrism,
  _rkJointSetDisCNTPrism,
  _rkJointXferPrism,
  _rkJointIncVelPrism,
  _rkJointIncAccOnVelPrism,
  _rkJointIncAccPrism,
  _rkJointCalcTrqPrism,
  _rkJointTorsionPrism,
  _rkJointSetFricPrism,
  _rkJointGetFricPrism,
  _rkJointGetSFricPrism,
  _rkJointGetKFricPrism,
  _rkJointSetRefPrism,
  _rkJointGetRefPrism,
  _rk_joint_axis_prism_ang,
  _rk_joint_axis_prism_lin,
  _rkJointQueryFReadPrism,
  _rkJointFWritePrism,
};

/* motor */
static byte _rkJointMotorPrism(void *prp);
static void _rkJointMotorSetInputPrism(void *prp, double *val);
static void _rkJointMotorInertiaPrism(void *prp, double *val);
static void _rkJointMotorInputTrqPrism(void *prp, double *val);
static void _rkJointMotorResistancePrism(void *prp, double *val);
static void _rkJointMotorDrivingTrqPrism(void *prp, double *val);

byte _rkJointMotorPrism(void *prp){
  return rkMotorType( &_rkc(prp)->m );
}
void _rkJointMotorSetInputPrism(void *prp, double *val){
  rkMotorSetInput( &_rkc(prp)->m, val );
}
void _rkJointMotorInertiaPrism(void *prp, double *val){
  *val = 0.0;
  rkMotorInertia( &_rkc(prp)->m, val );
}
void _rkJointMotorInputTrqPrism(void *prp, double *val){
  *val = 0.0;
  rkMotorInputTrq( &_rkc(prp)->m, val );
}
void _rkJointMotorResistancePrism(void *prp, double *val){
  *val = 0.0;
  rkMotorRegistance( &_rkc(prp)->m, &_rkc(prp)->dis, &_rkc(prp)->vel, val );
}
void _rkJointMotorDrivingTrqPrism(void *prp, double *val){
  *val = 0.0;
  rkMotorDrivingTrq( &_rkc(prp)->m, &_rkc(prp)->dis, &_rkc(prp)->vel, &_rkc(prp)->acc, val );
}

static rkJointMotorCom rk_joint_motor_prism = {
  _rkJointMotorPrism,
  _rkJointMotorSetInputPrism,
  _rkJointMotorInertiaPrism,
  _rkJointMotorInputTrqPrism,
  _rkJointMotorResistancePrism,
  _rkJointMotorDrivingTrqPrism,
};

/* ABI */
static void _rkJointABIAxisInertiaPrism(void *prp, zMat6D *m, zMat h, zMat ih);
static void _rkJointABIAddAbiPrism(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm);
static void _rkJointABIAddBiasPrism(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb);
static void _rkJointABIDrivingTorquePrism(void *prp);
static void _rkJointABIQAccPrism(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc);

void _rkJointABIAxisInertiaPrism(void *prp, zMat6D *m, zMat h, zMat ih)
{
  _rkJointMotorInertiaPrism( prp, zMatBuf(h) );
  zMatElem(h,0,0) += zMat6DMat3D(m,0,0)->e[2][2];
  if( !zIsTiny( zMatElem(h,0,0) ) )
    zMatElem(ih,0,0) = 1.0 / zMatElem(h,0,0);
  else
    zMatElem(ih,0,0) = 0.0;
}

void _rkJointABIAddAbiPrism(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm)
{
  zVec6D tmpv, tmpv2;
  zMat6D tmpm;

  zMat6DCol( m, zZ, &tmpv );
  zMat6DRow( m, zZ, &tmpv2 );
  zVec6DMulDRC( &tmpv, -1.0*zMatElem(h,0,0) );
  zMat6DDyad( &tmpm, &tmpv, &tmpv2 );
  zMat6DAddDRC( &tmpm, m );

  rkJointXferMat6D( f, &tmpm, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}

void _rkJointABIAddBiasPrism(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb)
{
  zVec6D tmpv, tmpv2;

  zMat6DCol( m, zZ, &tmpv );
  zVec6DMulDRC( &tmpv, _rkc(prp)->_u - b->e[zZ] );
  zVec6DSub( b, &tmpv, &tmpv2 );

  zMulMatVec6D( zFrame3DAtt( f ), &tmpv2, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}

void _rkJointABIDrivingTorquePrism(void *prp)
{
  double val;

  _rkJointMotorInputTrqPrism( prp, &_rkc(prp)->_u );
  _rkJointMotorResistancePrism( prp, &val );
  _rkc(prp)->_u -= val;
  _rkc(prp)->_u += _rkc(prp)->tf;
}

void _rkJointABIQAccPrism(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
{
  zVec6D tmpv;

  zMat6DRow( m, zZ, &tmpv );
  /* q */
  _rkc(prp)->acc = zMatElem(h,0,0)*( _rkc(prp)->_u - zVec6DInnerProd( &tmpv, jac ) - b->e[zZ] );
  /* acc */
  zVec6DCopy( jac, acc );
  acc->e[zZ] += _rkc(prp)->acc;
}

static rkJointABICom rk_joint_abi_prism = {
  _rkJointABIAxisInertiaPrism,
  _rkJointABIAddAbiPrism,
  _rkJointABIAddBiasPrism,
  _rkJointABIDrivingTorquePrism,
  _rkJointABIQAccPrism,
  _rkJointUpdateWrench,
};

/* rkJointCreatePrism
 * - create prismatic joint instance.
 */
rkJoint *rkJointCreatePrism(rkJoint *j)
{
  if( !( j->prp = zAlloc( rkJointPrpPrism, 1 ) ) )
    return NULL;
  _rkc(j->prp)->max = HUGE_VAL;
  _rkc(j->prp)->min =-HUGE_VAL;
  rkMotorCreate( &_rkc(j->prp)->m, RK_MOTOR_NONE );
  j->com = &rk_joint_prism;
  j->mcom = &rk_joint_motor_prism;
  j->acom = &rk_joint_abi_prism;
  return j;
}

#undef _rkc
