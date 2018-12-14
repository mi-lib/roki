/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_spher - joint structure: spherical joint
 */

#include <roki/rk_joint.h>

static void _rkJointLimDisSpher(void *prp, double *testval, double *limval);
static void _rkJointSetDisSpher(void *prp, double *val);
static void _rkJointSetVelSpher(void *prp, double *val);
static void _rkJointSetAccSpher(void *prp, double *val);
static void _rkJointSetTrqSpher(void *prp, double *val);
static void _rkJointGetDisSpher(void *prp, double *val);
static void _rkJointGetVelSpher(void *prp, double *val);
static void _rkJointGetAccSpher(void *prp, double *val);
static void _rkJointGetTrqSpher(void *prp, double *val);
static void _rkJointGetMotorSpher(void *prp, rkMotor **m);
static void _rkJointCatDisSpher(void *prp, double *dis, double k, double *val);
static void _rkJointSubDisSpher(void *prp, double *dis, double *sdis);
static void _rkJointSetDisCNTSpher(void *prp, double *val, double dt);

static zFrame3D *_rkJointXferSpher(void *prp, zFrame3D *fo, zFrame3D *f);
static void _rkJointIncVelSpher(void *prp, zVec6D *vel);
static void _rkJointIncAccOnVelSpher(void *prp, zVec3D *w, zVec6D *acc);
static void _rkJointIncAccSpher(void *prp, zVec6D *acc);

static void _rkJointCalcTrqSpher(void *prp, zVec6D *f);
static void _rkJointTorsionSpher(zFrame3D *dev, zVec6D *t, double dis[]);
static void _rkJointValSpher(void *prp, double *val);
static void _rkJointRefSpher(void *prp, rkJointRef *ref);

static zVec3D *_rkJointAxisSpher(void *prp, zFrame3D *f, zDir dir, zVec3D *a);
static zVec3D *_rkJointAxisXSpher(void *prp, zFrame3D *f, zVec3D *a);
static zVec3D *_rkJointAxisYSpher(void *prp, zFrame3D *f, zVec3D *a);
static zVec3D *_rkJointAxisZSpher(void *prp, zFrame3D *f, zVec3D *a);

static bool _rkJointQueryFReadSpher(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm);
static void _rkJointFWriteSpher(FILE *fp, void *prp, char *name);

#define _rkc(p) ((rkJointPrpSpher *)p)

/* limit joint displacement */
void _rkJointLimDisSpher(void *prp, double *testval, double *limval){
  zVec3DCopy( (zVec3D*)testval, (zVec3D*)limval );
}

/* joint displacement set function */
void _rkJointSetDisSpher(void *prp, double *val){
  _rkJointLimDisSpher( prp, val, _rkc(prp)->aa.e );
  zMat3DFromAA( &_rkc(prp)->_att, &_rkc(prp)->aa );
}

void _rkJointSetVelSpher(void *prp, double *val){
  zVec3DCopy( (zVec3D*)val, &_rkc(prp)->vel );
}

void _rkJointSetAccSpher(void *prp, double *val){
  zVec3DCopy( (zVec3D*)val, &_rkc(prp)->acc );
}

void _rkJointSetTrqSpher(void *prp, double *val){
  zVec3DCopy( (zVec3D*)val, &_rkc(prp)->trq );
}

/* get joint displacement, velocity, acceleration and torque */
void _rkJointGetDisSpher(void *prp, double *val){
  zVec3DCopy( &_rkc(prp)->aa, (zVec3D*)val );
}

void _rkJointGetVelSpher(void *prp, double *val){
  zVec3DCopy( &_rkc(prp)->vel, (zVec3D*)val );
}

void _rkJointGetAccSpher(void *prp, double *val){
  zVec3DCopy( &_rkc(prp)->acc, (zVec3D*)val );
}

void _rkJointGetTrqSpher(void *prp, double *val){
  zVec3DCopy( &_rkc(prp)->trq, (zVec3D*)val );
}

/* motor */
void _rkJointGetMotorSpher(void *prp, rkMotor **m){
  *m = &_rkc(prp)->m;
}

void _rkJointCatDisSpher(void *prp, double *dis, double k, double *val)
{
  zVec3D da, aa;

  zVec3DCopy( (zVec3D*)dis, &aa );
  zVec3DMul( (zVec3D*)val, k, &da );
  zAACascade( &aa, &da, (zVec3D*)dis );
}

void _rkJointSubDisSpher(void *prp, double *dis, double *sdis)
{
  zMat3D m, ms;
  zVec3D aa;

  zVec3DCopy( (zVec3D*)dis, &aa );
  zMat3DFromAA( &m, &aa );
  zMat3DFromAA( &ms, (zVec3D*)sdis );
  zMat3DError( &m, &ms, (zVec3D*)dis );
}

/* continuously update joint displacement */
void _rkJointSetDisCNTSpher(void *prp, double *val, double dt)
{
  zMat3D m_old;
  zVec3D v_old;

  /* previous state */
  zMat3DCopy( &_rkc(prp)->_att, &m_old );
  _rkJointGetVelSpher( prp, (double *)&v_old );
  /* update displacement */
  _rkJointSetDisSpher( prp, val );
  /* numerical differentiation */
  zMat3DError( &_rkc(prp)->_att, &m_old, &_rkc(prp)->vel );
  zVec3DDivDRC( &_rkc(prp)->vel, dt );
  zVec3DDif( &v_old, &_rkc(prp)->vel, dt, &_rkc(prp)->acc );
}

/* joint frame transfer function */
zFrame3D *_rkJointXferSpher(void *prp, zFrame3D *fo, zFrame3D *f)
{
  zVec3DCopy( zFrame3DPos(fo), zFrame3DPos(f) );
  zMulMatMat3D( zFrame3DAtt(fo), &_rkc(prp)->_att, zFrame3DAtt(f) );
  return f;
}

/* joint velocity transfer function */
void _rkJointIncVelSpher(void *prp, zVec6D *vel)
{
  zVec3D cw;

  zMulMatTVec3D( &_rkc(prp)->_att, &_rkc(prp)->vel, &cw );
  zVec3DAddDRC( zVec6DAng(vel), &cw );
}

void _rkJointIncAccOnVelSpher(void *prp, zVec3D *w, zVec6D *acc)
{
  zVec3D cw;

  zMulMatTVec3D( &_rkc(prp)->_att, &_rkc(prp)->vel, &cw );
  zVec3DOuterProd( w, &cw, &cw );
  zVec3DAddDRC( zVec6DAng(acc), &cw );
}

/* joint acceleration transfer function */
void _rkJointIncAccSpher(void *prp, zVec6D *acc)
{
  zVec3D cw;

  zMulMatTVec3D( &_rkc(prp)->_att, &_rkc(prp)->acc, &cw );
  zVec3DAddDRC( zVec6DAng(acc), &cw );
}

/* joint torque transfer function */
void _rkJointCalcTrqSpher(void *prp, zVec6D *f)
{
  zMulMatVec3D( &_rkc(prp)->_att, zVec6DAng(f), &_rkc(prp)->trq );
}

/* inverse computation of joint torsion and displacement */
void _rkJointTorsionSpher(zFrame3D *dev, zVec6D *t, double dis[])
{
  zMulMatTVec3D( zFrame3DAtt(dev), zFrame3DPos(dev), zVec6DLin(t) );
  zVec3DClear( zVec6DAng(t) );
  zMat3DToAA( zFrame3DAtt(dev), (zVec3D*)dis );
}

void _rkJointValSpher(void *prp, double *val){}
void _rkJointRefSpher(void *prp, rkJointRef *ref){}

/* joint axis function */
zVec3D *_rkJointAxisSpher(void *prp, zFrame3D *f, zDir dir, zVec3D *a){
  zVec3D al;

  zMat3DRow( &_rkc(prp)->_att, dir, &al );
  return zMulMatVec3D( zFrame3DAtt(f), &al, a );
}

zVec3D *_rkJointAxisXSpher(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisSpher( prp, f, zX, a );
}

zVec3D *_rkJointAxisYSpher(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisSpher( prp, f, zY, a );
}

zVec3D *_rkJointAxisZSpher(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisSpher( prp, f, zZ, a );
}

/* query joint properties */
bool _rkJointQueryFReadSpher(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm)
{
  rkMotor *mp;
  zVec3D aa;

  if( strcmp( buf, "dis" ) == 0 ){
    zVec3DFRead( fp, &aa );
    _rkJointSetDisSpher( prp, aa.e );
  } else
  if( strcmp( buf, "motor" ) == 0 ){
    zFToken( fp, buf, BUFSIZ );
    zNameFind( marray, nm, buf, mp );
    if( !mp ){
      ZRUNERROR( "invalid motor name %s detected", buf );
      return true;
    }
    if( rkMotorSize(mp) != 3 ){
      ZRUNERROR( "unmatched motor size" );
      return true;
    }
    rkMotorClone( mp, &_rkc(prp)->m );
  } else
  if( !rkMotorQueryFRead( fp, buf, &_rkc(prp)->m ) )
    return false;
  return true;
}

void _rkJointFWriteSpher(FILE *fp, void *prp, char *name)
{
  if( !zVec3DIsTiny( &_rkc(prp)->aa ) )
    fprintf( fp, "%s: %.10f %.10f %.10f\n", name,
      _rkc(prp)->aa.e[zX], _rkc(prp)->aa.e[zY], _rkc(prp)->aa.e[zZ] );
}

static zVec3D* (*_rk_joint_axis_spher_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisXSpher,
  _rkJointAxisYSpher,
  _rkJointAxisZSpher,
};
static zVec3D* (*_rk_joint_axis_spher_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
};
static rkJointCom rk_joint_spher = {
  3,
  _rkJointLimDisSpher,
  _rkJointSetDisSpher,
  _rkJointSetVelSpher,
  _rkJointSetAccSpher,
  _rkJointSetTrqSpher,
  _rkJointGetDisSpher,
  _rkJointGetVelSpher,
  _rkJointGetAccSpher,
  _rkJointGetTrqSpher,
  _rkJointGetMotorSpher,
  _rkJointCatDisSpher,
  _rkJointSubDisSpher,
  _rkJointSetDisCNTSpher,
  _rkJointXferSpher,
  _rkJointIncVelSpher,
  _rkJointIncAccOnVelSpher,
  _rkJointIncAccSpher,
  _rkJointCalcTrqSpher,
  _rkJointTorsionSpher,
  _rkJointValSpher,
  _rkJointValSpher,
  _rkJointValSpher,
  _rkJointValSpher,
  _rkJointRefSpher,
  _rkJointRefSpher,
  _rk_joint_axis_spher_ang,
  _rk_joint_axis_spher_lin,
  _rkJointQueryFReadSpher,
  _rkJointFWriteSpher,
};

/* motor */
static byte _rkJointMotorSpher(void *prp);
static void _rkJointMotorSetInputSpher(void *prp, double *val);
static void _rkJointMotorInertiaSpher(void *prp, double *val);
static void _rkJointMotorInputTrqSpher(void *prp, double *val);
static void _rkJointMotorResistanceSpher(void *prp, double *val);
static void _rkJointMotorDrivingTrqSpher(void *prp, double *val);

byte _rkJointMotorSpher(void *prp){
  return rkMotorType( &_rkc(prp)->m );
}
void _rkJointMotorSetInputSpher(void *prp, double *val){
  rkMotorSetInput( &_rkc(prp)->m, val );
}
void _rkJointMotorInertiaSpher(void *prp, double *val){
  zMat3DClear( (zMat3D *)val );
  rkMotorInertia( &_rkc(prp)->m, val );
}
void _rkJointMotorInputTrqSpher(void *prp, double *val){
  zVec3DClear( (zVec3D *)val );
  rkMotorInputTrq( &_rkc(prp)->m, val );
}
void _rkJointMotorResistanceSpher(void *prp, double *val){
  zVec3DClear( (zVec3D *)val );
  rkMotorRegistance( &_rkc(prp)->m, &_rkc(prp)->aa.e[zX], &_rkc(prp)->vel.e[zX], val );
}
void _rkJointMotorDrivingTrqSpher(void *prp, double *val){
  zVec3DClear( (zVec3D *)val );
  rkMotorDrivingTrq( &_rkc(prp)->m, &_rkc(prp)->aa.e[zX], &_rkc(prp)->vel.e[zX], &_rkc(prp)->acc.e[zX], val );
}

static rkJointMotorCom rk_joint_motor_spher = {
  _rkJointMotorSpher,
  _rkJointMotorSetInputSpher,
  _rkJointMotorInertiaSpher,
  _rkJointMotorInputTrqSpher,
  _rkJointMotorResistanceSpher,
  _rkJointMotorDrivingTrqSpher,
};

/* ABI */
static void _rkJointABIAxisInertiaSpher(void *prp, zMat6D *m, zMat h, zMat ih);
static void _rkJointABIAddAbiSpher(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm);
static void _rkJointABIAddBiasSpher(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb);
static void _rkJointABIDrivingTorqueSpher(void *prp);
static void _rkJointABIQAccSpher(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc);

void _rkJointABIAxisInertiaSpher(void *prp, zMat6D *m, zMat h, zMat ih)
{
  /* The inertia matrix is multiplied by R from the left side and
     by R transpose from the right side in the mathematically strict
     way. A nice property is that they are cancelled in the following
     computation and thus are omitted from the beginning. */
  _rkJointMotorInertiaSpher( prp, zMatBuf(h) );
  zMat3DT( zMat6DMat3D(m,1,1), (zMat3D *)&zMatElem(h,0,0) );
  zMatInv( h, ih );
}

void _rkJointABIAddAbiSpher(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm)
{
  eprintf("under construction error: abi update for spherical joint\n");
}

void _rkJointABIAddBiasSpher(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb)
{
  eprintf("under construction error: abi update for spherical joint\n");
}

void _rkJointABIDrivingTorqueSpher(void *prp)
{
  eprintf("under construction error: abi update for spherical joint\n");
}

void _rkJointABIQAccSpher(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
{
}

static rkJointABICom rk_joint_abi_spher = {
  _rkJointABIAxisInertiaSpher,
  _rkJointABIAddAbiSpher,
  _rkJointABIAddBiasSpher,
  _rkJointABIDrivingTorqueSpher,
  _rkJointABIQAccSpher,
  _rkJointUpdateWrench,
};

/* rkJointCreateSpher
 * - create spherical joint instance.
 */
rkJoint *rkJointCreateSpher(rkJoint *j)
{
  if( !( j->prp = zAlloc( rkJointPrpSpher, 1 ) ) )
    return NULL;
  rkMotorCreate( &_rkc(j->prp)->m, RK_MOTOR_NONE );
  j->com = &rk_joint_spher;
  j->mcom = &rk_joint_motor_spher;
  j->acom = &rk_joint_abi_spher;
  return j;
}

#undef _rkc
