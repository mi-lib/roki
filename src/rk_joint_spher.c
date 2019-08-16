/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_spher - joint structure: spherical joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointPrpSpher *)p)

static void _rkJointInitSpher(void *prp){
  rkMotorAssign( &_rkc(prp)->m, &rk_motor_none );
}

static void *_rkJointAllocSpher(void){ return zAlloc( rkJointPrpSpher, 1 ); }

/* limit joint displacement */
static void _rkJointLimDisSpher(void *prp, double *testval, double *limval){
  zVec3DCopy( (zVec3D*)testval, (zVec3D*)limval );
}

/* set joint displacement */
static void _rkJointSetDisSpher(void *prp, double *val){
  _rkJointLimDisSpher( prp, val, _rkc(prp)->aa.e );
  zMat3DFromAA( &_rkc(prp)->_att, &_rkc(prp)->aa );
}

static void _rkJointSetVelSpher(void *prp, double *val){
  zVec3DCopy( (zVec3D*)val, &_rkc(prp)->vel );
}

static void _rkJointSetAccSpher(void *prp, double *val){
  zVec3DCopy( (zVec3D*)val, &_rkc(prp)->acc );
}

static void _rkJointSetTrqSpher(void *prp, double *val){
  zVec3DCopy( (zVec3D*)val, &_rkc(prp)->trq );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointGetDisSpher(void *prp, double *val){
  zVec3DCopy( &_rkc(prp)->aa, (zVec3D*)val );
}

static void _rkJointGetVelSpher(void *prp, double *val){
  zVec3DCopy( &_rkc(prp)->vel, (zVec3D*)val );
}

static void _rkJointGetAccSpher(void *prp, double *val){
  zVec3DCopy( &_rkc(prp)->acc, (zVec3D*)val );
}

static void _rkJointGetTrqSpher(void *prp, double *val){
  zVec3DCopy( &_rkc(prp)->trq, (zVec3D*)val );
}

static void _rkJointCatDisSpher(void *prp, double *dis, double k, double *val)
{
  zVec3D da, aa;

  zVec3DCopy( (zVec3D*)dis, &aa );
  zVec3DMul( (zVec3D*)val, k, &da );
  zAACascade( &aa, &da, (zVec3D*)dis );
}

static void _rkJointSubDisSpher(void *prp, double *dis, double *sdis)
{
  zMat3D m, ms;
  zVec3D aa;

  zVec3DCopy( (zVec3D*)dis, &aa );
  zMat3DFromAA( &m, &aa );
  zMat3DFromAA( &ms, (zVec3D*)sdis );
  zMat3DError( &m, &ms, (zVec3D*)dis );
}

/* continuously update joint displacement */
static void _rkJointSetDisCNTSpher(void *prp, double *val, double dt)
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

/* joint frame transformation */
static zFrame3D *_rkJointXformSpher(void *prp, zFrame3D *fo, zFrame3D *f){
  zVec3DCopy( zFrame3DPos(fo), zFrame3DPos(f) );
  zMulMat3DMat3D( zFrame3DAtt(fo), &_rkc(prp)->_att, zFrame3DAtt(f) );
  return f;
}

/* joint velocity transformation */
static void _rkJointIncVelSpher(void *prp, zVec6D *vel){
  zVec3D cw;
  zMulMat3DTVec3D( &_rkc(prp)->_att, &_rkc(prp)->vel, &cw );
  zVec3DAddDRC( zVec6DAng(vel), &cw );
}

static void _rkJointIncAccOnVelSpher(void *prp, zVec3D *w, zVec6D *acc){
  zVec3D cw;
  zMulMat3DTVec3D( &_rkc(prp)->_att, &_rkc(prp)->vel, &cw );
  zVec3DOuterProd( w, &cw, &cw );
  zVec3DAddDRC( zVec6DAng(acc), &cw );
}

/* joint acceleration transformation */
static void _rkJointIncAccSpher(void *prp, zVec6D *acc){
  zVec3D cw;
  zMulMat3DTVec3D( &_rkc(prp)->_att, &_rkc(prp)->acc, &cw );
  zVec3DAddDRC( zVec6DAng(acc), &cw );
}

/* joint torque transformation */
static void _rkJointCalcTrqSpher(void *prp, zVec6D *f){
  zMulMat3DVec3D( &_rkc(prp)->_att, zVec6DAng(f), &_rkc(prp)->trq );
}

/* inverse computation of joint torsion and displacement */
static void _rkJointTorsionSpher(zFrame3D *dev, zVec6D *t, double dis[]){
  zMulMat3DTVec3D( zFrame3DAtt(dev), zFrame3DPos(dev), zVec6DLin(t) );
  zVec3DZero( zVec6DAng(t) );
  zMat3DToAA( zFrame3DAtt(dev), (zVec3D*)dis );
}

/* joint axes */
static zVec3D *_rkJointAxisSpher(void *prp, zFrame3D *f, zDir dir, zVec3D *a){
  zVec3D al;
  zMat3DRow( &_rkc(prp)->_att, dir, &al );
  return zMulMat3DVec3D( zFrame3DAtt(f), &al, a );
}

static zVec3D *_rkJointAxisXSpher(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisSpher( prp, f, zX, a );
}

static zVec3D *_rkJointAxisYSpher(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisSpher( prp, f, zY, a );
}

static zVec3D *_rkJointAxisZSpher(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisSpher( prp, f, zZ, a );
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

/* friction computation (to be implemented) */

static void _rkJointFrictionPivotSpher(void *prp, rkJointFrictionPivot *fp){}

static void _rkJointValSpher(void *prp, double *val){}

/* motor */
static rkMotor *_rkJointGetMotorSpher(void *prp){ return &_rkc(prp)->m; }

static void _rkJointMotorSetInputSpher(void *prp, double *val){
  rkMotorSetInput( &_rkc(prp)->m, val );
}

static void _rkJointMotorInertiaSpher(void *prp, double *val){
  zMat3DZero( (zMat3D *)val );
  rkMotorInertia( &_rkc(prp)->m, val );
}

static void _rkJointMotorInputTrqSpher(void *prp, double *val){
  zVec3DZero( (zVec3D *)val );
  rkMotorInputTrq( &_rkc(prp)->m, val );
}

static void _rkJointMotorResistanceSpher(void *prp, double *val){
  zVec3DZero( (zVec3D *)val );
  rkMotorRegistance( &_rkc(prp)->m, &_rkc(prp)->aa.e[zX], &_rkc(prp)->vel.e[zX], val );
}

static void _rkJointMotorDrivingTrqSpher(void *prp, double *val){
  zVec3DZero( (zVec3D *)val );
  rkMotorDrivingTrq( &_rkc(prp)->m, &_rkc(prp)->aa.e[zX], &_rkc(prp)->vel.e[zX], &_rkc(prp)->acc.e[zX], val );
}

/* ABI */
static void _rkJointABIAxisInertiaSpher(void *prp, zMat6D *m, zMat h, zMat ih)
{
  /* The inertia matrix is multiplied by R from the left side and
     by R transpose from the right side in the mathematically strict
     way. A nice property is that they are cancelled in the following
     computation and thus are omitted from the beginning. */
  _rkJointMotorInertiaSpher( prp, zMatBuf(h) );
  zMat3DT( &m->e[1][1], (zMat3D *)&zMatElemNC(h,0,0) );
  zMatInv( h, ih );
}

static void _rkJointABIAddAbiSpher(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  eprintf("under construction error: abi update for spherical joint\n");
}

static void _rkJointABIAddBiasSpher(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  eprintf("under construction error: abi update for spherical joint\n");
}

static void _rkJointABIDrivingTorqueSpher(void *prp){
  eprintf("under construction error: abi update for spherical joint\n");
}

static void _rkJointABIQAccSpher(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){}

/* query joint properties */
static bool _rkJointQueryFScanSpher(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm)
{
  rkMotor *mp;
  zVec3D aa;

  if( strcmp( buf, "dis" ) == 0 ){
    zVec3DFScan( fp, &aa );
    _rkJointSetDisSpher( prp, aa.e );
  } else
  if( strcmp( buf, "motor" ) == 0 ){
    zFToken( fp, buf, BUFSIZ );
    zNameFind( marray, nm, buf, mp );
    if( !mp ){
      ZRUNERROR( RK_ERR_MOTOR_UNKNOWN, buf );
      return true;
    }
    if( rkMotorSize(mp) != 3 ){
      ZRUNERROR( RK_ERR_JOINT_SIZMISMATCH );
      return true;
    }
    rkMotorClone( mp, &_rkc(prp)->m );
  } else
  if( !rkMotorQueryFScan( fp, buf, &_rkc(prp)->m ) )
    return false;
  return true;
}

static void *_rkJointDisFromZTKSpher(void *prp, int i, void *arg, ZTK *ztk){
  zVec3D aa;
  zVec3DFromZTK( &aa, ztk );
  _rkJointSetDisSpher( _rkc(prp), aa.e );
  return prp;
}
static void *_rkJointMotorFromZTKSpher(void *prp, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk) ) ) ) return NULL;
  return rkMotorClone( mp, &_rkc(prp)->m ) ? prp : NULL;
}

static void _rkJointDisFPrintSpher(FILE *fp, int i, void *prp){
  zVec3DFPrint( fp, &_rkc(prp)->aa );
}
static void _rkJointMotorFPrintSpher(FILE *fp, int i, void *prp){
  fprintf( fp, "%s\n", zName(&_rkc(prp)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_spher[] = {
  { "dis", 1, _rkJointDisFromZTKSpher, _rkJointDisFPrintSpher },
  { "motor", 1, _rkJointMotorFromZTKSpher, _rkJointMotorFPrintSpher },
};

static void *_rkJointFromZTKSpher(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_spher );
}

static void _rkJointFPrintSpher(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_spher );
}

rkJointCom rk_joint_spher = {
  "spherical",
  3,
  _rkJointInitSpher,
  _rkJointAllocSpher,
  _rkJointLimDisSpher,
  _rkJointSetDisSpher,
  _rkJointSetVelSpher,
  _rkJointSetAccSpher,
  _rkJointSetTrqSpher,
  _rkJointGetDisSpher,
  _rkJointGetVelSpher,
  _rkJointGetAccSpher,
  _rkJointGetTrqSpher,
  _rkJointCatDisSpher,
  _rkJointSubDisSpher,
  _rkJointSetDisCNTSpher,
  _rkJointXformSpher,
  _rkJointIncVelSpher,
  _rkJointIncAccOnVelSpher,
  _rkJointIncAccSpher,
  _rkJointCalcTrqSpher,
  _rkJointTorsionSpher,
  _rk_joint_axis_spher_ang,
  _rk_joint_axis_spher_lin,

  _rkJointFrictionPivotSpher,
  _rkJointFrictionPivotSpher,
  _rkJointValSpher,
  _rkJointValSpher,
  _rkJointValSpher,
  _rkJointValSpher,

  _rkJointGetMotorSpher,
  _rkJointMotorSetInputSpher,
  _rkJointMotorInertiaSpher,
  _rkJointMotorInputTrqSpher,
  _rkJointMotorResistanceSpher,
  _rkJointMotorDrivingTrqSpher,

  _rkJointABIAxisInertiaSpher,
  _rkJointABIAddAbiSpher,
  _rkJointABIAddBiasSpher,
  _rkJointABIDrivingTorqueSpher,
  _rkJointABIQAccSpher,
  _rkJointUpdateWrench,

  _rkJointQueryFScanSpher,
  _rkJointDisFromZTKSpher,
  _rkJointFromZTKSpher,
  _rkJointDisFPrintSpher,
  _rkJointFPrintSpher,
};

bool rkJointRegZTKSpher(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_spher ) ? true : false;
}

#undef _rkc
