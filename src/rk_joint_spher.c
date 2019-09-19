/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_spher - joint structure: spherical joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointSpherPrp *)p)

static void _rkJointSpherInit(void *prp){
  rkMotorAssign( &_rkc(prp)->m, &rk_motor_none );
}

static void *_rkJointSpherAlloc(void){ return zAlloc( rkJointSpherPrp, 1 ); }

/* limit joint displacement */
static void _rkJointSpherLimDis(void *prp, double *testval, double *limval){
  zVec3DCopy( (zVec3D*)testval, (zVec3D*)limval );
}

/* set joint displacement */
static void _rkJointSpherSetDis(void *prp, double *val){
  _rkJointSpherLimDis( prp, val, _rkc(prp)->aa.e );
  zMat3DFromAA( &_rkc(prp)->_att, &_rkc(prp)->aa );
}

static void _rkJointSpherSetVel(void *prp, double *val){
  zVec3DCopy( (zVec3D*)val, &_rkc(prp)->vel );
}

static void _rkJointSpherSetAcc(void *prp, double *val){
  zVec3DCopy( (zVec3D*)val, &_rkc(prp)->acc );
}

static void _rkJointSpherSetTrq(void *prp, double *val){
  zVec3DCopy( (zVec3D*)val, &_rkc(prp)->trq );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointSpherGetDis(void *prp, double *val){
  zVec3DCopy( &_rkc(prp)->aa, (zVec3D*)val );
}

static void _rkJointSpherGetVel(void *prp, double *val){
  zVec3DCopy( &_rkc(prp)->vel, (zVec3D*)val );
}

static void _rkJointSpherGetAcc(void *prp, double *val){
  zVec3DCopy( &_rkc(prp)->acc, (zVec3D*)val );
}

static void _rkJointSpherGetTrq(void *prp, double *val){
  zVec3DCopy( &_rkc(prp)->trq, (zVec3D*)val );
}

static void _rkJointSpherCatDis(void *prp, double *dis, double k, double *val)
{
  zVec3D da, aa;

  zVec3DCopy( (zVec3D*)dis, &aa );
  zVec3DMul( (zVec3D*)val, k, &da );
  zAACascade( &aa, &da, (zVec3D*)dis );
}

static void _rkJointSpherSubDis(void *prp, double *dis, double *sdis)
{
  zMat3D m, ms;
  zVec3D aa;

  zVec3DCopy( (zVec3D*)dis, &aa );
  zMat3DFromAA( &m, &aa );
  zMat3DFromAA( &ms, (zVec3D*)sdis );
  zMat3DError( &m, &ms, (zVec3D*)dis );
}

/* continuously update joint displacement */
static void _rkJointSpherSetDisCNT(void *prp, double *val, double dt)
{
  zMat3D m_old;
  zVec3D v_old;

  /* previous state */
  zMat3DCopy( &_rkc(prp)->_att, &m_old );
  _rkJointSpherGetVel( prp, (double *)&v_old );
  /* update displacement */
  _rkJointSpherSetDis( prp, val );
  /* numerical differentiation */
  zMat3DError( &_rkc(prp)->_att, &m_old, &_rkc(prp)->vel );
  zVec3DDivDRC( &_rkc(prp)->vel, dt );
  zVec3DDif( &v_old, &_rkc(prp)->vel, dt, &_rkc(prp)->acc );
}

/* joint frame transformation */
static zFrame3D *_rkJointSpherXform(void *prp, zFrame3D *fo, zFrame3D *f){
  zVec3DCopy( zFrame3DPos(fo), zFrame3DPos(f) );
  zMulMat3DMat3D( zFrame3DAtt(fo), &_rkc(prp)->_att, zFrame3DAtt(f) );
  return f;
}

/* joint velocity transformation */
static void _rkJointSpherIncVel(void *prp, zVec6D *vel){
  zVec3D cw;
  zMulMat3DTVec3D( &_rkc(prp)->_att, &_rkc(prp)->vel, &cw );
  zVec3DAddDRC( zVec6DAng(vel), &cw );
}

static void _rkJointSpherIncAccOnVel(void *prp, zVec3D *w, zVec6D *acc){
  zVec3D cw;
  zMulMat3DTVec3D( &_rkc(prp)->_att, &_rkc(prp)->vel, &cw );
  zVec3DOuterProd( w, &cw, &cw );
  zVec3DAddDRC( zVec6DAng(acc), &cw );
}

/* joint acceleration transformation */
static void _rkJointSpherIncAcc(void *prp, zVec6D *acc){
  zVec3D cw;
  zMulMat3DTVec3D( &_rkc(prp)->_att, &_rkc(prp)->acc, &cw );
  zVec3DAddDRC( zVec6DAng(acc), &cw );
}

/* joint torque transformation */
static void _rkJointSpherCalcTrq(void *prp, zVec6D *f){
  zMulMat3DVec3D( &_rkc(prp)->_att, zVec6DAng(f), &_rkc(prp)->trq );
}

/* inverse computation of joint torsion and displacement */
static void _rkJointSpherTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  zMulMat3DTVec3D( zFrame3DAtt(dev), zFrame3DPos(dev), zVec6DLin(t) );
  zVec3DZero( zVec6DAng(t) );
  zMat3DToAA( zFrame3DAtt(dev), (zVec3D*)dis );
}

/* joint axes */
static zVec3D *_rkJointSpherAxis(void *prp, zFrame3D *f, zDir dir, zVec3D *a){
  zVec3D al;
  zMat3DRow( &_rkc(prp)->_att, dir, &al );
  return zMulMat3DVec3D( zFrame3DAtt(f), &al, a );
}

static zVec3D *_rkJointSpherAxisX(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointSpherAxis( prp, f, zX, a );
}

static zVec3D *_rkJointSpherAxisY(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointSpherAxis( prp, f, zY, a );
}

static zVec3D *_rkJointSpherAxisZ(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointSpherAxis( prp, f, zZ, a );
}

static zVec3D* (*_rk_joint_spher_axis_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointSpherAxisX,
  _rkJointSpherAxisY,
  _rkJointSpherAxisZ,
};
static zVec3D* (*_rk_joint_spher_axis_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
};

/* friction computation (to be implemented) */

static void _rkJointSpherFrictionPivot(void *prp, rkJointFrictionPivot *fp){}

static void _rkJointSpherVal(void *prp, double *val){}

/* motor */
static rkMotor *_rkJointSpherGetMotor(void *prp){ return &_rkc(prp)->m; }

static void _rkJointSpherMotorSetInput(void *prp, double *val){
  rkMotorSetInput( &_rkc(prp)->m, val );
}

static void _rkJointSpherMotorInertia(void *prp, double *val){
  zMat3DZero( (zMat3D *)val );
  rkMotorInertia( &_rkc(prp)->m, val );
}

static void _rkJointSpherMotorInputTrq(void *prp, double *val){
  zVec3DZero( (zVec3D *)val );
  rkMotorInputTrq( &_rkc(prp)->m, val );
}

static void _rkJointSpherMotorResistance(void *prp, double *val){
  zVec3DZero( (zVec3D *)val );
  rkMotorRegistance( &_rkc(prp)->m, &_rkc(prp)->aa.e[zX], &_rkc(prp)->vel.e[zX], val );
}

static void _rkJointSpherMotorDrivingTrq(void *prp, double *val){
  zVec3DZero( (zVec3D *)val );
  rkMotorDrivingTrq( &_rkc(prp)->m, &_rkc(prp)->aa.e[zX], &_rkc(prp)->vel.e[zX], &_rkc(prp)->acc.e[zX], val );
}

/* ABI */
static void _rkJointSpherABIAxisInertia(void *prp, zMat6D *m, zMat h, zMat ih)
{
  /* The inertia matrix is multiplied by R from the left side and
     by R transpose from the right side in the mathematically strict
     way. A nice property is that they are cancelled in the following
     computation and thus are omitted from the beginning. */
  _rkJointSpherMotorInertia( prp, zMatBuf(h) );
  zMat3DT( &m->e[1][1], (zMat3D *)&zMatElemNC(h,0,0) );
  zMatInv( h, ih );
}

static void _rkJointSpherABIAddABI(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  eprintf("under construction error: abi update for spherical joint\n");
}

static void _rkJointSpherABIAddBias(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  eprintf("under construction error: abi update for spherical joint\n");
}

static void _rkJointSpherABIDrivingTorque(void *prp){
  eprintf("under construction error: abi update for spherical joint\n");
}

static void _rkJointSpherABIQAcc(void *prp, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){}

static void *_rkJointSpherDisFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  zVec3D aa;
  zVec3DFromZTK( &aa, ztk );
  _rkJointSpherSetDis( _rkc(prp), aa.e );
  return prp;
}
static void *_rkJointSpherMotorFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  rkMotor *mp;
  if( !( mp = rkMotorArrayFind( arg, ZTKVal(ztk) ) ) ) return NULL;
  return rkMotorClone( mp, &_rkc(prp)->m ) ? prp : NULL;
}

static void _rkJointSpherDisFPrintZTK(FILE *fp, int i, void *prp){
  zVec3DFPrint( fp, &_rkc(prp)->aa );
}
static void _rkJointSpherMotorFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%s\n", zName(&_rkc(prp)->m) );
}

static ZTKPrp __ztk_prp_rkjoint_spher[] = {
  { "dis", 1, _rkJointSpherDisFromZTK, _rkJointSpherDisFPrintZTK },
  { "motor", 1, _rkJointSpherMotorFromZTK, _rkJointSpherMotorFPrintZTK },
};

static bool _rkJointSpherRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_spher ) ? true : false;
}

static void *_rkJointSpherFromZTK(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_spher );
}

static void _rkJointSpherFPrintZTK(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_spher );
}

rkJointCom rk_joint_spher = {
  "spherical",
  3,
  _rkJointSpherInit,
  _rkJointSpherAlloc,
  _rkJointSpherLimDis,
  _rkJointSpherSetDis,
  _rkJointSpherSetVel,
  _rkJointSpherSetAcc,
  _rkJointSpherSetTrq,
  _rkJointSpherGetDis,
  _rkJointSpherGetVel,
  _rkJointSpherGetAcc,
  _rkJointSpherGetTrq,
  _rkJointSpherCatDis,
  _rkJointSpherSubDis,
  _rkJointSpherSetDisCNT,
  _rkJointSpherXform,
  _rkJointSpherIncVel,
  _rkJointSpherIncAccOnVel,
  _rkJointSpherIncAcc,
  _rkJointSpherCalcTrq,
  _rkJointSpherTorsion,
  _rk_joint_spher_axis_ang,
  _rk_joint_spher_axis_lin,

  _rkJointSpherFrictionPivot,
  _rkJointSpherFrictionPivot,
  _rkJointSpherVal,
  _rkJointSpherVal,
  _rkJointSpherVal,
  _rkJointSpherVal,

  _rkJointSpherGetMotor,
  _rkJointSpherMotorSetInput,
  _rkJointSpherMotorInertia,
  _rkJointSpherMotorInputTrq,
  _rkJointSpherMotorResistance,
  _rkJointSpherMotorDrivingTrq,

  _rkJointSpherABIAxisInertia,
  _rkJointSpherABIAddABI,
  _rkJointSpherABIAddBias,
  _rkJointSpherABIDrivingTorque,
  _rkJointSpherABIQAcc,
  _rkJointUpdateWrench,

  _rkJointSpherRegZTK,
  _rkJointSpherDisFromZTK,
  _rkJointSpherFromZTK,
  _rkJointSpherDisFPrintZTK,
  _rkJointSpherFPrintZTK,
};

#undef _rkc
