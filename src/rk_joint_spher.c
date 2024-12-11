/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_spher - joint structure: spherical joint
 */

#include <roki/rk_joint.h>

#define _rks(joint) ((rkJointSpherState *)((rkJoint *)(joint))->state)

static void _rkJointSpherInit(rkJoint *joint){}

static void *_rkJointSpherAllocPrp(void){ return NULL; }
static void _rkJointSpherCopyPrp(rkJoint *src, rkJoint *dst){}

RK_JOINT_COM_DEF_STATE_FUNC( Spher )

/* limit joint displacement */
static void _rkJointSpherLimDis(rkJoint *joint, double *testval, double *limval){
  zVec3DCopy( (zVec3D*)testval, (zVec3D*)limval );
}

/* set joint displacement */
static void _rkJointSpherSetDis(rkJoint *joint, double *val){
  _rkJointSpherLimDis( joint, val, _rks(joint)->aa.e );
  zMat3DFromAA( &_rks(joint)->_att, &_rks(joint)->aa );
}

static void _rkJointSpherSetMinMax(rkJoint *joint, double *val){}

static void _rkJointSpherSetVel(rkJoint *joint, double *val){
  zVec3DCopy( (zVec3D*)val, &_rks(joint)->vel );
}

static void _rkJointSpherSetAcc(rkJoint *joint, double *val){
  zVec3DCopy( (zVec3D*)val, &_rks(joint)->acc );
}

static void _rkJointSpherSetTrq(rkJoint *joint, double *val){
  zVec3DCopy( (zVec3D*)val, &_rks(joint)->trq );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointSpherGetDis(rkJoint *joint, double *val){
  zVec3DCopy( &_rks(joint)->aa, (zVec3D*)val );
}

static void _rkJointSpherGetMin(rkJoint *joint, double *val){
  val[0] = val[1] = val[2] = -HUGE_VAL;
}

static void _rkJointSpherGetMax(rkJoint *joint, double *val){
  val[0] = val[1] = val[2] = HUGE_VAL;
}

static void _rkJointSpherGetVel(rkJoint *joint, double *val){
  zVec3DCopy( &_rks(joint)->vel, (zVec3D*)val );
}

static void _rkJointSpherGetAcc(rkJoint *joint, double *val){
  zVec3DCopy( &_rks(joint)->acc, (zVec3D*)val );
}

static void _rkJointSpherGetTrq(rkJoint *joint, double *val){
  zVec3DCopy( &_rks(joint)->trq, (zVec3D*)val );
}

static void _rkJointSpherCatDis(rkJoint *joint, double *dis, double k, double *val)
{
  zVec3D da, aa;

  zVec3DCopy( (zVec3D*)dis, &aa );
  zVec3DMul( (zVec3D*)val, k, &da );
  zAACascade( &aa, &da, (zVec3D*)dis );
}

static void _rkJointSpherSubDis(rkJoint *joint, double *dis, double *sdis)
{
  zMat3D m, ms;

  zMat3DFromAA( &m, (zVec3D*)dis );
  zMat3DFromAA( &ms, (zVec3D*)sdis );
  zMat3DError( &m, &ms, (zVec3D*)dis );
}

/* continuously update joint displacement */
static void _rkJointSpherSetDisCNT(rkJoint *joint, double *val, double dt)
{
  zMat3D m_old;
  zVec3D v_old;

  /* previous state */
  zMat3DCopy( &_rks(joint)->_att, &m_old );
  _rkJointSpherGetVel( joint, (double *)&v_old );
  /* update displacement */
  _rkJointSpherSetDis( joint, val );
  /* numerical differentiation */
  zMat3DError( &_rks(joint)->_att, &m_old, &_rks(joint)->vel );
  zVec3DDivDRC( &_rks(joint)->vel, dt );
  zVec3DDif( &v_old, &_rks(joint)->vel, dt, &_rks(joint)->acc );
}

/* joint frame transformation */
static zFrame3D *_rkJointSpherXform(rkJoint *joint, zFrame3D *fo, zFrame3D *f){
  zVec3DCopy( zFrame3DPos(fo), zFrame3DPos(f) );
  zMulMat3DMat3D( zFrame3DAtt(fo), &_rks(joint)->_att, zFrame3DAtt(f) );
  return f;
}

/* joint velocity transformation */
static void _rkJointSpherIncVel(rkJoint *joint, zVec6D *vel){
  zVec3D cw;
  zMulMat3DTVec3D( &_rks(joint)->_att, &_rks(joint)->vel, &cw );
  zVec3DAddDRC( zVec6DAng(vel), &cw );
}

static void _rkJointSpherIncAccOnVel(rkJoint *joint, zVec3D *w, zVec6D *acc){
  zVec3D cw;
  zMulMat3DTVec3D( &_rks(joint)->_att, &_rks(joint)->vel, &cw );
  zVec3DOuterProd( w, &cw, &cw );
  zVec3DAddDRC( zVec6DAng(acc), &cw );
}

/* joint acceleration transformation */
static void _rkJointSpherIncAcc(rkJoint *joint, zVec6D *acc){
  zVec3D cw;
  zMulMat3DTVec3D( &_rks(joint)->_att, &_rks(joint)->acc, &cw );
  zVec3DAddDRC( zVec6DAng(acc), &cw );
}

/* joint torque transformation */
static void _rkJointSpherCalcTrq(rkJoint *joint, zVec6D *f){
  zMulMat3DVec3D( &_rks(joint)->_att, zVec6DAng(f), &_rks(joint)->trq );
}

/* inverse computation of joint torsion and displacement */
static void _rkJointSpherTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  zMulMat3DTVec3D( zFrame3DAtt(dev), zFrame3DPos(dev), zVec6DLin(t) );
  zVec3DZero( zVec6DAng(t) );
  zMat3DToAA( zFrame3DAtt(dev), (zVec3D*)dis );
}

/* joint axes */

static zVec3D *_rkJointSpherAxis(rkJoint *joint, zFrame3D *f, zDir dir, zVec3D *a){
  zVec3D al;
  zMat3DRow( &_rks(joint)->_att, dir, &al );
  return zMulMat3DVec3D( zFrame3DAtt(f), &al, a );
}

static zVec3D *_rkJointSpherAxisX(rkJoint *joint, zFrame3D *f, zVec3D *a){
  return _rkJointSpherAxis( joint, f, zX, a );
}

static zVec3D *_rkJointSpherAxisY(rkJoint *joint, zFrame3D *f, zVec3D *a){
  return _rkJointSpherAxis( joint, f, zY, a );
}

static zVec3D *_rkJointSpherAxisZ(rkJoint *joint, zFrame3D *f, zVec3D *a){
  return _rkJointSpherAxis( joint, f, zZ, a );
}

static zVec3D* (*_rk_joint_spher_axis_ang[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointSpherAxisX,
  _rkJointSpherAxisY,
  _rkJointSpherAxisZ,
};
static zVec3D* (*_rk_joint_spher_axis_lin[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
};

/* CRB method */

static void _rkJointSpherCRBWrench(rkJoint *joint, rkMP *crb, zVec6D wi[]){
  zMat3D icrb;
  zVec3D a;
  int i;

  rkMPOrgInertia( crb, &icrb );
  for( i=0; i<3; i++ ){
    _zMat3DRow( &_rks(joint)->_att, i, &a );
    _zVec3DOuterProd( rkMPCOM(crb), &a, zVec6DLin(&wi[i]) );
    _zVec3DMulDRC( zVec6DLin(&wi[i]), -rkMPMass(crb) );
    _zMulMat3DVec3D( &icrb, &a, zVec6DAng(&wi[i]) );
  }
}
static void _rkJointSpherCRBXform(rkJoint *joint, zFrame3D *f, zVec6D si[]){
  zMat3D r;
  int i;

  zMulMat3DMat3DT( zFrame3DAtt(f), &_rks(joint)->_att, &r );
  for( i=0; i<3; i++ ){
    _zVec3DOuterProd( zFrame3DPos(f), &r.v[i], zVec6DLin(&si[i]) );
    zVec3DCopy( &r.v[i], zVec6DAng(&si[i]) );
  }
}

/* friction computation (to be implemented) */

static void _rkJointSpherFrictionPivot(rkJoint *joint, rkJointFrictionPivot *fp){}

static void _rkJointSpherVal(rkJoint *joint, double *val){}

/* motor */

static void _rkJointSpherMotorSetInput(rkJoint *joint, double *val){
  rkMotorSetInput( rkJointMotor(joint), val );
}

static void _rkJointSpherMotorInertia(rkJoint *joint, double *val){
  zMat3DZero( (zMat3D *)val );
  rkMotorInertia( rkJointMotor(joint), val );
}

static void _rkJointSpherMotorInputTrq(rkJoint *joint, double *val){
  zVec3DZero( (zVec3D *)val );
  rkMotorInputTrq( rkJointMotor(joint), val );
}

static void _rkJointSpherMotorResistance(rkJoint *joint, double *val){
  zVec3DZero( (zVec3D *)val );
  rkMotorRegistance( rkJointMotor(joint), &_rks(joint)->aa.e[zX], &_rks(joint)->vel.e[zX], val );
}

static void _rkJointSpherMotorDrivingTrq(rkJoint *joint, double *val){
  zVec3DZero( (zVec3D *)val );
  rkMotorDrivingTrq( rkJointMotor(joint), &_rks(joint)->aa.e[zX], &_rks(joint)->vel.e[zX], &_rks(joint)->acc.e[zX], val );
}

/* ABI */

static void _rkJointSpherABIAxisInertia(rkJoint *joint, zMat6D *m, zMat h, zMat ih)
{
  /* The inertia matrix is multiplied by R from the left side and
     by R transpose from the right side in the mathematically strict
     way. A nice property is that they are cancelled in the following
     computation and thus are omitted from the beginning. */
  _rkJointSpherMotorInertia( joint, zMatBufNC(h) );
  zMat3DT( &m->e[1][1], (zMat3D *)&zMatElemNC(h,0,0) );
  zMatInv( h, ih );
}

static void _rkJointSpherABIAddABI(rkJoint *joint, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  eprintf("under construction error: abi update for spherical joint\n");
}

static void _rkJointSpherABIAddBias(rkJoint *joint, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  eprintf("under construction error: abi update for spherical joint\n");
}

static void _rkJointSpherABIDrivingTorque(rkJoint *joint){
  eprintf("under construction error: abi update for spherical joint\n");
}

static void _rkJointSpherABIQAcc(rkJoint *joint, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){}

/* ZTK */

static void *_rkJointSpherDisFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  zVec3D aa;
  zVec3DFromZTK( &aa, ztk );
  _rkJointSpherSetDis( (rkJoint *)joint, aa.e );
  return joint;
}
static void *_rkJointSpherMotorFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  return rkJointAssignMotorByStr( (rkJoint *)joint, (rkMotorSpecArray *)arg, ZTKVal(ztk) );
}

static bool _rkJointSpherDisFPrintZTK(FILE *fp, int i, void *joint){
  if( zVec3DIsTiny( &_rks(joint)->aa ) ) return false;
  zVec3DValueNLFPrint( fp, &_rks(joint)->aa );
  return true;
}

static bool _rkJointSpherMotorFPrintZTK(FILE *fp, int i, void *joint){
  if( !((rkJoint *)joint)->motor ) return false;
  fprintf( fp, "%s\n", rkMotorName( ((rkJoint *)joint)->motor ) );
  return true;
}

static const ZTKPrp __ztk_prp_rkjoint_spher[] = {
  { ZTK_KEY_ROKI_JOINT_DIS,   1, _rkJointSpherDisFromZTK, _rkJointSpherDisFPrintZTK },
  { ZTK_KEY_ROKI_JOINT_MOTOR, 1, _rkJointSpherMotorFromZTK, _rkJointSpherMotorFPrintZTK },
};

static rkJoint *_rkJointSpherFromZTK(rkJoint *joint, rkMotorSpecArray *motorspecarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorspecarray, ztk, __ztk_prp_rkjoint_spher );
}

static void _rkJointSpherFPrintZTK(FILE *fp, rkJoint *joint, char *name)
{
  ZTKPrpKeyFPrint( fp, joint, __ztk_prp_rkjoint_spher );
}

rkJointCom rk_joint_spher = {
  "spherical",
  3,
  _rkJointSpherInit,
  _rkJointSpherAllocPrp,
  _rkJointSpherAllocState,
  _rkJointSpherCopyPrp,
  _rkJointSpherCopyState,
  _rkJointSpherLimDis,
  _rkJointSpherSetDis,
  _rkJointSpherSetMinMax,
  _rkJointSpherSetMinMax,
  _rkJointSpherSetVel,
  _rkJointSpherSetAcc,
  _rkJointSpherSetTrq,
  _rkJointSpherGetDis,
  _rkJointSpherGetMin,
  _rkJointSpherGetMax,
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

  _rkJointSpherCRBWrench,
  _rkJointSpherCRBXform,

  _rkJointSpherFrictionPivot,
  _rkJointSpherFrictionPivot,
  _rkJointSpherVal,
  _rkJointSpherVal,
  _rkJointSpherVal,
  _rkJointSpherVal,

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

  _rkJointSpherDisFromZTK,
  _rkJointSpherFromZTK,
  _rkJointSpherDisFPrintZTK,
  _rkJointSpherFPrintZTK,
};

#undef _rks
