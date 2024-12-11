/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_float - joint structure: free-floating joint
 */

#include <roki/rk_joint.h>

#define _rks(joint) ((rkJointFloatState *)((rkJoint *)(joint))->state)

static void _rkJointFloatInit(rkJoint *joint){}

static void *_rkJointFloatAllocPrp(void){ return NULL; }
static void _rkJointFloatCopyPrp(rkJoint *src, rkJoint *dst){}

RK_JOINT_COM_DEF_STATE_FUNC( Float )

/* limit joint displacement */
static void _rkJointFloatLimDis(rkJoint *joint, double *testval, double *limval){
  zVec6DCopy( (zVec6D*)testval, (zVec6D*)limval );
}

/* joint displacement set function */
static void _rkJointFloatSetDis(rkJoint *joint, double *val){
  _rkJointFloatLimDis( joint, val, _rks(joint)->dis.e );
  zMat3DFromAA( &_rks(joint)->_att, zVec6DAng(&_rks(joint)->dis) );
}

static void _rkJointFloatSetMinMax(rkJoint *joint, double *val){}

static void _rkJointFloatSetVel(rkJoint *joint, double *val){
  zVec6DCopy( (zVec6D*)val, &_rks(joint)->vel );
}

static void _rkJointFloatSetAcc(rkJoint *joint, double *val){
  zVec6DCopy( (zVec6D*)val, &_rks(joint)->acc );
}

static void _rkJointFloatSetTrq(rkJoint *joint, double *val){
  zVec6DCopy( (zVec6D*)val, &_rks(joint)->trq );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointFloatGetDis(rkJoint *joint, double *val){
  zVec6DCopy( &_rks(joint)->dis, (zVec6D*)val );
}

static void _rkJointFloatGetMin(rkJoint *joint, double *val){
  val[0] = val[1] = val[2] = val[3] = val[4] = val[5] = -HUGE_VAL;
}

static void _rkJointFloatGetMax(rkJoint *joint, double *val){
  val[0] = val[1] = val[2] = val[3] = val[4] = val[5] = HUGE_VAL;
}

static void _rkJointFloatGetVel(rkJoint *joint, double *val){
  zVec6DCopy( &_rks(joint)->vel, (zVec6D*)val );
}

static void _rkJointFloatGetAcc(rkJoint *joint, double *val){
  zVec6DCopy( &_rks(joint)->acc, (zVec6D*)val );
}

static void _rkJointFloatGetTrq(rkJoint *joint, double *val){
  zVec6DCopy( &_rks(joint)->trq, (zVec6D*)val );
}

static void _rkJointFloatCatDis(rkJoint *joint, double *dis, double k, double *val){
  zVec3D d, aa;
  /* concatenate position */
  zVec3DCatDRC( (zVec3D*)&dis[0], k, (zVec3D*)&val[0] );
  /* concatenate attitude */
  zVec3DCopy( (zVec3D*)&dis[3], &aa );
  zVec3DMul( (zVec3D*)&val[3], k, &d );
  zAACascade( &aa, &d, (zVec3D*)&dis[3] );
}

static void _rkJointFloatSubDis(rkJoint *joint, double *dis, double *sdis){
  zMat3D m, ms;

  zVec3DSubDRC( zVec6DLin((zVec6D*)dis), zVec6DLin((zVec6D*)sdis) );
  zMat3DFromAA( &m, zVec6DAng((zVec6D*)dis) );
  zMat3DFromAA( &ms, zVec6DAng((zVec6D*)sdis) );
  zMat3DError( &m, &ms, zVec6DAng((zVec6D*)dis) );
}

/* continuously update joint displacement */
static void _rkJointFloatSetDisCNT(rkJoint *joint, double *val, double dt){
  zMat3D m_old;
  zVec3D p_old;
  zVec6D v_old;

  /* previous state */
  zVec3DCopy( zVec6DLin(&_rks(joint)->dis), &p_old );
  zMat3DCopy( &_rks(joint)->_att, &m_old );
  _rkJointFloatGetVel( joint, v_old.e );
  /* update displacement */
  _rkJointFloatSetDis( joint, val );
  /* numerical differentiation */
  zVec3DDif( &p_old, zVec6DLin(&_rks(joint)->dis), dt, zVec6DLin(&_rks(joint)->vel) );
  zMat3DError( &_rks(joint)->_att, &m_old, zVec6DAng(&_rks(joint)->vel) );
  zVec3DDivDRC( zVec6DAng(&_rks(joint)->vel), dt );
  zVec6DDif( &v_old, &_rks(joint)->vel, dt, &_rks(joint)->acc );
}

/* joint frame transformation */
static zFrame3D *_rkJointFloatXform(rkJoint *joint, zFrame3D *fo, zFrame3D *f){
  /* position */
  zXform3D( fo, zVec6DLin(&_rks(joint)->dis), zFrame3DPos(f) );
  /* attitude */
  zMulMat3DMat3D( zFrame3DAtt(fo), &_rks(joint)->_att, zFrame3DAtt(f) );
  return f;
}

/* joint velocity transformation */
static void _rkJointFloatIncVel(rkJoint *joint, zVec6D *vel){
  zVec6D vl;
  zMulMat3DTVec6D( &_rks(joint)->_att, &_rks(joint)->vel, &vl );
  zVec6DAddDRC( vel, &vl );
}

static void _rkJointFloatIncAccOnVel(rkJoint *joint, zVec3D *w, zVec6D *acc){
  zVec6D vl;
  zVec3D tmp;
  /* FIXME: _att -> ^pR_j */
  zMulMat3DTVec6D( &_rks(joint)->_att, &_rks(joint)->vel, &vl );
  zVec3DOuterProd( w, zVec6DLin(&vl), &tmp );
  zVec3DCatDRC( zVec6DLin(acc), 2, &tmp );
  zVec3DOuterProd( w, zVec6DAng(&vl), &tmp );
  zVec3DAddDRC( zVec6DAng(acc), &tmp );
}

/* joint acceleration transformation */
static void _rkJointFloatIncAcc(rkJoint *joint, zVec6D *acc){
  zVec6D al;
  zMulMat3DTVec6D( &_rks(joint)->_att, &_rks(joint)->acc, &al );
  zVec6DAddDRC( acc, &al );
}

/* joint torque transformation */
static void _rkJointFloatCalcTrq(rkJoint *joint, zVec6D *f){
  zMulMat3DVec6D( &_rks(joint)->_att, f, &_rks(joint)->trq );
}

/* inverse computation of joint torsion and displacement */
static void _rkJointFloatTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  zVec6DZero( t );
  zFrame3DToVec6DAA( dev, (zVec6D*)dis );
}

/* joint axes */

static zVec3D *_rkJointFloatAxis(rkJoint *joint, zFrame3D *f, zDir dir, zVec3D *a){
  zVec3D al;
  zMat3DRow( &_rks(joint)->_att, dir, &al );
  return zMulMat3DVec3D( zFrame3DAtt(f), &al, a );
}

static zVec3D *_rkJointFloatAxisX(rkJoint *joint, zFrame3D *f, zVec3D *a){
  return _rkJointFloatAxis( joint, f, zX, a );
}

static zVec3D *_rkJointFloatAxisY(rkJoint *joint, zFrame3D *f, zVec3D *a){
  return _rkJointFloatAxis( joint, f, zY, a );
}

static zVec3D *_rkJointFloatAxisZ(rkJoint *joint, zFrame3D *f, zVec3D *a){
  return _rkJointFloatAxis( joint, f, zZ, a );
}

static zVec3D* (*_rk_joint_float_axis_ang[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointFloatAxisX,
  _rkJointFloatAxisY,
  _rkJointFloatAxisZ,
};
static zVec3D* (*_rk_joint_float_axis_lin[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointFloatAxisX,
  _rkJointFloatAxisY,
  _rkJointFloatAxisZ,
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
};

/* CRB method */

static void _rkJointFloatCRBWrench(rkJoint *joint, rkMP *crb, zVec6D wi[]){
  zMat3D icrb;
  zVec3D a;
  int i;

  rkMPOrgInertia( crb, &icrb );
  for( i=0; i<3; i++ ){
    _zMat3DRow( &_rks(joint)->_att, i, &a );
    _zVec3DMul( &a, rkMPMass(crb), zVec6DLin(&wi[i]) );
    _zVec3DOuterProd( rkMPCOM(crb), &a, zVec6DAng(&wi[i]) );
    _zVec3DMulDRC( zVec6DAng(&wi[i]), rkMPMass(crb) );
    _zVec3DRev( zVec6DAng(&wi[i]), zVec6DLin(&wi[i+3]) );
    _zMulMat3DVec3D( &icrb, &a, zVec6DAng(&wi[i+3]) );
  }
}

static void _rkJointFloatCRBXform(rkJoint *joint, zFrame3D *f, zVec6D si[]){
  zMat3D r;
  int i;

  zMulMat3DMat3DT( zFrame3DAtt(f), &_rks(joint)->_att, &r );
  for( i=0; i<3; i++ ){
    zVec3DCopy( &r.v[i], zVec6DLin(&si[i]) );
    _zVec3DZero( zVec6DAng(&si[i]) );
    _zVec3DOuterProd( zFrame3DPos(f), &r.v[i], zVec6DLin(&si[i+3]) );
    zVec3DCopy( &r.v[i], zVec6DAng(&si[i+3]) );
  }
}

static void _rkJointFloatFrictionPivot(rkJoint *joint, rkJointFrictionPivot *fp){}
static void _rkJointFloatVal(rkJoint *joint, double *val){}

/* any actuator cannot be mounted on the free-floating joint. */

static void _rkJointFloatMotorInertia(rkJoint *joint, double *val){ zMat6DZero( (zMat6D *)val ); }
static void _rkJointFloatMotorInputTrq(rkJoint *joint, double *val){ zVec6DZero( (zVec6D *)val ); }
static void _rkJointFloatMotorResistance(rkJoint *joint, double *val){ zVec6DZero( (zVec6D *)val ); }
static void _rkJointFloatMotorDrivingTrq(rkJoint *joint, double *val){ zVec6DZero( (zVec6D *)val ); }

/* ABI */

static void _rkJointFloatABIAxisInertia(rkJoint *joint, zMat6D *m, zMat h, zMat ih){
  int i, j;

  _rkJointFloatMotorInertia( joint, zMatBufNC(h) );
  for( i=0; i<3; i++ )
    for( j=0; j<3; j++ ){
      zMatElemNC(h,i,  j)   += m->e[0][0].e[j][i];
      zMatElemNC(h,i+3,j)   += m->e[0][1].e[j][i];
      zMatElemNC(h,i,  j+3) += m->e[1][0].e[j][i];
      zMatElemNC(h,i+3,j+3) += m->e[1][1].e[j][i];
    }
  zMatInv( h, ih );
}

static void _rkJointFloatABIAddABI(rkJoint *joint, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){}

static void _rkJointFloatABIAddBias(rkJoint *joint, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){}

static void _rkJointFloatABIDrivingTorque(rkJoint *joint){}

static void _rkJointFloatABIQAcc(rkJoint *joint, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  zVec6D tmpv, tmpv2;
  int i;
  /* acc */
  zVec6DRev( b, &tmpv2 );
  for( i=zX; i<=zZA; i++ )
    tmpv.e[i] = zVec6DInnerProd( (zVec6D *)&zMatElemNC(h,i,0), &tmpv2 );
  zVec6DCopy( &tmpv, acc );
  /* q */
  zVec6DSubDRC( &tmpv, jac );
  zMulMat3DVec6D( &_rks(joint)->_att, &tmpv, &_rks(joint)->acc );
}

/* ZTK */

static void *_rkJointFloatDisFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  zVec6D dis;
  zVec6DFromZTK( &dis, ztk );
  _rkJointFloatSetDis( (rkJoint *)joint, dis.e );
  return joint;
}

static bool _rkJointFloatDisFPrintZTK(FILE *fp, int i, void *joint){
  if( zVec6DIsTiny( &_rks(joint)->dis ) ) return false;
  zVec6DValueNLFPrint( fp, &_rks(joint)->dis );
  return true;
}

static const ZTKPrp __ztk_prp_rkjoint_float[] = {
  { ZTK_KEY_ROKI_JOINT_DIS, 1, _rkJointFloatDisFromZTK, _rkJointFloatDisFPrintZTK },
};

static rkJoint *_rkJointFloatFromZTK(rkJoint *joint, rkMotorSpecArray *motorspecarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorspecarray, ztk, __ztk_prp_rkjoint_float );
}

static void _rkJointFloatFPrintZTK(FILE *fp, rkJoint *joint, char *name)
{
  ZTKPrpKeyFPrint( fp, joint, __ztk_prp_rkjoint_float );
}

rkJointCom rk_joint_float = {
  "float",
  6,
  _rkJointFloatInit,
  _rkJointFloatAllocPrp,
  _rkJointFloatAllocState,
  _rkJointFloatCopyPrp,
  _rkJointFloatCopyState,
  _rkJointFloatLimDis,
  _rkJointFloatSetDis,
  _rkJointFloatSetMinMax,
  _rkJointFloatSetMinMax,
  _rkJointFloatSetVel,
  _rkJointFloatSetAcc,
  _rkJointFloatSetTrq,
  _rkJointFloatGetDis,
  _rkJointFloatGetMin,
  _rkJointFloatGetMax,
  _rkJointFloatGetVel,
  _rkJointFloatGetAcc,
  _rkJointFloatGetTrq,
  _rkJointFloatCatDis,
  _rkJointFloatSubDis,
  _rkJointFloatSetDisCNT,
  _rkJointFloatXform,
  _rkJointFloatIncVel,
  _rkJointFloatIncAccOnVel,
  _rkJointFloatIncAcc,
  _rkJointFloatCalcTrq,
  _rkJointFloatTorsion,
  _rk_joint_float_axis_ang,
  _rk_joint_float_axis_lin,

  _rkJointFloatCRBWrench,
  _rkJointFloatCRBXform,

  _rkJointFloatFrictionPivot,
  _rkJointFloatFrictionPivot,
  _rkJointFloatVal,
  _rkJointFloatVal,
  _rkJointFloatVal,
  _rkJointFloatVal,

  rkJointMotorSetValDummy,
  _rkJointFloatMotorInertia,
  _rkJointFloatMotorInputTrq,
  _rkJointFloatMotorResistance,
  _rkJointFloatMotorDrivingTrq,

  _rkJointFloatABIAxisInertia,
  _rkJointFloatABIAddABI,
  _rkJointFloatABIAddBias,
  _rkJointFloatABIDrivingTorque,
  _rkJointFloatABIQAcc,
  _rkJointUpdateWrench,

  _rkJointFloatDisFromZTK,
  _rkJointFloatFromZTK,
  _rkJointFloatDisFPrintZTK,
  _rkJointFloatFPrintZTK,
};

#undef _rks
