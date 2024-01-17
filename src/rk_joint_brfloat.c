/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_brfloat - joint structure: breakable free-floating joint
 */

#include <roki/rk_joint.h>

#define _rkc(joint) ((rkJointBrFloatPrp *)((rkJoint *)(joint))->prp)

/* ********************************************************** */
/* broken float
 * ********************************************************** */

static void _rkJointBrFloatInit(rkJoint *joint){
  _rkc(joint)->ep_f = HUGE_VAL;
  _rkc(joint)->ep_t = HUGE_VAL;
}

static void *_rkJointBrFloatAllocPrp(void){ return zAlloc( rkJointBrFloatPrp, 1 ); }

static void _rkJointBrFloatCopyPrp(rkJoint *src, rkJoint *dst){
  _rkc(dst)->ep_f = _rkc(src)->ep_f;
  _rkc(dst)->ep_t = _rkc(src)->ep_t;
}

/* limit joint displacement */
static void _rkJointBrFloatLimDis(rkJoint *joint, double *testval, double *limval){
  zVec6DCopy( (zVec6D*)testval, (zVec6D*)limval );
}

/* set joint displacement */
static void _rkJointBrFloatSetDis(rkJoint *joint, double *val){
  _rkJointBrFloatLimDis( joint, val, _rkc(joint)->dis.e );
  zMat3DFromAA( &_rkc(joint)->_att, zVec6DAng(&_rkc(joint)->dis) );
}

static void _rkJointBrFloatSetMinMax(rkJoint *joint, double *val){}

static void _rkJointBrFloatSetVel(rkJoint *joint, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(joint)->vel );
}

static void _rkJointBrFloatSetAcc(rkJoint *joint, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(joint)->acc );
}

static void _rkJointBrFloatSetTrq(rkJoint *joint, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(joint)->trq );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointBrFloatGetDis(rkJoint *joint, double *val){
  zVec6DCopy( &_rkc(joint)->dis, (zVec6D*)val );
}

static void _rkJointBrFloatGetMin(rkJoint *joint, double *val){
  val[0] = val[1] = val[2] = val[3] = val[4] = val[5] = -HUGE_VAL;
}

static void _rkJointBrFloatGetMax(rkJoint *joint, double *val){
  val[0] = val[1] = val[2] = val[3] = val[4] = val[5] = HUGE_VAL;
}

static void _rkJointBrFloatGetVel(rkJoint *joint, double *val){
  zVec6DCopy( &_rkc(joint)->vel, (zVec6D*)val );
}

static void _rkJointBrFloatGetAcc(rkJoint *joint, double *val){
  zVec6DCopy( &_rkc(joint)->acc, (zVec6D*)val );
}

static void _rkJointBrFloatGetTrq(rkJoint *joint, double *val){
  zVec6DCopy( &_rkc(joint)->trq, (zVec6D*)val );
}

static void _rkJointBrFloatCatDis(rkJoint *joint, double *dis, double k, double *val){
  zVec3D d, aa;
  /* concatenate position */
  zVec3DCatDRC( (zVec3D*)&dis[0], k, (zVec3D*)&val[0] );
  /* concatenate attitude */
  zVec3DCopy( (zVec3D*)&dis[3], &aa );
  zVec3DMul( (zVec3D*)&val[3], k, &d );
  zAACascade( &aa, &d, (zVec3D*)&dis[3] );
}

static void _rkJointBrFloatSubDis(rkJoint *joint, double *dis, double *sdis){
  zMat3D m, ms;
  zVec3D aa;

  zVec3DSubDRC( zVec6DLin((zVec6D*)dis), zVec6DLin((zVec6D*)sdis) );
  zVec3DCopy( zVec6DAng((zVec6D*)dis), &aa );
  zMat3DFromAA( &m, &aa );
  zMat3DFromAA( &ms, zVec6DAng((zVec6D*)sdis) );
  zMat3DError( &m, &ms, zVec6DAng((zVec6D*)dis) );
}

/* continuously update joint displacement over delta time */
static void _rkJointBrFloatSetDisCNT(rkJoint *joint, double *val, double dt){
  zMat3D m_old;
  zVec3D p_old;
  zVec6D v_old;

  /* previous state */
  zVec3DCopy( zVec6DLin(&_rkc(joint)->dis), &p_old );
  zMat3DCopy( &_rkc(joint)->_att, &m_old );
  _rkJointBrFloatGetVel( joint, v_old.e );
  /* update displacement */
  _rkJointBrFloatSetDis( joint, val );
  /* numerical differentiation */
  zVec3DDif( &p_old, zVec6DLin(&_rkc(joint)->dis), dt, zVec6DLin(&_rkc(joint)->vel) );
  zMat3DError( &_rkc(joint)->_att, &m_old, zVec6DAng(&_rkc(joint)->vel) );
  zVec3DDivDRC( zVec6DAng(&_rkc(joint)->vel), dt );
  zVec6DDif( &v_old, &_rkc(joint)->vel, dt, &_rkc(joint)->acc );
}

/* joint frame transfer function */
static zFrame3D *_rkJointBrFloatXform(rkJoint *joint, zFrame3D *fo, zFrame3D *f){
  /* position */
  zXform3D( fo, zVec6DLin(&_rkc(joint)->dis), zFrame3DPos(f) );
  /* attitude */
  zMulMat3DMat3D( zFrame3DAtt(fo), &_rkc(joint)->_att, zFrame3DAtt(f) );
  return f;
}

/* joint velocity transfer function */
static void _rkJointBrFloatIncVel(rkJoint *joint, zVec6D *vel){
  zVec6D vl;
  zMulMat3DTVec6D( &_rkc(joint)->_att, &_rkc(joint)->vel, &vl );
  zVec6DAddDRC( vel, &vl );
}

static void _rkJointBrFloatIncAccOnVel(rkJoint *joint, zVec3D *w, zVec6D *acc){
  zVec6D vl;
  zVec3D tmp;

  /* FIXME: _att -> ^pR_j */
  zMulMat3DTVec6D( &_rkc(joint)->_att, &_rkc(joint)->vel, &vl );
  zVec3DOuterProd( w, zVec6DLin(&vl), &tmp );
  zVec3DCatDRC( zVec6DLin(acc), 2, &tmp );
  zVec3DOuterProd( w, zVec6DAng(&vl), &tmp );
  zVec3DAddDRC( zVec6DAng(acc), &tmp );
}

/* joint acceleration transfer function */
static void _rkJointBrFloatIncAcc(rkJoint *joint, zVec6D *acc){
  zVec6D al;
  zMulMat3DTVec6D( &_rkc(joint)->_att, &_rkc(joint)->acc, &al );
  zVec6DAddDRC( acc, &al );
}

/* joint torque transfer function */
static void _rkJointBrFloatCalcTrq(rkJoint *joint, zVec6D *f){
  zMulMat3DVec6D( &_rkc(joint)->_att, f, &_rkc(joint)->trq );
}

/* inverse computation of joint torsion and displacement */
static void _rkJointBrFloatTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  zVec6DZero( t );
  zFrame3DToVec6DAA( dev, (zVec6D*)dis );
}

/* joint axis function */

static zVec3D *_rkJointBrFloatAxis(rkJoint *joint, zFrame3D *f, zDir dir, zVec3D *a){
  zVec3D al;
  zMat3DRow( &_rkc(joint)->_att, dir, &al );
  return zMulMat3DVec3D( zFrame3DAtt(f), &al, a );
}
static zVec3D *_rkJointBrFloatAxisX(rkJoint *joint, zFrame3D *f, zVec3D *a){
  return _rkJointBrFloatAxis( joint, f, zX, a );
}
static zVec3D *_rkJointBrFloatAxisY(rkJoint *joint, zFrame3D *f, zVec3D *a){
  return _rkJointBrFloatAxis( joint, f, zY, a );
}
static zVec3D *_rkJointBrFloatAxisZ(rkJoint *joint, zFrame3D *f, zVec3D *a){
  return _rkJointBrFloatAxis( joint, f, zZ, a );
}
static zVec3D* (*_rk_joint_float_axis_ang[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointBrFloatAxisX,
  _rkJointBrFloatAxisY,
  _rkJointBrFloatAxisZ,
};
static zVec3D* (*_rk_joint_float_axis_lin[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointBrFloatAxisX,
  _rkJointBrFloatAxisY,
  _rkJointBrFloatAxisZ,
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
};

/* CRB method */

static void _rkJointBrFloatCRBWrench(rkJoint *joint, rkMP *crb, zVec6D wi[]){
  zMat3D icrb;
  zVec3D a;
  int i;

  rkMPOrgInertia( crb, &icrb );
  for( i=0; i<3; i++ ){
    _zMat3DRow( &_rkc(joint)->_att, i, &a );
    _zVec3DMul( &a, rkMPMass(crb), zVec6DLin(&wi[i]) );
    _zVec3DOuterProd( rkMPCOM(crb), &a, zVec6DAng(&wi[i]) );
    _zVec3DMulDRC( zVec6DAng(&wi[i]), rkMPMass(crb) );
    _zVec3DRev( zVec6DAng(&wi[i]), zVec6DLin(&wi[i+3]) );
    _zMulMat3DVec3D( &icrb, &a, zVec6DAng(&wi[i+3]) );
  }
}
static void _rkJointBrFloatCRBXform(rkJoint *joint, zFrame3D *f, zVec6D si[]){
  zMat3D r;
  int i;

  zMulMat3DMat3DT( zFrame3DAtt(f), &_rkc(joint)->_att, &r );
  for( i=0; i<3; i++ ){
    zVec3DCopy( &r.v[i], zVec6DLin(&si[i]) );
    _zVec3DZero( zVec6DAng(&si[i]) );
    _zVec3DOuterProd( zFrame3DPos(f), &r.v[i], zVec6DLin(&si[i+3]) );
    zVec3DCopy( &r.v[i], zVec6DAng(&si[i+3]) );
  }
}

static void _rkJointBrFloatFrictionPivot(rkJoint *joint, rkJointFrictionPivot *fp){}
static void _rkJointBrFloatVal(rkJoint *joint, double *val){}

/* motor */

/* any actuator cannot be mounted on the free-floating joint. */
static rkMotor *_rkJointBrFloatGetMotor(rkJoint *joint){ return NULL; }

static void _rkJointBrFloatMotorSetInput(rkJoint *joint, double *val){}
static void _rkJointBrFloatMotorInertia(rkJoint *joint, double *val){ zMat6DZero( (zMat6D *)val ); }
static void _rkJointBrFloatMotorInputTrq(rkJoint *joint, double *val){ zVec6DZero( (zVec6D *)val ); }
static void _rkJointBrFloatMotorResistance(rkJoint *joint, double *val){ zVec6DZero( (zVec6D *)val ); }
static void _rkJointBrFloatMotorDrivingTrq(rkJoint *joint, double *val){ zVec6DZero( (zVec6D *)val ); }

/* ABI (for breakable joints) */

static void _rkJointBrFloatABIAxisInertia(rkJoint *joint, zMat6D *m, zMat h, zMat ih){
  int i, j;
  _rkJointBrFloatMotorInertia( joint, zMatBufNC(h) );
  for( i=0; i<3; i++ )
    for( j=0; j<3; j++ ){
      zMatElemNC(h,i,  j)   += m->e[0][0].e[j][i];
      zMatElemNC(h,i+3,j)   += m->e[0][1].e[j][i];
      zMatElemNC(h,i,  j+3) += m->e[1][0].e[j][i];
      zMatElemNC(h,i+3,j+3) += m->e[1][1].e[j][i];
    }
  zMatInv( h, ih );
}

static void _rkJointBrFloatABIAddABI(rkJoint *joint, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){}
static void _rkJointBrFloatABIAddBias(rkJoint *joint, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){}
static void _rkJointBrFloatABIDrivingTorque(rkJoint *joint){}
static void _rkJointBrFloatABIQAcc(rkJoint *joint, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  zVec6D tmpv, tmpv2;
  int i;
  /* acc */
  zVec6DRev( b, &tmpv2 );
  for( i=zX; i<=zZA; i++ )
    tmpv.e[i] = zVec6DInnerProd( (zVec6D *)&zMatElemNC(h,i,0), &tmpv2 );
  zVec6DCopy( &tmpv, acc );
  /* q */
  zVec6DSubDRC( &tmpv, jac );
  zMulMat3DVec6D( &_rkc(joint)->_att, &tmpv, &_rkc(joint)->acc );
}

/* ABI (for unbreakable joints) */

static void _rkJointBrFloatFixedABIAxisInertia(rkJoint *joint, zMat6D *m, zMat h, zMat ih){}
static void _rkJointBrFloatFixedABIAddABI(rkJoint *joint, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  zMat6D tmpm;
  rkJointXformMat6D( f, m, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}
static void _rkJointBrFloatFixedABIAddBias(rkJoint *joint, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  zVec6D tmpv;
  zMulMat3DVec6D( zFrame3DAtt(f), b, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}
static void _rkJointBrFloatFixedABIDrivingTorque(rkJoint *joint){}
static void _rkJointBrFloatFixedABIQAcc(rkJoint *joint, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  zVec6DCopy( jac, acc );
}
static void _rkJointBrFloatFixedUpdateWrench(rkJoint *joint, zMat6D *i, zVec6D *b, zVec6D *acc){
  _rkJointUpdateWrench( joint, i, b, acc );
  if( ( zVec3DNorm( zVec6DLin(rkJointWrench(joint)) ) > _rkc(joint)->ep_f ) ||
      ( zVec3DNorm( zVec6DAng(rkJointWrench(joint)) ) > _rkc(joint)->ep_t ) ){
    joint->com->_axinertia = _rkJointBrFloatFixedABIAxisInertia;
    joint->com->_addabi = _rkJointBrFloatFixedABIAddABI;
    joint->com->_addbias = _rkJointBrFloatFixedABIAddBias;
    joint->com->_dtrq = _rkJointBrFloatFixedABIDrivingTorque;
    joint->com->_qacc = _rkJointBrFloatFixedABIQAcc;
  }
}

/* ZTK */

static void *_rkJointBrFloatDisFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  zVec6D dis;
  zVec6DFromZTK( &dis, ztk );
  _rkJointBrFloatSetDis( joint, dis.e );
  return joint;
}
static void *_rkJointBrFloatForceThFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->ep_f = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointBrFloatTorqueThFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkc(joint)->ep_t = ZTKDouble(ztk);
  return joint;
}

static void _rkJointBrFloatDisFPrintZTK(FILE *fp, int i, void *joint){
  zVec6DDataNLFPrint( fp, &_rkc(joint)->dis );
}
static void _rkJointBrFloatForceThFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkc(joint)->ep_f );
}
static void _rkJointBrFloatTorqueThFPrintZTK(FILE *fp, int i, void *joint){
  fprintf( fp, "%.10g\n", _rkc(joint)->ep_t );
}

static ZTKPrp __ztk_prp_rkjoint_brfloat[] = {
  { "dis", 1, _rkJointBrFloatDisFromZTK, _rkJointBrFloatDisFPrintZTK },
  { "forcethreshold", 1, _rkJointBrFloatForceThFromZTK, _rkJointBrFloatForceThFPrintZTK },
  { "torquethreshold", 1, _rkJointBrFloatTorqueThFromZTK, _rkJointBrFloatTorqueThFPrintZTK },
};

static rkJoint *_rkJointBrFloatFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorarray, ztk, __ztk_prp_rkjoint_brfloat );
}

static void _rkJointBrFloatFPrintZTK(FILE *fp, rkJoint *joint, char *name)
{
  ZTKPrpKeyFPrint( fp, joint, __ztk_prp_rkjoint_brfloat );
}

rkJointCom rk_joint_brfloat = {
  "breakablefloat",
  6,
  _rkJointBrFloatInit,
  _rkJointBrFloatAllocPrp,
  _rkJointBrFloatCopyPrp,
  _rkJointBrFloatLimDis,
  _rkJointBrFloatSetDis,
  _rkJointBrFloatSetMinMax,
  _rkJointBrFloatSetMinMax,
  _rkJointBrFloatSetVel,
  _rkJointBrFloatSetAcc,
  _rkJointBrFloatSetTrq,
  _rkJointBrFloatGetDis,
  _rkJointBrFloatGetMin,
  _rkJointBrFloatGetMax,
  _rkJointBrFloatGetVel,
  _rkJointBrFloatGetAcc,
  _rkJointBrFloatGetTrq,
  _rkJointBrFloatCatDis,
  _rkJointBrFloatSubDis,
  _rkJointBrFloatSetDisCNT,
  _rkJointBrFloatXform,
  _rkJointBrFloatIncVel,
  _rkJointBrFloatIncAccOnVel,
  _rkJointBrFloatIncAcc,
  _rkJointBrFloatCalcTrq,
  _rkJointBrFloatTorsion,
  _rk_joint_float_axis_ang,
  _rk_joint_float_axis_lin,

  _rkJointBrFloatCRBWrench,
  _rkJointBrFloatCRBXform,

  _rkJointBrFloatFrictionPivot,
  _rkJointBrFloatFrictionPivot,
  _rkJointBrFloatVal,
  _rkJointBrFloatVal,
  _rkJointBrFloatVal,
  _rkJointBrFloatVal,

  _rkJointBrFloatGetMotor,
  _rkJointBrFloatMotorSetInput,
  _rkJointBrFloatMotorInertia,
  _rkJointBrFloatMotorInputTrq,
  _rkJointBrFloatMotorResistance,
  _rkJointBrFloatMotorDrivingTrq,

  _rkJointBrFloatABIAxisInertia,
  _rkJointBrFloatABIAddABI,
  _rkJointBrFloatABIAddBias,
  _rkJointBrFloatABIDrivingTorque,
  _rkJointBrFloatABIQAcc,
  _rkJointBrFloatFixedUpdateWrench,

  _rkJointBrFloatDisFromZTK,
  _rkJointBrFloatFromZTK,
  _rkJointBrFloatDisFPrintZTK,
  _rkJointBrFloatFPrintZTK,
};

#undef _rkc
