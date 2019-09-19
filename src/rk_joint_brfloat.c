/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_brfloat - joint structure: breakable free-floating joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointBrFloatPrp *)p)

/* ********************************************************** */
/* broken float
 * ********************************************************** */

static void _rkJointBrFloatInit(void *prp){
  _rkc(prp)->ep_f = HUGE_VAL;
  _rkc(prp)->ep_t = HUGE_VAL;
}

static void *_rkJointBrFloatAlloc(void){
  return zAlloc( rkJointBrFloatPrp, 1 );
}

/* limit joint displacement */
static void _rkJointBrFloatLimDis(void *prp, double *testval, double *limval){
  zVec6DCopy( (zVec6D*)testval, (zVec6D*)limval );
}

/* set joint displacement */
static void _rkJointBrFloatSetDis(void *prp, double *val){
  _rkJointBrFloatLimDis( prp, val, _rkc(prp)->dis.e );
  zMat3DFromAA( &_rkc(prp)->_att, zVec6DAng(&_rkc(prp)->dis) );
}

static void _rkJointBrFloatSetVel(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->vel );
}

static void _rkJointBrFloatSetAcc(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->acc );
}

static void _rkJointBrFloatSetTrq(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->trq );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointBrFloatGetDis(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->dis, (zVec6D*)val );
}

static void _rkJointBrFloatGetVel(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->vel, (zVec6D*)val );
}

static void _rkJointBrFloatGetAcc(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->acc, (zVec6D*)val );
}

static void _rkJointBrFloatGetTrq(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->trq, (zVec6D*)val );
}

static void _rkJointBrFloatCatDis(void *prp, double *dis, double k, double *val){
  zVec3D d, aa;
  /* concatenate position */
  zVec3DCatDRC( (zVec3D*)&dis[0], k, (zVec3D*)&val[0] );
  /* concatenate attitude */
  zVec3DCopy( (zVec3D*)&dis[3], &aa );
  zVec3DMul( (zVec3D*)&val[3], k, &d );
  zAACascade( &aa, &d, (zVec3D*)&dis[3] );
}

static void _rkJointBrFloatSubDis(void *prp, double *dis, double *sdis){
  zMat3D m, ms;
  zVec3D aa;

  zVec3DSubDRC( zVec6DLin((zVec6D*)dis), zVec6DLin((zVec6D*)sdis) );
  zVec3DCopy( zVec6DAng((zVec6D*)dis), &aa );
  zMat3DFromAA( &m, &aa );
  zMat3DFromAA( &ms, zVec6DAng((zVec6D*)sdis) );
  zMat3DError( &m, &ms, zVec6DAng((zVec6D*)dis) );
}

/* continuously update joint displacement over delta time */
static void _rkJointBrFloatSetDisCNT(void *prp, double *val, double dt){
  zMat3D m_old;
  zVec3D p_old;
  zVec6D v_old;

  /* previous state */
  zVec3DCopy( zVec6DLin(&_rkc(prp)->dis), &p_old );
  zMat3DCopy( &_rkc(prp)->_att, &m_old );
  _rkJointBrFloatGetVel( prp, v_old.e );
  /* update displacement */
  _rkJointBrFloatSetDis( prp, val );
  /* numerical differentiation */
  zVec3DDif( &p_old, zVec6DLin(&_rkc(prp)->dis), dt, zVec6DLin(&_rkc(prp)->vel) );
  zMat3DError( &_rkc(prp)->_att, &m_old, zVec6DAng(&_rkc(prp)->vel) );
  zVec3DDivDRC( zVec6DAng(&_rkc(prp)->vel), dt );
  zVec6DDif( &v_old, &_rkc(prp)->vel, dt, &_rkc(prp)->acc );
}

/* joint frame transfer function */
static zFrame3D *_rkJointBrFloatXform(void *prp, zFrame3D *fo, zFrame3D *f){
  /* position */
  zXform3D( fo, zVec6DLin(&_rkc(prp)->dis), zFrame3DPos(f) );
  /* attitude */
  zMulMat3DMat3D( zFrame3DAtt(fo), &_rkc(prp)->_att, zFrame3DAtt(f) );
  return f;
}

/* joint velocity transfer function */
static void _rkJointBrFloatIncVel(void *prp, zVec6D *vel){
  zVec6D vl;
  zMulMat3DTVec6D( &_rkc(prp)->_att, &_rkc(prp)->vel, &vl );
  zVec6DAddDRC( vel, &vl );
}

static void _rkJointBrFloatIncAccOnVel(void *prp, zVec3D *w, zVec6D *acc){
  zVec6D vl;
  zVec3D tmp;

  /* FIXME: _att -> ^pR_j */
  zMulMat3DTVec6D( &_rkc(prp)->_att, &_rkc(prp)->vel, &vl );
  zVec3DOuterProd( w, zVec6DLin(&vl), &tmp );
  zVec3DCatDRC( zVec6DLin(acc), 2, &tmp );
  zVec3DOuterProd( w, zVec6DAng(&vl), &tmp );
  zVec3DAddDRC( zVec6DAng(acc), &tmp );
}

/* joint acceleration transfer function */
static void _rkJointBrFloatIncAcc(void *prp, zVec6D *acc){
  zVec6D al;
  zMulMat3DTVec6D( &_rkc(prp)->_att, &_rkc(prp)->acc, &al );
  zVec6DAddDRC( acc, &al );
}

/* joint torque transfer function */
static void _rkJointBrFloatCalcTrq(void *prp, zVec6D *f){
  zMulMat3DVec6D( &_rkc(prp)->_att, f, &_rkc(prp)->trq );
}

/* inverse computation of joint torsion and displacement */
static void _rkJointBrFloatTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  zVec6DZero( t );
  zFrame3DToVec6DAA( dev, (zVec6D*)dis );
}

/* joint axis function */
static zVec3D *_rkJointBrFloatAxis(void *prp, zFrame3D *f, zDir dir, zVec3D *a){
  zVec3D al;
  zMat3DRow( &_rkc(prp)->_att, dir, &al );
  return zMulMat3DVec3D( zFrame3DAtt(f), &al, a );
}
static zVec3D *_rkJointBrFloatAxisX(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointBrFloatAxis( prp, f, zX, a );
}
static zVec3D *_rkJointBrFloatAxisY(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointBrFloatAxis( prp, f, zY, a );
}
static zVec3D *_rkJointBrFloatAxisZ(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointBrFloatAxis( prp, f, zZ, a );
}
static zVec3D* (*_rk_joint_float_axis_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointBrFloatAxisX,
  _rkJointBrFloatAxisY,
  _rkJointBrFloatAxisZ,
};
static zVec3D* (*_rk_joint_float_axis_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointBrFloatAxisX,
  _rkJointBrFloatAxisY,
  _rkJointBrFloatAxisZ,
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
};

static void _rkJointBrFloatFrictionPivot(void *prp, rkJointFrictionPivot *fp){}
static void _rkJointBrFloatVal(void *prp, double *val){}

/* any actuator cannot be mounted on the free-floating joint. */
static rkMotor *_rkJointBrFloatGetMotor(void *prp){ return NULL; }

/* motor */
static void _rkJointBrFloatMotorSetInput(void *prp, double *val){}
static void _rkJointBrFloatMotorInertia(void *prp, double *val){ zMat6DZero( (zMat6D *)val ); }
static void _rkJointBrFloatMotorInputTrq(void *prp, double *val){ zVec6DZero( (zVec6D *)val ); }
static void _rkJointBrFloatMotorResistance(void *prp, double *val){ zVec6DZero( (zVec6D *)val ); }
static void _rkJointBrFloatMotorDrivingTrq(void *prp, double *val){ zVec6DZero( (zVec6D *)val ); }

/* ABI for breakable joints */
static void _rkJointBrFloatABIAxisInertia(void *prp, zMat6D *m, zMat h, zMat ih){
  register int i, j;
  _rkJointBrFloatMotorInertia( prp, zMatBuf(h) );
  for( i=0; i<3; i++ )
    for( j=0; j<3; j++ ){
      zMatElemNC(h,i,  j)   += m->e[0][0].e[j][i];
      zMatElemNC(h,i+3,j)   += m->e[0][1].e[j][i];
      zMatElemNC(h,i,  j+3) += m->e[1][0].e[j][i];
      zMatElemNC(h,i+3,j+3) += m->e[1][1].e[j][i];
    }
  zMatInv( h, ih );
}

static void _rkJointBrFloatABIAddABI(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){}
static void _rkJointBrFloatABIAddBias(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){}
static void _rkJointBrFloatABIDrivingTorque(void *prp){}
static void _rkJointBrFloatABIQAcc(void *prp, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  zVec6D tmpv, tmpv2;
  register int i;
  /* acc */
  zVec6DRev( b, &tmpv2 );
  for( i=zX; i<=zZA; i++ )
    tmpv.e[i] = zVec6DInnerProd( (zVec6D *)&zMatElemNC(h,i,0), &tmpv2 );
  zVec6DCopy( &tmpv, acc );
  /* q */
  zVec6DSubDRC( &tmpv, jac );
  zMulMat3DVec6D( &_rkc(prp)->_att, &tmpv, &_rkc(prp)->acc );
}

/* ABI for unbreakable joints */

static void _rkJointBrFloatFixedABIAxisInertia(void *prp, zMat6D *m, zMat h, zMat ih){}
static void _rkJointBrFloatFixedABIAddABI(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  zMat6D tmpm;
  rkJointXformMat6D( f, m, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}
static void _rkJointBrFloatFixedABIAddBias(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  zVec6D tmpv;
  zMulMat3DVec6D( zFrame3DAtt(f), b, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}
static void _rkJointBrFloatFixedABIDrivingTorque(void *prp){}
static void _rkJointBrFloatFixedABIQAcc(void *prp, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  zVec6DCopy( jac, acc );
}
static void _rkJointBrFloatFixedUpdateWrench(rkJoint *j, zMat6D *i, zVec6D *b, zVec6D *acc){
  _rkJointUpdateWrench( j, i, b, acc );
  if( ( zVec3DNorm( zVec6DLin(rkJointWrench(j)) ) > _rkc(j->prp)->ep_f ) ||
      ( zVec3DNorm( zVec6DAng(rkJointWrench(j)) ) > _rkc(j->prp)->ep_t ) ){
    j->com->_axinertia = _rkJointBrFloatFixedABIAxisInertia;
    j->com->_addabi = _rkJointBrFloatFixedABIAddABI;
    j->com->_addbias = _rkJointBrFloatFixedABIAddBias;
    j->com->_dtrq = _rkJointBrFloatFixedABIDrivingTorque;
    j->com->_qacc = _rkJointBrFloatFixedABIQAcc;
    j->com->_wrench = _rkJointBrFloatFixedUpdateWrench; /* unnecessary */
  }
}

static void *_rkJointBrFloatDisFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  zVec6D dis;
  zVec6DFromZTK( &dis, ztk );
  _rkJointBrFloatSetDis( prp, dis.e );
  return prp;
}
static void *_rkJointBrFloatForceThFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->ep_f = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointBrFloatTorqueThFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->ep_t = ZTKDouble(ztk);
  return prp;
}

static void _rkJointBrFloatDisFPrintZTK(FILE *fp, int i, void *prp){
  zVec6DDataNLFPrint( fp, &_rkc(prp)->dis );
}
static void _rkJointBrFloatForceThFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->ep_f );
}
static void _rkJointBrFloatTorqueThFPrintZTK(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->ep_t );
}

static ZTKPrp __ztk_prp_rkjoint_brfloat[] = {
  { "dis", 1, _rkJointBrFloatDisFromZTK, _rkJointBrFloatDisFPrintZTK },
  { "forcethreshold", 1, _rkJointBrFloatForceThFromZTK, _rkJointBrFloatForceThFPrintZTK },
  { "torquethreshold", 1, _rkJointBrFloatTorqueThFromZTK, _rkJointBrFloatTorqueThFPrintZTK },
};

static bool _rkJointBrFloatRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_brfloat ) ? true : false;
}

static void *_rkJointBrFloatFromZTK(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_brfloat );
}

static void _rkJointBrFloatFPrintZTK(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_brfloat );
}

rkJointCom rk_joint_brfloat = {
  "breakablefloat",
  6,
  _rkJointBrFloatInit,
  _rkJointBrFloatAlloc,
  _rkJointBrFloatLimDis,
  _rkJointBrFloatSetDis,
  _rkJointBrFloatSetVel,
  _rkJointBrFloatSetAcc,
  _rkJointBrFloatSetTrq,
  _rkJointBrFloatGetDis,
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

  _rkJointBrFloatRegZTK,
  _rkJointBrFloatDisFromZTK,
  _rkJointBrFloatFromZTK,
  _rkJointBrFloatDisFPrintZTK,
  _rkJointBrFloatFPrintZTK,
};

#undef _rkc
