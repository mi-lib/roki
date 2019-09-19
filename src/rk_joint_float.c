/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_float - joint structure: free-floating joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointFloatPrp *)p)

static void _rkJointFloatInit(void *prp){}

static void *_rkJointFloatAlloc(void){ return zAlloc( rkJointFloatPrp, 1 ); }

/* limit joint displacement */
static void _rkJointFloatLimDis(void *prp, double *testval, double *limval){
  zVec6DCopy( (zVec6D*)testval, (zVec6D*)limval );
}

/* joint displacement set function */
static void _rkJointFloatSetDis(void *prp, double *val){
  _rkJointFloatLimDis( prp, val, _rkc(prp)->dis.e );
  zMat3DFromAA( &_rkc(prp)->_att, zVec6DAng(&_rkc(prp)->dis) );
}

static void _rkJointFloatSetVel(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->vel );
}

static void _rkJointFloatSetAcc(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->acc );
}

static void _rkJointFloatSetTrq(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->trq );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointFloatGetDis(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->dis, (zVec6D*)val );
}

static void _rkJointFloatGetVel(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->vel, (zVec6D*)val );
}

static void _rkJointFloatGetAcc(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->acc, (zVec6D*)val );
}

static void _rkJointFloatGetTrq(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->trq, (zVec6D*)val );
}

static void _rkJointFloatCatDis(void *prp, double *dis, double k, double *val){
  zVec3D d, aa;
  /* concatenate position */
  zVec3DCatDRC( (zVec3D*)&dis[0], k, (zVec3D*)&val[0] );
  /* concatenate attitude */
  zVec3DCopy( (zVec3D*)&dis[3], &aa );
  zVec3DMul( (zVec3D*)&val[3], k, &d );
  zAACascade( &aa, &d, (zVec3D*)&dis[3] );
}

static void _rkJointFloatSubDis(void *prp, double *dis, double *sdis){
  zMat3D m, ms;
  zVec3D aa;

  zVec3DSubDRC( zVec6DLin((zVec6D*)dis), zVec6DLin((zVec6D*)sdis) );
  zVec3DCopy( zVec6DAng((zVec6D*)dis), &aa );
  zMat3DFromAA( &m, &aa );
  zMat3DFromAA( &ms, zVec6DAng((zVec6D*)sdis) );
  zMat3DError( &m, &ms, zVec6DAng((zVec6D*)dis) );
}

/* continuously update joint displacement */
static void _rkJointFloatSetDisCNT(void *prp, double *val, double dt){
  zMat3D m_old;
  zVec3D p_old;
  zVec6D v_old;

  /* previous state */
  zVec3DCopy( zVec6DLin(&_rkc(prp)->dis), &p_old );
  zMat3DCopy( &_rkc(prp)->_att, &m_old );
  _rkJointFloatGetVel( prp, v_old.e );
  /* update displacement */
  _rkJointFloatSetDis( prp, val );
  /* numerical differentiation */
  zVec3DDif( &p_old, zVec6DLin(&_rkc(prp)->dis), dt, zVec6DLin(&_rkc(prp)->vel) );
  zMat3DError( &_rkc(prp)->_att, &m_old, zVec6DAng(&_rkc(prp)->vel) );
  zVec3DDivDRC( zVec6DAng(&_rkc(prp)->vel), dt );
  zVec6DDif( &v_old, &_rkc(prp)->vel, dt, &_rkc(prp)->acc );
}

/* joint frame transformation */
static zFrame3D *_rkJointFloatXform(void *prp, zFrame3D *fo, zFrame3D *f){
  /* position */
  zXform3D( fo, zVec6DLin(&_rkc(prp)->dis), zFrame3DPos(f) );
  /* attitude */
  zMulMat3DMat3D( zFrame3DAtt(fo), &_rkc(prp)->_att, zFrame3DAtt(f) );
  return f;
}

/* joint velocity transformation */
static void _rkJointFloatIncVel(void *prp, zVec6D *vel){
  zVec6D vl;
  zMulMat3DTVec6D( &_rkc(prp)->_att, &_rkc(prp)->vel, &vl );
  zVec6DAddDRC( vel, &vl );
}

static void _rkJointFloatIncAccOnVel(void *prp, zVec3D *w, zVec6D *acc){
  zVec6D vl;
  zVec3D tmp;
  /* FIXME: _att -> ^pR_j */
  zMulMat3DTVec6D( &_rkc(prp)->_att, &_rkc(prp)->vel, &vl );
  zVec3DOuterProd( w, zVec6DLin(&vl), &tmp );
  zVec3DCatDRC( zVec6DLin(acc), 2, &tmp );
  zVec3DOuterProd( w, zVec6DAng(&vl), &tmp );
  zVec3DAddDRC( zVec6DAng(acc), &tmp );
}

/* joint acceleration transformation */
static void _rkJointFloatIncAcc(void *prp, zVec6D *acc){
  zVec6D al;
  zMulMat3DTVec6D( &_rkc(prp)->_att, &_rkc(prp)->acc, &al );
  zVec6DAddDRC( acc, &al );
}

/* joint torque transformation */
static void _rkJointFloatCalcTrq(void *prp, zVec6D *f){
  zMulMat3DVec6D( &_rkc(prp)->_att, f, &_rkc(prp)->trq );
}

/* inverse computation of joint torsion and displacement */
static void _rkJointFloatTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  zVec6DZero( t );
  zFrame3DToVec6DAA( dev, (zVec6D*)dis );
}

/* joint axes */
static zVec3D *_rkJointFloatAxis(void *prp, zFrame3D *f, zDir dir, zVec3D *a){
  zVec3D al;
  zMat3DRow( &_rkc(prp)->_att, dir, &al );
  return zMulMat3DVec3D( zFrame3DAtt(f), &al, a );
}

static zVec3D *_rkJointFloatAxisX(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointFloatAxis( prp, f, zX, a );
}

static zVec3D *_rkJointFloatAxisY(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointFloatAxis( prp, f, zY, a );
}

static zVec3D *_rkJointFloatAxisZ(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointFloatAxis( prp, f, zZ, a );
}

static zVec3D* (*_rk_joint_float_axis_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointFloatAxisX,
  _rkJointFloatAxisY,
  _rkJointFloatAxisZ,
};
static zVec3D* (*_rk_joint_float_axis_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointFloatAxisX,
  _rkJointFloatAxisY,
  _rkJointFloatAxisZ,
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
};

static void _rkJointFloatFrictionPivot(void *prp, rkJointFrictionPivot *fp){}
static void _rkJointFloatVal(void *prp, double *val){}

/* any actuator cannot be mounted on the free-floating joint. */
static rkMotor *_rkJointFloatGetMotor(void *prp){ return NULL; }

static void _rkJointFloatMotorSetInput(void *prp, double *val){}
static void _rkJointFloatMotorInertia(void *prp, double *val){ zMat6DZero( (zMat6D *)val ); }
static void _rkJointFloatMotorInputTrq(void *prp, double *val){ zVec6DZero( (zVec6D *)val ); }
static void _rkJointFloatMotorResistance(void *prp, double *val){ zVec6DZero( (zVec6D *)val ); }
static void _rkJointFloatMotorDrivingTrq(void *prp, double *val){ zVec6DZero( (zVec6D *)val ); }

/* ABI */
static void _rkJointFloatABIAxisInertia(void *prp, zMat6D *m, zMat h, zMat ih){
  register int i, j;

  _rkJointFloatMotorInertia( prp, zMatBuf(h) );
  for( i=0; i<3; i++ )
    for( j=0; j<3; j++ ){
      zMatElemNC(h,i,  j)   += m->e[0][0].e[j][i];
      zMatElemNC(h,i+3,j)   += m->e[0][1].e[j][i];
      zMatElemNC(h,i,  j+3) += m->e[1][0].e[j][i];
      zMatElemNC(h,i+3,j+3) += m->e[1][1].e[j][i];
    }
  zMatInv( h, ih );
}

static void _rkJointFloatABIAddABI(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){}

static void _rkJointFloatABIAddBias(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){}

static void _rkJointFloatABIDrivingTorque(void *prp){}

static void _rkJointFloatABIQAcc(void *prp, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
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

static void *_rkJointFloatDisFromZTK(void *prp, int i, void *arg, ZTK *ztk){
  zVec6D dis;
  zVec6DFromZTK( &dis, ztk );
  _rkJointFloatSetDis( prp, dis.e );
  return prp;
}

static void _rkJointFloatDisFPrintZTK(FILE *fp, int i, void *prp){
  zVec6DDataNLFPrint( fp, &_rkc(prp)->dis );
}

static ZTKPrp __ztk_prp_rkjoint_float[] = {
  { "dis", 1, _rkJointFloatDisFromZTK, _rkJointFloatDisFPrintZTK },
};

static bool _rkJointFloatRegZTK(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_float ) ? true : false;
}

static void *_rkJointFloatFromZTK(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_float );
}

static void _rkJointFloatFPrintZTK(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_float );
}

rkJointCom rk_joint_float = {
  "float",
  6,
  _rkJointFloatInit,
  _rkJointFloatAlloc,
  _rkJointFloatLimDis,
  _rkJointFloatSetDis,
  _rkJointFloatSetVel,
  _rkJointFloatSetAcc,
  _rkJointFloatSetTrq,
  _rkJointFloatGetDis,
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

  _rkJointFloatFrictionPivot,
  _rkJointFloatFrictionPivot,
  _rkJointFloatVal,
  _rkJointFloatVal,
  _rkJointFloatVal,
  _rkJointFloatVal,

  _rkJointFloatGetMotor,
  _rkJointFloatMotorSetInput,
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

  _rkJointFloatRegZTK,
  _rkJointFloatDisFromZTK,
  _rkJointFloatFromZTK,
  _rkJointFloatDisFPrintZTK,
  _rkJointFloatFPrintZTK,
};

#undef _rkc
