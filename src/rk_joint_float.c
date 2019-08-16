/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_float - joint structure: free-floating joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointPrpFloat *)p)

static void _rkJointInitFloat(void *prp){}

static void *_rkJointAllocFloat(void){ return zAlloc( rkJointPrpFloat, 1 ); }

/* limit joint displacement */
static void _rkJointLimDisFloat(void *prp, double *testval, double *limval){
  zVec6DCopy( (zVec6D*)testval, (zVec6D*)limval );
}

/* joint displacement set function */
static void _rkJointSetDisFloat(void *prp, double *val){
  _rkJointLimDisFloat( prp, val, _rkc(prp)->dis.e );
  zMat3DFromAA( &_rkc(prp)->_att, zVec6DAng(&_rkc(prp)->dis) );
}

static void _rkJointSetVelFloat(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->vel );
}

static void _rkJointSetAccFloat(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->acc );
}

static void _rkJointSetTrqFloat(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->trq );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointGetDisFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->dis, (zVec6D*)val );
}

static void _rkJointGetVelFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->vel, (zVec6D*)val );
}

static void _rkJointGetAccFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->acc, (zVec6D*)val );
}

static void _rkJointGetTrqFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->trq, (zVec6D*)val );
}

static void _rkJointCatDisFloat(void *prp, double *dis, double k, double *val){
  zVec3D d, aa;
  /* concatenate position */
  zVec3DCatDRC( (zVec3D*)&dis[0], k, (zVec3D*)&val[0] );
  /* concatenate attitude */
  zVec3DCopy( (zVec3D*)&dis[3], &aa );
  zVec3DMul( (zVec3D*)&val[3], k, &d );
  zAACascade( &aa, &d, (zVec3D*)&dis[3] );
}

static void _rkJointSubDisFloat(void *prp, double *dis, double *sdis){
  zMat3D m, ms;
  zVec3D aa;

  zVec3DSubDRC( zVec6DLin((zVec6D*)dis), zVec6DLin((zVec6D*)sdis) );
  zVec3DCopy( zVec6DAng((zVec6D*)dis), &aa );
  zMat3DFromAA( &m, &aa );
  zMat3DFromAA( &ms, zVec6DAng((zVec6D*)sdis) );
  zMat3DError( &m, &ms, zVec6DAng((zVec6D*)dis) );
}

/* continuously update joint displacement */
static void _rkJointSetDisCNTFloat(void *prp, double *val, double dt){
  zMat3D m_old;
  zVec3D p_old;
  zVec6D v_old;

  /* previous state */
  zVec3DCopy( zVec6DLin(&_rkc(prp)->dis), &p_old );
  zMat3DCopy( &_rkc(prp)->_att, &m_old );
  _rkJointGetVelFloat( prp, v_old.e );
  /* update displacement */
  _rkJointSetDisFloat( prp, val );
  /* numerical differentiation */
  zVec3DDif( &p_old, zVec6DLin(&_rkc(prp)->dis), dt, zVec6DLin(&_rkc(prp)->vel) );
  zMat3DError( &_rkc(prp)->_att, &m_old, zVec6DAng(&_rkc(prp)->vel) );
  zVec3DDivDRC( zVec6DAng(&_rkc(prp)->vel), dt );
  zVec6DDif( &v_old, &_rkc(prp)->vel, dt, &_rkc(prp)->acc );
}

/* joint frame transformation */
static zFrame3D *_rkJointXformFloat(void *prp, zFrame3D *fo, zFrame3D *f){
  /* position */
  zXform3D( fo, zVec6DLin(&_rkc(prp)->dis), zFrame3DPos(f) );
  /* attitude */
  zMulMat3DMat3D( zFrame3DAtt(fo), &_rkc(prp)->_att, zFrame3DAtt(f) );
  return f;
}

/* joint velocity transformation */
static void _rkJointIncVelFloat(void *prp, zVec6D *vel){
  zVec6D vl;
  zMulMat3DTVec6D( &_rkc(prp)->_att, &_rkc(prp)->vel, &vl );
  zVec6DAddDRC( vel, &vl );
}

static void _rkJointIncAccOnVelFloat(void *prp, zVec3D *w, zVec6D *acc){
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
static void _rkJointIncAccFloat(void *prp, zVec6D *acc){
  zVec6D al;
  zMulMat3DTVec6D( &_rkc(prp)->_att, &_rkc(prp)->acc, &al );
  zVec6DAddDRC( acc, &al );
}

/* joint torque transformation */
static void _rkJointCalcTrqFloat(void *prp, zVec6D *f){
  zMulMat3DVec6D( &_rkc(prp)->_att, f, &_rkc(prp)->trq );
}

/* inverse computation of joint torsion and displacement */
static void _rkJointTorsionFloat(zFrame3D *dev, zVec6D *t, double dis[]){
  zVec6DZero( t );
  zFrame3DToVec6DAA( dev, (zVec6D*)dis );
}

/* joint axes */
static zVec3D *_rkJointAxisFloat(void *prp, zFrame3D *f, zDir dir, zVec3D *a){
  zVec3D al;
  zMat3DRow( &_rkc(prp)->_att, dir, &al );
  return zMulMat3DVec3D( zFrame3DAtt(f), &al, a );
}

static zVec3D *_rkJointAxisXFloat(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisFloat( prp, f, zX, a );
}

static zVec3D *_rkJointAxisYFloat(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisFloat( prp, f, zY, a );
}

static zVec3D *_rkJointAxisZFloat(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisFloat( prp, f, zZ, a );
}

static zVec3D* (*_rk_joint_axis_float_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisXFloat,
  _rkJointAxisYFloat,
  _rkJointAxisZFloat,
};
static zVec3D* (*_rk_joint_axis_float_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisXFloat,
  _rkJointAxisYFloat,
  _rkJointAxisZFloat,
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
};

static void _rkJointFrictionPivotFloat(void *prp, rkJointFrictionPivot *fp){}
static void _rkJointValFloat(void *prp, double *val){}

/* any actuator cannot be mounted on the free-floating joint. */
static rkMotor *_rkJointGetMotorFloat(void *prp){ return NULL; }

static void _rkJointMotorSetInputFloat(void *prp, double *val){}
static void _rkJointMotorInertiaFloat(void *prp, double *val){ zMat6DZero( (zMat6D *)val ); }
static void _rkJointMotorInputTrqFloat(void *prp, double *val){ zVec6DZero( (zVec6D *)val ); }
static void _rkJointMotorResistanceFloat(void *prp, double *val){ zVec6DZero( (zVec6D *)val ); }
static void _rkJointMotorDrivingTrqFloat(void *prp, double *val){ zVec6DZero( (zVec6D *)val ); }

/* ABI */
static void _rkJointABIAxisInertiaFloat(void *prp, zMat6D *m, zMat h, zMat ih){
  register int i, j;

  _rkJointMotorInertiaFloat( prp, zMatBuf(h) );
  for( i=0; i<3; i++ )
    for( j=0; j<3; j++ ){
      zMatElemNC(h,i,  j)   += m->e[0][0].e[j][i];
      zMatElemNC(h,i+3,j)   += m->e[0][1].e[j][i];
      zMatElemNC(h,i,  j+3) += m->e[1][0].e[j][i];
      zMatElemNC(h,i+3,j+3) += m->e[1][1].e[j][i];
    }
  zMatInv( h, ih );
}

static void _rkJointABIAddAbiFloat(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){}

static void _rkJointABIAddBiasFloat(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){}

static void _rkJointABIDrivingTorqueFloat(void *prp){}

static void _rkJointABIQAccFloat(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  zVec6D tmpv, tmpv2;
  register int i;
  /* acc */
  zVec6DRev(b, &tmpv2);
  for(i=zX;i<=zZA;i++)
    tmpv.e[i] = zVec6DInnerProd( (zVec6D *)&zMatElemNC(h,i,0), &tmpv2 );
  zVec6DCopy( &tmpv, acc );
  /* q */
  zVec6DSubDRC(&tmpv, jac);
  zMulMat3DVec6D( r, &tmpv, &_rkc(prp)->acc );
}

/* query joint properties */
static bool _rkJointQueryFScanFloat(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm){
  zVec6D dis;
  if( strcmp( buf, "dis" ) == 0 ){
    zVec6DFScan( fp, &dis );
    _rkJointSetDisFloat( prp, dis.e );
    return true;
  }
  return false;
}

static void *_rkJointDisFromZTKFloat(void *prp, int i, void *arg, ZTK *ztk){
  zVec6D dis;
  zVec6DFromZTK( &dis, ztk );
  _rkJointSetDisFloat( prp, dis.e );
  return prp;
}

static void _rkJointDisFPrintFloat(FILE *fp, int i, void *prp){
  zVec6DDataNLFPrint( fp, &_rkc(prp)->dis );
}

static ZTKPrp __ztk_prp_rkjoint_float[] = {
  { "dis", 1, _rkJointDisFromZTKFloat, _rkJointDisFPrintFloat },
};

static void *_rkJointFromZTKFloat(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_float );
}

static void _rkJointFPrintFloat(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_float );
}

rkJointCom rk_joint_float = {
  "float",
  6,
  _rkJointInitFloat,
  _rkJointAllocFloat,
  _rkJointLimDisFloat,
  _rkJointSetDisFloat,
  _rkJointSetVelFloat,
  _rkJointSetAccFloat,
  _rkJointSetTrqFloat,
  _rkJointGetDisFloat,
  _rkJointGetVelFloat,
  _rkJointGetAccFloat,
  _rkJointGetTrqFloat,
  _rkJointCatDisFloat,
  _rkJointSubDisFloat,
  _rkJointSetDisCNTFloat,
  _rkJointXformFloat,
  _rkJointIncVelFloat,
  _rkJointIncAccOnVelFloat,
  _rkJointIncAccFloat,
  _rkJointCalcTrqFloat,
  _rkJointTorsionFloat,
  _rk_joint_axis_float_ang,
  _rk_joint_axis_float_lin,

  _rkJointFrictionPivotFloat,
  _rkJointFrictionPivotFloat,
  _rkJointValFloat,
  _rkJointValFloat,
  _rkJointValFloat,
  _rkJointValFloat,

  _rkJointGetMotorFloat,
  _rkJointMotorSetInputFloat,
  _rkJointMotorInertiaFloat,
  _rkJointMotorInputTrqFloat,
  _rkJointMotorResistanceFloat,
  _rkJointMotorDrivingTrqFloat,

  _rkJointABIAxisInertiaFloat,
  _rkJointABIAddAbiFloat,
  _rkJointABIAddBiasFloat,
  _rkJointABIDrivingTorqueFloat,
  _rkJointABIQAccFloat,
  _rkJointUpdateWrench,

  _rkJointQueryFScanFloat,
  _rkJointDisFromZTKFloat,
  _rkJointFromZTKFloat,
  _rkJointDisFPrintFloat,
  _rkJointFPrintFloat,
};

bool rkJointRegZTKFloat(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_float ) ? true : false;
}

#undef _rkc
