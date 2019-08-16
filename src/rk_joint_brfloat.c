/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_brfloat - joint structure: breakable free-floating joint
 */

#include <roki/rk_joint.h>

#define _rkc(p) ((rkJointPrpBrFloat *)p)

/* ********************************************************** */
/* broken float
 * ********************************************************** */

static void _rkJointInitBrFloat(void *prp){
  _rkc(prp)->ep_f = HUGE_VAL;
  _rkc(prp)->ep_t = HUGE_VAL;
}

static void *_rkJointAllocBrFloat(void){
  return zAlloc( rkJointPrpBrFloat, 1 );
}

/* limit joint displacement */
static void _rkJointLimDisBrFloat(void *prp, double *testval, double *limval){
  zVec6DCopy( (zVec6D*)testval, (zVec6D*)limval );
}

/* set joint displacement */
static void _rkJointSetDisBrFloat(void *prp, double *val){
  _rkJointLimDisBrFloat( prp, val, _rkc(prp)->dis.e );
  zMat3DFromAA( &_rkc(prp)->_att, zVec6DAng(&_rkc(prp)->dis) );
}

static void _rkJointSetVelBrFloat(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->vel );
}

static void _rkJointSetAccBrFloat(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->acc );
}

static void _rkJointSetTrqBrFloat(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->trq );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointGetDisBrFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->dis, (zVec6D*)val );
}

static void _rkJointGetVelBrFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->vel, (zVec6D*)val );
}

static void _rkJointGetAccBrFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->acc, (zVec6D*)val );
}

static void _rkJointGetTrqBrFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->trq, (zVec6D*)val );
}

static void _rkJointCatDisBrFloat(void *prp, double *dis, double k, double *val){
  zVec3D d, aa;
  /* concatenate position */
  zVec3DCatDRC( (zVec3D*)&dis[0], k, (zVec3D*)&val[0] );
  /* concatenate attitude */
  zVec3DCopy( (zVec3D*)&dis[3], &aa );
  zVec3DMul( (zVec3D*)&val[3], k, &d );
  zAACascade( &aa, &d, (zVec3D*)&dis[3] );
}

static void _rkJointSubDisBrFloat(void *prp, double *dis, double *sdis){
  zMat3D m, ms;
  zVec3D aa;

  zVec3DSubDRC( zVec6DLin((zVec6D*)dis), zVec6DLin((zVec6D*)sdis) );
  zVec3DCopy( zVec6DAng((zVec6D*)dis), &aa );
  zMat3DFromAA( &m, &aa );
  zMat3DFromAA( &ms, zVec6DAng((zVec6D*)sdis) );
  zMat3DError( &m, &ms, zVec6DAng((zVec6D*)dis) );
}

/* continuously update joint displacement over delta time */
static void _rkJointSetDisCNTBrFloat(void *prp, double *val, double dt){
  zMat3D m_old;
  zVec3D p_old;
  zVec6D v_old;

  /* previous state */
  zVec3DCopy( zVec6DLin(&_rkc(prp)->dis), &p_old );
  zMat3DCopy( &_rkc(prp)->_att, &m_old );
  _rkJointGetVelBrFloat( prp, v_old.e );
  /* update displacement */
  _rkJointSetDisBrFloat( prp, val );
  /* numerical differentiation */
  zVec3DDif( &p_old, zVec6DLin(&_rkc(prp)->dis), dt, zVec6DLin(&_rkc(prp)->vel) );
  zMat3DError( &_rkc(prp)->_att, &m_old, zVec6DAng(&_rkc(prp)->vel) );
  zVec3DDivDRC( zVec6DAng(&_rkc(prp)->vel), dt );
  zVec6DDif( &v_old, &_rkc(prp)->vel, dt, &_rkc(prp)->acc );
}

/* joint frame transfer function */
static zFrame3D *_rkJointXformBrFloat(void *prp, zFrame3D *fo, zFrame3D *f){
  /* position */
  zXform3D( fo, zVec6DLin(&_rkc(prp)->dis), zFrame3DPos(f) );
  /* attitude */
  zMulMat3DMat3D( zFrame3DAtt(fo), &_rkc(prp)->_att, zFrame3DAtt(f) );
  return f;
}

/* joint velocity transfer function */
static void _rkJointIncVelBrFloat(void *prp, zVec6D *vel){
  zVec6D vl;
  zMulMat3DTVec6D( &_rkc(prp)->_att, &_rkc(prp)->vel, &vl );
  zVec6DAddDRC( vel, &vl );
}

static void _rkJointIncAccOnVelBrFloat(void *prp, zVec3D *w, zVec6D *acc){
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
static void _rkJointIncAccBrFloat(void *prp, zVec6D *acc){
  zVec6D al;
  zMulMat3DTVec6D( &_rkc(prp)->_att, &_rkc(prp)->acc, &al );
  zVec6DAddDRC( acc, &al );
}

/* joint torque transfer function */
static void _rkJointCalcTrqBrFloat(void *prp, zVec6D *f){
  zMulMat3DVec6D( &_rkc(prp)->_att, f, &_rkc(prp)->trq );
}

/* inverse computation of joint torsion and displacement */
static void _rkJointTorsionBrFloat(zFrame3D *dev, zVec6D *t, double dis[]){
  zVec6DZero( t );
  zFrame3DToVec6DAA( dev, (zVec6D*)dis );
}

/* joint axis function */
static zVec3D *_rkJointAxisBrFloat(void *prp, zFrame3D *f, zDir dir, zVec3D *a){
  zVec3D al;
  zMat3DRow( &_rkc(prp)->_att, dir, &al );
  return zMulMat3DVec3D( zFrame3DAtt(f), &al, a );
}
static zVec3D *_rkJointAxisXBrFloat(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisBrFloat( prp, f, zX, a );
}
static zVec3D *_rkJointAxisYBrFloat(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisBrFloat( prp, f, zY, a );
}
static zVec3D *_rkJointAxisZBrFloat(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisBrFloat( prp, f, zZ, a );
}
static zVec3D* (*_rk_joint_axis_float_ang[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisXBrFloat,
  _rkJointAxisYBrFloat,
  _rkJointAxisZBrFloat,
};
static zVec3D* (*_rk_joint_axis_float_lin[])(void*,zFrame3D*,zVec3D*) = {
  _rkJointAxisXBrFloat,
  _rkJointAxisYBrFloat,
  _rkJointAxisZBrFloat,
  _rkJointAxisNull,
  _rkJointAxisNull,
  _rkJointAxisNull,
};

static void _rkJointFrictionPivotBrFloat(void *prp, rkJointFrictionPivot *fp){}
static void _rkJointValBrFloat(void *prp, double *val){}

/* any actuator cannot be mounted on the free-floating joint. */
static rkMotor *_rkJointGetMotorBrFloat(void *prp){ return NULL; }

/* motor */
static void _rkJointMotorSetInputBrFloat(void *prp, double *val){}
static void _rkJointMotorInertiaBrFloat(void *prp, double *val){ zMat6DZero( (zMat6D *)val ); }
static void _rkJointMotorInputTrqBrFloat(void *prp, double *val){ zVec6DZero( (zVec6D *)val ); }
static void _rkJointMotorResistanceBrFloat(void *prp, double *val){ zVec6DZero( (zVec6D *)val ); }
static void _rkJointMotorDrivingTrqBrFloat(void *prp, double *val){ zVec6DZero( (zVec6D *)val ); }

/* ABI for breakable joints */
static void _rkJointABIAxisInertiaBrFloat(void *prp, zMat6D *m, zMat h, zMat ih){
  register int i, j;
  _rkJointMotorInertiaBrFloat( prp, zMatBuf(h) );
  for( i=0; i<3; i++ )
    for( j=0; j<3; j++ ){
      zMatElemNC(h,i,  j)   += m->e[0][0].e[j][i];
      zMatElemNC(h,i+3,j)   += m->e[0][1].e[j][i];
      zMatElemNC(h,i,  j+3) += m->e[1][0].e[j][i];
      zMatElemNC(h,i+3,j+3) += m->e[1][1].e[j][i];
    }
  zMatInv( h, ih );
}

static void _rkJointABIAddAbiBrFloat(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){}
static void _rkJointABIAddBiasBrFloat(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){}
static void _rkJointABIDrivingTorqueBrFloat(void *prp){}
static void _rkJointABIQAccBrFloat(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  zVec6D tmpv, tmpv2;
  register int i;
  /* acc */
  zVec6DRev(b, &tmpv2);
  for(i=zX;i<=zZA;i++)
    tmpv.e[i] = zVec6DInnerProd( (zVec6D *)&zMatElemNC(h,i,0), &tmpv2 );
  zVec6DCopy( &tmpv, acc );
  /* q */
  zVec6DSubDRC( &tmpv, jac );
  zMulMat3DVec6D( r, &tmpv, &_rkc(prp)->acc );
}

/* ABI for unbreakable joints */

static void _rkJointABIAxisInertiaBrFloatFixed(void *prp, zMat6D *m, zMat h, zMat ih){}
static void _rkJointABIAddAbiBrFloatFixed(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  zMat6D tmpm;
  rkJointXformMat6D( f, m, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}
static void _rkJointABIAddBiasBrFloatFixed(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  zVec6D tmpv;
  zMulMat3DVec6D( zFrame3DAtt(f), b, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}
static void _rkJointABIDrivingTorqueBrFloatFixed(void *prp){}
static void _rkJointABIQAccBrFloatFixed(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  zVec6DCopy( jac, acc );
}
static void _rkJointUpdateWrenchBrFloatFixed(rkJoint *j, zMat6D *i, zVec6D *b, zVec6D *acc){
  _rkJointUpdateWrench( j, i, b, acc );
  if( ( zVec3DNorm( zVec6DLin(rkJointWrench(j)) ) > _rkc(j->prp)->ep_f ) ||
      ( zVec3DNorm( zVec6DAng(rkJointWrench(j)) ) > _rkc(j->prp)->ep_t ) ){
    j->com->_axinertia = _rkJointABIAxisInertiaBrFloatFixed;
    j->com->_addabi = _rkJointABIAddAbiBrFloatFixed;
    j->com->_addbias = _rkJointABIAddBiasBrFloatFixed;
    j->com->_dtrq = _rkJointABIDrivingTorqueBrFloatFixed;
    j->com->_qacc = _rkJointABIQAccBrFloatFixed;
    j->com->_wrench = _rkJointUpdateWrenchBrFloatFixed; /* unnecessary */
  }
}

/* query joint properties */
static bool _rkJointQueryFScanBrFloat(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm){
  zVec6D dis;

  if( strcmp( buf, "dis" ) == 0 ){
    zVec6DFScan( fp, &dis );
    _rkJointSetDisBrFloat( prp, dis.e );
  } else
  if( strcmp( buf, "forcethreshold" ) == 0 )
    _rkc(prp)->ep_f = zFDouble(fp);
  else
  if( strcmp( buf, "torquethreshold" ) == 0 )
    _rkc(prp)->ep_t = zFDouble(fp);
  else
    return false;
  return false;
}

static void *_rkJointDisFromZTKBrFloat(void *prp, int i, void *arg, ZTK *ztk){
  zVec6D dis;
  zVec6DFromZTK( &dis, ztk );
  _rkJointSetDisBrFloat( prp, dis.e );
  return prp;
}
static void *_rkJointForceThFromZTKBrFloat(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->ep_f = ZTKDouble(ztk);
  return prp;
}
static void *_rkJointTorqueThFromZTKBrFloat(void *prp, int i, void *arg, ZTK *ztk){
  _rkc(prp)->ep_t = ZTKDouble(ztk);
  return prp;
}

static void _rkJointDisFPrintBrFloat(FILE *fp, int i, void *prp){
  zVec6DDataNLFPrint( fp, &_rkc(prp)->dis );
}
static void _rkJointForceThFPrintBrFloat(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->ep_f );
}
static void _rkJointTorqueThFPrintBrFloat(FILE *fp, int i, void *prp){
  fprintf( fp, "%.10g\n", _rkc(prp)->ep_t );
}

static ZTKPrp __ztk_prp_rkjoint_brfloat[] = {
  { "dis", 1, _rkJointDisFromZTKBrFloat, _rkJointDisFPrintBrFloat },
  { "forcethreshold", 1, _rkJointForceThFromZTKBrFloat, _rkJointForceThFPrintBrFloat },
  { "torquethreshold", 1, _rkJointTorqueThFromZTKBrFloat, _rkJointTorqueThFPrintBrFloat },
};

static void *_rkJointFromZTKBrFloat(void *prp, rkMotorArray *motorarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( prp, motorarray, ztk, __ztk_prp_rkjoint_brfloat );
}

static void _rkJointFPrintBrFloat(FILE *fp, void *prp, char *name)
{
  ZTKPrpKeyFPrint( fp, prp, __ztk_prp_rkjoint_brfloat );
}

rkJointCom rk_joint_brfloat = {
  "breakablefloat",
  6,
  _rkJointInitBrFloat,
  _rkJointAllocBrFloat,
  _rkJointLimDisBrFloat,
  _rkJointSetDisBrFloat,
  _rkJointSetVelBrFloat,
  _rkJointSetAccBrFloat,
  _rkJointSetTrqBrFloat,
  _rkJointGetDisBrFloat,
  _rkJointGetVelBrFloat,
  _rkJointGetAccBrFloat,
  _rkJointGetTrqBrFloat,
  _rkJointCatDisBrFloat,
  _rkJointSubDisBrFloat,
  _rkJointSetDisCNTBrFloat,
  _rkJointXformBrFloat,
  _rkJointIncVelBrFloat,
  _rkJointIncAccOnVelBrFloat,
  _rkJointIncAccBrFloat,
  _rkJointCalcTrqBrFloat,
  _rkJointTorsionBrFloat,
  _rk_joint_axis_float_ang,
  _rk_joint_axis_float_lin,

  _rkJointFrictionPivotBrFloat,
  _rkJointFrictionPivotBrFloat,
  _rkJointValBrFloat,
  _rkJointValBrFloat,
  _rkJointValBrFloat,
  _rkJointValBrFloat,

  _rkJointGetMotorBrFloat,
  _rkJointMotorSetInputBrFloat,
  _rkJointMotorInertiaBrFloat,
  _rkJointMotorInputTrqBrFloat,
  _rkJointMotorResistanceBrFloat,
  _rkJointMotorDrivingTrqBrFloat,

  _rkJointABIAxisInertiaBrFloat,
  _rkJointABIAddAbiBrFloat,
  _rkJointABIAddBiasBrFloat,
  _rkJointABIDrivingTorqueBrFloat,
  _rkJointABIQAccBrFloat,
  _rkJointUpdateWrenchBrFloatFixed,

  _rkJointQueryFScanBrFloat,
  _rkJointDisFromZTKBrFloat,
  _rkJointFromZTKBrFloat,
  _rkJointDisFPrintBrFloat,
  _rkJointFPrintBrFloat,
};

bool rkJointRegZTKBrFloat(ZTK *ztk, char *tag)
{
  return ZTKDefRegPrp( ztk, tag, __ztk_prp_rkjoint_brfloat ) ? true : false;
}

#undef _rkc
