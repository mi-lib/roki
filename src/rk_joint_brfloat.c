/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_brfloat - joint structure: breakable free-floating joint
 */

#include <roki/rk_joint.h>

/* ********************************************************** */
/* broken float
 * ********************************************************** */
static void _rkJointLimDisBrFloat(void *prp, double *testval, double *limval);
static void _rkJointSetDisBrFloat(void *prp, double *val);
static void _rkJointSetVelBrFloat(void *prp, double *val);
static void _rkJointSetAccBrFloat(void *prp, double *val);
static void _rkJointSetTrqBrFloat(void *prp, double *val);
static void _rkJointGetDisBrFloat(void *prp, double *val);
static void _rkJointGetVelBrFloat(void *prp, double *val);
static void _rkJointGetAccBrFloat(void *prp, double *val);
static void _rkJointGetTrqBrFloat(void *prp, double *val);
static void _rkJointGetMotorBrFloat(void *prp, rkMotor **m);
static void _rkJointCatDisBrFloat(void *prp, double *dis, double k, double *val);
static void _rkJointSubDisBrFloat(void *prp, double *dis, double *sdis);
static void _rkJointSetDisCNTBrFloat(void *prp, double *val, double dt);

static zFrame3D *_rkJointXferBrFloat(void *prp, zFrame3D *fo, zFrame3D *f);
static void _rkJointIncVelBrFloat(void *prp, zVec6D *vel);
static void _rkJointIncAccOnVelBrFloat(void *prp, zVec3D *w, zVec6D *acc);
static void _rkJointIncAccBrFloat(void *prp, zVec6D *acc);

static void _rkJointCalcTrqBrFloat(void *prp, zVec6D *f);
static void _rkJointTorsionBrFloat(zFrame3D *dev, zVec6D *t, double dis[]);

static void _rkJointValBrFloat(void *prp, double *val);
static void _rkJointRefBrFloat(void *prp, rkJointRef *ref);

static zVec3D *_rkJointAxisBrFloat(void *prp, zFrame3D *f, zDir dir, zVec3D *a);
static zVec3D *_rkJointAxisXBrFloat(void *prp, zFrame3D *f, zVec3D *a);
static zVec3D *_rkJointAxisYBrFloat(void *prp, zFrame3D *f, zVec3D *a);
static zVec3D *_rkJointAxisZBrFloat(void *prp, zFrame3D *f, zVec3D *a);

static bool _rkJointQueryFReadBrFloat(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm);
static void _rkJointFWriteBrFloat(FILE *fp, void *prp, char *name);

#define _rkc(p) ((rkJointPrpBrFloat *)p)

/* limit joint displacement */
void _rkJointLimDisBrFloat(void *prp, double *testval, double *limval){
  zVec6DCopy( (zVec6D*)testval, (zVec6D*)limval );
}

/* joint displacement set function */
void _rkJointSetDisBrFloat(void *prp, double *val){
  _rkJointLimDisBrFloat( prp, val, _rkc(prp)->dis.e );
  zMat3DFromAA( &_rkc(prp)->_att, zVec6DAng(&_rkc(prp)->dis) );
}

void _rkJointSetVelBrFloat(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->vel );
}

void _rkJointSetAccBrFloat(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->acc );
}

void _rkJointSetTrqBrFloat(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->trq );
}

/* get joint displacement, velocity, acceleration and torque */
void _rkJointGetDisBrFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->dis, (zVec6D*)val );
}

void _rkJointGetVelBrFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->vel, (zVec6D*)val );
}

void _rkJointGetAccBrFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->acc, (zVec6D*)val );
}

void _rkJointGetTrqBrFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->trq, (zVec6D*)val );
}

/* any actuator cannot be mounted on the free-floating joint. */
void _rkJointGetMotorBrFloat(void *prp, rkMotor **m){
  *m = NULL;
}

void _rkJointCatDisBrFloat(void *prp, double *dis, double k, double *val)
{
  zVec3D d, aa;

  /* concatenate position */
  zVec3DCatDRC( (zVec3D*)&dis[0], k, (zVec3D*)&val[0] );
  /* concatenate attitude */
  zVec3DCopy( (zVec3D*)&dis[3], &aa );
  zVec3DMul( (zVec3D*)&val[3], k, &d );
  zAACascade( &aa, &d, (zVec3D*)&dis[3] );
}

void _rkJointSubDisBrFloat(void *prp, double *dis, double *sdis)
{
  zMat3D m, ms;
  zVec3D aa;

  zVec3DSubDRC( zVec6DLin((zVec6D*)dis), zVec6DLin((zVec6D*)sdis) );
  zVec3DCopy( zVec6DAng((zVec6D*)dis), &aa );
  zMat3DFromAA( &m, &aa );
  zMat3DFromAA( &ms, zVec6DAng((zVec6D*)sdis) );
  zMat3DError( &m, &ms, zVec6DAng((zVec6D*)dis) );
}

/* continuously update joint displacement */
void _rkJointSetDisCNTBrFloat(void *prp, double *val, double dt)
{
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
zFrame3D *_rkJointXferBrFloat(void *prp, zFrame3D *fo, zFrame3D *f)
{
  /* position */
  zXfer3D( fo, zVec6DLin(&_rkc(prp)->dis), zFrame3DPos(f) );
  /* attitude */
  zMulMatMat3D( zFrame3DAtt(fo), &_rkc(prp)->_att, zFrame3DAtt(f) );
  return f;
}

/* joint velocity transfer function */
void _rkJointIncVelBrFloat(void *prp, zVec6D *vel)
{
  zVec6D vl;

  zMulMatTVec6D( &_rkc(prp)->_att, &_rkc(prp)->vel, &vl );
  zVec6DAddDRC( vel, &vl );
}

void _rkJointIncAccOnVelBrFloat(void *prp, zVec3D *w, zVec6D *acc)
{
  zVec6D vl;
  zVec3D tmp;

	/* FIXME: _att -> ^pR_j */
  zMulMatTVec6D( &_rkc(prp)->_att, &_rkc(prp)->vel, &vl );
  zVec3DOuterProd( w, zVec6DLin(&vl), &tmp );
  zVec3DCatDRC( zVec6DLin(acc), 2, &tmp );
  zVec3DOuterProd( w, zVec6DAng(&vl), &tmp );
  zVec3DAddDRC( zVec6DAng(acc), &tmp );
}

/* joint acceleration transfer function */
void _rkJointIncAccBrFloat(void *prp, zVec6D *acc)
{
  zVec6D al;

  zMulMatTVec6D( &_rkc(prp)->_att, &_rkc(prp)->acc, &al );
  zVec6DAddDRC( acc, &al );
}

/* joint torque transfer function */
void _rkJointCalcTrqBrFloat(void *prp, zVec6D *f)
{
  zMulMatVec6D( &_rkc(prp)->_att, f, &_rkc(prp)->trq );
}

/* inverse computation of joint torsion and displacement */
void _rkJointTorsionBrFloat(zFrame3D *dev, zVec6D *t, double dis[])
{
  zVec6DClear( t );
  zFrame3DToVec6DAA( dev, (zVec6D*)dis );
}

void _rkJointValBrFloat(void *prp, double *val){}
void _rkJointRefBrFloat(void *prp, rkJointRef *ref){}

/* joint axis function */
zVec3D *_rkJointAxisBrFloat(void *prp, zFrame3D *f, zDir dir, zVec3D *a){
  zVec3D al;

  zMat3DRow( &_rkc(prp)->_att, dir, &al );
  return zMulMatVec3D( zFrame3DAtt(f), &al, a );
}

zVec3D *_rkJointAxisXBrFloat(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisBrFloat( prp, f, zX, a );
}

zVec3D *_rkJointAxisYBrFloat(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisBrFloat( prp, f, zY, a );
}

zVec3D *_rkJointAxisZBrFloat(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisBrFloat( prp, f, zZ, a );
}

/* query joint properties */
bool _rkJointQueryFReadBrFloat(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm)
{
  zVec6D dis;

  if( strcmp( buf, "dis" ) == 0 ){
    zVec6DFRead( fp, &dis );
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

void _rkJointFWriteBrFloat(FILE *fp, void *prp, char *name)
{
  if( !zVec6DIsTiny( &_rkc(prp)->dis ) )
    fprintf( fp, "%s: %.10f %.10f %.10f %.10f %.10f %.10f\n", name,
      _rkc(prp)->dis.e[zX],  _rkc(prp)->dis.e[zY],  _rkc(prp)->dis.e[zZ],
      _rkc(prp)->dis.e[zXA], _rkc(prp)->dis.e[zYA], _rkc(prp)->dis.e[zZA] );
  fprintf( fp, "forcethreshold: %.10f\n", _rkc(prp)->ep_f );
  fprintf( fp, "torquethreshold: %.10f\n", _rkc(prp)->ep_t );
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
static rkJointCom rk_joint_brfloat = {
  6,
  _rkJointLimDisBrFloat,
  _rkJointSetDisBrFloat,
  _rkJointSetVelBrFloat,
  _rkJointSetAccBrFloat,
  _rkJointSetTrqBrFloat,
  _rkJointGetDisBrFloat,
  _rkJointGetVelBrFloat,
  _rkJointGetAccBrFloat,
  _rkJointGetTrqBrFloat,
  _rkJointGetMotorBrFloat,
  _rkJointCatDisBrFloat,
  _rkJointSubDisBrFloat,
  _rkJointSetDisCNTBrFloat,
  _rkJointXferBrFloat,
  _rkJointIncVelBrFloat,
  _rkJointIncAccOnVelBrFloat,
  _rkJointIncAccBrFloat,
  _rkJointCalcTrqBrFloat,
  _rkJointTorsionBrFloat,
  _rkJointValBrFloat,
  _rkJointValBrFloat,
  _rkJointValBrFloat,
  _rkJointValBrFloat,
  _rkJointRefBrFloat,
  _rkJointRefBrFloat,
  _rk_joint_axis_float_ang,
  _rk_joint_axis_float_lin,
  _rkJointQueryFReadBrFloat,
  _rkJointFWriteBrFloat,
};

/* motor */
static byte _rkJointMotorBrFloat(void *prp);
static void _rkJointMotorSetInputBrFloat(void *prp, double *val);
static void _rkJointMotorInertiaBrFloat(void *prp, double *val);
static void _rkJointMotorInputTrqBrFloat(void *prp, double *val);
static void _rkJointMotorResistanceBrFloat(void *prp, double *val);
static void _rkJointMotorDrivingTrqBrFloat(void *prp, double *val);

byte _rkJointMotorBrFloat(void *prp){return RK_MOTOR_INVALID;}
void _rkJointMotorSetInputBrFloat(void *prp, double *val){}
void _rkJointMotorInertiaBrFloat(void *prp, double *val){ zMat6DClear( (zMat6D *)val ); }
void _rkJointMotorInputTrqBrFloat(void *prp, double *val){ zVec6DClear( (zVec6D *)val ); }
void _rkJointMotorResistanceBrFloat(void *prp, double *val){ zVec6DClear( (zVec6D *)val ); }
void _rkJointMotorDrivingTrqBrFloat(void *prp, double *val){ zVec6DClear( (zVec6D *)val ); }

static rkJointMotorCom rk_joint_motor_brfloat = {
  _rkJointMotorBrFloat,
  _rkJointMotorSetInputBrFloat,
  _rkJointMotorInertiaBrFloat,
  _rkJointMotorInputTrqBrFloat,
  _rkJointMotorResistanceBrFloat,
  _rkJointMotorDrivingTrqBrFloat,
};

/* ABI */
static void _rkJointABIAxisInertiaBrFloat(void *prp, zMat6D *m, zMat h, zMat ih);
static void _rkJointABIAddAbiBrFloat(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm);
static void _rkJointABIAddBiasBrFloat(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb);
static void _rkJointABIDrivingTorqueBrFloat(void *prp);
static void _rkJointABIQAccBrFloat(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc);

void _rkJointABIAxisInertiaBrFloat(void *prp, zMat6D *m, zMat h, zMat ih){
  register int i, j;

  _rkJointMotorInertiaBrFloat( prp, zMatBuf(h) );
  for( i=0; i<3; i++ )
    for( j=0; j<3; j++ ){
      zMatElem(h,i,  j)   += zMat6DMat3D(m,0,0)->e[j][i];
      zMatElem(h,i+3,j)   += zMat6DMat3D(m,1,0)->e[j][i];
      zMatElem(h,i,  j+3) += zMat6DMat3D(m,0,1)->e[j][i];
      zMatElem(h,i+3,j+3) += zMat6DMat3D(m,1,1)->e[j][i];
    }
  zMatInv( h, ih );
}
void _rkJointABIAddAbiBrFloat(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){}
void _rkJointABIAddBiasBrFloat(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){}
void _rkJointABIDrivingTorqueBrFloat(void *prp){}
void _rkJointABIQAccBrFloat(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
{
  zVec6D tmpv, tmpv2;
  register int i;

  /* acc */
  zVec6DRev(b, &tmpv2);
  for(i=zX;i<=zZA;i++)
    tmpv.e[i] = zVec6DInnerProd( (zVec6D *)&zMatElem(h,i,0), &tmpv2 );
  zVec6DCopy( &tmpv, acc );

  /* q */
  zVec6DSubDRC(&tmpv, jac);
  zMulMatVec6D(r, &tmpv, &_rkc(prp)->acc);
}

static rkJointABICom rk_joint_abi_brfloat = {
  _rkJointABIAxisInertiaBrFloat,
  _rkJointABIAddAbiBrFloat,
  _rkJointABIAddBiasBrFloat,
  _rkJointABIDrivingTorqueBrFloat,
  _rkJointABIQAccBrFloat,
  _rkJointUpdateWrench,
};

/* ********************************************************** */
/* not broken joint (fixed joint)
 * ********************************************************** */
/* ABI */
static void _rkJointABIAxisInertiaBrFloatFixed(void *prp, zMat6D *m, zMat h, zMat ih);
static void _rkJointABIAddAbiBrFloatFixed(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm);
static void _rkJointABIAddBiasBrFloatFixed(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb);
static void _rkJointABIDrivingTorqueBrFloatFixed(void *prp);
static void _rkJointABIQAccBrFloatFixed(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc);
static void _rkJointUpdateWrenchBrFloatFixed(rkJoint *j, zMat6D *i, zVec6D *b, zVec6D *acc);

void _rkJointABIAxisInertiaBrFloatFixed(void *prp, zMat6D *m, zMat h, zMat ih){}
void _rkJointABIAddAbiBrFloatFixed(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm)
{
  zMat6D tmpm;

  rkJointXferMat6D( f, m, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}

void _rkJointABIAddBiasBrFloatFixed(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb)
{
  zVec6D tmpv;

  zMulMatVec6D( zFrame3DAtt(f), b, &tmpv );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec6DAddDRC( pb, &tmpv );
}

void _rkJointABIDrivingTorqueBrFloatFixed(void *prp){}
void _rkJointABIQAccBrFloatFixed(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
{
  zVec6DCopy( jac, acc );
}

void _rkJointUpdateWrenchBrFloatFixed(rkJoint *j, zMat6D *i, zVec6D *b, zVec6D *acc)
{
  _rkJointUpdateWrench( j, i, b, acc );
  if( ( zVec3DNorm( zVec6DLin(rkJointWrench(j)) ) > _rkc(j->prp)->ep_f ) ||
      ( zVec3DNorm( zVec6DAng(rkJointWrench(j)) ) > _rkc(j->prp)->ep_t ) ){
    j->acom = &rk_joint_abi_brfloat;
  }
}

static rkJointABICom rk_joint_abi_brfloatfixed = {
  _rkJointABIAxisInertiaBrFloatFixed,
  _rkJointABIAddAbiBrFloatFixed,
  _rkJointABIAddBiasBrFloatFixed,
  _rkJointABIDrivingTorqueBrFloatFixed,
  _rkJointABIQAccBrFloatFixed,
  _rkJointUpdateWrenchBrFloatFixed,
};

/* rkJointCreateBrFloat
 * - create free-floating joint instance.
 */
rkJoint *rkJointCreateBrFloat(rkJoint *j)
{
  if( !( j->prp = zAlloc( rkJointPrpBrFloat, 1 ) ) )
    return NULL;
  _rkc(j->prp)->ep_f = HUGE_VAL;
  _rkc(j->prp)->ep_t = HUGE_VAL;
  j->com = &rk_joint_brfloat;
  j->mcom = &rk_joint_motor_brfloat;
  j->acom = &rk_joint_abi_brfloatfixed;
  return j;
}

#undef _rkc
