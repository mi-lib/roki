/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_float - joint structure: free-floating joint
 */

#include <roki/rk_joint.h>

static void _rkJointLimDisFloat(void *prp, double *testval, double *limval);
static void _rkJointSetDisFloat(void *prp, double *val);
static void _rkJointSetVelFloat(void *prp, double *val);
static void _rkJointSetAccFloat(void *prp, double *val);
static void _rkJointSetTrqFloat(void *prp, double *val);
static void _rkJointGetDisFloat(void *prp, double *val);
static void _rkJointGetVelFloat(void *prp, double *val);
static void _rkJointGetAccFloat(void *prp, double *val);
static void _rkJointGetTrqFloat(void *prp, double *val);
static void _rkJointGetMotorFloat(void *prp, rkMotor **m);
static void _rkJointCatDisFloat(void *prp, double *dis, double k, double *val);
static void _rkJointSubDisFloat(void *prp, double *dis, double *sdis);
static void _rkJointSetDisCNTFloat(void *prp, double *val, double dt);

static zFrame3D *_rkJointXferFloat(void *prp, zFrame3D *fo, zFrame3D *f);
static void _rkJointIncVelFloat(void *prp, zVec6D *vel);
static void _rkJointIncAccOnVelFloat(void *prp, zVec3D *w, zVec6D *acc);
static void _rkJointIncAccFloat(void *prp, zVec6D *acc);

static void _rkJointCalcTrqFloat(void *prp, zVec6D *f);
static void _rkJointTorsionFloat(zFrame3D *dev, zVec6D *t, double dis[]);

static void _rkJointValFloat(void *prp, double *val);
static void _rkJointRefFloat(void *prp, rkJointRef *ref);

static zVec3D *_rkJointAxisFloat(void *prp, zFrame3D *f, zDir dir, zVec3D *a);
static zVec3D *_rkJointAxisXFloat(void *prp, zFrame3D *f, zVec3D *a);
static zVec3D *_rkJointAxisYFloat(void *prp, zFrame3D *f, zVec3D *a);
static zVec3D *_rkJointAxisZFloat(void *prp, zFrame3D *f, zVec3D *a);

static bool _rkJointQueryFReadFloat(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm);
static void _rkJointFWriteFloat(FILE *fp, void *prp, char *name);

#define _rkc(p) ((rkJointPrpFloat *)p)

/* limit joint displacement */
void _rkJointLimDisFloat(void *prp, double *testval, double *limval){
  zVec6DCopy( (zVec6D*)testval, (zVec6D*)limval );
}

/* joint displacement set function */
void _rkJointSetDisFloat(void *prp, double *val){
  _rkJointLimDisFloat( prp, val, _rkc(prp)->dis.e );
  zMat3DFromAA( &_rkc(prp)->_att, zVec6DAng(&_rkc(prp)->dis) );
}

void _rkJointSetVelFloat(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->vel );
}

void _rkJointSetAccFloat(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->acc );
}

void _rkJointSetTrqFloat(void *prp, double *val){
  zVec6DCopy( (zVec6D*)val, &_rkc(prp)->trq );
}

/* get joint displacement, velocity, acceleration and torque */
void _rkJointGetDisFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->dis, (zVec6D*)val );
}

void _rkJointGetVelFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->vel, (zVec6D*)val );
}

void _rkJointGetAccFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->acc, (zVec6D*)val );
}

void _rkJointGetTrqFloat(void *prp, double *val){
  zVec6DCopy( &_rkc(prp)->trq, (zVec6D*)val );
}

/* any actuator cannot be mounted on the free-floating joint. */
void _rkJointGetMotorFloat(void *prp, rkMotor **m){
  *m = NULL;
}

void _rkJointCatDisFloat(void *prp, double *dis, double k, double *val)
{
  zVec3D d, aa;

  /* concatenate position */
  zVec3DCatDRC( (zVec3D*)&dis[0], k, (zVec3D*)&val[0] );
  /* concatenate attitude */
  zVec3DCopy( (zVec3D*)&dis[3], &aa );
  zVec3DMul( (zVec3D*)&val[3], k, &d );
  zAACascade( &aa, &d, (zVec3D*)&dis[3] );
}

void _rkJointSubDisFloat(void *prp, double *dis, double *sdis)
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
void _rkJointSetDisCNTFloat(void *prp, double *val, double dt)
{
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

/* joint frame transfer function */
zFrame3D *_rkJointXferFloat(void *prp, zFrame3D *fo, zFrame3D *f)
{
  /* position */
  zXfer3D( fo, zVec6DLin(&_rkc(prp)->dis), zFrame3DPos(f) );
  /* attitude */
  zMulMatMat3D( zFrame3DAtt(fo), &_rkc(prp)->_att, zFrame3DAtt(f) );
  return f;
}

/* joint velocity transfer function */
void _rkJointIncVelFloat(void *prp, zVec6D *vel)
{
  zVec6D vl;

  zMulMatTVec6D( &_rkc(prp)->_att, &_rkc(prp)->vel, &vl );
  zVec6DAddDRC( vel, &vl );
}

void _rkJointIncAccOnVelFloat(void *prp, zVec3D *w, zVec6D *acc)
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
void _rkJointIncAccFloat(void *prp, zVec6D *acc)
{
  zVec6D al;

  zMulMatTVec6D( &_rkc(prp)->_att, &_rkc(prp)->acc, &al );
  zVec6DAddDRC( acc, &al );
}

/* joint torque transfer function */
void _rkJointCalcTrqFloat(void *prp, zVec6D *f)
{
  zMulMatVec6D( &_rkc(prp)->_att, f, &_rkc(prp)->trq );
}

/* inverse computation of joint torsion and displacement */
void _rkJointTorsionFloat(zFrame3D *dev, zVec6D *t, double dis[])
{
  zVec6DClear( t );
  zFrame3DToVec6DAA( dev, (zVec6D*)dis );
}

void _rkJointValFloat(void *prp, double *val){}
void _rkJointRefFloat(void *prp, rkJointRef *ref){}

/* joint axis function */
zVec3D *_rkJointAxisFloat(void *prp, zFrame3D *f, zDir dir, zVec3D *a){
  zVec3D al;

  zMat3DRow( &_rkc(prp)->_att, dir, &al );
  return zMulMatVec3D( zFrame3DAtt(f), &al, a );
}

zVec3D *_rkJointAxisXFloat(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisFloat( prp, f, zX, a );
}

zVec3D *_rkJointAxisYFloat(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisFloat( prp, f, zY, a );
}

zVec3D *_rkJointAxisZFloat(void *prp, zFrame3D *f, zVec3D *a){
  return _rkJointAxisFloat( prp, f, zZ, a );
}

/* query joint properties */
bool _rkJointQueryFReadFloat(FILE *fp, char *buf, void *prp, rkMotor *marray, int nm)
{
  zVec6D dis;

  if( strcmp( buf, "dis" ) == 0 ){
    zVec6DFRead( fp, &dis );
    _rkJointSetDisFloat( prp, dis.e );
    return true;
  }
  return false;
}

void _rkJointFWriteFloat(FILE *fp, void *prp, char *name)
{
  if( !zVec6DIsTiny( &_rkc(prp)->dis ) )
    fprintf( fp, "%s: %.10f %.10f %.10f %.10f %.10f %.10f\n", name,
      _rkc(prp)->dis.e[zX],  _rkc(prp)->dis.e[zY],  _rkc(prp)->dis.e[zZ],
      _rkc(prp)->dis.e[zXA], _rkc(prp)->dis.e[zYA], _rkc(prp)->dis.e[zZA] );
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
static rkJointCom rk_joint_float = {
  6,
  _rkJointLimDisFloat,
  _rkJointSetDisFloat,
  _rkJointSetVelFloat,
  _rkJointSetAccFloat,
  _rkJointSetTrqFloat,
  _rkJointGetDisFloat,
  _rkJointGetVelFloat,
  _rkJointGetAccFloat,
  _rkJointGetTrqFloat,
  _rkJointGetMotorFloat,
  _rkJointCatDisFloat,
  _rkJointSubDisFloat,
  _rkJointSetDisCNTFloat,
  _rkJointXferFloat,
  _rkJointIncVelFloat,
  _rkJointIncAccOnVelFloat,
  _rkJointIncAccFloat,
  _rkJointCalcTrqFloat,
  _rkJointTorsionFloat,
  _rkJointValFloat,
  _rkJointValFloat,
  _rkJointValFloat,
  _rkJointValFloat,
  _rkJointRefFloat,
  _rkJointRefFloat,
  _rk_joint_axis_float_ang,
  _rk_joint_axis_float_lin,
  _rkJointQueryFReadFloat,
  _rkJointFWriteFloat,
};

/* motor */
static byte _rkJointMotorFloat(void *prp);
static void _rkJointMotorSetInputFloat(void *prp, double *val);
static void _rkJointMotorInertiaFloat(void *prp, double *val);
static void _rkJointMotorInputTrqFloat(void *prp, double *val);
static void _rkJointMotorResistanceFloat(void *prp, double *val);
static void _rkJointMotorDrivingTrqFloat(void *prp, double *val);

byte _rkJointMotorFloat(void *prp){return RK_MOTOR_INVALID;}
void _rkJointMotorSetInputFloat(void *prp, double *val){}
void _rkJointMotorInertiaFloat(void *prp, double *val){ zMat6DClear( (zMat6D *)val ); }
void _rkJointMotorInputTrqFloat(void *prp, double *val){ zVec6DClear( (zVec6D *)val ); }
void _rkJointMotorResistanceFloat(void *prp, double *val){ zVec6DClear( (zVec6D *)val ); }
void _rkJointMotorDrivingTrqFloat(void *prp, double *val){ zVec6DClear( (zVec6D *)val ); }

static rkJointMotorCom rk_joint_motor_float = {
  _rkJointMotorFloat,
  _rkJointMotorSetInputFloat,
  _rkJointMotorInertiaFloat,
  _rkJointMotorInputTrqFloat,
  _rkJointMotorResistanceFloat,
  _rkJointMotorDrivingTrqFloat,
};

/* ABI */
static void _rkJointABIAxisInertiaFloat(void *prp, zMat6D *m, zMat h, zMat ih);
static void _rkJointABIAddAbiFloat(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm);
static void _rkJointABIAddBiasFloat(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb);
static void _rkJointABIDrivingTorqueFloat(void *prp);
static void _rkJointABIQAccFloat(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc);

void _rkJointABIAxisInertiaFloat(void *prp, zMat6D *m, zMat h, zMat ih){
  register int i, j;

  _rkJointMotorInertiaFloat( prp, zMatBuf(h) );
  for( i=0; i<3; i++ )
    for( j=0; j<3; j++ ){
      zMatElem(h,i,  j)   += zMat6DMat3D(m,0,0)->e[j][i];
      zMatElem(h,i+3,j)   += zMat6DMat3D(m,1,0)->e[j][i];
      zMatElem(h,i,  j+3) += zMat6DMat3D(m,0,1)->e[j][i];
      zMatElem(h,i+3,j+3) += zMat6DMat3D(m,1,1)->e[j][i];
    }
  zMatInv( h, ih );
}
void _rkJointABIAddAbiFloat(void *prp, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){}
void _rkJointABIAddBiasFloat(void *prp, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){}
void _rkJointABIDrivingTorqueFloat(void *prp){}
void _rkJointABIQAccFloat(void *prp, zMat3D *r, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc)
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

static rkJointABICom rk_joint_abi_float = {
  _rkJointABIAxisInertiaFloat,
  _rkJointABIAddAbiFloat,
  _rkJointABIAddBiasFloat,
  _rkJointABIDrivingTorqueFloat,
  _rkJointABIQAccFloat,
  _rkJointUpdateWrench,
};

/* rkJointCreateFloat
 * - create free-floating joint instance.
 */
rkJoint *rkJointCreateFloat(rkJoint *j)
{
  if( !( j->prp = zAlloc( rkJointPrpFloat, 1 ) ) )
    return NULL;
  j->com = &rk_joint_float;
  j->mcom = &rk_joint_motor_float;
  j->acom = &rk_joint_abi_float;
  return j;
}

#undef _rkc
