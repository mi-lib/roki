/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_cylin - joint structure: cylindrical joint
 */

#include <roki/rk_joint.h>

#define _rks(joint) ((rkJointCylinState *)((rkJoint *)(joint))->state)
#define _rkp(joint) ((rkJointCylinPrp   *)((rkJoint *)(joint))->prp)

static void _rkJointCylinInit(rkJoint *joint){
  _rkp(joint)->max[0] = HUGE_VAL;
  _rkp(joint)->min[0] =-HUGE_VAL;
  _rkp(joint)->max[1] = HUGE_VAL;
  _rkp(joint)->min[1] =-HUGE_VAL;
}

RK_JOINT_COM_DEF_PRP_FUNC( Cylin )
RK_JOINT_COM_DEF_STATE_FUNC( Cylin )

/* test joint displacement */
static void _rkJointCylinTestDis(rkJoint *joint, double *testval, double *val){
  double angle;
  /* 0: prismatic */
  val[0] = zLimit( testval[0], _rkp(joint)->min[0], _rkp(joint)->max[0] );
  /* 1: revolutional */
  angle = zPhaseNormalize( testval[1] );
  val[1] = zLimit( angle, _rkp(joint)->min[1], _rkp(joint)->max[1] );
}

/* set joint displacement */
static void _rkJointCylinSetDis(rkJoint *joint, double *val){
  _rkJointCylinTestDis( joint, val, _rks(joint)->dis );
  zSinCos( _rks(joint)->dis[1], &_rks(joint)->_s, &_rks(joint)->_c );
}

static void _rkJointCylinSetMin(rkJoint *joint, double *val){
  memcpy( _rkp(joint)->min, val, sizeof(double)*2 );
}

static void _rkJointCylinSetMax(rkJoint *joint, double *val){
  memcpy( _rkp(joint)->max, val, sizeof(double)*2 );
}

static void _rkJointCylinSetVel(rkJoint *joint, double *val){
  memcpy( _rks(joint)->vel, val, sizeof(double)*2 );
}

static void _rkJointCylinSetAcc(rkJoint *joint, double *val){
  memcpy( _rks(joint)->acc, val, sizeof(double)*2 );
}

static void _rkJointCylinSetTrq(rkJoint *joint, double *val){
  memcpy( _rks(joint)->trq, val, sizeof(double)*2 );
}

/* get joint displacement, velocity, acceleration and torque */
static void _rkJointCylinGetDis(rkJoint *joint, double *val){
  memcpy( val, _rks(joint)->dis, sizeof(double)*2 );
}

static void _rkJointCylinGetMin(rkJoint *joint, double *val){
  memcpy( val, _rkp(joint)->min, sizeof(double)*2 );
}

static void _rkJointCylinGetMax(rkJoint *joint, double *val){
  memcpy( val, _rkp(joint)->max, sizeof(double)*2 );
}

static void _rkJointCylinGetVel(rkJoint *joint, double *val){
  memcpy( val, _rks(joint)->vel, sizeof(double)*2 );
}

static void _rkJointCylinGetAcc(rkJoint *joint, double *val){
  memcpy( val, _rks(joint)->acc, sizeof(double)*2 );
}

static void _rkJointCylinGetTrq(rkJoint *joint, double *val){
  memcpy( val, _rks(joint)->trq, sizeof(double)*2 );
}

static void _rkJointCylinCatDis(rkJoint *joint, double *dis, double k, double *val){
  dis[0] += val[0] * k;
  dis[1] += val[1] * k;
}

static void _rkJointCylinSubDis(rkJoint *joint, double *dis, double *sdis){
  dis[0] -= sdis[0];
  dis[1] -= sdis[1];
}

/* continuously update joint displacement over delta time */
static void _rkJointCylinSetDisCNT(rkJoint *joint, double *val, double dt){
  double olddis[2], oldvel[2];

  _rkJointCylinGetDis( joint, olddis );
  _rkJointCylinGetVel( joint, oldvel );
  _rkJointCylinSetDis( joint, val );
  _rks(joint)->vel[0] = ( val[0] - olddis[0] ) / dt;
  _rks(joint)->vel[1] = ( val[1] - olddis[1] ) / dt;
  _rks(joint)->acc[0] = ( _rks(joint)->vel[0] - oldvel[0] ) / dt;
  _rks(joint)->acc[1] = ( _rks(joint)->vel[1] - oldvel[1] ) / dt;
}

/* joint frame transformation */
static zFrame3D *_rkJointCylinXform(rkJoint *joint, zFrame3D *fo, zFrame3D *f){
  /* rotation */
  _rkJointRotateZ( joint, fo, f );
  /* slide */
  zVec3DCat( zFrame3DPos(fo), _rks(joint)->dis[0], &zFrame3DAtt(fo)->v[2], zFrame3DPos(f) );
  return f;
}

/* joint velocity transformation */
static void _rkJointCylinIncVel(rkJoint *joint, zVec6D *vel){
  vel->e[zZ ] += _rks(joint)->vel[0];
  vel->e[zZA] += _rks(joint)->vel[1];
}

static void _rkJointCylinIncAccOnVel(rkJoint *joint, zVec3D *w, zVec6D *acc){
  acc->e[zX ] += 2 * _rks(joint)->vel[0] * w->e[zY];
  acc->e[zY ] -= 2 * _rks(joint)->vel[0] * w->e[zX];
  acc->e[zXA] += _rks(joint)->vel[1] * w->e[zY];
  acc->e[zYA] -= _rks(joint)->vel[1] * w->e[zX];
}

/* joint acceleration transformation */
static void _rkJointCylinIncAcc(rkJoint *joint, zVec6D *acc){
  acc->e[zZ ] += _rks(joint)->acc[0];
  acc->e[zZA] += _rks(joint)->acc[1];
}

/* joint torque transformation */
static void _rkJointCylinCalcTrq(rkJoint *joint, zVec6D *f){
  _rks(joint)->trq[0] = f->e[zZ];
  _rks(joint)->trq[1] = f->e[zZA];
}

/* inverse computation of joint torsion and displacement */
static void _rkJointCylinTorsion(zFrame3D *dev, zVec6D *t, double dis[]){
  dis[0] = rkJointPrismTorsionDis( dev, t );
  dis[1] = rkJointRevolTorsionDis( dev, t );
}

/* joint axes */
static zVec3D* (*_rk_joint_cylin_axis_ang[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisNull,
  _rkJointAxisZ,
};
static zVec3D* (*_rk_joint_cylin_axis_lin[])(rkJoint*,zFrame3D*,zVec3D*) = {
  _rkJointAxisZ,
  _rkJointAxisNull,
};

/* CRB method */

static void _rkJointCylinCRBWrench(rkJoint *joint, rkMP *crb, zVec6D wi[]){
  _rkJointCRBWrenchLinZ( crb, &wi[0] );
  _rkJointCRBWrenchAngZ( crb, &wi[1] );
}
static void _rkJointCylinCRBXform(rkJoint *joint, zFrame3D *f, zVec6D si[]){
  _rkJointCRBXformLin( f, zZ, &si[0] );
  _rkJointCRBXformAng( f, zZ, &si[1] );
}

static void _rkJointCylinSetFrictionPivot(rkJoint *joint, rkJointFrictionPivot *fp){
  fp[0] = _rks(joint)->_fp[0];
  fp[1] = _rks(joint)->_fp[1];
}

static void _rkJointCylinGetFrictionPivot(rkJoint *joint, rkJointFrictionPivot *fp){
  _rks(joint)->_fp[0] = fp[0];
  _rks(joint)->_fp[1] = fp[1];
}

static void _rkJointCylinSetFriction(rkJoint *joint, double *val)
{
  _rkp(joint)->tf[0] = val[0];
  _rkp(joint)->tf[1] = val[1];
}
static void _rkJointCylinGetFriction(rkJoint *joint, double *val){
  val[0] = _rkp(joint)->tf[0];
  val[1] = _rkp(joint)->tf[1];
}
static void _rkJointCylinGetSFriction(rkJoint *joint, double *val){
  val[0] = _rkp(joint)->sf[0];
  val[1] = _rkp(joint)->sf[1];
}
static void _rkJointCylinGetKFriction(rkJoint *joint, double *val){
  val[0] = _rkJointRestTrq( _rkp(joint)->stiffness[0], _rkp(joint)->viscosity[0], _rkp(joint)->coulomb[0], _rks(joint)->dis[0], _rks(joint)->vel[0] );
  val[1] = _rkJointRestTrq( _rkp(joint)->stiffness[1], _rkp(joint)->viscosity[1], _rkp(joint)->coulomb[1], _rks(joint)->dis[1], _rks(joint)->vel[1] );
}

/* motor */

static void _rkJointCylinMotorInertia(rkJoint *joint, double *val){ val[0] = val[1] = val[2] = val[3] = 0; }
static void _rkJointCylinMotorInputTrq(rkJoint *joint, double *val){ val[0] = val[1] = 0; }
static void _rkJointCylinMotorResistance(rkJoint *joint, double *val){ val[0] = val[1] = 0; }
static void _rkJointCylinMotorDrivingTrq(rkJoint *joint, double *val){ val[0] = val[1] = 0; }

/* ABI */

static void _rkJointCylinABIAxisInertia(rkJoint *joint, zMat6D *m, zMat h, zMat ih){
  zMatElemNC(h,0,0) = m->e[0][0].e[2][2];
  zMatElemNC(h,1,0) = m->e[0][1].e[2][2];
  zMatElemNC(h,0,1) = m->e[1][0].e[2][2];
  zMatElemNC(h,1,1) = m->e[1][1].e[2][2];
  zMatInv( h, ih );
}

static void _rkJointCylinABIAddABI(rkJoint *joint, zMat6D *m, zFrame3D *f, zMat h, zMat6D *pm){
  zVec6D v13, v31, v16, v61, tmpv;
  zMat6D tmpm, tmpm2;

  zMat6DCol( m, zZ,  &v13 );
  zMat6DCol( m, zZA, &v16 );
  zMat6DRow( m, zZ,  &v31 );
  zMat6DRow( m, zZA, &v61 );

  zVec6DMul( &v13, zMatElemNC(h,0,0), &tmpv );
  zMat6DDyad( &tmpm, &tmpv, &v31 );
  zVec6DMul( &v13, zMatElemNC(h,0,1), &tmpv );
  zMat6DDyad( &tmpm2, &tmpv, &v61 );
  zMat6DAddDRC( &tmpm, &tmpm2 );
  zVec6DMul( &v16, zMatElemNC(h,1,0), &tmpv );
  zMat6DDyad( &tmpm2, &tmpv, &v31 );
  zMat6DAddDRC( &tmpm, &tmpm2 );
  zVec6DMul( &v16, zMatElemNC(h,1,1), &tmpv );
  zMat6DDyad( &tmpm2, &tmpv, &v61 );
  zMat6DAddDRC( &tmpm, &tmpm2 );

  rkJointXformMat6D( f, &tmpm, &tmpm );
  zMat6DAddDRC( pm, &tmpm );
}

static void _rkJointCylinABIAddBias(rkJoint *joint, zMat6D *m, zVec6D *b, zFrame3D *f, zMat h, zVec6D *pb){
  zVec6D v13, v16, tmpv;
  zVec3D tmp3v;

  zMat6DCol( m, zZ,  &v13 );
  zMat6DCol( m, zZA, &v16 );
  zVec6DCat(b,        (_rks(joint)->_u[0] - b->e[zZ])*zMatElemNC(h,0,0) + (_rks(joint)->_u[1] - b->e[zZA])*zMatElemNC(h,0,1), &v13, &tmpv );
  zVec6DCatDRC(&tmpv, (_rks(joint)->_u[0] - b->e[zZ])*zMatElemNC(h,1,0) + (_rks(joint)->_u[1] - b->e[zZA])*zMatElemNC(h,1,1), &v16 );

  zMulMat3DVec6D( zFrame3DAtt(f), &tmpv, &v13 );
  zVec6DAngShiftDRC( &tmpv, zFrame3DPos(f) );
  zVec3DAddDRC( zVec6DAng(&v13), zVec3DOuterProd( zFrame3DPos(f), zVec6DLin(&v13), &tmp3v ) );
  zVec6DAddDRC( pb, &v13 );
}

static void _rkJointCylinABIDrivingTorque(rkJoint *joint){
  _rks(joint)->_u[0] = _rkp(joint)->tf[0];
  _rks(joint)->_u[1] = _rkp(joint)->tf[1];
}

static void _rkJointCylinABIQAcc(rkJoint *joint, zMat6D *m, zVec6D *b, zVec6D *jac, zMat h, zVec6D *acc){
  double u[2];
  zVec6D v31, v61;

  zMat6DRow( m, zZ, &v31 );
  zMat6DRow( m, zZA, &v61 );
  u[0] = _rks(joint)->_u[0] - zVec6DInnerProd( &v31, jac ) + b->e[zZ];
  u[1] = _rks(joint)->_u[1] - zVec6DInnerProd( &v61, jac ) + b->e[zZA];
  /* q */
  _rks(joint)->acc[0] = u[0]*zMatElemNC(h,0,0) + u[1]*zMatElemNC(h,0,1);
  _rks(joint)->acc[1] = u[0]*zMatElemNC(h,1,0) + u[1]*zMatElemNC(h,1,1);
  /* acc */
  zVec6DCopy( jac, acc );
  acc->e[zZ ] += _rks(joint)->acc[0];
  acc->e[zZA] += _rks(joint)->acc[1];
}

/* ZTK */

static void *_rkJointCylinDisFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  double dis[2];
  dis[0] = ZTKDouble(ztk);
  dis[1] = zDeg2Rad(ZTKDouble(ztk));
  _rkJointCylinSetDis( (rkJoint *)joint, dis );
  return joint;
}
static void *_rkJointCylinMinFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->min[0] = ZTKDouble(ztk);
  _rkp(joint)->min[1] = zDeg2Rad(ZTKDouble(ztk));
  return joint;
}
static void *_rkJointCylinMaxFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->max[0] = ZTKDouble(ztk);
  _rkp(joint)->max[1] = zDeg2Rad(ZTKDouble(ztk));
  return joint;
}
static void *_rkJointCylinStiffnessFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->stiffness[0] = ZTKDouble(ztk);
  _rkp(joint)->stiffness[1] = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointCylinViscosityFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->viscosity[0] = ZTKDouble(ztk);
  _rkp(joint)->viscosity[1] = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointCylinCoulombFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->coulomb[0] = ZTKDouble(ztk);
  _rkp(joint)->coulomb[1] = ZTKDouble(ztk);
  return joint;
}
static void *_rkJointCylinStaticFrictionFromZTK(void *joint, int i, void *arg, ZTK *ztk){
  _rkp(joint)->sf[0] = ZTKDouble(ztk);
  _rkp(joint)->sf[1] = ZTKDouble(ztk);
  return joint;
}

static bool _rkJointCylinDisFPrintZTK(FILE *fp, int i, void *joint){
  if( zIsTiny( _rks(joint)->dis[0] ) && zIsTiny( _rks(joint)->dis[1] ) ) return false;
  fprintf( fp, "%.10g %.10g\n",
    _rks(joint)->dis[0],
    zRad2Deg(_rks(joint)->dis[1]) );
  return true;
}
static bool _rkJointCylinMinFPrintZTK(FILE *fp, int i, void *joint){
  if( zIsInf( -_rkp(joint)->min[0] ) && zIsInf( -_rkp(joint)->min[1] ) ) return false;
  fprintf( fp, "%.10g %.10g\n",
    _rkp(joint)->min[0],
    zRad2Deg(_rkp(joint)->min[1]) );
  return true;
}
static bool _rkJointCylinMaxFPrintZTK(FILE *fp, int i, void *joint){
  if( zIsInf( _rkp(joint)->max[0] ) && zIsInf( _rkp(joint)->max[1] ) ) return false;
  fprintf( fp, "%.10g %.10g\n",
    _rkp(joint)->max[0],
    zRad2Deg(_rkp(joint)->max[1]) );
  return true;
}
static bool _rkJointCylinStiffnessFPrintZTK(FILE *fp, int i, void *joint){
  if( zIsTiny( _rkp(joint)->stiffness[0] ) && zIsTiny( _rkp(joint)->stiffness[1] ) ) return false;
  fprintf( fp, "%.10g %.10g\n",
    _rkp(joint)->stiffness[0],
    _rkp(joint)->stiffness[1] );
  return true;
}
static bool _rkJointCylinViscosityFPrintZTK(FILE *fp, int i, void *joint){
  if( zIsTiny( _rkp(joint)->viscosity[0] ) && zIsTiny( _rkp(joint)->viscosity[1] ) ) return false;
  fprintf( fp, "%.10g %.10g\n",
    _rkp(joint)->viscosity[0],
    _rkp(joint)->viscosity[1] );
  return true;
}
static bool _rkJointCylinCoulombFPrintZTK(FILE *fp, int i, void *joint){
  if( zIsTiny( _rkp(joint)->coulomb[0] ) && zIsTiny( _rkp(joint)->coulomb[1] ) ) return false;
  fprintf( fp, "%.10g %.10g\n",
    _rkp(joint)->coulomb[0],
    _rkp(joint)->coulomb[1] );
  return true;
}
static bool _rkJointCylinStaticFrictionFPrintZTK(FILE *fp, int i, void *joint){
  if( zIsTiny( _rkp(joint)->sf[0] ) && zIsTiny( _rkp(joint)->sf[1] ) ) return false;
  fprintf( fp, "%.10g %.10g\n",
    _rkp(joint)->sf[0],
    _rkp(joint)->sf[1] );
  return true;
}

static const ZTKPrp __ztk_prp_rkjoint_cylin[] = {
  { ZTK_KEY_ROKI_JOINT_DIS,            1, _rkJointCylinDisFromZTK, _rkJointCylinDisFPrintZTK },
  { ZTK_KEY_ROKI_JOINT_MIN,            1, _rkJointCylinMinFromZTK, _rkJointCylinMinFPrintZTK },
  { ZTK_KEY_ROKI_JOINT_MAX,            1, _rkJointCylinMaxFromZTK, _rkJointCylinMaxFPrintZTK },
  { ZTK_KEY_ROKI_JOINT_STIFFNESS,      1, _rkJointCylinStiffnessFromZTK, _rkJointCylinStiffnessFPrintZTK },
  { ZTK_KEY_ROKI_JOINT_VISCOSITY,      1, _rkJointCylinViscosityFromZTK, _rkJointCylinViscosityFPrintZTK },
  { ZTK_KEY_ROKI_JOINT_COULOMB,        1, _rkJointCylinCoulombFromZTK, _rkJointCylinCoulombFPrintZTK },
  { ZTK_KEY_ROKI_JOINT_STATICFRICTION, 1, _rkJointCylinStaticFrictionFromZTK, _rkJointCylinStaticFrictionFPrintZTK },
};

static rkJoint *_rkJointCylinFromZTK(rkJoint *joint, rkMotorSpecArray *motorspecarray, ZTK *ztk)
{
  return rkJointPrpFromZTK( joint, motorspecarray, ztk, __ztk_prp_rkjoint_cylin );
}

static void _rkJointCylinFPrintZTK(FILE *fp, rkJoint *joint, char *name)
{
  _ZTKPrpKeyFPrint( fp, joint, __ztk_prp_rkjoint_cylin );
}

rkJointCom rk_joint_cylin = {
  "cylindrical",
  2,
  _rkJointCylinInit,
  _rkJointCylinAllocPrp,
  _rkJointCylinAllocState,
  _rkJointCylinCopyPrp,
  _rkJointCylinCopyState,
  _rkJointCylinTestDis,
  _rkJointCylinSetDis,
  _rkJointCylinSetMin,
  _rkJointCylinSetMax,
  _rkJointCylinSetVel,
  _rkJointCylinSetAcc,
  _rkJointCylinSetTrq,
  _rkJointCylinGetDis,
  _rkJointCylinGetMin,
  _rkJointCylinGetMax,
  _rkJointCylinGetVel,
  _rkJointCylinGetAcc,
  _rkJointCylinGetTrq,
  _rkJointCylinCatDis,
  _rkJointCylinSubDis,
  _rkJointCylinSetDisCNT,
  _rkJointCylinXform,
  _rkJointCylinIncVel,
  _rkJointCylinIncAccOnVel,
  _rkJointCylinIncAcc,
  _rkJointCylinCalcTrq,
  _rkJointCylinTorsion,
  _rk_joint_cylin_axis_ang,
  _rk_joint_cylin_axis_lin,

  _rkJointCylinCRBWrench,
  _rkJointCylinCRBXform,

  _rkJointCylinSetFrictionPivot,
  _rkJointCylinGetFrictionPivot,
  _rkJointCylinSetFriction,
  _rkJointCylinGetFriction,
  _rkJointCylinGetSFriction,
  _rkJointCylinGetKFriction,

  rkJointMotorSetValDummy,
  _rkJointCylinMotorInertia,
  _rkJointCylinMotorInputTrq,
  _rkJointCylinMotorResistance,
  _rkJointCylinMotorDrivingTrq,

  _rkJointCylinABIAxisInertia,
  _rkJointCylinABIAddABI,
  _rkJointCylinABIAddBias,
  _rkJointCylinABIDrivingTorque,
  _rkJointCylinABIQAcc,
  _rkJointUpdateWrench,

  _rkJointCylinDisFromZTK,
  _rkJointCylinFromZTK,
  _rkJointCylinDisFPrintZTK,
  _rkJointCylinFPrintZTK,
};

#undef _rks
#undef _rkp
