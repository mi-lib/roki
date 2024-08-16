/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint - joint structure
 */

#include <roki/rk_joint.h>

/* ********************************************************** */
/* CLASS: rkJoint
 * joint class
 * ********************************************************** */

RK_JOINT_COM_ARRAY;

rkJoint *rkJointAssign(rkJoint *joint, rkJointCom *com)
{
  rkJointInit( joint );
  if( ( joint->state = ( joint->com = com )->_alloc_state() ) &&
      ( joint->prp   = ( joint->com = com )->_alloc_prp() ) )
    joint->com->_init( joint );
  rkJointNeutralize( joint );
  return joint;
}

rkJoint *rkJointAssignByStr(rkJoint *joint, const char *str)
{
  int i;

  for( i=0; rk_joint_com[i]; i++ )
    if( strcmp( rk_joint_com[i]->typestr, str ) == 0 )
      return rkJointAssign( joint, rk_joint_com[i] );
  return NULL;
}

/* destroy a joint object. */
void rkJointDestroy(rkJoint *joint)
{
  zFree( joint->state );
  zFree( joint->prp );
  if( rkJointMotor(joint) ){
    rkMotorDestroy( rkJointMotor(joint) );
    zFree( rkJointMotor(joint) );
  }
  rkJointInit( joint );
}

/* clone a joint. */
rkJoint *rkJointClone(rkJoint *org, rkJoint *cln, rkMotorSpecArray *msarray_org, rkMotorSpecArray *msarray_cln)
{
  rkMotor *motor_cln;

  if( !rkJointAssign( cln, org->com ) ) return NULL;
  rkJointCopyState( org, cln );
  rkJointCopyPrp( org, cln );
  if( rkJointMotor(org) ){
    if( !( motor_cln = zAlloc( rkMotor, 1 ) ) ){
      ZALLOCERROR();
      return NULL;
    }
    if( !rkMotorClone( rkJointMotor(org), motor_cln, msarray_org, msarray_cln ) ){
      free( motor_cln );
      rkJointDestroy( cln );
      return NULL;
    }
    rkJointSetMotor( cln, motor_cln );
  }
  return cln;
}

/* copy joint state. */
rkJoint *rkJointCopyState(rkJoint *src, rkJoint *dst)
{
  if( src->com != dst->com ) return NULL;
  src->com->_copy_state( src, dst );
  return dst;
}

/* copy joint propety */
rkJoint *rkJointCopyPrp(rkJoint *src, rkJoint *dst)
{
  if( src->com != dst->com ) return NULL;
  src->com->_copy_prp( src, dst );
  return dst;
}

/* increment motion rate due to joint rate. */
void rkJointIncRate(rkJoint *joint, zVec3D *w, zVec6D *vel, zVec6D *acc)
{
  rkJointIncVel( joint, vel );
  rkJointIncAccOnVel( joint, w, acc );
  rkJointIncAcc( joint, acc );
}

/* neutralize joint displacement. */
void rkJointNeutralize(rkJoint *joint)
{
  double dis[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  rkJointSetDis( joint, dis );
}

/* check if joint displacement is neutral. */
bool rkJointIsNeutral(rkJoint *joint)
{
  double dis[6];
  int i;

  rkJointGetDis( joint, dis );
  for( i=0; i<rkJointDOF(joint); i++ )
    if( !zIsTiny( dis[i] ) ) return false;
  return true;
}

/* NOTE: The following macros and functions are for sharing
 * some operation codes. Do not use them in users programs. */
zVec3D *_rkJointAxisNull(rkJoint *joint, zFrame3D *f, zVec3D *a){
  return NULL;
}

zVec3D *_rkJointAxisZ(rkJoint *joint, zFrame3D *f, zVec3D *a){
  zVec3DCopy( &zFrame3DAtt(f)->e[zZ], a );
  return a;
}

/* joint torsion */

double rkJointRevolTorsionDis(zFrame3D *dev, zVec6D *t)
{
  zMat3D rm;
  zVec3D aa;
  double l, angle;

  zVec3DCreate( &aa, -zFrame3DAtt(dev)->e[2][1], zFrame3DAtt(dev)->e[2][0], 0 );
  l = sqrt( zSqr(aa.e[zX]) + zSqr(aa.e[zY]) );
  angle = atan2( l, zFrame3DAtt(dev)->e[2][2] );
  zIsTiny( angle ) ?
    zVec3DZero( &aa ) : zVec3DMulDRC( &aa, angle/l );
  zMulMat3DTVec3D( zFrame3DAtt(dev), &aa, zVec6DAng(t) );
  /* intermediate attitude */
  zMat3DFromAA( &rm, &aa );
  /* joint displacement */
  return 0.5 *
    ( zVec3DAngle( &rm.v[zX], &zFrame3DAtt(dev)->v[zX], &rm.v[zZ] )
    + zVec3DAngle( &rm.v[zY], &zFrame3DAtt(dev)->v[zY], &rm.v[zZ] ) );
}

double rkJointPrismTorsionDis(zFrame3D *dev, zVec6D *t)
{
  double q;

  zMulMat3DTVec3D( zFrame3DAtt(dev), zFrame3DPos(dev), zVec6DLin(t) );
  /* joint displacement */
  q = t->e[zZ];
  t->e[zZ] = 0;
  return q;
}

/* for ABI */
zMat6D *rkJointXformMat6D(zFrame3D *frame, zMat6D *i, zMat6D *m)
{
  zMat3D tmpm, tmpm2;

  zRotMat3D( zFrame3DAtt(frame), &i->e[0][0], &m->e[0][0] );
  zRotMat3D( zFrame3DAtt(frame), &i->e[0][1], &m->e[0][1] );
  zRotMat3D( zFrame3DAtt(frame), &i->e[1][1], &m->e[1][1] );

  zMulVec3DOuterProdMat3D( zFrame3DPos(frame), &m->e[0][0], &tmpm );
  zMat3DT( &m->e[0][1], &tmpm2 );
  zMat3DAddDRC( &m->e[0][1], &tmpm );
  zMat3DT( &m->e[0][1], &m->e[1][0] );
  zMulVec3DOuterProdMat3D( zFrame3DPos(frame), &m->e[1][0], &tmpm );
  zMat3DAddDRC( &m->e[1][1], &tmpm );
  zMulVec3DOuterProdMat3D( zFrame3DPos(frame), &tmpm2, &tmpm );
  zMat3DT( &tmpm, &tmpm );
  zMat3DAddDRC( &m->e[1][1], &tmpm );
  return m;
}

void _rkJointUpdateWrench(rkJoint *joint, zMat6D *i, zVec6D *b, zVec6D *acc)
{
  zMulMat6DVec6D( i, acc, rkJointWrench(joint) );
  zVec6DAddDRC( rkJointWrench(joint), b );
}

/* motor */

void rkJointMotorSetValDummy(rkJoint *joint, double *val){}
void rkJointMotorGetValDummy(rkJoint *joint, double *val){}

rkJoint *rkJointAssignMotorByStr(rkJoint *joint, rkMotorSpecArray *msarray, const char *str)
{
  rkMotorSpec *ms;

  if( !( ms = rkMotorSpecArrayFind( msarray, str ) ) ){
    ZRUNERROR( RK_ERR_MOTOR_UNKNOWN, str );
    return NULL;
  }
  if( rkJointDOF(joint) != rkMotorSpecDOF(ms) ){
    ZRUNERROR( RK_ERR_JOINT_MISMATCH_DOF, rkJointDOF(joint), rkMotorSpecDOF(ms) );
    return NULL;
  }
  if( !( rkJointMotor(joint) = zAlloc( rkMotor, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  return rkMotorCreate( rkJointMotor(joint), ms ) ? joint : NULL;
}

/* ZTK */

rkJoint *rkJointFromZTK(rkJoint *joint, rkMotorSpecArray *motorspecarray, ZTK *ztk)
{
  return joint->com->_fromZTK( joint, motorspecarray, ztk );
}
