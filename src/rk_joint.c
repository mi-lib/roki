/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint - joint structure
 */

#include <roki/rk_joint.h>

/* ********************************************************** */
/* joint type
 * ********************************************************** */
static char *__rkjointtypename[] = {
  "fix", "revolute", "prism", "cylinder", "hooke", "sphere", "float", "breakablefloat",
  NULL
};

/* convert joint type to a string. */
char *rkJointTypeExpr(byte type)
{
  return __rkjointtypename[zLimit(type,RK_JOINT_FIXED,RK_JOINT_BRFLOAT)];
}

/* convert a string to joint type. */
byte rkJointTypeFromStr(char *str)
{
  char **jp;
  byte type;

  for( type=RK_JOINT_FIXED, jp=__rkjointtypename; *jp; jp++, type++ )
    if( !strcmp( str, *jp ) ) return type;
  return RK_JOINT_FIXED;
}

/* ********************************************************** */
/* CLASS: rkJoint
 * joint class
 * ********************************************************** */

static rkJoint *(* rk_joint_create[])(rkJoint*) = {
  rkJointCreateFixed,
  rkJointCreateRevol,
  rkJointCreatePrism,
  rkJointCreateCylin,
  rkJointCreateHooke,
  rkJointCreateSpher,
  rkJointCreateFloat,
  rkJointCreateBrFloat,
};

/* create a joint object. */
rkJoint *rkJointCreate(rkJoint *j, byte type)
{
  if( type < RK_JOINT_FIXED || type > RK_JOINT_BRFLOAT ){
    ZRUNERROR( RK_ERR_JOINT_INVTYPE, type );
    return NULL;
  }
  rkJointInit( j );
  if( !rk_joint_create[( (j)->type = type )]( j ) ){
    ZRUNERROR( RK_ERR_JOINT_FAILED );
    rkJointDestroy( j );
    return NULL;
  }
  rkJointNeutral( j );
  return j;
}

/* destroy a joint object. */
void rkJointDestroy(rkJoint *j)
{
  zFree( j->prp );
  rkJointInit( j );
}

/* neutralize joint displacement. */
void rkJointNeutral(rkJoint *j)
{
  double dis[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  rkJointSetDis( j, dis );
}

/* check if joint displacement is neutral. */
bool rkJointIsNeutral(rkJoint *j)
{
  double dis[6];
  register int i;

  rkJointGetDis( j, dis );
  for( i=0; i<rkJointSize(j); i++ )
    if( !zIsTiny( dis[i] ) ) return false;
  return true;
}

/* clone a joint. */
rkJoint *rkJointClone(rkJoint *org, rkJoint *cln)
{
  rkMotor *morg, *mcln;

  if( !rkJointCreate( cln, rkJointType(org) ) ){
    ZALLOCERROR();
    return NULL;
  }
  rkJointCopyState( org, cln );
  rkJointGetMotor( org, &morg );
  rkJointGetMotor( cln, &mcln );
  if( morg != NULL && mcln != NULL )
    if( !rkMotorClone( morg, mcln ) ){
      ZALLOCERROR();
      return NULL;
    }
  return cln;
}

/* copy joint state. */
rkJoint *rkJointCopyState(rkJoint *src, rkJoint *dst)
{
  double val[6];

  rkJointGetDis( src, val ); rkJointSetDis( dst, val );
  rkJointGetVel( src, val ); rkJointSetVel( dst, val );
  rkJointGetAcc( src, val ); rkJointSetAcc( dst, val );
  rkJointGetTrq( src, val ); rkJointSetTrq( dst, val );
  return dst;
}

/* increment motion rate due to joint rate. */
void rkJointIncRate(rkJoint *j, zVec3D *w, zVec6D *vel, zVec6D *acc)
{
  rkJointIncVel( j, vel );
  rkJointIncAccOnVel( j, w, acc );
  rkJointIncAcc( j, acc );
}

/* NOTE: The following macros and functions are for sharing
 * some operation codes. Do not use them in users programs. */
zVec3D *_rkJointAxisNull(void *prp, zFrame3D *f, zVec3D *a){
  return NULL;
}

zVec3D *_rkJointAxisZ(void *prp, zFrame3D *f, zVec3D *a){
  zVec3DCopy( &zFrame3DAtt(f)->e[2], a );
  return a;
}

/* joint torsion */

double rkJointTorsionDisRevol(zFrame3D *dev, zVec6D *t)
{
  zMat3D rm;
  zVec3D aa;
  double l, angle;

  zVec3DCreate( &aa, -zFrame3DAtt(dev)->e[2][1], zFrame3DAtt(dev)->e[2][0], 0 );
  l = sqrt( zSqr(aa.e[zX]) + zSqr(aa.e[zY]) );
  angle = atan2( l, zFrame3DAtt(dev)->e[2][2] );
  zIsTiny( angle ) ?
    zVec3DClear( &aa ) : zVec3DMulDRC( &aa, angle/l );
  zMulMat3DTVec3D( zFrame3DAtt(dev), &aa, zVec6DAng(t) );
  /* intermediate attitude */
  zMat3DFromAA( &rm, &aa );
  /* joint displacement */
  return 0.5 *
    ( zVec3DAngle( &rm.v[zX], &zFrame3DAtt(dev)->v[zX], &rm.v[zZ] )
    + zVec3DAngle( &rm.v[zY], &zFrame3DAtt(dev)->v[zY], &rm.v[zZ] ) );
}

double rkJointTorsionDisPrism(zFrame3D *dev, zVec6D *t)
{
  double q;

  zMulMat3DTVec3D( zFrame3DAtt(dev), zFrame3DPos(dev), zVec6DLin(t) );
  /* joint displacement */
  q = t->e[zZ];
  t->e[zZ] = 0;
  return q;
}

/* for ABI */
zMat6D *rkJointXferMat6D(zFrame3D *f, zMat6D *i, zMat6D *m)
{
  zMat3D tmpm, tmpm2;

  zRotMat3D( zFrame3DAtt(f), &i->e[0][0], &m->e[0][0] );
  zRotMat3D( zFrame3DAtt(f), &i->e[0][1], &m->e[0][1] );
  zRotMat3D( zFrame3DAtt(f), &i->e[1][1], &m->e[1][1] );

  zMulVec3DOPMat3D( zFrame3DPos(f), &m->e[0][0], &tmpm );
  zMat3DT( &m->e[0][1], &tmpm2 );
  zMat3DAddDRC( &m->e[0][1], &tmpm );
  zMat3DT( &m->e[0][1], &m->e[1][0] );
  zMulVec3DOPMat3D( zFrame3DPos(f), &m->e[1][0], &tmpm );
  zMat3DAddDRC( &m->e[1][1], &tmpm );
  zMulVec3DOPMat3D( zFrame3DPos(f), &tmpm2, &tmpm );
  zMat3DT( &tmpm, &tmpm );
  zMat3DAddDRC( &m->e[1][1], &tmpm );
  return m;
}

void _rkJointUpdateWrench(rkJoint *j, zMat6D *i, zVec6D *b, zVec6D *acc)
{
  zMulMat6DVec6D( i, acc, rkJointWrench(j) );
  zVec6DAddDRC( rkJointWrench(j), b );
}
