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

/* rkJointTypeExpr
 * - convert joint type to string.
 */
char *rkJointTypeExpr(byte type)
{
  return __rkjointtypename[zLimit(type,RK_JOINT_FIXED,RK_JOINT_BRFLOAT)];
}

/* rkJointTypeFromStr
 * - convert string to joint type.
 */
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

/* rkJointCreate
 * - create joint object.
 */
rkJoint *rkJointCreate(rkJoint *j, byte type)
{
  if( type < RK_JOINT_FIXED || type > RK_JOINT_BRFLOAT ){
    ZRUNERROR( "invalid joint type specified - %d", type );
    return NULL;
  }
  rkJointInit( j );
  if( !rk_joint_create[( (j)->type = type )]( j ) ){
    ZRUNERROR( "cannot create joint instance" );
    rkJointDestroy( j );
    return NULL;
  }
  rkJointNeutral( j );
  return j;
}

/* rkJointDestroy
 * - destroy joint object.
 */
void rkJointDestroy(rkJoint *j)
{
  zFree( j->prp );
  rkJointInit( j );
}

/* rkJointNeutral
 * - neutralize joint displacement.
 */
void rkJointNeutral(rkJoint *j)
{
  double dis[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  rkJointSetDis( j, dis );
}

/* rkJointIsNeutral
 * - check if joint displacement is neutral.
 */
bool rkJointIsNeutral(rkJoint *j)
{
  double dis[6];
  register int i;

  rkJointGetDis( j, dis );
  for( i=0; i<rkJointSize(j); i++ )
    if( !zIsTiny( dis[i] ) ) return false;
  return true;
}

/* rkJointClone
 * - clone a joint .
 */
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

/* rkJointCopyState
 * - copy joint state.
 */
rkJoint *rkJointCopyState(rkJoint *src, rkJoint *dst)
{
  double val[6];

  rkJointGetDis( src, val ); rkJointSetDis( dst, val );
  rkJointGetVel( src, val ); rkJointSetVel( dst, val );
  rkJointGetAcc( src, val ); rkJointSetAcc( dst, val );
  rkJointGetTrq( src, val ); rkJointSetTrq( dst, val );
  return dst;
}

/* rkJointIncRate
 * - increment motion rate due to the joint rate.
 */
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
  zMulMatTVec3D( zFrame3DAtt(dev), &aa, zVec6DAng(t) );
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

  zMulMatTVec3D( zFrame3DAtt(dev), zFrame3DPos(dev), zVec6DLin(t) );
  /* joint displacement */
  q = t->e[zZ];
  t->e[zZ] = 0;
  return q;
}

/* for ABI */
zMat6D *rkJointXferMat6D(zFrame3D *f, zMat6D *i, zMat6D *m)
{
  zMat3D tmpm, tmpm2;

  zMulMatMat3D( zFrame3DAtt( f ), zMat6DMat3D( i, 0, 0 ), zMat6DMat3D( m, 0, 0 ) );
  zMulMatMatT3DDRC( zMat6DMat3D( m, 0, 0 ), zFrame3DAtt( f ) );
  zMulMatMat3D( zFrame3DAtt( f ), zMat6DMat3D( i, 1, 0 ), zMat6DMat3D( m, 1, 0 ) );
  zMulMatMatT3DDRC( zMat6DMat3D( m, 1, 0 ), zFrame3DAtt( f ) );
  zMulMatMat3D( zFrame3DAtt( f ), zMat6DMat3D( i, 1, 1 ), zMat6DMat3D( m, 1, 1 ) );
  zMulMatMatT3DDRC( zMat6DMat3D( m, 1, 1 ), zFrame3DAtt( f ) );

  zMulVecOPMat3D( zFrame3DPos( f ), zMat6DMat3D( m, 0, 0 ), &tmpm );
  zMat3DT( zMat6DMat3D( m, 1, 0 ), &tmpm2 );
  zMat3DAddDRC( zMat6DMat3D( m, 1, 0 ), &tmpm );
  zMat3DT( zMat6DMat3D( m, 1, 0 ), zMat6DMat3D( m, 0, 1 ) );
  zMulVecOPMat3D( zFrame3DPos( f ), zMat6DMat3D( m, 0, 1 ), &tmpm );
  zMat3DAddDRC( zMat6DMat3D( m, 1, 1 ), &tmpm );
  zMulVecOPMat3D( zFrame3DPos( f ), &tmpm2, &tmpm );
  zMat3DT( &tmpm, &tmpm );
  zMat3DAddDRC( zMat6DMat3D( m, 1, 1 ), &tmpm );
  return m;
}

/* void _rkJointUpdateWrench(void *prp, zMat6D *i, zVec6D *b, zVec6D *acc, zVec6D *w) */
/* { */
/*   zMulMat6DVec6D( i, acc, w ); */
/*   zVec6DAddDRC( w, b ); */
/* } */

void _rkJointUpdateWrench(rkJoint *j, zMat6D *i, zVec6D *b, zVec6D *acc)
{
  zMulMat6DVec6D( i, acc, rkJointWrench(j) );
  zVec6DAddDRC( rkJointWrench(j), b );
}
