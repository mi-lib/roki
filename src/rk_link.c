/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_link - link structure, kinematics and dynamics
 */

#include <roki/rk_link.h>

/* ********************************************************** */
/* CLASS: rkLink
 * link class
 * ********************************************************** */

/* rkLinkInit
 * - initialize a link.
 */
void rkLinkInit(rkLink *l)
{
  rkLinkSetOffset( l, -1 );
  zNameSetPtr( l, NULL );
  rkJointInit( rkLinkJoint(l) );
  rkBodyInit( rkLinkBody(l) );

  rkLinkSetOrgFrame( l, ZFRAME3DIDENT );
  rkLinkSetAdjFrame( l, ZFRAME3DIDENT );
  rkLinkSetWrench( l, ZVEC6DZERO );

  rkLinkSetParent( l, NULL );
  rkLinkSetChild( l, NULL );
  rkLinkSetSibl( l, NULL );

  l->_util = NULL;
}

/* rkLinkDestroy
 * - destroy a link.
 */
void rkLinkDestroy(rkLink *l)
{
  zNameDestroy( l );
  rkJointDestroy( rkLinkJoint(l) );
  rkLinkExtWrenchDestroy( l );
  rkLinkShapeDestroy( l );
  rkLinkInit( l );
}

/* rkLinkClone
 * - clone a link.
 */
rkLink *rkLinkClone(rkLink *org, rkLink *cln, zMShape3D *so, zMShape3D *sc)
{
  if( !zNameSet( cln, zName(org) ) ||
      !rkJointClone( rkLinkJoint(org), rkLinkJoint(cln) ) ||
      !rkBodyClone( rkLinkBody(org), rkLinkBody(cln), so, sc ) ){
    ZALLOCERROR();
    return NULL;
  }
  rkLinkSetOffset( cln, rkLinkOffset(org) );
  rkLinkSetOrgFrame( cln, rkLinkOrgFrame(org) );

  rkLinkSetParent( cln,
    rkLinkParent(org) ? rkLinkParent(org) - org + cln : NULL );
  rkLinkSetChild( cln,
    rkLinkChild(org) ? rkLinkChild(org) - org + cln : NULL );
  rkLinkSetSibl( cln,
    rkLinkSibl(org) ? rkLinkSibl(org) - org + cln : NULL );
  return cln;
}

/* rkLinkCopyState
 * - copy link state.
 */
rkLink *rkLinkCopyState(rkLink *src, rkLink *dst)
{
  rkJointCopyState( rkLinkJoint(src), rkLinkJoint(dst) );
  rkBodyCopyState( rkLinkBody(src), rkLinkBody(dst) );
  zFrame3DCopy( rkLinkAdjFrame(src), rkLinkAdjFrame(dst) );
  zVec6DCopy( rkLinkWrench(src), rkLinkWrench(dst) );
  return dst;
}

/* rkLinkAddSibl
 * - add a sibling link.
 */
rkLink *rkLinkAddSibl(rkLink *l, rkLink *bl)
{
  for( ; rkLinkSibl(l); l=rkLinkSibl(l) );
  return rkLinkSetSibl( l, bl );
}

/* rkLinkAddChild
 * - add a child link.
 */
rkLink *rkLinkAddChild(rkLink *l, rkLink *cl)
{
  rkLinkSetParent( cl, l );
  if( !rkLinkChild(l) ) return rkLinkSetChild( l, cl );
  return rkLinkAddSibl( rkLinkChild(l), cl );
}

/* rkLinkPointVel
 * - calculate velocity of a point with respect to the inertial frame.
 */
zVec3D *rkLinkPointVel(rkLink *l, zVec3D *p, zVec3D *v)
{
  zVec3DOuterProd( rkLinkAngVel(l), p, v );
  return zVec3DAddDRC( v, rkLinkLinVel(l) );
}

/* rkLinkPointAcc
 * - calculate accerelation of a point with respect to the inertial frame.
 */
zVec3D *rkLinkPointAcc(rkLink *l, zVec3D *p, zVec3D *a)
{
  zVec3D tmp;

  zVec3DTripleProd( rkLinkAngVel(l), rkLinkAngVel(l), p, a );
  zVec3DOuterProd( rkLinkAngAcc(l), p, &tmp );
  zVec3DAddDRC( a, &tmp );
  return zVec3DAddDRC( a, rkLinkLinAcc(l) );
}

/* rkLinkWldInertia
 * - compute inertia tensor of a link with respect to the inertial frame.
 */
zMat3D *rkLinkWldInertia(rkLink *l, zMat3D *i)
{
  return zRotMat3D( rkLinkWldAtt(l), rkLinkInertia(l), i );
}

/* rkLinkUpdateFrame
 *  - update link frame with respect to the world frame.
 */
void rkLinkUpdateFrame(rkLink *l, zFrame3D *pwf)
{
  rkJointXfer( rkLinkJoint(l), rkLinkOrgFrame(l), rkLinkAdjFrame(l) );
  zFrame3DCascade( pwf, rkLinkAdjFrame(l), rkLinkWldFrame(l) );
  rkBodyUpdateCOM( rkLinkBody(l) );

  if( rkLinkChild(l) )
    rkLinkUpdateFrame( rkLinkChild(l), rkLinkWldFrame(l) );
  if( rkLinkSibl(l) )
    rkLinkUpdateFrame( rkLinkSibl(l), rkLinkWldFrame(rkLinkParent(l)) );
}

void _rkLinkUpdateVel(rkLink *l, zVec6D *pvel)
{
  /* velocity */
  zXfer6DLin( rkLinkAdjFrame(l), pvel, rkLinkVel(l) );
  /* joint motion rate */
  rkJointIncVel( rkLinkJoint(l), rkLinkVel(l) );
  /* COM velocity and acceleration */
  rkBodyUpdateCOMVel( rkLinkBody(l) );
}

void rkLinkUpdateVel(rkLink *l, zVec6D *pvel)
{
  _rkLinkUpdateVel( l, pvel );
  if( rkLinkChild(l) )
    rkLinkUpdateVel( rkLinkChild(l), rkLinkVel(l) );
  if( rkLinkSibl(l) )
    rkLinkUpdateVel( rkLinkSibl(l), rkLinkVel(rkLinkParent(l)) );
}

void _rkLinkUpdateAcc(rkLink *l, zVec6D *pvel, zVec6D *pacc)
{
  zVec3D wp, tmp;

  /* acceleration */
  zVec6DLinShift( pacc, rkLinkAdjPos(l), rkLinkAcc(l) );
  zVec3DOuterProd( zVec6DAng(pvel), rkLinkAdjPos(l), &wp );
  zVec3DOuterProd( zVec6DAng(pvel), &wp, &tmp );
  zVec3DAddDRC( rkLinkLinAcc(l), &tmp );
  zMulMatTVec6DDRC( rkLinkAdjAtt(l), rkLinkAcc(l) );
  /* joint motion rate */
  zVec3DCopy( rkLinkAngVel(l), &tmp );
  rkJointIncAccOnVel( rkLinkJoint(l), &tmp, rkLinkAcc(l) );
  rkJointIncAcc( rkLinkJoint(l), rkLinkAcc(l) );
  /* COM velocity and acceleration */
  rkBodyUpdateCOMAcc( rkLinkBody(l) );
}

void rkLinkUpdateAcc(rkLink *l, zVec6D *pvel, zVec6D *pacc)
{
  _rkLinkUpdateAcc( l, pvel, pacc );
  if( rkLinkChild(l) )
    rkLinkUpdateAcc( rkLinkChild(l), rkLinkVel(l), rkLinkAcc(l) );
  if( rkLinkSibl(l) )
    rkLinkUpdateAcc( rkLinkSibl(l), rkLinkVel(rkLinkParent(l)), rkLinkAcc(rkLinkParent(l)) );
}

/* rkLinkUpdateRate
 * - update link motion rate with respect to the inertial frame.
 */
void rkLinkUpdateRate(rkLink *l, zVec6D *pvel, zVec6D *pacc)
{
  _rkLinkUpdateVel( l, pvel );
  _rkLinkUpdateAcc( l, pvel, pacc );
  if( rkLinkChild(l) )
    rkLinkUpdateRate( rkLinkChild(l), rkLinkVel(l), rkLinkAcc(l) );
  if( rkLinkSibl(l) )
    rkLinkUpdateRate( rkLinkSibl(l), rkLinkVel(rkLinkParent(l)), rkLinkAcc(rkLinkParent(l)) );
}

/* rkLinkUpdateWrench
 * - update joint torque of link based on Neuton=Euler's equation.
 */
void rkLinkUpdateWrench(rkLink *l)
{
  zVec6D w;
  rkLink *child;

  /* inertia force */
  rkBodyNetWrench( rkLinkBody(l), rkLinkWrench(l) );
  zVec6DAngShiftDRC( rkLinkWrench(l), rkLinkCOM(l) );
  /* reaction force propagation from children */
  if( ( child = rkLinkChild(l) ) ){
    rkLinkUpdateWrench( child );
    for( ; child; child=rkLinkSibl(child) ){
      zXfer6DAng( rkLinkAdjFrame(child), rkLinkWrench(child), &w );
      zVec6DAddDRC( rkLinkWrench(l), &w );
    }
  }
  rkLinkCalcExtWrench( l, &w ); /* external wrench */
  zVec6DSubDRC( rkLinkWrench(l), &w );
  /* joint torque resolution */
  rkJointCalcTrq( rkLinkJoint(l), rkLinkWrench(l) );
  /* branch */
  if( rkLinkSibl(l) )
    rkLinkUpdateWrench( rkLinkSibl(l) );
}

void rkLinkConfToJointDis(rkLink *link)
{
  zFrame3D org, dev;
  double dis[6];
  zVec6D tor;

  if( rkLinkParent(link) ){
    zFrame3DCascade( rkLinkWldFrame(rkLinkParent(link)), rkLinkOrgFrame(link), &org );
  } else{
    zFrame3DCopy( rkLinkOrgFrame(link), &org );
  }
  zFrame3DXfer( &org, rkLinkWldFrame(link), &dev );
  rkJointTorsion( rkLinkJoint(link), &dev, &tor, dis );
  rkJointSetDis( rkLinkJoint(link), dis );
  /* recursive computation */
  if( rkLinkChild(link) )
    rkLinkConfToJointDis( rkLinkChild(link) );
  if( rkLinkSibl(link) )
    rkLinkConfToJointDis( rkLinkSibl(link) );
}

typedef struct{
  rkLink *l;
  rkLink *larray;
  int nl;
  zShape3D *sarray;
  int ns;
  rkMotor *marray;
  int nm;
} _rkLinkParam;

static bool _rkLinkFRead(FILE *fp, void *instance, char *buf, bool *success);

/* rkLinkFRead
 * - input link properties from file.
 */
bool _rkLinkFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  _rkLinkParam *prm;
  rkLink *pl, *cl;
  zShape3D *sp;

  prm = instance;
  if( strcmp( buf, "name" ) == 0 ){
    if( strlen( zFToken( fp, buf, BUFSIZ ) ) >= BUFSIZ ){
      buf[BUFSIZ-1] = '\0';
      ZRUNWARN( "too long link name, truncated to %s", buf );
    }
    zNameFind( prm->larray, prm->nl, buf, cl );
    if( cl ){
      ZRUNERROR( "twofolded link %s exists", buf );
      return false;
    }
    if( !( zNameSet( prm->l, buf ) ) ){
      ZALLOCERROR();
      return ( *success = false );
    }
  } else if( strcmp( buf, "jointtype" ) == 0 )
    rkJointCreate( rkLinkJoint(prm->l),
      rkJointTypeFromStr( zFToken( fp, buf, BUFSIZ ) ) );
  else if( strcmp( buf, "mass" ) == 0 )
    rkLinkSetMass( prm->l, zFDouble( fp ) );
  else if( strcmp( buf, "stuff" ) == 0 ){
    if( strlen( zFToken( fp, buf, BUFSIZ ) ) >= BUFSIZ ){
      buf[BUFSIZ-1] = '\0';
      ZRUNWARN( "too long stuff name, truncated to %s", buf );
    }
    rkLinkSetStuff( prm->l, buf );
  } else if( strcmp( buf, "COM" ) == 0 )
    zVec3DFRead( fp, rkLinkCOM(prm->l) );
  else if( strcmp( buf, "inertia" ) == 0 )
    zMat3DFRead( fp, rkLinkInertia(prm->l) );
  else if( strcmp( buf, "pos" ) == 0 )
    zVec3DFRead( fp, rkLinkOrgPos(prm->l) );
  else if( strcmp( buf, "att" ) == 0 )
    zMat3DFRead( fp, rkLinkOrgAtt(prm->l) );
  else if( strcmp( buf, "frame" ) == 0 )
    zFrame3DFRead( fp, rkLinkOrgFrame(prm->l) );
  else if( strcmp( buf, "DH" ) == 0 )
    zFrame3DDHFRead( fp, rkLinkOrgFrame(prm->l) );
  else if( strcmp( buf, "parent" ) == 0 ){
    zFToken( fp, buf, BUFSIZ );
    zNameFind( prm->larray, prm->nl, buf, pl );
    if( !pl ){
      ZRUNERROR( "link %s not found", buf );
      return ( *success = false );
    }
    rkLinkAddChild( pl, prm->l );
  } else if( strcmp( buf, "shape" ) == 0 ){
    zFToken( fp, buf, BUFSIZ );
    zNameFind( prm->sarray, prm->ns, buf, sp );
    if( !sp ){
      ZRUNERROR( "shape %s not found", buf );
      return ( *success = false );
    }
    rkLinkShapePush( prm->l, sp );
  } else if( !rkJointQueryFRead( fp, buf, rkLinkJoint(prm->l), prm->marray, prm->nm ) )
    return false;
  return true;
}

rkLink *rkLinkFRead(FILE *fp, rkLink *l, rkLink *larray, int nl, zShape3D *sarray, int ns, rkMotor *marray, int nm)
{
  _rkLinkParam prm;

  prm.l = l;
  prm.larray = larray;
  prm.nl = nl;
  prm.sarray = sarray;
  prm.ns = ns;
  prm.marray = marray;
  prm.nm = nm;
  rkJointCreate( rkLinkJoint(l), RK_JOINT_FIXED );
  if( !zFieldFRead( fp, _rkLinkFRead, &prm ) ) return NULL;
  if( zNamePtr(l) ) return l;
  ZRUNERROR( "unnamed link exists" );
  return NULL;
}

/* rkLinkFWrite
 * - output of link properties to file.
 */
void rkLinkFWrite(FILE *fp, rkLink *l)
{
  zShapeListCell *cp;

  if( !l ){
    fprintf( fp, "(null link)\n" );
    return;
  }
  fprintf( fp, "name: %s\n", zName(l) );
  fprintf( fp, "jointtype: %s\n", rkJointTypeExpr( rkLinkJointType(l) ) );
  rkJointFWrite( fp, rkLinkJoint(l), NULL );
  rkMPFWrite( fp, rkLinkMP(l) );
  if( rkLinkStuff(l) ) fprintf( fp, "stuff: %s\n", rkLinkStuff(l) );
  fprintf( fp, "frame: " );
  zFrame3DFWrite( fp, rkLinkOrgFrame(l) );
  if( !rkLinkShapeIsEmpty(l) )
    zListForEach( rkLinkShapeList(l), cp )
      fprintf( fp, "shape: %s\n", zName( zShapeListCellShape(cp) ) );
  if( rkLinkParent(l) )
    fprintf( fp, "parent: %s\n", zName( rkLinkParent(l) ) );
  fprintf( fp, "\n" );
}

/* rkLinkPostureFWrite
 * - output of link posture to file.
 */
void rkLinkPostureFWrite(FILE *fp, rkLink *l)
{
  fprintf( fp, "Link(name:%s offset:%d)\n", zName(l), rkLinkOffset(l) );
  fprintf( fp, " adjacent frame:\n" );
  zFrame3DFWrite( fp, rkLinkAdjFrame( l ) );
  fprintf( fp, " world frame:\n" );
  zFrame3DFWrite( fp, rkLinkWldFrame( l ) );
}

/* rkLinkConnectionFWrite
 * - output of link connectivity to file.
 */
#define RK_LINK_CONNECTION_INDENT 2
void rkLinkConnectionFWrite(FILE *fp, rkLink *l, int n)
{
  zIndentF( fp, n );
  fprintf( fp, "|-%s (%s:%d)\n", zName(l), rkJointTypeExpr(rkLinkJointType(l)), rkLinkOffset(l) );

  if( rkLinkChild( l ) )
    rkLinkConnectionFWrite( fp, rkLinkChild(l), n+RK_LINK_CONNECTION_INDENT );
  if( rkLinkSibl( l ) )
    rkLinkConnectionFWrite( fp, rkLinkSibl(l), n );
}

/* rkLinkExtWrenchFWrite
 * - output of external wrenches applied to link to file.
 */
void rkLinkExtWrenchFWrite(FILE *fp, rkLink *l)
{
  rkWrench *c;

  zListForEach( rkLinkExtWrench(l), c )
    rkWrenchFWrite( fp, c );
}
