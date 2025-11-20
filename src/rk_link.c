/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_link - link structure, kinematics and dynamics
 */

#include <roki/rk_link.h>

/* ********************************************************** */
/* link class
 * ********************************************************** */

/* initialize a link. */
void rkLinkInit(rkLink *link)
{
  rkLinkSetJointIDOffset( link, -1 );
  zNameSetPtr( link, NULL );
  rkJointInit( rkLinkJoint(link) );
  rkBodyInit( rkLinkBody(link) );

  rkLinkSetOrgFrame( link, ZFRAME3DIDENT );
  rkLinkSetAdjFrame( link, ZFRAME3DIDENT );
  rkLinkSetJointWrench( link, ZVEC6DZERO );

  rkLinkSetParent( link, NULL );
  rkLinkSetChild( link, NULL );
  rkLinkSetSibl( link, NULL );
}

/* destroy a link. */
void rkLinkDestroy(rkLink *link)
{
  zNameFree( link );
  rkLinkJointDestroy( link );
  rkLinkStuffDestroy( link );
  rkLinkShapeDestroy( link );
  rkLinkInit( link );
}

/* clone a link. */
rkLink *rkLinkClone(rkLink *org, rkLink *cln, zMShape3D *shape_org, zMShape3D *shape_cln, rkMotorSpecArray *msarray_org, rkMotorSpecArray *msarray_cln)
{
  if( !org || !cln ){
    ZRUNERROR( RK_WARN_LINK_NULL );
    return NULL;
  }
  if( !zNameSet( cln, zName(org) ) ||
      !rkJointClone( rkLinkJoint(org), rkLinkJoint(cln), msarray_org, msarray_cln ) ||
      !rkBodyClone( rkLinkBody(org), rkLinkBody(cln), shape_org, shape_cln ) ){
    ZALLOCERROR();
    return NULL;
  }
  rkMPCopy( rkLinkCRB(org), rkLinkCRB(cln) );
  rkLinkSetJointIDOffset( cln, rkLinkJointIDOffset(org) );
  rkLinkSetOrgFrame( cln, rkLinkOrgFrame(org) );

  rkLinkSetParent( cln,
    rkLinkParent(org) ? rkLinkParent(org) - org + cln : NULL );
  rkLinkSetChild( cln,
    rkLinkChild(org) ? rkLinkChild(org) - org + cln : NULL );
  rkLinkSetSibl( cln,
    rkLinkSibl(org) ? rkLinkSibl(org) - org + cln : NULL );
  return cln;
}

/* copy state of a link. */
rkLink *rkLinkCopyState(rkLink *src, rkLink *dst)
{
  rkJointCopyState( rkLinkJoint(src), rkLinkJoint(dst) );
  rkBodyCopyState( rkLinkBody(src), rkLinkBody(dst) );
  zFrame3DCopy( rkLinkAdjFrame(src), rkLinkAdjFrame(dst) );
  zVec6DCopy( rkLinkJointWrench(src), rkLinkJointWrench(dst) );
  return dst;
}

/* add a sibling link. */
rkLink *rkLinkAddSibl(rkLink *link, rkLink *sibl)
{
  for( ; rkLinkSibl(link); link=rkLinkSibl(link) );
  return rkLinkSetSibl( link, sibl );
}

/* add a child link. */
rkLink *rkLinkAddChild(rkLink *link, rkLink *child)
{
  rkLinkSetParent( child, link );
  if( !rkLinkChild(link) ) return rkLinkSetChild( link, child );
  return rkLinkAddSibl( rkLinkChild(link), child );
}

/* detach a link from its parent and siblings. */
rkLink *rkLinkDetach(rkLink *link)
{
  rkLink *parent, *sibl;

  if( !( parent = rkLinkParent(link) ) ) return link; /* no need to detach link. */
  if( !( sibl = rkLinkChild( parent ) ) ){
    ZRUNERROR( RK_WARN_LINK_NOT_CONNECTED, zName(link), zName(parent) );
    return NULL;
  }
  if( sibl == link ){
    rkLinkSetChild( parent, rkLinkSibl(link) );
  } else{
    while( rkLinkSibl(sibl) != link ){
      if( !( sibl = rkLinkSibl(sibl) ) ){
        ZRUNERROR( RK_WARN_LINK_NOT_CONNECTED, zName(link), zName(parent) );
        return NULL;
      }
    }
    rkLinkSetSibl( sibl, rkLinkSibl(link) ); /* skip link. */
  }
  rkLinkSetParent( link, NULL );
  rkLinkSetSibl( link, NULL );
  return link;
}

/* calculate velocity of a point with respect to the inertial frame. */
zVec3D *rkLinkPointVel(const rkLink *link, const zVec3D *p, zVec3D *v)
{
  _zVec3DOuterProd( rkLinkAngVel(link), p, v );
  _zVec3DAddDRC( v, rkLinkLinVel(link) );
  return v;
}

/* calculate accerelation of a point with respect to the inertial frame. */
zVec3D *rkLinkPointAcc(const rkLink *link, const zVec3D *p, zVec3D *a)
{
  zVec3D tmp;

  _zVec3DTripleProd( rkLinkAngVel(link), rkLinkAngVel(link), p, a );
  _zVec3DOuterProd( rkLinkAngAcc(link), p, &tmp );
  _zVec3DAddDRC( a, &tmp );
  _zVec3DAddDRC( a, rkLinkLinAcc(link) );
  return a;
}

/* compute inertia tensor of a link with respect to the inertial frame. */
zMat3D *rkLinkWldInertia(const rkLink *link, zMat3D *inertia)
{
  return zRotMat3D( rkLinkWldAtt(link), rkLinkInertia(link), inertia );
}

/* update link frame with respect to the world frame. */
void rkLinkUpdateFrame(rkLink *link, const zFrame3D *pwf)
{
  rkJointXform( rkLinkJoint(link), rkLinkOrgFrame(link), rkLinkAdjFrame(link) );
  zFrame3DCascade( pwf, rkLinkAdjFrame(link), rkLinkWldFrame(link) );
  rkBodyUpdateCOM( rkLinkBody(link) );

  if( rkLinkChild(link) )
    rkLinkUpdateFrame( rkLinkChild(link), rkLinkWldFrame(link) );
  if( rkLinkSibl(link) )
    rkLinkUpdateFrame( rkLinkSibl(link), rkLinkWldFrame(rkLinkParent(link)) );
}

/* update velocity of a link in the inertial frame, where the orientation is with respect to the link frame. */
static void _rkLinkUpdateVel(rkLink *link, const zVec6D *pvel)
{
  /* velocity */
  zXform6DLin( rkLinkAdjFrame(link), pvel, rkLinkVel(link) );
  /* joint motion rate */
  rkJointIncVel( rkLinkJoint(link), rkLinkVel(link) );
  /* COM velocity and acceleration */
  rkBodyUpdateCOMVel( rkLinkBody(link) );
}
void rkLinkUpdateVel(rkLink *link, const zVec6D *pvel)
{
  _rkLinkUpdateVel( link, pvel );
  if( rkLinkChild(link) )
    rkLinkUpdateVel( rkLinkChild(link), rkLinkVel(link) );
  if( rkLinkSibl(link) )
    rkLinkUpdateVel( rkLinkSibl(link), rkLinkVel(rkLinkParent(link)) );
}

/* update acceleration of a link in the inertial frame, where the orientation is with respect to the link frame. */
static void _rkLinkUpdateAcc(rkLink *link, const zVec6D *pvel, const zVec6D *pacc)
{
  zVec3D wp, tmp;

  /* acceleration */
  zVec6DLinShift( pacc, rkLinkAdjPos(link), rkLinkAcc(link) );
  _zVec3DOuterProd( zVec6DAng(pvel), rkLinkAdjPos(link), &wp );
  _zVec3DOuterProd( zVec6DAng(pvel), &wp, &tmp );
  _zVec3DAddDRC( rkLinkLinAcc(link), &tmp );
  zMulMat3DTVec6DDRC( rkLinkAdjAtt(link), rkLinkAcc(link) );
  /* joint motion rate */
  zVec3DCopy( rkLinkAngVel(link), &tmp );
  rkJointIncAccOnVel( rkLinkJoint(link), &tmp, rkLinkAcc(link) );
  rkJointIncAcc( rkLinkJoint(link), rkLinkAcc(link) );
  /* COM velocity and acceleration */
  rkBodyUpdateCOMAcc( rkLinkBody(link) );
}
void rkLinkUpdateAcc(rkLink *link, const zVec6D *pvel, const zVec6D *pacc)
{
  _rkLinkUpdateAcc( link, pvel, pacc );
  if( rkLinkChild(link) )
    rkLinkUpdateAcc( rkLinkChild(link), rkLinkVel(link), rkLinkAcc(link) );
  if( rkLinkSibl(link) )
    rkLinkUpdateAcc( rkLinkSibl(link), rkLinkVel(rkLinkParent(link)), rkLinkAcc(rkLinkParent(link)) );
}

/* update link motion rate in the inertial frame, where the orientation is with respect to the link frame. */
void rkLinkUpdateRate(rkLink *link, const zVec6D *pvel, const zVec6D *pacc)
{
  _rkLinkUpdateVel( link, pvel );
  _rkLinkUpdateAcc( link, pvel, pacc );
  if( rkLinkChild(link) )
    rkLinkUpdateRate( rkLinkChild(link), rkLinkVel(link), rkLinkAcc(link) );
  if( rkLinkSibl(link) )
    rkLinkUpdateRate( rkLinkSibl(link), rkLinkVel(rkLinkParent(link)), rkLinkAcc(rkLinkParent(link)) );
}

/* update joint wrench of a link based on Neuton-Euler's backward computation. */
void rkLinkUpdateJointWrench(rkLink *link)
{
  zVec6D w;
  rkLink *child;

  /* inertia force */
  rkBodyInertialWrench( rkLinkBody(link), rkLinkJointWrench(link) );
  zVec6DAngShiftDRC( rkLinkJointWrench(link), rkLinkCOM(link) );
  /* reaction force propagation from children */
  if( ( child = rkLinkChild(link) ) ){
    rkLinkUpdateJointWrench( child );
    for( ; child; child=rkLinkSibl(child) ){
      zXform6DAng( rkLinkAdjFrame(child), rkLinkJointWrench(child), &w );
      _zVec6DAddDRC( rkLinkJointWrench(link), &w );
    }
  }
  _zVec6DSubDRC( rkLinkJointWrench(link), rkLinkExtWrench(link) );
  /* joint torque resolution */
  rkJointCalcTrq( rkLinkJoint(link), rkLinkJointWrench(link) );
  /* branch */
  if( rkLinkSibl(link) )
    rkLinkUpdateJointWrench( rkLinkSibl(link) );
}

/* merge mass properties of a link. */
rkMP *rkLinkMergeMP(const rkLink *link, rkMP *mp)
{
  rkMP link_mp;

  rkMPXform( rkLinkMP(link), rkLinkWldFrame(link), &link_mp );
  return rkMPMerge( mp, &link_mp );
}

/* update mass of the composite rigit body of a link. */
double rkLinkUpdateCRBMass(rkLink *link)
{
  rkLink *lp;

  rkMPSetMass( rkLinkCRB(link), rkLinkMass(link) );
  for( lp=rkLinkChild(link); lp; lp=rkLinkSibl(lp) )
    rkLinkCRBMass(link) += rkLinkUpdateCRBMass( lp );
  return rkLinkCRBMass(link);
}

/* update the composite rigit body of a link. */
rkMP *rkLinkUpdateCRB(rkLink *link)
{
  rkLink *lp;
  zMat3D tmpi;
  zVec3D tmpr;

  if( !rkLinkChild(link) ) return rkLinkCRB(link);
  /* composite COM */
  _zVec3DMul( rkLinkCOM(link), rkLinkMass(link), rkLinkCRBCOM(link) );
  for( lp=rkLinkChild(link); lp; lp=rkLinkSibl(lp) ){
    rkLinkUpdateCRB( lp );
    _zXform3D( rkLinkAdjFrame(lp), rkLinkCRBCOM(lp), &tmpr );
    _zVec3DCatDRC( rkLinkCRBCOM(link), rkLinkCRBMass(lp), &tmpr );
  }
  zVec3DDivDRC( rkLinkCRBCOM(link), rkLinkCRBMass(link) );
  /* composite inertia */
  _zVec3DSub( rkLinkCOM(link), rkLinkCRBCOM(link), &tmpr );
  rkMPShiftInertia( rkLinkMP(link), &tmpr, rkLinkCRBInertia(link) );
  for( lp=rkLinkChild(link); lp; lp=rkLinkSibl(lp) ){
    zRotMat3D( rkLinkAdjAtt(lp), rkLinkCRBInertia(lp), &tmpi );
    _zXform3D( rkLinkAdjFrame(lp), rkLinkCRBCOM(lp), &tmpr );
    _zVec3DSubDRC( &tmpr, rkLinkCRBCOM(link) );
    _zMat3DCatVec3DDoubleOuterProdDRC( &tmpi, -rkLinkCRBMass(lp), &tmpr );
    _zMat3DAddDRC( rkLinkCRBInertia(link), &tmpi );
  }
  return rkLinkCRB(link);
}

/* convert 6D configuration of a link to joint displacement. */
void rkLinkConfToJointDis(rkLink *link)
{
  zFrame3D org;
  double dis[6];
  zVec6D tor;

  if( rkLinkParent(link) ){
    zFrame3DCascade( rkLinkWldFrame(rkLinkParent(link)), rkLinkOrgFrame(link), &org );
  } else{
    zFrame3DCopy( rkLinkOrgFrame(link), &org );
  }
  zFrame3DXform( &org, rkLinkWldFrame(link), rkLinkAdjFrame(link) );
  rkJointTorsion( rkLinkJoint(link), rkLinkAdjFrame(link), &tor, dis );
  rkJointSetDis( rkLinkJoint(link), dis );

  /* recursive computation */
  if( rkLinkChild(link) )
    rkLinkConfToJointDis( rkLinkChild(link) );
  if( rkLinkSibl(link) )
    rkLinkConfToJointDis( rkLinkSibl(link) );
}

/* recursively computes linear momentum of a link. */
zVec3D *rkLinkLinearMomentumRecursive(const rkLink *link, zVec3D *momentum)
{
  rkLink *lp;
  zVec3D m, tmp;

  rkLinkLinearMomentum( link, momentum );
  for( lp=rkLinkChild(link); lp; lp=rkLinkSibl(lp) ){
    rkLinkLinearMomentumRecursive( lp, &m );
    _zMulMat3DVec3D( rkLinkAdjAtt(lp), &m, &tmp );
    _zVec3DAddDRC( momentum, &tmp );
  }
  return momentum;
}

/* recursively computes angular momentum of a link. */
zVec3D *rkLinkAngularMomentumRecursive(const rkLink *link, const zVec3D *pos, zVec3D *am)
{
  rkLink *lp;
  zVec3D tp, m, tmp;

  _zXform3DInv( rkLinkWldFrame(link), pos, &tp );
  rkLinkAngularMomentum( link, &tp, am );
  for( lp=rkLinkChild(link); lp; lp=rkLinkSibl(lp) ){
    rkLinkAngularMomentumRecursive( lp, pos, &m );
    _zMulMat3DVec3D( rkLinkAdjAtt(lp), &m, &tmp );
    _zVec3DAddDRC( am, &tmp );
  }
  return am;
}

/* ********************************************************** */
/* rkLinkArray
 * array of links
 * ********************************************************** */

rkLinkArray *rkLinkArrayAlloc(rkLinkArray *linkarray, int size)
{
  zArrayAlloc( linkarray, rkLink, size );
  return zArraySize(linkarray) == size ? linkarray : NULL;
}

void rkLinkArrayDestroy(rkLinkArray *linkarray)
{
  int i;

  for( i=0; i<zArraySize(linkarray); i++ )
    rkLinkDestroy( zArrayElemNC(linkarray,i) );
  zArrayFree( linkarray );
}

rkLinkArray *rkLinkArrayClone(rkLinkArray *org, rkLinkArray *cln, zMShape3D *shape_org, zMShape3D *shape_cln, rkMotorSpecArray *msarray_org, rkMotorSpecArray *msarray_cln)
{
  int i;

  if( zArraySize(org) > 0 ){
    zArrayAlloc( cln, rkLink, zArraySize(org) );
    if( zArraySize(cln) != zArraySize(org) ) return NULL;
    for( i=0; i<zArraySize(cln); i++ )
      if( !rkLinkClone( zArrayElemNC(org,i), zArrayElemNC(cln,i), shape_org, shape_cln, msarray_org, msarray_cln ) )
        return NULL;
  } else
    zArrayInit( cln );
  return cln;
}

void rkLinkArrayFPrintZTK(FILE *fp, rkLinkArray *linkarray)
{
  int i;

  for( i=0; i<zArraySize(linkarray); i++ ){
    fprintf( fp, "[%s]\n", ZTK_TAG_ROKI_LINK );
    rkLinkFPrintZTK( fp, zArrayElemNC(linkarray,i) );
  }
}

/* ZTK processing */

typedef struct{
  rkLinkArray *link_array;
  zShape3DArray *shape_array;
  bool given_density;
  bool given_mass;
  bool auto_com;
  bool auto_inertia;
  double density;
} _rkLinkRefPrp;

static void *_rkLinkNameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkLink *link;
  zArrayFindName( ((_rkLinkRefPrp*)arg)->link_array, ZTKVal(ztk), link );
  if( link ){
    ZRUNWARN( RK_WARN_LINK_DUPLICATE_NAME, ZTKVal(ztk) );
    return NULL;
  }
  zNameSet( (rkLink*)obj, ZTKVal(ztk) );
  return zNamePtr((rkLink*)obj) ? obj : NULL;
}
static void *_rkLinkJointTypeFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  if( !rkJointAssignByStr( rkLinkJoint((rkLink*)obj), ZTKVal(ztk) ) ) return NULL;
  if( ZTKValNext( ztk ) ){
    if( strcmp( ZTKVal(ztk), ZTK_VAL_ROKI_JOINT_PASSIVE ) == 0 )
      rkLinkJoint((rkLink*)obj)->is_active = false;
  }
  return obj;
}
static void *_rkLinkMassFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((_rkLinkRefPrp*)arg)->given_mass = true;
  rkLinkSetMass( (rkLink*)obj, ZTKDouble(ztk) );
  return obj;
}
static void *_rkLinkDensityFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  ((_rkLinkRefPrp*)arg)->given_density = true;
  ((_rkLinkRefPrp*)arg)->density = ZTKDouble(ztk);
  return obj;
}
static void *_rkLinkStuffFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkLinkSetStuff( (rkLink*)obj, ZTKVal(ztk) ) ? obj : NULL;
}
static void *_rkLinkCOMFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zVec3D com;
  if( strcmp( ZTKVal(ztk), "auto" ) == 0 ){
    ((_rkLinkRefPrp*)arg)->auto_com = true;
  } else{
    zVec3DFromZTK( &com, ztk );
    rkLinkSetCOM( (rkLink*)obj, &com );
  }
  return obj;
}
static void *_rkLinkInertiaFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zMat3D inertia;
  if( strcmp( ZTKVal(ztk), "auto" ) == 0 ){
    ((_rkLinkRefPrp*)arg)->auto_inertia = true;
  } else{
    zMat3DFromZTK( &inertia, ztk );
    rkLinkSetInertia( (rkLink*)obj, &inertia );
  }
  return obj;
}
static void *_rkLinkPosFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zVec3DFromZTK( rkLinkOrgPos((rkLink*)obj), ztk );
  return obj;
}
static void *_rkLinkAttFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zMat3DFromZTK( rkLinkOrgAtt((rkLink*)obj), ztk );
  return obj;
}
static void *_rkLinkRotFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zVec3D aa;
  zAAFromZTK( &aa, ztk );
  zMat3DRotDRC( rkLinkOrgAtt((rkLink*)obj), &aa );
  return obj;
}
static void *_rkLinkFrameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zFrame3DFromZTK( rkLinkOrgFrame((rkLink*)obj), ztk );
  return obj;
}
static void *_rkLinkDHFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zFrame3DDHFromZTK( rkLinkOrgFrame((rkLink*)obj), ztk );
  return obj;
}
static void *_rkLinkParentFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkLink *parent;
  zArrayFindName( ((_rkLinkRefPrp*)arg)->link_array, ZTKVal(ztk), parent );
  if( !parent ){
    ZRUNERROR( RK_ERR_LINK_UNKNOWN, ZTKVal(ztk) );
    return NULL;
  }
  rkLinkAddChild( parent, (rkLink*)obj );
  return obj;
}
static void *_rkLinkShapeFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zShape3D *sp;
  do{
    zArrayFindName( ((_rkLinkRefPrp*)arg)->shape_array, ZTKVal(ztk), sp );
    if( !sp ){
      ZRUNERROR( RK_ERR_SHAPE_UNKNOWN, ZTKVal(ztk) );
      return NULL;
    }
    if( !rkLinkShapePush( (rkLink*)obj, sp ) ) return NULL;
  } while( ZTKValNext( ztk ) );
  return obj;
}
static void *_rkLinkBindFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkLink *identlink;
  zArrayFindName( ((_rkLinkRefPrp*)arg)->link_array, ZTKVal(ztk), identlink );
  if( !identlink ){
    ZRUNERROR( RK_ERR_LINK_UNKNOWN, ZTKVal(ztk) );
    return NULL;
  }
  rkLinkSetIdent( (rkLink*)obj, identlink );
  return obj;
}

static bool _rkLinkNameFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName((rkLink*)obj) );
  return true;
}
static bool _rkLinkJointTypeFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s", rkLinkJointTypeStr((rkLink*)obj) );
  if( !rkJointIsActive( rkLinkJoint((rkLink*)obj) ) )
    fprintf( fp, " %s", ZTK_VAL_ROKI_JOINT_PASSIVE );
  fprintf( fp, "\n" );
  rkJointFPrintZTK( fp, rkLinkJoint((rkLink*)obj), zName((rkLink*)obj) );
  return true;
}
static bool _rkLinkMassFPrintZTK(FILE *fp, int i, void *obj){
  if( zIsTiny( rkLinkMass((rkLink*)obj) ) ) return false;
  fprintf( fp, "%.10g\n", rkLinkMass((rkLink*)obj) );
  return true;
}
static bool _rkLinkStuffFPrintZTK(FILE *fp, int i, void *obj){
  if( !rkLinkStuff((rkLink*)obj) ) return false;
  fprintf( fp, "%s\n", rkLinkStuff((rkLink*)obj) );
  return true;
}
static bool _rkLinkCOMFPrintZTK(FILE *fp, int i, void *obj){
  if( zVec3DIsTiny( rkLinkCOM((rkLink*)obj) ) ) return false;
  zVec3DFPrint( fp, rkLinkCOM((rkLink*)obj) );
  return true;
}
static bool _rkLinkInertiaFPrintZTK(FILE *fp, int i, void *obj){
  if( zMat3DIsTiny( rkLinkInertia((rkLink*)obj) ) ) return false;
  zMat3DFPrint( fp, rkLinkInertia((rkLink*)obj) );
  return true;
}
static bool _rkLinkPosFPrintZTK(FILE *fp, int i, void *obj){
  if( zVec3DIsTiny( rkLinkOrgPos((rkLink*)obj) ) ) return false;
  zVec3DFPrint( fp, rkLinkOrgPos((rkLink*)obj) );
  return true;
}
static bool _rkLinkAttFPrintZTK(FILE *fp, int i, void *obj){
  if( zMat3DIsIdent( rkLinkOrgAtt((rkLink*)obj) ) ) return false;
  zMat3DFPrint( fp, rkLinkOrgAtt((rkLink*)obj) );
  return true;
}
static bool _rkLinkParentFPrintZTK(FILE *fp, int i, void *obj){
  if( !rkLinkParent((rkLink*)obj) ) return false;
  fprintf( fp, "%s\n", zName(rkLinkParent((rkLink*)obj)) );
  return true;
}
static bool _rkLinkBindFPrintZTK(FILE *fp, int i, void *obj){
  if( !rkLinkIdent((rkLink*)obj) ) return false;
  fprintf( fp, "%s\n", zName(rkLinkIdent((rkLink*)obj)) );
  return true;
}

static const ZTKPrp __ztk_prp_rklink[] = {
  { ZTK_KEY_ROKI_LINK_NAME,      1, _rkLinkNameFromZTK, _rkLinkNameFPrintZTK },
  { ZTK_KEY_ROKI_LINK_JOINTTYPE, 1, _rkLinkJointTypeFromZTK, _rkLinkJointTypeFPrintZTK },
  { ZTK_KEY_ROKI_LINK_MASS,      1, _rkLinkMassFromZTK, _rkLinkMassFPrintZTK },
  { ZTK_KEY_ROKI_LINK_DENSITY,   1, _rkLinkDensityFromZTK, NULL },
  { ZTK_KEY_ROKI_LINK_STUFF,     1, _rkLinkStuffFromZTK, _rkLinkStuffFPrintZTK },
  { ZTK_KEY_ROKI_LINK_COM,       1, _rkLinkCOMFromZTK, _rkLinkCOMFPrintZTK },
  { ZTK_KEY_ROKI_LINK_INERTIA,   1, _rkLinkInertiaFromZTK, _rkLinkInertiaFPrintZTK },
  { ZTK_KEY_ROKI_LINK_POS,       1, _rkLinkPosFromZTK, _rkLinkPosFPrintZTK },
  { ZTK_KEY_ROKI_LINK_ATT,       1, _rkLinkAttFromZTK, _rkLinkAttFPrintZTK },
  { ZTK_KEY_ROKI_LINK_ROT,      -1, _rkLinkRotFromZTK, NULL },
  { ZTK_KEY_ROKI_LINK_FRAME,     1, _rkLinkFrameFromZTK, NULL },
  { ZTK_KEY_ROKI_LINK_DH,        1, _rkLinkDHFromZTK, NULL },
  { ZTK_KEY_ROKI_LINK_SHAPE,    -1, _rkLinkShapeFromZTK, NULL },
};

static const ZTKPrp __ztk_prp_rklink_connect[] = {
  { ZTK_KEY_ROKI_LINK_PARENT,    1, _rkLinkParentFromZTK, _rkLinkParentFPrintZTK },
  { ZTK_KEY_ROKI_LINK_BIND,      1, _rkLinkBindFromZTK, _rkLinkBindFPrintZTK },
};

/* build a link from ZTK. */
rkLink *rkLinkFromZTK(rkLink *link, rkLinkArray *link_array, zShape3DArray *shape_array, rkMotorSpecArray *motorspec_array, ZTK *ztk)
{
  _rkLinkRefPrp prp;
  rkMP mp;
  double v;

  rkLinkInit( link );
  prp.link_array = link_array;
  prp.shape_array = shape_array;
  prp.given_density = false;
  prp.given_mass = false;
  prp.auto_com = false;
  prp.auto_inertia = false;
  prp.density = 1.0; /* dummy */

  if( !_ZTKEvalKey( link, &prp, ztk, __ztk_prp_rklink ) ) return NULL;
  /* automatic mass property computation */
  if( prp.given_density ){ /* from density */
    if( prp.given_mass )
      ZRUNWARN( RK_WARN_LINK_DUALLYDEFINED_MASS );
    if( zIsTiny( prp.density ) )
      ZRUNWARN( RK_WARN_LINK_TOO_SMALL_DENSITY );
    else if( rkLinkShapeIsEmpty( link ) )
      ZRUNWARN( RK_WARN_LINK_EMPTY_SHAPE );
    else
      rkLinkShapeMP( link, prp.density, &mp );
    rkLinkSetMass( link, rkMPMass(&mp) );
  } else
  if( prp.auto_com || prp.auto_inertia ){ /* from mass */
    if( !prp.given_mass )
      ZRUNWARN( RK_WARN_LINK_UNSPECIFIED_MASS );
    else
    if( zIsTiny( rkLinkMass(link) ) )
      ZRUNWARN( RK_WARN_LINK_TOO_SMALL_MASS );
    else if( rkLinkShapeIsEmpty( link ) )
      ZRUNWARN( RK_WARN_LINK_EMPTY_SHAPE );
    else if( zIsTiny( ( v = rkLinkShapeVolume(link) ) ) ){
      ZRUNWARN( RK_WARN_LINK_TOO_SMALL_VOLUME );
      v = 1.0; /* dummy */
    } else
      rkLinkShapeMP( link, rkLinkMass(link)/v, &mp );
  }
  if( prp.auto_com ) rkLinkSetCOM( link, rkMPCOM(&mp) );
  if( prp.auto_inertia ) rkLinkSetInertia( link, rkMPInertia(&mp) );
  if( !zMat3DIsSymmetric( rkLinkInertia(link) ) ){ /* check if inertia tensor is symmetric */
    ZRUNWARN( RK_WARN_BODY_NON_SYMMETRIC_INERTIA );
    zMat3DSymmetrizeDRC( rkLinkInertia(link) );
  }
  if( !rkLinkJoint(link)->com ) rkJointAssign( rkLinkJoint(link), &rk_joint_fixed );
  rkJointFromZTK( rkLinkJoint(link), motorspec_array, ztk );
  if( !zMat3DIsRightHand( rkLinkOrgAtt(link) ) ){ /* check if attitude matrix is right-handed */
    ZRUNWARN( RK_WARN_LINK_NON_RIGHTHAND_ATT );
  }
  if( !zMat3DIsOrthonormal( rkLinkOrgAtt(link) ) ){ /* check if attitude matrix is orthonormal */
    ZRUNWARN( RK_WARN_LINK_NON_ORTHONORMAL_ATT );
    zMat3DOrthonormalizeDRC( rkLinkOrgAtt(link), zZ, zX );
  }
  return link;
}

/* connect links based on ZTK. */
rkLink *rkLinkConnectFromZTK(rkLink *link, rkLinkArray *link_array, ZTK *ztk)
{
  _rkLinkRefPrp prp;

  prp.link_array = link_array;
  if( !_ZTKEvalKey( link, &prp, ztk, __ztk_prp_rklink_connect ) ) return NULL;
  return link;
}

/* print out link properties to a file in ZTK format. */
void rkLinkFPrintZTK(FILE *fp, rkLink *link)
{
  zShapeListCell *cp;

  _ZTKPrpKeyFPrint( fp, link, __ztk_prp_rklink );
  if( !rkLinkShapeIsEmpty(link) ){
    fprintf( fp, "%s:", ZTK_KEY_ROKI_LINK_SHAPE );
    zListForEach( rkLinkShapeList(link), cp )
      fprintf( fp, " %s", zName( zShapeListCellShape(cp) ) );
    fprintf( fp, "\n" );
  }
  _ZTKPrpKeyFPrint( fp, link, __ztk_prp_rklink_connect );
  fprintf( fp, "\n" );
}

/* print current 6D posture of a link out to a file. */
void rkLinkPostureFPrint(FILE *fp, rkLink *link)
{
  fprintf( fp, "Link(name:%s joint ID offset:%d)\n", zName(link), rkLinkJointIDOffset(link) );
  fprintf( fp, " adjacent frame:\n" );
  zFrame3DFPrint( fp, rkLinkAdjFrame( link ) );
  fprintf( fp, " world frame:\n" );
  zFrame3DFPrint( fp, rkLinkWldFrame( link ) );
}

/* visualize branches of a kinematic chain in a text file. */
static void _rkLinkConnectivityBranchFPrint(FILE *fp, ulong branch_bit, int depth)
{
  for( ; depth>0; depth--, branch_bit>>=1 )
    fprintf( fp, "%c ", branch_bit & 0x1 ? '|' : ' ' );
}

/* print connectivity of a link of a kinematic chain out to a file. */
void rkLinkConnectivityFPrint(FILE *fp, rkLink *link, rkLink *root, ulong branch_bit, int depth)
{
  _rkLinkConnectivityBranchFPrint( fp, branch_bit, depth );
  fprintf( fp, "|-[%ld] %s (%s", (long)( link - root ), zName(link), rkJointTypeStr(rkLinkJoint(link)) );
  if( rkLinkJointIDOffset(link) >= 0 )
    fprintf( fp, ":%d", rkLinkJointIDOffset(link) );
  fprintf( fp, ")\n" );
  if( rkLinkChild( link ) ){
    if( rkLinkSibl( link ) )
      branch_bit |= ( 0x1 << depth );
    rkLinkConnectivityFPrint( fp, rkLinkChild(link), root, branch_bit, depth+1 );
    branch_bit &= ~( 0x1 << depth );
  }
  if( rkLinkSibl( link ) )
    rkLinkConnectivityFPrint( fp, rkLinkSibl(link), root, branch_bit, depth );
}

/* print external wrench applied to a link out to a file. */
void rkLinkExtWrenchFPrint(FILE *fp, rkLink *link)
{
  fprintf( fp, " force: " );
  zVec3DFPrint( fp, zVec6DLin( rkLinkExtWrench(link) ) );
  fprintf( fp, " torque: " );
  zVec3DFPrint( fp, zVec6DAng( rkLinkExtWrench(link) ) );
}
