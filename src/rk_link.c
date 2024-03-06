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

/* initialize a link. */
void rkLinkInit(rkLink *link)
{
  rkLinkSetJointIDOffset( link, -1 );
  zNameSetPtr( link, NULL );
  rkJointInit( rkLinkJoint(link) );
  rkBodyInit( rkLinkBody(link) );

  rkLinkSetOrgFrame( link, ZFRAME3DIDENT );
  rkLinkSetAdjFrame( link, ZFRAME3DIDENT );
  rkLinkSetWrench( link, ZVEC6DZERO );

  rkLinkSetParent( link, NULL );
  rkLinkSetChild( link, NULL );
  rkLinkSetSibl( link, NULL );
}

/* destroy a link. */
void rkLinkDestroy(rkLink *link)
{
  zNameFree( link );
  rkJointDestroy( rkLinkJoint(link) );
  rkLinkExtWrenchDestroy( link );
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
  zVec6DCopy( rkLinkWrench(src), rkLinkWrench(dst) );
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

/* calculate velocity of a point with respect to the inertial frame. */
zVec3D *rkLinkPointVel(rkLink *link, zVec3D *p, zVec3D *v)
{
  zVec3DOuterProd( rkLinkAngVel(link), p, v );
  return zVec3DAddDRC( v, rkLinkLinVel(link) );
}

/* calculate accerelation of a point with respect to the inertial frame. */
zVec3D *rkLinkPointAcc(rkLink *link, zVec3D *p, zVec3D *a)
{
  zVec3D tmp;

  zVec3DTripleProd( rkLinkAngVel(link), rkLinkAngVel(link), p, a );
  zVec3DOuterProd( rkLinkAngAcc(link), p, &tmp );
  zVec3DAddDRC( a, &tmp );
  return zVec3DAddDRC( a, rkLinkLinAcc(link) );
}

/* compute inertia tensor of a link with respect to the inertial frame. */
zMat3D *rkLinkWldInertia(rkLink *link, zMat3D *i)
{
  return zRotMat3D( rkLinkWldAtt(link), rkLinkInertia(link), i );
}

/* update link frame with respect to the world frame. */
void rkLinkUpdateFrame(rkLink *link, zFrame3D *pwf)
{
  rkJointXform( rkLinkJoint(link), rkLinkOrgFrame(link), rkLinkAdjFrame(link) );
  zFrame3DCascade( pwf, rkLinkAdjFrame(link), rkLinkWldFrame(link) );
  rkBodyUpdateCOM( rkLinkBody(link) );

  if( rkLinkChild(link) )
    rkLinkUpdateFrame( rkLinkChild(link), rkLinkWldFrame(link) );
  if( rkLinkSibl(link) )
    rkLinkUpdateFrame( rkLinkSibl(link), rkLinkWldFrame(rkLinkParent(link)) );
}

void _rkLinkUpdateVel(rkLink *link, zVec6D *pvel)
{
  /* velocity */
  zXform6DLin( rkLinkAdjFrame(link), pvel, rkLinkVel(link) );
  /* joint motion rate */
  rkJointIncVel( rkLinkJoint(link), rkLinkVel(link) );
  /* COM velocity and acceleration */
  rkBodyUpdateCOMVel( rkLinkBody(link) );
}

void rkLinkUpdateVel(rkLink *link, zVec6D *pvel)
{
  _rkLinkUpdateVel( link, pvel );
  if( rkLinkChild(link) )
    rkLinkUpdateVel( rkLinkChild(link), rkLinkVel(link) );
  if( rkLinkSibl(link) )
    rkLinkUpdateVel( rkLinkSibl(link), rkLinkVel(rkLinkParent(link)) );
}

void _rkLinkUpdateAcc(rkLink *link, zVec6D *pvel, zVec6D *pacc)
{
  zVec3D wp, tmp;

  /* acceleration */
  zVec6DLinShift( pacc, rkLinkAdjPos(link), rkLinkAcc(link) );
  zVec3DOuterProd( zVec6DAng(pvel), rkLinkAdjPos(link), &wp );
  zVec3DOuterProd( zVec6DAng(pvel), &wp, &tmp );
  zVec3DAddDRC( rkLinkLinAcc(link), &tmp );
  zMulMat3DTVec6DDRC( rkLinkAdjAtt(link), rkLinkAcc(link) );
  /* joint motion rate */
  zVec3DCopy( rkLinkAngVel(link), &tmp );
  rkJointIncAccOnVel( rkLinkJoint(link), &tmp, rkLinkAcc(link) );
  rkJointIncAcc( rkLinkJoint(link), rkLinkAcc(link) );
  /* COM velocity and acceleration */
  rkBodyUpdateCOMAcc( rkLinkBody(link) );
}

void rkLinkUpdateAcc(rkLink *link, zVec6D *pvel, zVec6D *pacc)
{
  _rkLinkUpdateAcc( link, pvel, pacc );
  if( rkLinkChild(link) )
    rkLinkUpdateAcc( rkLinkChild(link), rkLinkVel(link), rkLinkAcc(link) );
  if( rkLinkSibl(link) )
    rkLinkUpdateAcc( rkLinkSibl(link), rkLinkVel(rkLinkParent(link)), rkLinkAcc(rkLinkParent(link)) );
}

/* update link motion rate with respect to the inertial frame. */
void rkLinkUpdateRate(rkLink *link, zVec6D *pvel, zVec6D *pacc)
{
  _rkLinkUpdateVel( link, pvel );
  _rkLinkUpdateAcc( link, pvel, pacc );
  if( rkLinkChild(link) )
    rkLinkUpdateRate( rkLinkChild(link), rkLinkVel(link), rkLinkAcc(link) );
  if( rkLinkSibl(link) )
    rkLinkUpdateRate( rkLinkSibl(link), rkLinkVel(rkLinkParent(link)), rkLinkAcc(rkLinkParent(link)) );
}

/* update joint torque of link based on Neuton=Euler's equation. */
void rkLinkUpdateWrench(rkLink *link)
{
  zVec6D w;
  rkLink *child;

  /* inertia force */
  rkBodyNetWrench( rkLinkBody(link), rkLinkWrench(link) );
  zVec6DAngShiftDRC( rkLinkWrench(link), rkLinkCOM(link) );
  /* reaction force propagation from children */
  if( ( child = rkLinkChild(link) ) ){
    rkLinkUpdateWrench( child );
    for( ; child; child=rkLinkSibl(child) ){
      zXform6DAng( rkLinkAdjFrame(child), rkLinkWrench(child), &w );
      zVec6DAddDRC( rkLinkWrench(link), &w );
    }
  }
  rkLinkNetExtWrench( link, &w ); /* external wrench */
  zVec6DSubDRC( rkLinkWrench(link), &w );
  /* joint torque resolution */
  rkJointCalcTrq( rkLinkJoint(link), rkLinkWrench(link) );
  /* branch */
  if( rkLinkSibl(link) )
    rkLinkUpdateWrench( rkLinkSibl(link) );
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
    zMat3DCatVec3DDoubleOuterProdDRC( &tmpi, -rkLinkCRBMass(lp), &tmpr );
    zMat3DAddDRC( rkLinkCRBInertia(link), &tmpi );
  }
  return rkLinkCRB(link);
}

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
    fprintf( fp, "[%s]\n", ZTK_TAG_RKLINK );
    rkLinkFPrintZTK( fp, zArrayElemNC(linkarray,i) );
  }
}

/* ZTK processing */

typedef struct{
  rkLinkArray *larray;
  zShape3DArray *sarray;
  bool given_density;
  bool given_mass;
  bool auto_com;
  bool auto_inertia;
  double density;
} _rkLinkRefPrp;

static void *_rkLinkNameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkLink *link;
  zArrayFindName( ((_rkLinkRefPrp*)arg)->larray, ZTKVal(ztk), link );
  if( link ){
    ZRUNWARN( RK_WARN_LINK_DUP, ZTKVal(ztk) );
    return NULL;
  }
  zNameSet( (rkLink*)obj, ZTKVal(ztk) );
  return zNamePtr((rkLink*)obj) ? obj : NULL;
}
static void *_rkLinkJointTypeFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkJointAssignByStr( rkLinkJoint((rkLink*)obj), ZTKVal(ztk) );
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
  zArrayFindName( ((_rkLinkRefPrp*)arg)->larray, ZTKVal(ztk), parent );
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
    zArrayFindName( ((_rkLinkRefPrp*)arg)->sarray, ZTKVal(ztk), sp );
    if( !sp ){
      ZRUNERROR( RK_ERR_SHAPE_UNKNOWN, ZTKVal(ztk) );
      return NULL;
    }
    if( !rkLinkShapePush( (rkLink*)obj, sp ) ) return NULL;
  } while( ZTKValNext( ztk ) );
  return obj;
}

static void _rkLinkNameFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName((rkLink*)obj) );
}
static void _rkLinkJointTypeFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", rkLinkJointTypeStr((rkLink*)obj) );
}
static void _rkLinkMassFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", rkLinkMass((rkLink*)obj) );
}
static void _rkLinkStuffFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", rkLinkStuff((rkLink*)obj) );
}
static void _rkLinkCOMFPrintZTK(FILE *fp, int i, void *obj){
  zVec3DFPrint( fp, rkLinkCOM((rkLink*)obj) );
}
static void _rkLinkInertiaFPrintZTK(FILE *fp, int i, void *obj){
  zMat3DFPrint( fp, rkLinkInertia((rkLink*)obj) );
}
static void _rkLinkPosFPrintZTK(FILE *fp, int i, void *obj){
  zVec3DFPrint( fp, rkLinkOrgPos((rkLink*)obj) );
}
static void _rkLinkAttFPrintZTK(FILE *fp, int i, void *obj){
  zMat3DFPrint( fp, rkLinkOrgAtt((rkLink*)obj) );
}

static ZTKPrp __ztk_prp_rklink[] = {
  { "name", 1, _rkLinkNameFromZTK, _rkLinkNameFPrintZTK },
  { "jointtype", 1, _rkLinkJointTypeFromZTK, _rkLinkJointTypeFPrintZTK },
  { "mass", 1, _rkLinkMassFromZTK, _rkLinkMassFPrintZTK },
  { "density", 1, _rkLinkDensityFromZTK, NULL },
  { "stuff", 1, _rkLinkStuffFromZTK, _rkLinkStuffFPrintZTK },
  { "COM", 1, _rkLinkCOMFromZTK, _rkLinkCOMFPrintZTK },
  { "inertia", 1, _rkLinkInertiaFromZTK, _rkLinkInertiaFPrintZTK },
  { "pos", 1, _rkLinkPosFromZTK, _rkLinkPosFPrintZTK },
  { "att", 1, _rkLinkAttFromZTK, _rkLinkAttFPrintZTK },
  { "rot", -1, _rkLinkRotFromZTK, NULL },
  { "frame", 1, _rkLinkFrameFromZTK, NULL },
  { "DH", 1, _rkLinkDHFromZTK, NULL },
  { "shape", -1, _rkLinkShapeFromZTK, NULL },
};

static ZTKPrp __ztk_prp_rklink_parent[] = {
  { "parent", 1, _rkLinkParentFromZTK, NULL },
};

rkLink *rkLinkFromZTK(rkLink *link, rkLinkArray *larray, zShape3DArray *sarray, rkMotorSpecArray *msarray, ZTK *ztk)
{
  _rkLinkRefPrp prp;
  rkMP mp;
  double v;

  rkLinkInit( link );
  prp.larray = larray;
  prp.sarray = sarray;
  prp.given_density = false;
  prp.given_mass = false;
  prp.auto_com = false;
  prp.auto_inertia = false;
  prp.density = 1.0; /* dummy */

  if( !ZTKEvalKey( link, &prp, ztk, __ztk_prp_rklink ) ) return NULL;
  /* automatic mass property computation */
  if( prp.given_density ){ /* from density */
    if( prp.given_mass )
      ZRUNWARN( RK_WARN_LINK_DUP_MASS_DENS );
    if( zIsTiny( prp.density ) )
      ZRUNWARN( RK_WARN_LINK_TOO_SMALL_DENS );
    else if( rkLinkShapeIsEmpty( link ) )
      ZRUNWARN( RK_WARN_LINK_EMPTY_SHAPE );
    else
      rkLinkShapeMP( link, prp.density, &mp );
    rkLinkSetMass( link, rkMPMass(&mp) );
  } else
  if( prp.auto_com || prp.auto_inertia ){ /* from mass */
    if( !prp.given_mass )
      ZRUNWARN( RK_WARN_LINK_NO_MASS_DENS );
    else
    if( zIsTiny( rkLinkMass(link) ) )
      ZRUNWARN( RK_WARN_LINK_TOO_SMALL_MASS );
    else if( rkLinkShapeIsEmpty( link ) )
      ZRUNWARN( RK_WARN_LINK_EMPTY_SHAPE );
    else if( zIsTiny( ( v = rkLinkShapeVolume(link) ) ) ){
      ZRUNWARN( RK_WARN_LINK_TOO_SMALL_VOL );
      v = 1.0; /* dummy */
    } else
      rkLinkShapeMP( link, rkLinkMass(link)/v, &mp );
  }
  if( prp.auto_com ) rkLinkSetCOM( link, rkMPCOM(&mp) );
  if( prp.auto_inertia ) rkLinkSetInertia( link, rkMPInertia(&mp) );

  if( !rkLinkJoint(link)->com ) rkJointAssign( rkLinkJoint(link), &rk_joint_fixed );
  rkJointFromZTK( rkLinkJoint(link), msarray, ztk );
  return link;
}

rkLink *rkLinkConnectFromZTK(rkLink *link, rkLinkArray *larray, ZTK *ztk)
{
  _rkLinkRefPrp prp;

  prp.larray = larray;
  if( !ZTKEvalKey( link, &prp, ztk, __ztk_prp_rklink_parent ) ) return NULL;
  return link;
}

void rkLinkFPrintZTK(FILE *fp, rkLink *link)
{
  zShapeListCell *cp;

  ZTKPrpKeyFPrint( fp, link, __ztk_prp_rklink );
  rkJointFPrintZTK( fp, rkLinkJoint(link), zName(link) );
  if( !rkLinkShapeIsEmpty(link) )
    zListForEach( rkLinkShapeList(link), cp )
      fprintf( fp, "%s: %s\n", ZTK_TAG_SHAPE, zName( zShapeListCellShape(cp) ) );
  if( rkLinkParent(link) )
    fprintf( fp, "parent: %s\n", zName(rkLinkParent(link)) );
  fprintf( fp, "\n" );
}

/* print link posture out to a file. */
void rkLinkPostureFPrint(FILE *fp, rkLink *link)
{
  fprintf( fp, "Link(name:%s joint ID offset:%d)\n", zName(link), rkLinkJointIDOffset(link) );
  fprintf( fp, " adjacent frame:\n" );
  zFrame3DFPrint( fp, rkLinkAdjFrame( link ) );
  fprintf( fp, " world frame:\n" );
  zFrame3DFPrint( fp, rkLinkWldFrame( link ) );
}

/* print link connectivity out to a file. */
#define RK_LINK_CONNECTION_INDENT 2
void rkLinkConnectionFPrint(FILE *fp, rkLink *link, int n)
{
  zFIndent( fp, n );
  fprintf( fp, "|-%s (%s:%d)\n", zName(link), rkJointTypeStr(rkLinkJoint(link)), rkLinkJointIDOffset(link) );

  if( rkLinkChild( link ) )
    rkLinkConnectionFPrint( fp, rkLinkChild(link), n+RK_LINK_CONNECTION_INDENT );
  if( rkLinkSibl( link ) )
    rkLinkConnectionFPrint( fp, rkLinkSibl(link), n );
}

/* print external wrenches applied to a link out to a file. */
void rkLinkExtWrenchFPrint(FILE *fp, rkLink *link)
{
  rkWrench *c;

  zListForEach( rkLinkExtWrench(link), c )
    rkWrenchFPrint( fp, c );
}
