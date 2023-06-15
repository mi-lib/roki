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
void rkLinkInit(rkLink *l)
{
  rkLinkSetJointIDOffset( l, -1 );
  zNameSetPtr( l, NULL );
  rkJointInit( rkLinkJoint(l) );
  rkBodyInit( rkLinkBody(l) );

  rkLinkSetOrgFrame( l, ZFRAME3DIDENT );
  rkLinkSetAdjFrame( l, ZFRAME3DIDENT );
  rkLinkSetWrench( l, ZVEC6DZERO );

  rkLinkSetParent( l, NULL );
  rkLinkSetChild( l, NULL );
  rkLinkSetSibl( l, NULL );
}

/* destroy a link. */
void rkLinkDestroy(rkLink *l)
{
  zNameFree( l );
  rkJointDestroy( rkLinkJoint(l) );
  rkLinkExtWrenchDestroy( l );
  rkLinkShapeDestroy( l );
  rkLinkInit( l );
}

/* clone a link. */
rkLink *rkLinkClone(rkLink *org, rkLink *cln, zMShape3D *so, zMShape3D *sc)
{
  if( !org || !cln ){
    ZRUNERROR( RK_WARN_LINK_NULL );
    return NULL;
  }
  if( !zNameSet( cln, zName(org) ) ||
      !rkJointClone( rkLinkJoint(org), rkLinkJoint(cln) ) ||
      !rkBodyClone( rkLinkBody(org), rkLinkBody(cln), so, sc ) ){
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
rkLink *rkLinkAddSibl(rkLink *l, rkLink *bl)
{
  for( ; rkLinkSibl(l); l=rkLinkSibl(l) );
  return rkLinkSetSibl( l, bl );
}

/* add a child link. */
rkLink *rkLinkAddChild(rkLink *l, rkLink *cl)
{
  rkLinkSetParent( cl, l );
  if( !rkLinkChild(l) ) return rkLinkSetChild( l, cl );
  return rkLinkAddSibl( rkLinkChild(l), cl );
}

/* calculate velocity of a point with respect to the inertial frame. */
zVec3D *rkLinkPointVel(rkLink *l, zVec3D *p, zVec3D *v)
{
  zVec3DOuterProd( rkLinkAngVel(l), p, v );
  return zVec3DAddDRC( v, rkLinkLinVel(l) );
}

/* calculate accerelation of a point with respect to the inertial frame. */
zVec3D *rkLinkPointAcc(rkLink *l, zVec3D *p, zVec3D *a)
{
  zVec3D tmp;

  zVec3DTripleProd( rkLinkAngVel(l), rkLinkAngVel(l), p, a );
  zVec3DOuterProd( rkLinkAngAcc(l), p, &tmp );
  zVec3DAddDRC( a, &tmp );
  return zVec3DAddDRC( a, rkLinkLinAcc(l) );
}

/* compute inertia tensor of a link with respect to the inertial frame. */
zMat3D *rkLinkWldInertia(rkLink *l, zMat3D *i)
{
  return zRotMat3D( rkLinkWldAtt(l), rkLinkInertia(l), i );
}

/* update link frame with respect to the world frame. */
void rkLinkUpdateFrame(rkLink *l, zFrame3D *pwf)
{
  rkJointXform( rkLinkJoint(l), rkLinkOrgFrame(l), rkLinkAdjFrame(l) );
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
  zXform6DLin( rkLinkAdjFrame(l), pvel, rkLinkVel(l) );
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
  zMulMat3DTVec6DDRC( rkLinkAdjAtt(l), rkLinkAcc(l) );
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

/* update link motion rate with respect to the inertial frame. */
void rkLinkUpdateRate(rkLink *l, zVec6D *pvel, zVec6D *pacc)
{
  _rkLinkUpdateVel( l, pvel );
  _rkLinkUpdateAcc( l, pvel, pacc );
  if( rkLinkChild(l) )
    rkLinkUpdateRate( rkLinkChild(l), rkLinkVel(l), rkLinkAcc(l) );
  if( rkLinkSibl(l) )
    rkLinkUpdateRate( rkLinkSibl(l), rkLinkVel(rkLinkParent(l)), rkLinkAcc(rkLinkParent(l)) );
}

/* update joint torque of link based on Neuton=Euler's equation. */
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
      zXform6DAng( rkLinkAdjFrame(child), rkLinkWrench(child), &w );
      zVec6DAddDRC( rkLinkWrench(l), &w );
    }
  }
  rkLinkNetExtWrench( l, &w ); /* external wrench */
  zVec6DSubDRC( rkLinkWrench(l), &w );
  /* joint torque resolution */
  rkJointCalcTrq( rkLinkJoint(l), rkLinkWrench(l) );
  /* branch */
  if( rkLinkSibl(l) )
    rkLinkUpdateWrench( rkLinkSibl(l) );
}

/* update mass of the composite rigit body of a link. */
double rkLinkUpdateCRBMass(rkLink *link)
{
  rkLink *l;

  rkMPSetMass( rkLinkCRB(link), rkLinkMass(link) );
  for( l=rkLinkChild(link); l; l=rkLinkSibl(l) )
    rkLinkCRBMass(link) += rkLinkUpdateCRBMass( l );
  return rkLinkCRBMass(link);
}

/* update the composite rigit body of a link. */
rkMP *rkLinkUpdateCRB(rkLink *link)
{
  rkLink *l;
  zMat3D tmpi;
  zVec3D tmpr;

  if( !rkLinkChild(link) ) return rkLinkCRB(link);
  /* composite COM */
  _zVec3DMul( rkLinkCOM(link), rkLinkMass(link), rkLinkCRBCOM(link) );
  for( l=rkLinkChild(link); l; l=rkLinkSibl(l) ){
    rkLinkUpdateCRB( l );
    _zXform3D( rkLinkAdjFrame(l), rkLinkCRBCOM(l), &tmpr );
    _zVec3DCatDRC( rkLinkCRBCOM(link), rkLinkCRBMass(l), &tmpr );
  }
  zVec3DDivDRC( rkLinkCRBCOM(link), rkLinkCRBMass(link) );
  /* composite inertia */
  _zVec3DSub( rkLinkCOM(link), rkLinkCRBCOM(link), &tmpr );
  rkMPShiftInertia( rkLinkMP(link), &tmpr, rkLinkCRBInertia(link) );
  for( l=rkLinkChild(link); l; l=rkLinkSibl(l) ){
    zRotMat3D( rkLinkAdjAtt(l), rkLinkCRBInertia(l), &tmpi );
    _zXform3D( rkLinkAdjFrame(l), rkLinkCRBCOM(l), &tmpr );
    _zVec3DSubDRC( &tmpr, rkLinkCRBCOM(link) );
    zMat3DCatVec3DDoubleOuterProdDRC( &tmpi, -rkLinkCRBMass(l), &tmpr );
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
  return rkJointQueryAssign( rkLinkJoint((rkLink*)obj), ZTKVal(ztk) );
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

rkLink *rkLinkFromZTK(rkLink *link, rkLinkArray *larray, zShape3DArray *sarray, rkMotorArray *motorarray, ZTK *ztk)
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
  rkJointFromZTK( rkLinkJoint(link), motorarray, ztk );
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
void rkLinkPostureFPrint(FILE *fp, rkLink *l)
{
  fprintf( fp, "Link(name:%s joint ID offset:%d)\n", zName(l), rkLinkJointIDOffset(l) );
  fprintf( fp, " adjacent frame:\n" );
  zFrame3DFPrint( fp, rkLinkAdjFrame( l ) );
  fprintf( fp, " world frame:\n" );
  zFrame3DFPrint( fp, rkLinkWldFrame( l ) );
}

/* print link connectivity out to a file. */
#define RK_LINK_CONNECTION_INDENT 2
void rkLinkConnectionFPrint(FILE *fp, rkLink *l, int n)
{
  zFIndent( fp, n );
  fprintf( fp, "|-%s (%s:%d)\n", zName(l), rkJointTypeStr(rkLinkJoint(l)), rkLinkJointIDOffset(l) );

  if( rkLinkChild( l ) )
    rkLinkConnectionFPrint( fp, rkLinkChild(l), n+RK_LINK_CONNECTION_INDENT );
  if( rkLinkSibl( l ) )
    rkLinkConnectionFPrint( fp, rkLinkSibl(l), n );
}

/* print external wrenches applied to a link out to a file. */
void rkLinkExtWrenchFPrint(FILE *fp, rkLink *l)
{
  rkWrench *c;

  zListForEach( rkLinkExtWrench(l), c )
    rkWrenchFPrint( fp, c );
}
