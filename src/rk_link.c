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
  zFrame3DXform( &org, rkLinkWldFrame(link), &dev );
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

static bool _rkLinkFScan(FILE *fp, void *instance, char *buf, bool *success);

/* scan link properties from a file. */
bool _rkLinkFScan(FILE *fp, void *instance, char *buf, bool *success)
{
  _rkLinkParam *prm;
  rkLink *pl, *cl;
  zShape3D *sp;
  zVec3D axis;
  double angle;

  prm = instance;
  if( strcmp( buf, "name" ) == 0 ){
    if( strlen( zFToken( fp, buf, BUFSIZ ) ) >= BUFSIZ ){
      buf[BUFSIZ-1] = '\0';
      ZRUNWARN( RK_WARN_TOOLNG_NAME, buf );
    }
    zNameFind( prm->larray, prm->nl, buf, cl );
    if( cl ){
      ZRUNERROR( RK_WARN_TWOFOLDNAME, buf );
      return false;
    }
    if( !( zNameSet( prm->l, buf ) ) ){
      ZALLOCERROR();
      return ( *success = false );
    }
  } else if( strcmp( buf, "jointtype" ) == 0 )
    rkJointQueryAssign( rkLinkJoint(prm->l), zFToken( fp, buf, BUFSIZ ) );
  else if( strcmp( buf, "mass" ) == 0 )
    rkLinkSetMass( prm->l, zFDouble( fp ) );
  else if( strcmp( buf, "stuff" ) == 0 ){
    if( strlen( zFToken( fp, buf, BUFSIZ ) ) >= BUFSIZ ){
      buf[BUFSIZ-1] = '\0';
      ZRUNWARN( RK_WARN_TOOLNG_NAME, buf );
    }
    rkLinkSetStuff( prm->l, buf );
  } else if( strcmp( buf, "COM" ) == 0 )
    zVec3DFScan( fp, rkLinkCOM(prm->l) );
  else if( strcmp( buf, "inertia" ) == 0 )
    zMat3DFScan( fp, rkLinkInertia(prm->l) );
  else if( strcmp( buf, "pos" ) == 0 )
    zVec3DFScan( fp, rkLinkOrgPos(prm->l) );
  else if( strcmp( buf, "att" ) == 0 )
    zMat3DFScan( fp, rkLinkOrgAtt(prm->l) );
  else if( strcmp( buf, "rot" ) == 0 ){
    zVec3DFScan( fp, &axis );
    angle = zDeg2Rad( zFDouble( fp ) );
    if( zVec3DNormalizeDRC( &axis ) > 0 ){
      zVec3DMulDRC( &axis, angle );
      zMat3DRot( rkLinkOrgAtt(prm->l), &axis, rkLinkOrgAtt(prm->l) );
    }
  } else if( strcmp( buf, "frame" ) == 0 )
    zFrame3DFScan( fp, rkLinkOrgFrame(prm->l) );
  else if( strcmp( buf, "DH" ) == 0 )
    zFrame3DDHFScan( fp, rkLinkOrgFrame(prm->l) );
  else if( strcmp( buf, "parent" ) == 0 ){
    zFToken( fp, buf, BUFSIZ );
    zNameFind( prm->larray, prm->nl, buf, pl );
    if( !pl ){
      ZRUNERROR( RK_ERR_LINK_UNKNOWN, buf );
      return ( *success = false );
    }
    rkLinkAddChild( pl, prm->l );
  } else if( strcmp( buf, "shape" ) == 0 ){
    zFToken( fp, buf, BUFSIZ );
    zNameFind( prm->sarray, prm->ns, buf, sp );
    if( !sp ){
      ZRUNERROR( RK_ERR_SHAPE_UNKNOWN, buf );
      return ( *success = false );
    }
    rkLinkShapePush( prm->l, sp );
  } else if( !rkJointQueryFScan( fp, buf, rkLinkJoint(prm->l), prm->marray, prm->nm ) )
    return false;
  return true;
}

rkLink *rkLinkFScan(FILE *fp, rkLink *l, rkLink *larray, int nl, zShape3D *sarray, int ns, rkMotor *marray, int nm)
{
  _rkLinkParam prm;

  prm.l = l;
  prm.larray = larray;
  prm.nl = nl;
  prm.sarray = sarray;
  prm.ns = ns;
  prm.marray = marray;
  prm.nm = nm;
  rkJointAssign( rkLinkJoint(l), &rk_joint_fixed );
  if( !zFieldFScan( fp, _rkLinkFScan, &prm ) ) return NULL;
  if( zNamePtr(l) ) return l;
  ZRUNERROR( RK_ERR_LINK_UNNAMED );
  return NULL;
}

typedef struct{
  rkLinkArray *larray;
  zShape3DArray *sarray;
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
  rkLinkSetMass( (rkLink*)obj, ZTKDouble(ztk) );
  return obj;
}
static void *_rkLinkStuffFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkLinkSetStuff( (rkLink*)obj, ZTKVal(ztk) ) ? obj : NULL;
}
static void *_rkLinkCOMFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zVec3DFromZTK( rkLinkCOM((rkLink*)obj), ztk );
  return obj;
}
static void *_rkLinkInertiaFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zMat3DFromZTK( rkLinkInertia((rkLink*)obj), ztk );
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
  zVec3D axis;
  double angle;
  zVec3DFromZTK( &axis, ztk );
  angle = zDeg2Rad( ZTKDouble( ztk ) );
  if( zVec3DNormalizeDRC( &axis ) > 0 ){
    zVec3DMulDRC( &axis, angle );
    zMat3DRot( rkLinkOrgAtt((rkLink*)obj), &axis, rkLinkOrgAtt((rkLink*)obj) );
  }
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
  zArrayFindName( ((_rkLinkRefPrp*)arg)->sarray, ZTKVal(ztk), sp );
  if( !sp ){
    ZRUNERROR( RK_ERR_SHAPE_UNKNOWN, ZTKVal(ztk) );
    return NULL;
  }
  return rkLinkShapePush( (rkLink*)obj, sp ) ? obj : NULL;
}

static void _rkLinkNameFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName((rkLink*)obj) );
}
static void _rkLinkJointTypeFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", rkLinkJointTypeStr((rkLink*)obj) );
}
static void _rkLinkMassFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%.10g\n", rkLinkMass((rkLink*)obj) );
}
static void _rkLinkStuffFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", rkLinkStuff((rkLink*)obj) );
}
static void _rkLinkCOMFPrint(FILE *fp, int i, void *obj){
  zVec3DFPrint( fp, rkLinkCOM((rkLink*)obj) );
}
static void _rkLinkInertiaFPrint(FILE *fp, int i, void *obj){
  zMat3DFPrint( fp, rkLinkInertia((rkLink*)obj) );
}
static void _rkLinkPosFPrint(FILE *fp, int i, void *obj){
  zVec3DFPrint( fp, rkLinkOrgPos((rkLink*)obj) );
}
static void _rkLinkAttFPrint(FILE *fp, int i, void *obj){
  zMat3DFPrint( fp, rkLinkOrgAtt((rkLink*)obj) );
}

static ZTKPrp __ztk_prp_rklink[] = {
  { "name", 1, _rkLinkNameFromZTK, _rkLinkNameFPrint },
  { "jointtype", 1, _rkLinkJointTypeFromZTK, _rkLinkJointTypeFPrint },
  { "mass", 1, _rkLinkMassFromZTK, _rkLinkMassFPrint },
  { "stuff", 1, _rkLinkStuffFromZTK, _rkLinkStuffFPrint },
  { "COM", 1, _rkLinkCOMFromZTK, _rkLinkCOMFPrint },
  { "inertia", 1, _rkLinkInertiaFromZTK, _rkLinkInertiaFPrint },
  { "pos", 1, _rkLinkPosFromZTK, _rkLinkPosFPrint },
  { "att", 1, _rkLinkAttFromZTK, _rkLinkAttFPrint },
  { "rot", -1, _rkLinkRotFromZTK, NULL },
  { "frame", 1, _rkLinkFrameFromZTK, NULL },
  { "DH", 1, _rkLinkDHFromZTK, NULL },
  { "parent", 1, _rkLinkParentFromZTK, NULL },
  { "shape", -1, _rkLinkShapeFromZTK, NULL },
};

bool rkLinkRegZTK(ZTK *ztk)
{
  return rkJointRegZTKRevol( ztk, ZTK_TAG_RKLINK ) &&
         rkJointRegZTKPrism( ztk, ZTK_TAG_RKLINK ) &&
         rkJointRegZTKCylin( ztk, ZTK_TAG_RKLINK ) &&
         rkJointRegZTKHooke( ztk, ZTK_TAG_RKLINK ) &&
         rkJointRegZTKSpher( ztk, ZTK_TAG_RKLINK ) &&
         rkJointRegZTKFloat( ztk, ZTK_TAG_RKLINK ) &&
         rkJointRegZTKBrFloat( ztk, ZTK_TAG_RKLINK ) &&
         ZTKDefRegPrp( ztk, ZTK_TAG_RKLINK, __ztk_prp_rklink ) ? true : false;
}

rkLink *rkLinkFromZTK(rkLink *link, rkLinkArray *larray, zShape3DArray *sarray, rkMotorArray *motorarray, ZTK *ztk)
{
  _rkLinkRefPrp prp;

  rkLinkInit( link );
  prp.larray = larray;
  prp.sarray = sarray;
  if( !ZTKEncodeKey( link, &prp, ztk, __ztk_prp_rklink ) ) return NULL;
  rkJointFromZTK( rkLinkJoint(link), motorarray, ztk );
  return link;
}

void rkLinkFPrint(FILE *fp, rkLink *link)
{
  zShapeListCell *cp;

  ZTKPrpKeyFPrint( fp, link, __ztk_prp_rklink );
  rkJointFPrint( fp, rkLinkJoint(link), zName(link) );
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
  fprintf( fp, "Link(name:%s offset:%d)\n", zName(l), rkLinkOffset(l) );
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
  fprintf( fp, "|-%s (%s:%d)\n", zName(l), rkJointTypeStr(rkLinkJoint(l)), rkLinkOffset(l) );

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
