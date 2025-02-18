/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_ik_cell - inverse kinematics: cell
 */

#include <roki/rk_chain.h>

/* initialize IK attribute */
rkIKAttr* rkIKAttrInit(rkIKAttr *attr)
{
  attr->user_defined_type = -1;
  attr->id = -1;
  attr->id_sub = -1;
  zVec3DZero( &attr->attention_point );
  zVec3DCreate( &attr->weight, 1.0, 1.0, 1.0 ); /* default weight on constraint*/
  attr->mask = RK_IK_ATTR_MASK_NONE;
  return attr;
}

/* ********************************************************** */
/* CLASS: rkIKCell
 * inverse kinematics cell class
 * ********************************************************** */

/* initialize an IK cell. */
void rkIKCellInit(rkIKCell *cell)
{
  zNameSetPtr( &cell->data, NULL );
  cell->data.constraint = NULL;
  rkIKRefClear( rkIKCellRef(cell) );
  rkIKAttrInit( &cell->data.attr );
  cell->data.priority = 0;
  cell->data.mode = RK_IK_CELL_MODE_NONE;
  rkIKCellAcmZero( cell );
  cell->data._eval = 0;
  cell->data._util = NULL;
}

/* assign priority, attributes and a constraint to an IK cell. */
void rkIKCellAssign(rkIKCell *cell, int priority, rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util)
{
  zNameSetPtr( &cell->data, NULL );
  rkIKAttrInit( &cell->data.attr );
  if( attr ){
    rkIKCellLinkID(cell) = ( mask & RK_IK_ATTR_MASK_ID ) ? attr->id : 0;
    rkIKCellLinkID2(cell) = ( mask & RK_IK_ATTR_MASK_ID_SUB ) ? attr->id_sub : 0;
    if( mask & RK_IK_ATTR_MASK_ATTENTION_POINT )
      zVec3DCopy( &attr->attention_point, rkIKCellAttentionPoint(cell) );
    if( mask & RK_IK_ATTR_MASK_WEIGHT )
      zVec3DCopy( &attr->weight, rkIKCellWeight(cell) );
    cell->data.attr.mask = mask;
  }
  cell->data.priority = priority;
  cell->data.mode = RK_IK_CELL_MODE_XYZ;

  rkIKRefClear( rkIKCellRef(cell) );
  rkIKCellAcmZero( cell );
  rkIKCellDisable( cell );
  cell->data.constraint = constraint;
  cell->data._eval = 0;
  cell->data._util = util;
}

/* allocate an IK cell */
static rkIKCell *_rkIKCellAlloc(void)
{
  rkIKCell *cell;

  if( !( cell = zAlloc( rkIKCell, 1 ) ) )
    ZALLOCERROR();
  return cell;
}

/* set name of an IK cell */
static bool _rkIKCellSetName(rkIKCell *cell, const char* name)
{
  if( name ){
    zNameSet( &cell->data, name );
    if( !zNamePtr( &cell->data ) ){
      ZALLOCERROR();
      return false;
    }
  }
  return true;
}

/* allocate an empty IK cell. */
rkIKCell *rkIKCellAlloc(void)
{
  rkIKCell *cell;

  if( ( cell = _rkIKCellAlloc() ) )
    rkIKCellInit( cell );
  return cell;
}

/* create an IK cell. */
rkIKCell *rkIKCellCreate(const char *name, int priority, rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util)
{
  rkIKCell *cell;

  if( !( cell = _rkIKCellAlloc() ) ) return NULL;
  rkIKCellAssign( cell, priority, attr, mask, constraint, util );
  if( !_rkIKCellSetName( cell, name ) )
    zFree( cell );
  return cell;
}

/* clone an IK cell. */
rkIKCell *rkIKCellClone(rkIKCell *src)
{
  rkIKCell *cell;

  if( !( cell = _rkIKCellAlloc() ) ) return NULL;
  zCopy( rkIKCell, src, cell );
  if( !_rkIKCellSetName( cell, zNamePtr(&src->data) ) )
    zFree( cell );
  return cell;
}

/* destroy an IK cell. */
void rkIKCellDestroy(rkIKCell *cell)
{
  zNameFree( &cell->data );
  rkIKCellInit( cell );
}

/* set weight on a constraint of an IK cell */
void rkIKCellSetWeight(rkIKCell *cell, double w1, double w2, double w3)
{
  rkIKAttrSetWeight( rkIKCellAttr(cell), w1, w2, w3 );
}

/* set the weight vector on a constraint of an IK cell */
void rkIKCellSetWeightVec(rkIKCell *cell, zVec3D *weight)
{
  rkIKAttrSetWeight( rkIKCellAttr(cell), weight->c.x, weight->c.y, weight->c.z );
}

/* set the referential position of an IK cell */
void rkIKCellSetRef(rkIKCell *cell, double v1, double v2, double v3)
{
  rkIKCellEnable( cell );
  cell->data.constraint->ref_fp( rkIKCellRef(cell), v1, v2, v3 );
}

/* set the referential position vector of an IK cell */
void rkIKCellSetRefVec(rkIKCell *cell, zVec3D *vec)
{
  rkIKCellSetRef( cell, vec->c.x, vec->c.y, vec->c.z );
}

/* set the referential attitude matrix of an IK cell */
void rkIKCellSetRefAtt(rkIKCell *cell, zMat3D *att)
{
  rkIKCellEnable( cell );
  zMat3DCopy( att, rkIKCellRefAtt(cell) );
}

/* zero the accumulated error of a highly-prioritized IK constraint. */
void rkIKCellAcmZero(rkIKCell *cell)
{
  zVec3DZero( &cell->data._acm.ae.p );
  zVec3DCreate( &cell->data._acm.e_old.p, HUGE_VAL, HUGE_VAL, HUGE_VAL );
  zVec3DCreate( &cell->data._acm.h_old, HUGE_VAL, HUGE_VAL, HUGE_VAL );
}

/* reference */

void rkIKRefSetPos(rkIKRef *ref, double x, double y, double z)
{ /* referential position */
  zVec3DCreate( &ref->pos, x, y, z );
}

void rkIKRefSetZYX(rkIKRef *ref, double azim, double elev, double tilt)
{ /* referential attitude by z-y-x Eulerian angle */
  zMat3DFromZYX( &ref->att, azim, elev, tilt );
}

void rkIKRefSetZYZ(rkIKRef *ref, double heading, double pitch, double bank)
{ /* referential attitude by z-y-z Eulerian angle */
  zMat3DFromZYZ( &ref->att, heading, pitch, bank );
}

void rkIKRefSetAA(rkIKRef *ref, double x, double y, double z)
{ /* referential attitude by angle-axis vector */
  zVec3D aa;

  zMat3DFromAA( &ref->att, zVec3DCreate( &aa, x, y, z ) );
}

/* Jacobian matrix */

zMat rkIKJacobiLinkWldLin(rkChain *chain, rkIKAttr *attr, zMat j)
{ /* linear motion of a link in the world frame */
  return rkChainLinkWldLinJacobi( chain, attr->id, &attr->attention_point, j );
}

zMat rkIKJacobiLinkWldAng(rkChain *chain, rkIKAttr *attr, zMat j)
{ /* angular motion of a link in the world frame */
  return rkChainLinkWldAngJacobi( chain, attr->id, j );
}

zMat rkIKJacobiLinkL2LLin(rkChain *chain, rkIKAttr *attr, zMat j)
{ /* relative linear motion of a link with respect to another link */
  return rkChainLinkToLinkLinJacobi( chain, attr->id, attr->id_sub, &attr->attention_point, j );
}

zMat rkIKJacobiLinkL2LAng(rkChain *chain, rkIKAttr *attr, zMat j)
{ /* relative angular motion of a link with respect to another link */
  return rkChainLinkToLinkAngJacobi( chain, attr->id, attr->id_sub, j );
}

zMat rkIKJacobiCOM(rkChain *chain, rkIKAttr *attr, zMat j)
{ /* COM motion of a kinematic chain in the world frame */
  return rkChainCOMJacobi( chain, j );
}

zMat rkIKJacobiAM(rkChain *chain, rkIKAttr *attr, zMat j)
{ /* angular momentum about a point of a kinematic chain */
  return rkChainAMMat( chain, &attr->attention_point, j );
}

zMat rkIKJacobiAMCOM(rkChain *chain, rkIKAttr *attr, zMat j)
{ /* angular momentum about COM of a kinematic chain */
  return rkChainAMCOMMat( chain, j );
}

/* displacement error */

zVec3D *rkIKLinkWldPosErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* position error of a link in the world frame */
  zVec3D p;

  zXform3D( rkChainLinkWldFrame(chain,attr->id), &attr->attention_point, &p );
  return zVec3DSub( &ref->pos, &p, err );
}

zVec3D *rkIKLinkWldAttErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* attitude error of a link in the world frame */
  return zMat3DError( &ref->att, rkChainLinkWldAtt(chain,attr->id), err );
}

zVec3D *rkIKLinkL2LPosErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* position error of a link with respect to another link */
  zVec3D p;

  zXform3D( rkChainLinkWldFrame(chain,attr->id_sub), &attr->attention_point, &p );
  zVec3DSubDRC( &p, rkChainLinkWldPos(chain,attr->id) );
  return zVec3DSub( &ref->pos, &p, err );
}

zVec3D *rkIKLinkL2LAttErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* attitude error of a link with respect to another link */
  zMat3D m;
  zVec3D e;

  zMulMat3DTMat3D( rkChainLinkWldAtt(chain,attr->id), rkChainLinkWldAtt(chain,attr->id_sub), &m );
  zMat3DError( &ref->att, &m, &e );
  return zMulMat3DVec3D( rkChainLinkWldAtt(chain,attr->id), &e, err );
}

zVec3D *rkIKCOMErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* COM error of a kinematic chain in the world frame */
  return zVec3DSub( &ref->pos, rkChainWldCOM(chain), err );
}

zVec3D *rkIKAMErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* angular momentum cancelleration */
  rkChainAngularMomentum( chain, &attr->attention_point, err );
  return zVec3DRevDRC( err );
}

zVec3D *rkIKAMCOMErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* angular momentum cancelleration */
  rkChainAngularMomentum( chain, rkChainWldCOM(chain), err );
  return zVec3DRevDRC( err );
}

/* bind current position/attitude */

void rkIKBindLinkWldPos(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref)
{ /* current position of a link in the world frame */
  zXform3D( rkChainLinkWldFrame(chain,attr->id), &attr->attention_point, &ref->pos );
}

void rkIKBindLinkWldAtt(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref)
{ /* current attitude of a link in the world frame */
  zMat3DCopy( rkChainLinkWldAtt(chain,attr->id), &ref->att );
}

void rkIKBindLinkL2LPos(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref)
{ /* current position of a link with respect to another link */
  zVec3D p;

  zXform3D( rkChainLinkWldFrame(chain,attr->id_sub), &attr->attention_point, &p );
  zVec3DSub( &p, rkChainLinkWldPos(chain,attr->id), &ref->pos );
}

void rkIKBindLinkL2LAtt(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref)
{ /* current attitude of a link with respect to another link */
  zMulMat3DTMat3D( rkChainLinkWldAtt(chain,attr->id), rkChainLinkWldAtt(chain,attr->id_sub), &ref->att );
}

void rkIKBindCOM(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref)
{ /* current COM of a kinematic chain in the world frame */
  /* NOTE: chain COM has to be updated in advance. */
  zVec3DCopy( rkChainWldCOM(chain), &ref->pos );
}

void rkIKBindAM(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref)
{ /* current angular momentum about a point of a kinematic chain in the world frame */
  rkChainAngularMomentum( chain, &attr->attention_point, &ref->pos );
}

void rkIKBindAMCOM(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref)
{ /* current angular momentum about COM of a kinematic chain in the world frame */
  rkChainAngularMomentum( chain, rkChainWldCOM(chain), &ref->pos );
}

/* error accumulation correction */

zVec3D *rkIKAcmPos(rkChain *chain, rkIKAcm *acm, void *util, zVec3D *vec)
{
  return zVec3DAddDRC( vec, zVec3DCatDRC( &acm->ae.p, 1.0, vec ) );
}

zVec3D *rkIKAcmAtt(rkChain *chain, rkIKAcm *acm, void *util, zVec3D *vec)
{
  zEP e;

  zAA2EP( vec, &e );
  zEPCatDRC( &acm->ae.e, 1.0, &e );
  zEPCascade( &e, &acm->ae.e, &e );
  return zEP2AA( &e, vec );
}

/* clone an IK cell list \a src to \a dest. */
rkIKCellList *rkIKCellListClone(rkIKCellList *src, rkIKCellList *dest)
{
  rkIKCell *scp, *new_cell;

  zListInit( dest );
  zListForEach( src, scp ){
    if( !( new_cell = rkIKCellClone( scp ) ) ){
      ZALLOCERROR();
      break;
    }
    zListInsertHead( dest, new_cell );
  }
  if( zListSize( dest ) != zListSize( src ) ){
    rkIKCellListDestroy( dest );
    return NULL;
  }
  return dest;
}

/* destroy a list of IK cells. */
void rkIKCellListDestroy(rkIKCellList *list)
{
  rkIKCell *cell;

  while( !zListIsEmpty( list ) ){
    zListDeleteHead( list, &cell );
    rkIKCellDestroy( cell );
    zFree( cell );
  }
}

/* ********************************************************** */
/* inverse kinematics constraint class
 * ********************************************************** */

/* read an attention point of an IK cell from ZTK. */
static bool _rkIKConstraintAttentionPointFromZTK(rkIKAttr *attr, ubyte *mask, ZTK *ztk)
{
  if( !ZTKValCmp( ztk, "at" ) ) return false;
  ZTKValNext( ztk );
  zVec3DFromZTK( &attr->attention_point, ztk );
  *mask |= RK_IK_ATTR_MASK_ATTENTION_POINT;
  return true;
}

/* read a weighting vector of an IK cell from ZTK. */
static bool _rkIKConstraintWeightFromZTK(rkIKAttr *attr, ubyte *mask, ZTK *ztk)
{
  if( !ZTKValCmp( ztk, "w" ) ) return false;
  ZTKValNext( ztk );
  zVec3DFromZTK( &attr->weight, ztk );
  *mask |= RK_IK_ATTR_MASK_WEIGHT;
  return true;
}

/* read a name of a link for an IK cell from ZTK. */
static bool _rkIKConstraintLinkFromZTK(rkChain *chain, int num, rkIKAttr *attr, ubyte *mask, ZTK *ztk)
{
  rkLink *link;

  if( !( link = rkChainFindLink( chain, ZTKVal(ztk) ) ) ){
    ZRUNERROR( RK_ERR_LINK_UNKNOWN, ZTKVal(ztk) );
    return false;
  }
  if( num == -1 ){
    ZRUNERROR( RK_WARN_IK_CONSTRAINT_DUALLYDEFINED_LINK, ZTKVal(ztk) );
    return false;
  } else
  if( num == 0 ){
    attr->id = link - rkChainRoot(chain);
    *mask |= RK_IK_ATTR_MASK_ID;
  } else{
    attr->id_sub = link - rkChainRoot(chain);
    *mask |= RK_IK_ATTR_MASK_ID_SUB;
  }
  ZTKValNext( ztk );
  return true;
}

/* print an attention point of a link of an IK cell to a file in ZTK format. */
static void _rkIKConstraintAttentionPointFPrintZTK(FILE *fp, rkChain *chain, rkIKCell *cell){
  if( cell->data.attr.mask & RK_IK_ATTR_MASK_ATTENTION_POINT ){
    fprintf( fp, " at" );
    zVec3DValueFPrint( fp, rkIKCellAttentionPoint(cell) );
  }
}

/* print a weighting vector of a link of an IK cell to a file in ZTK format. */
static void _rkIKConstraintWeightFPrintZTK(FILE *fp, rkChain *chain, rkIKCell *cell){
  if( cell->data.attr.mask & RK_IK_ATTR_MASK_WEIGHT ){
    fprintf( fp, " w" );
    zVec3DValueFPrint( fp, rkIKCellWeight(cell) );
  }
}

/* print a name of a link of an IK cell to a file in ZTK format. */
static void _rkIKConstraintLinkFPrintZTK(FILE *fp, rkChain *chain, rkIKCell *cell){
  if( cell->data.attr.mask & RK_IK_ATTR_MASK_ID )
    fprintf( fp, " %s", rkChainLinkName(chain,rkIKCellLinkID(cell)) );
}

/* print a name of a sublink of an IK cell to a file in ZTK format. */
static void _rkIKConstraintLinkSubFPrintZTK(FILE *fp, rkChain *chain, rkIKCell *cell){
  if( cell->data.attr.mask & RK_IK_ATTR_MASK_ID_SUB )
    fprintf( fp, " %s", rkChainLinkName(chain,rkIKCellLinkID2(cell)) );
}

/* IK constraint: link_world_pos */
static bool _rkIKConstraintWldPosFromZTK(rkChain *chain, rkIKAttr *attr, ubyte *mask, ZTK *ztk){
  int linknum = 0;
  while( ZTKValPtr(ztk) ){
    if( !_rkIKConstraintAttentionPointFromZTK( attr, mask, ztk ) &&
        !_rkIKConstraintWeightFromZTK( attr, mask, ztk ) ){
      if( !_rkIKConstraintLinkFromZTK( chain, linknum, attr, mask, ztk ) ){
        ZRUNERROR( ZEDA_ERR_ZTK_UNKNOWN_VAL, ZTKVal(ztk), ZTKTag(ztk), ZTKKey(ztk) );
        return false;
      }
      if( linknum == 0 ) linknum = -1;
    }
  }
  return true;
}
static void _rkIKConstraintWldPosFPrintZTK(FILE *fp, rkChain *chain, rkIKCell *cell){
  _rkIKConstraintLinkFPrintZTK( fp, chain, cell );
  _rkIKConstraintAttentionPointFPrintZTK( fp, chain, cell );
  _rkIKConstraintWeightFPrintZTK( fp, chain, cell );
  fprintf( fp, "\n" );
}

const rkIKConstraint rk_ik_constraint_link_world_pos = {
  typestr: "world_pos",
  ref_fp: rkIKRefSetPos,
  cmat_fp: rkIKJacobiLinkWldLin,
  cvec_fp: rkIKLinkWldPosErr,
  bind_fp: rkIKBindLinkWldPos,
  acm_fp: rkIKAcmPos,
  fromZTK: _rkIKConstraintWldPosFromZTK,
  fprintZTK: _rkIKConstraintWldPosFPrintZTK,
};

/* IK constraint: link_world_att */
static bool _rkIKConstraintWldAttFromZTK(rkChain *chain, rkIKAttr *attr, ubyte *mask, ZTK *ztk){
  int linknum = 0;
  while( ZTKValPtr(ztk) ){
    if( !_rkIKConstraintWeightFromZTK( attr, mask, ztk ) ){
      if( !_rkIKConstraintLinkFromZTK( chain, linknum, attr, mask, ztk ) ){
        ZRUNERROR( ZEDA_ERR_ZTK_UNKNOWN_VAL, ZTKVal(ztk), ZTKTag(ztk), ZTKKey(ztk) );
        return false;
      }
      if( linknum == 0 ) linknum = -1;
    }
  }
  return true;
}
static void _rkIKConstraintWldAttFPrintZTK(FILE *fp, rkChain *chain, rkIKCell *cell){
  _rkIKConstraintWeightFPrintZTK( fp, chain, cell );
  _rkIKConstraintLinkFPrintZTK( fp, chain, cell );
  fprintf( fp, "\n" );
}

const rkIKConstraint rk_ik_constraint_link_world_att = {
  typestr: "world_att",
  ref_fp: rkIKRefSetZYX,
  cmat_fp: rkIKJacobiLinkWldAng,
  cvec_fp: rkIKLinkWldAttErr,
  bind_fp: rkIKBindLinkWldAtt,
  acm_fp: rkIKAcmAtt,
  fromZTK: _rkIKConstraintWldAttFromZTK,
  fprintZTK: _rkIKConstraintWldAttFPrintZTK,
};

/* IK constraint: link_to_link_pos */
static bool _rkIKConstraintL2LPosFromZTK(rkChain *chain, rkIKAttr *attr, ubyte *mask, ZTK *ztk){
  int linknum = 0;
  while( ZTKValPtr(ztk) ){
    if( !_rkIKConstraintAttentionPointFromZTK( attr, mask, ztk ) &&
        !_rkIKConstraintWeightFromZTK( attr, mask, ztk ) ){
      if( !_rkIKConstraintLinkFromZTK( chain, linknum, attr, mask, ztk ) ){
        ZRUNERROR( ZEDA_ERR_ZTK_UNKNOWN_VAL, ZTKVal(ztk), ZTKTag(ztk), ZTKKey(ztk) );
        return false;
      }
      if( linknum > -1 ){
        if( ++linknum > 1 ) linknum = -1;
      }
    }
  }
  return true;
}
static void _rkIKConstraintL2LPosFPrintZTK(FILE *fp, rkChain *chain, rkIKCell *cell){
  _rkIKConstraintLinkFPrintZTK( fp, chain, cell );
  _rkIKConstraintLinkSubFPrintZTK( fp, chain, cell );
  _rkIKConstraintAttentionPointFPrintZTK( fp, chain, cell );
  _rkIKConstraintWeightFPrintZTK( fp, chain, cell );
  fprintf( fp, "\n" );
}

const rkIKConstraint rk_ik_constraint_link2link_pos = {
  typestr: "l2l_pos",
  ref_fp: rkIKRefSetPos,
  cmat_fp: rkIKJacobiLinkL2LLin,
  cvec_fp: rkIKLinkL2LPosErr,
  bind_fp: rkIKBindLinkL2LPos,
  acm_fp: rkIKAcmPos,
  fromZTK: _rkIKConstraintL2LPosFromZTK,
  fprintZTK: _rkIKConstraintL2LPosFPrintZTK,
};

/* IK constraint: link_to_link_att */
static bool _rkIKConstraintL2LAttFromZTK(rkChain *chain, rkIKAttr *attr, ubyte *mask, ZTK *ztk){
  int linknum = 0;
  while( ZTKValPtr(ztk) ){
    if( !_rkIKConstraintWeightFromZTK( attr, mask, ztk ) ){
      if( !_rkIKConstraintLinkFromZTK( chain, linknum, attr, mask, ztk ) ){
        ZRUNERROR( ZEDA_ERR_ZTK_UNKNOWN_VAL, ZTKVal(ztk), ZTKTag(ztk), ZTKKey(ztk) );
        return false;
      }
      if( linknum > -1 ){
        if( ++linknum > 1 ) linknum = -1;
      }
    }
  }
  return true;
}
static void _rkIKConstraintL2LAttFPrintZTK(FILE *fp, rkChain *chain, rkIKCell *cell){
  _rkIKConstraintLinkFPrintZTK( fp, chain, cell );
  _rkIKConstraintLinkSubFPrintZTK( fp, chain, cell );
  _rkIKConstraintWeightFPrintZTK( fp, chain, cell );
  fprintf( fp, "\n" );
}

const rkIKConstraint rk_ik_constraint_link2link_att = {
  typestr: "l2l_att",
  ref_fp: rkIKRefSetZYX,
  cmat_fp: rkIKJacobiLinkL2LAng,
  cvec_fp: rkIKLinkL2LAttErr,
  bind_fp: rkIKBindLinkL2LAtt,
  acm_fp: rkIKAcmAtt,
  fromZTK: _rkIKConstraintL2LAttFromZTK,
  fprintZTK: _rkIKConstraintL2LAttFPrintZTK,
};

/* IK constraint: center_of_mass_pos */
static bool _rkIKConstraintCOMFromZTK(rkChain *chain, rkIKAttr *attr, ubyte *mask, ZTK *ztk){
  while( ZTKValPtr(ztk) ){
    if( !_rkIKConstraintWeightFromZTK( attr, mask, ztk ) ){
      ZRUNERROR( ZEDA_ERR_ZTK_UNKNOWN_VAL, ZTKVal(ztk), ZTKTag(ztk), ZTKKey(ztk) );
      return false;
    }
  }
  return true;
}
static void _rkIKConstraintCOMFPrintZTK(FILE *fp, rkChain *chain, rkIKCell *cell){
  _rkIKConstraintWeightFPrintZTK( fp, chain, cell );
  fprintf( fp, "\n" );
}

const rkIKConstraint rk_ik_constraint_world_com = {
  typestr: "com",
  ref_fp: rkIKRefSetPos,
  cmat_fp: rkIKJacobiCOM,
  cvec_fp: rkIKCOMErr,
  bind_fp: rkIKBindCOM,
  acm_fp: rkIKAcmPos,
  fromZTK: _rkIKConstraintCOMFromZTK,
  fprintZTK: _rkIKConstraintCOMFPrintZTK,
};

/* IK constraint: angular_momentum */
static bool _rkIKConstraintAMFromZTK(rkChain *chain, rkIKAttr *attr, ubyte *mask, ZTK *ztk){
  while( ZTKValPtr(ztk) ){
    if( !_rkIKConstraintAttentionPointFromZTK( attr, mask, ztk ) &&
        !_rkIKConstraintWeightFromZTK( attr, mask, ztk ) ){
      ZRUNERROR( ZEDA_ERR_ZTK_UNKNOWN_VAL, ZTKVal(ztk), ZTKTag(ztk), ZTKKey(ztk) );
      return false;
    }
  }
  return true;
}
static void _rkIKConstraintAMFPrintZTK(FILE *fp, rkChain *chain, rkIKCell *cell){
  _rkIKConstraintAttentionPointFPrintZTK( fp, chain, cell );
  _rkIKConstraintWeightFPrintZTK( fp, chain, cell );
  fprintf( fp, "\n" );
}

const rkIKConstraint rk_ik_constraint_world_angular_momentum = {
  typestr: "angular_momentum",
  ref_fp: rkIKRefSetPos,
  cmat_fp: rkIKJacobiAM,
  cvec_fp: rkIKAMErr,
  bind_fp: rkIKBindAM,
  acm_fp: rkIKAcmAtt,
  fromZTK: _rkIKConstraintAMFromZTK,
  fprintZTK: _rkIKConstraintAMFPrintZTK,
};

/* IK constraint: angular_momentum_about_com */
const rkIKConstraint rk_ik_constraint_world_angular_momentum_about_com = {
  typestr: "angular_momentum_about_com",
  ref_fp: rkIKRefSetPos,
  cmat_fp: rkIKJacobiAMCOM,
  cvec_fp: rkIKAMCOMErr,
  bind_fp: rkIKBindAMCOM,
  acm_fp: rkIKAcmAtt,
  fromZTK: _rkIKConstraintCOMFromZTK,
  fprintZTK: _rkIKConstraintCOMFPrintZTK,
};

/* an array of pre-defined IK constraints */
const rkIKConstraint *rk_ik_constraint_array[] = {
  &rk_ik_constraint_link_world_pos,
  &rk_ik_constraint_link_world_att,
  &rk_ik_constraint_link2link_pos,
  &rk_ik_constraint_link2link_att,
  &rk_ik_constraint_world_com,
  &rk_ik_constraint_world_angular_momentum,
  &rk_ik_constraint_world_angular_momentum_about_com,
};

/* a list of user-defined IK constraints */
static rkIKConstraintList rk_ik_constraint_list
#ifndef __cplusplus
 = {
  size: 0,
  root: { prev: &rk_ik_constraint_list.root, next: &rk_ik_constraint_list.root },
}
#endif
;

/* find an IK constraint from the array of pre-defined constraints */
static const rkIKConstraint *_rkIKConstraintFindFromArray(const char *typestr)
{
  int i, n;

  n = sizeof(rk_ik_constraint_array) / sizeof(rkIKConstraint*);
  for( i=0; i<n; i++ ){
    if( strcmp( typestr, rk_ik_constraint_array[i]->typestr ) == 0 )
      return rk_ik_constraint_array[i];
  }
  return NULL;
}

/* find an IK constraint from the list of user-defined constraints */
static rkIKConstraintListCell *_rkIKConstraintFindFromList(const char *typestr)
{
  rkIKConstraintListCell *cp;

  if( !zListIsEmpty( &rk_ik_constraint_list ) )
    zListForEach( &rk_ik_constraint_list, cp )
      if( strcmp( typestr, cp->data->typestr ) == 0 ) return cp;
  return NULL;
}

/* find an IK constraint */
const rkIKConstraint *rkIKConstraintFind(const char *typestr)
{
  const rkIKConstraint *constraint;
  rkIKConstraintListCell *cp;

  if( ( constraint = _rkIKConstraintFindFromArray( typestr ) ) ) return constraint;
  if( ( cp = _rkIKConstraintFindFromList( typestr ) ) ) return cp->data;
  ZRUNERROR( RK_ERR_IK_CONSTRAINT_NOTFOUND, typestr );
  return NULL;
}

/* add a user-defined IK constraint */
rkIKConstraintListCell *rkIKConstraintListAdd(const rkIKConstraint *constraint)
{
  rkIKConstraintListCell *cp;

  if( _rkIKConstraintFindFromArray( constraint->typestr ) ){
    ZRUNERROR( RK_ERR_IK_CONSTRAINT_PREDEFINED, constraint->typestr );
    return NULL;
  }
  if( ( cp = _rkIKConstraintFindFromList( constraint->typestr ) ) ){
    ZRUNWARN( RK_WARN_IK_CONSTRAINT_ALREADY_REGISTERED, constraint->typestr );
  } else{
    if( !( cp = zAlloc( rkIKConstraintListCell, 1 ) ) ){
      ZALLOCERROR();
      return NULL;
    }
    zListInsertHead( &rk_ik_constraint_list, cp );
  }
  cp->data = constraint;
  return cp;
}

/* destroy the lisft of user-defined IK constraints */
void rkIKConstraintListDestroy(void)
{
  zListDestroy( rkIKConstraintListCell, &rk_ik_constraint_list );
}
