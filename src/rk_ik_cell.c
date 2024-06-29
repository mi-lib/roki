/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_ik_cell - inverse kinematics: cell
 */

#include <roki/rk_chain.h>

/* ********************************************************** */
/* CLASS: rkIKCell
 * inverse kinematics cell class
 * ********************************************************** */

/* initialize constraint cell. */
void rkIKCellInit(rkIKCell *cell, rkIKAttr *attr, int mask, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKCVec_fp vf, rkIKBind_fp bf, rkIKAcm_fp af, void *util)
{
  zNameSetPtr( &cell->data, NULL );
  rkIKCellLinkID(cell)  = attr && ( mask & RK_IK_ATTR_ID ) ? attr->id : 0;
  rkIKCellLinkID2(cell) = attr && ( mask & RK_IK_ATTR_ID_SUB ) ? attr->id_sub : 0;
  if( attr && ( mask & RK_IK_ATTR_ATTENTION_POINT ) )
    zVec3DCopy( &attr->attention_point, rkIKCellAttentionPoint(cell) );
  else
    zVec3DZero( rkIKCellAttentionPoint(cell) );
  rkIKCellMode(cell) = mask & RK_IK_ATTR_FORCE ? RK_IK_CELL_FORCE : 0;
  if( attr && ( mask & RK_IK_ATTR_WEIGHT ) )
    zVec3DCopy( &attr->weight, rkIKCellWeight(cell) );
  else
    rkIKCellSetWeight( cell, 1.0, 1.0, 1.0 ); /* default weight on constraint*/

  rkIKRefClear( rkIKCellRef(cell) );
  rkIKCellAcmZero( cell );
  rkIKCellDisable( cell );
  cell->data._ref_fp = rf;
  cell->data._cmat_fp = mf;
  cell->data._cvec_fp = vf;
  cell->data._bind_fp = bf;
  cell->data._acm_fp = af;
  rkIKCellIndexOffset(cell) = 0;
  cell->data._eval = 0;
  cell->data._util = util;
}

#define _RK_IK_CELL_ALLOC_FUNC(__name,__operation) \
  rkIKCell *cell; \
  if( !( cell = zAlloc( rkIKCell, 1 ) ) ){ \
    ZALLOCERROR(); \
    return NULL; \
  } \
  __operation; \
  if( __name ){ \
    zNameSet( &cell->data, __name ); \
    if( !zNamePtr(&cell->data) ){ \
      ZALLOCERROR(); \
      zFree( cell ); \
      return NULL; \
    } \
  } \
  return cell

/* create an IK cell. */
rkIKCell *rkIKCellCreate(const char *name, rkIKAttr *attr, int mask, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKCVec_fp vf, rkIKBind_fp bf, rkIKAcm_fp af, void *util)
{
  _RK_IK_CELL_ALLOC_FUNC( name, rkIKCellInit( cell, attr, mask, rf, mf, vf, bf, af, util ) );
}

/* clone an IK cell. */
rkIKCell *rkIKCellClone(rkIKCell *src)
{
  _RK_IK_CELL_ALLOC_FUNC( zNamePtr(&src->data), zCopy( rkIKCell, src, cell ) );
}

/* destroy an IK cell. */
void rkIKCellDestroy(rkIKCell *cell)
{
  zNameFree( &cell->data );
  rkIKCellInit( cell, NULL, 0x0, NULL, NULL, NULL, NULL, NULL, NULL );
}

/* zero the accumulated error of a highly-prioritized IK constraint. */
void rkIKCellAcmZero(rkIKCell *cell)
{
  zVec3DZero( &cell->data.acm.ae.p );
  zVec3DCreate( &cell->data.acm.e_old.p, HUGE_VAL, HUGE_VAL, HUGE_VAL );
  zVec3DCreate( &cell->data.acm.h_old, HUGE_VAL, HUGE_VAL, HUGE_VAL );
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
  rkChainAM( chain, &attr->attention_point, err );
  return zVec3DRevDRC( err );
}

zVec3D *rkIKAMCOMErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* angular momentum cancelleration */
  rkChainAM( chain, rkChainWldCOM(chain), err );
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
  rkChainAM( chain, &attr->attention_point, &ref->pos );
}

void rkIKBindAMCOM(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref)
{ /* current angular momentum about COM of a kinematic chain in the world frame */
  rkChainAM( chain, rkChainWldCOM(chain), &ref->pos );
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
