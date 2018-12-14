/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_ik_cell - inverse kinematics: cell
 */

#include <roki/rk_ik.h>

/* ********************************************************** */
/* CLASS: rkIKCell
 * inverse kinematics cell class
 * ********************************************************** */

/* rkIKCellInit
 * - initialize constraint cell.
 */
void rkIKCellInit(rkIKCell *cell, rkIKCellAttr *attr, int mask, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKSRV_fp vf, rkIKBind_fp bf, rkIKAcm_fp af, void *util)
{
  cell->data.id = -1; /* dummy identifier */
  cell->data.attr.id = attr && ( mask & RK_IK_CELL_ATTR_ID ) ? attr->id : 0;
  cell->data.attr.id_sub = attr && ( mask & RK_IK_CELL_ATTR_ID_SUB ) ? attr->id_sub : 0;
  if( attr && ( mask & RK_IK_CELL_ATTR_AP ) )
    zVec3DCopy( &attr->ap, &cell->data.attr.ap );
  else
    zVec3DClear( &cell->data.attr.ap );
  cell->data.attr.mode = mask & RK_IK_CELL_ATTR_FORCE ? RK_IK_CELL_FORCE : 0;
  if( attr && ( mask & RK_IK_CELL_ATTR_WEIGHT ) )
    zVec3DCopy( &attr->w, &cell->data.attr.w );
  else
    rkIKCellSetWeight( cell, 1.0, 1.0, 1.0 ); /* default weight on constraint*/

  memset( &cell->data.ref, 0, sizeof(rkIKRef) );
  rkIKCellAcmClear( cell );
  rkIKCellDisable( cell );
  cell->data._ref_fp = rf;
  cell->data._cmat_fp = mf;
  cell->data._srv_fp = vf;
  cell->data._bind_fp = bf;
  cell->data._acm_fp = af;
  cell->data.index_offset = 0;
  cell->data._eval = 0;
  cell->data._util = util;
}

void rkIKCellAcmClear(rkIKCell *cell)
{
  zVec3DClear( &cell->data.acm.ae.p );
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

zMat rkIKJacobiLinkWldLin(rkChain *chain, rkIKCellAttr *attr, zMat j)
{ /* linear motion of a link in the world frame */
  return rkChainLinkWldLinJacobi( chain, attr->id, &attr->ap, j );
}

zMat rkIKJacobiLinkWldAng(rkChain *chain, rkIKCellAttr *attr, zMat j)
{ /* angular motion of a link in the world frame */
  return rkChainLinkWldAngJacobi( chain, attr->id, j );
}

zMat rkIKJacobiLinkL2LLin(rkChain *chain, rkIKCellAttr *attr, zMat j)
{ /* relative linear motion of a link with respect to another link */
  return rkChainLinkToLinkLinJacobi( chain, attr->id, attr->id_sub, &attr->ap, j );
}

zMat rkIKJacobiLinkL2LAng(rkChain *chain, rkIKCellAttr *attr, zMat j)
{ /* relative angular motion of a link with respect to another link */
  return rkChainLinkToLinkAngJacobi( chain, attr->id, attr->id_sub, j );
}

zMat rkIKJacobiCOM(rkChain *chain, rkIKCellAttr *attr, zMat j)
{ /* COM motion of a kinematic chain in the world frame */
  return rkChainCOMJacobi( chain, j );
}

zMat rkIKJacobiAM(rkChain *chain, rkIKCellAttr *attr, zMat j)
{ /* angular momentum about a point of a kinematic chain */
  return rkChainAMJacobi( chain, &attr->ap, j );
}

zMat rkIKJacobiAMCOM(rkChain *chain, rkIKCellAttr *attr, zMat j)
{ /* angular momentum about COM of a kinematic chain */
  return rkChainAMCOMJacobi( chain, j );
}

/* displacement error */

zVec3D *rkIKLinkWldPosErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* position error of a link in the world frame */
  zVec3D p;

  zXfer3D( rkChainLinkWldFrame(chain,attr->id), &attr->ap, &p );
  return zVec3DSub( &ref->pos, &p, err );
}

zVec3D *rkIKLinkWldAttErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* attitude error of a link in the world frame */
  return zMat3DError( &ref->att, rkChainLinkWldAtt(chain,attr->id), err );
}

zVec3D *rkIKLinkL2LPosErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* position error of a link with respect to another link */
  zVec3D p;

  zXfer3D( rkChainLinkWldFrame(chain,attr->id_sub), &attr->ap, &p );
  zVec3DSubDRC( &p, rkChainLinkWldPos(chain,attr->id) );
  return zVec3DSub( &ref->pos, &p, err );
}

zVec3D *rkIKLinkL2LAttErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* attitude error of a link with respect to another link */
  zMat3D m;
  zVec3D e;

  zMulMatTMat3D( rkChainLinkWldAtt(chain,attr->id), rkChainLinkWldAtt(chain,attr->id_sub), &m );
  zMat3DError( &ref->att, &m, &e );
  return zMulMatVec3D( rkChainLinkWldAtt(chain,attr->id), &e, err );
}

zVec3D *rkIKCOMErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* COM error of a kinematic chain in the world frame */
  return zVec3DSub( &ref->pos, rkChainWldCOM(chain), err );
}

zVec3D *rkIKAMErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* angular momentum cancelleration */
  rkChainAM( chain, &attr->ap, err );
  return zVec3DRevDRC( err );
}

zVec3D *rkIKAMCOMErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err)
{ /* angular momentum cancelleration */
  rkChainAM( chain, rkChainWldCOM(chain), err );
  return zVec3DRevDRC( err );
}

/* bind current position/attitude */

void rkIKBindLinkWldPos(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref)
{ /* current position of a link in the world frame */
  zXfer3D( rkChainLinkWldFrame(chain,attr->id), &attr->ap, &ref->pos );
}

void rkIKBindLinkWldAtt(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref)
{ /* current attitude of a link in the world frame */
  zMat3DCopy( rkChainLinkWldAtt(chain,attr->id), &ref->att );
}

void rkIKBindLinkL2LPos(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref)
{ /* current position of a link with respect to another link */
  zVec3D p;

  zXfer3D( rkChainLinkWldFrame(chain,attr->id_sub), &attr->ap, &p );
  zVec3DSub( &p, rkChainLinkWldPos(chain,attr->id), &ref->pos );
}

void rkIKBindLinkL2LAtt(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref)
{ /* current attitude of a link with respect to another link */
  zMulMatTMat3D( rkChainLinkWldAtt(chain,attr->id), rkChainLinkWldAtt(chain,attr->id_sub), &ref->att );
}

void rkIKBindCOM(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref)
{ /* current COM of a kinematic chain in the world frame */
  /* NOTE: chain COM has to be updated in advance. */
  zVec3DCopy( rkChainWldCOM(chain), &ref->pos );
}

void rkIKBindAM(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref)
{ /* current angular momentum about a point of a kinematic chain in the world frame */
  rkChainAM( chain, &attr->ap, &ref->pos );
}

void rkIKBindAMCOM(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref)
{ /* current angular momentum about COM of a kinematic chain in the world frame */
  rkChainAM( chain, rkChainWldCOM(chain), &ref->pos );
}

/* error accumulation correction */

zVec3D *rkIKAcmPos(rkChain *chain, rkIKAcm *acm, void *util, zVec3D *srv)
{
  return zVec3DAddDRC( srv, zVec3DCatDRC( &acm->ae.p, 1.0, srv ) );
}

zVec3D *rkIKAcmAtt(rkChain *chain, rkIKAcm *acm, void *util, zVec3D *srv)
{
  zEP e;

  zAA2EP( srv, &e );
  zEPCatDRC( &acm->ae.e, 1.0, &e );
  zEPCascade( &e, &acm->ae.e, &e );
  return zEP2AA( &e, srv );
}
