/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_ik_imp - inverse kinematics: impedance control
 */

#include <roki/rk_ik.h>

static zVec3D *_rkIKImpSRV(zVec3D *err, zVec3D *v, rkIKImp *imp, zVec3D *srv);

/* (static)
 * _rkIKImpSRV
 * - strict referential velocity for impedance control.
 */
zVec3D *_rkIKImpSRV(zVec3D *err, zVec3D *v, rkIKImp *imp, zVec3D *srv)
{
  zVec3D dv;

  zVec3DAmp( err, &imp->k, srv );
  zVec3DAmp( v, &imp->c, &dv );
  zVec3DSubDRC( srv, &dv );
  zVec3DAddDRC( srv, v );
  return srv;
}

/* rkIKImpWldAtt
 * - attitude control with respect to the world frame.
 */
zVec3D *rkIKImpWldAtt(rkChain *chain, rkIKCellAttr *attr, void *priv, rkIKRef *ref, zVec3D *srv)
{
  zVec3D v, err;

  /* attitude error */
  zMat3DError( &ref->att, rkChainLinkWldAtt(chain,attr->id), &err );
  /* rotation velocity */
  zMulMatVec3D( rkChainLinkWldAtt(chain,attr->id),
    rkChainLinkAngVel(chain,attr->id), &v );
  return _rkIKImpSRV( &err, &v, priv, srv );
}

/* rkIKImpWldPos
 * - position control with respect to the world frame.
 */
zVec3D *rkIKImpWldPos(rkChain *chain, rkIKCellAttr *attr, void *priv, rkIKRef *ref, zVec3D *srv)
{
  zVec3D v, err;

  /* position error */
  zXfer3D( rkChainLinkWldFrame(chain,attr->id), &attr->ap, &v );
  zVec3DSub( &ref->pos, &v, &err );
  /* velocity */
  rkChainLinkPointVel( chain, attr->id, &attr->ap, &v );
  zMulMatVec3DDRC( rkChainLinkWldAtt(chain,attr->id), &v );
  return _rkIKImpSRV( &err, &v, priv, srv );
}

/* rkIKImpWldCOM
 * - COM position control with respect to the world frame.
 */
zVec3D *rkIKImpWldCOM(rkChain *chain, rkIKCellAttr *attr, void *priv, rkIKRef *ref, zVec3D *srv)
{
  zVec3D err;

  /* position error */
  zVec3DSub( &ref->pos, rkChainWldCOM(chain), &err );
  /* velocity */
  return _rkIKImpSRV( &err, rkChainCOMVel(chain), priv, srv );
}
