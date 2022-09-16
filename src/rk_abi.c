/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_abi - Articulated Body Inertia Method.
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_abi.h>

/* initialize invariant mass properties. */
/* RENAME: _rkLinkABIInitInertia -> _rkLinkInitABIInertia */
static void _rkLinkABIInitInertia(rkLink *link)
{
  rkABIPrp *ap;
  zVec3D pc;
  zMat3D mpcross2;

  ap = rkLinkABIPrp(link);
  zMat3DCreate( &ap->m.e[0][0], rkLinkMass(link), 0, 0, 0, rkLinkMass(link), 0, 0, 0, rkLinkMass(link) );
  zVec3DMul( rkLinkCOM(link), rkLinkMass(link), &pc );
  zVec3DOuterProd2Mat3D( &pc, &ap->m.e[0][1] );
  zMat3DRev( &ap->m.e[0][1], &ap->m.e[1][0] );
  zVec3DTripleProd2Mat3D( &pc, rkLinkCOM(link), &mpcross2 );
  zMat3DSub( rkLinkInertia(link), &mpcross2, &ap->m.e[1][1] );
}

/* allocate memory for ABI of a link. */
/* RENAME: rkLinkABIAlloc -> rkLinkAllocABI */
/* TODO: _abiprp -> *_abiprp */
rkLink *rkLinkABIAlloc(rkLink *link)
{
  rkABIPrp *ap;
  bool result = true;

  ap = rkLinkABIPrp(link);
  memset( (void *)ap, 0, sizeof(rkABIPrp) );
  if( rkLinkJointSize(link) == 0 ){
    ap->axi = ap->iaxi = NULL;
  } else{
    if( !( ap->axi  = zMatAllocSqr( rkLinkJointSize(link) ) ) ||
        !( ap->iaxi = zMatAllocSqr( rkLinkJointSize(link) ) ) ) result = false;
  }
  zListInit( &ap->wlist );
  _rkLinkABIInitInertia( link );
  if( !result ){
    rkLinkABIDestroy( link );
    return NULL;
  }
  return link;
}

/* allocate memory for ABI of a kinematic chain. */
/* RENAME: rkChainABIAlloc -> rkChainAllocABI */
rkChain *rkChainABIAlloc(rkChain *chain)
{
  uint i;
  bool result = true;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( !rkLinkABIAlloc( rkChainLink(chain,i) ) ) result = false;
  if( !result ){
    rkChainABIDestroy( chain );
    return NULL;
  }
  return chain;
}

/* destroy ABI of a link. */
/* RENAME: rkLinkABIDestroy -> rkLinkDestroyABI */
/* TODO: free _abiprp */
void rkLinkABIDestroy(rkLink *link)
{
  zMatFree( rkLinkABIPrp(link)->axi );
  zMatFree( rkLinkABIPrp(link)->iaxi );
  rkWrenchListDestroy( &rkLinkABIPrp(link)->wlist );
}

/* destroy ABI of a kinematic chain. */
/* RENAME: rkChainABIDestroy -> rkChainDestroyABI */
void rkChainABIDestroy(rkChain *chain)
{
  uint i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkLinkABIDestroy( rkChainLink(chain,i) );
}

/* initialize ABI of a link for recursive computation. */
/* RENAME: rkLinkABIUpdateInit -> rkLinkInitABI */
void rkLinkABIUpdateInit(rkLink *link, zVec6D *pvel)
{
  rkABIPrp *ap;
  zVec3D tmp;

  ap = rkLinkABIPrp(link);
  /* I */
  zMat6DCopy( &ap->m, &ap->i );
  /* b */
  zVec3DTripleProd( rkLinkAngVel(link), rkLinkAngVel(link), rkLinkCOM(link), &tmp);
  zVec3DMul( &tmp, rkLinkMass(link), zVec6DLin(&ap->f) );
  zMulMat3DVec3D( &ap->i.e[1][1], rkLinkAngVel(link), &tmp );
  zVec3DOuterProd( rkLinkAngVel(link), &tmp, zVec6DAng(&ap->f) );
  zVec6DCopy( &ap->f, &ap->b );
  /* total external forces */
  rkWrenchListNet( &ap->wlist, &ap->w ); /* temporary contact forces */
  rkLinkNetExtWrench( link, &ap->w0 ); /* external forces */
  zVec3DCreate( &tmp, 0, 0, -RK_G * rkLinkMass(link) ); /* gravity force */
  zMulMat3DTVec3DDRC( rkLinkWldAtt(link), &tmp );
  zVec3DAddDRC( zVec6DLin(&ap->w0), &tmp );
  zVec3DOuterProd( rkLinkCOM(link), &tmp, &tmp );
  zVec3DAddDRC( zVec6DAng(&ap->w0), &tmp );
  zVec6DAddDRC( &ap->w, &ap->w0 );
  /* c */
  zVec6DZero( &ap->c );
  zMulMat3DTVec3D( rkLinkAdjAtt(link), zVec6DAng(pvel), &tmp );
  rkJointIncAccOnVel( rkLinkJoint(link), &tmp, &ap->c );
  zVec3DTripleProd( zVec6DAng(pvel), zVec6DAng(pvel), rkLinkAdjPos(link), &tmp );
  zMulMat3DTVec3DDRC( rkLinkAdjAtt(link), &tmp );
  zVec3DAddDRC( zVec6DLin(&ap->c), &tmp );
}

/* initialize ABI of a kinematic chain for recursive computation. */
/* RENAME: rkChainABIUpdateInit -> rkChainInitABI */
void rkChainABIUpdateInit(rkChain *chain)
{
  uint i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkLinkABIUpdateInit( rkChainLink(chain,i), rkChainLinkParent(chain,i) ?
      rkLinkVel(rkChainLinkParent(chain,i)) : ZVEC6DZERO );
}

/* add bias acceleration term in backward computation to update ABI of a link. */
/* RENAME: _rkLinkABIAddBias -> _rkLinkUpdateABIBias */
static void _rkLinkABIAddBias(rkLink *link)
{
  zVec6D icb;

  /* IH (HIH)^-1 (u - H^T (Ic + b)) */
  zMulMat6DVec6D( &rkLinkABIPrp(link)->i, &rkLinkABIPrp(link)->c, &icb );
  zVec6DAddDRC( &icb, &rkLinkABIPrp(link)->b );
  rkJointABIAddBias( rkLinkJoint(link), &rkLinkABIPrp(link)->i, &icb, rkLinkAdjFrame(link), rkLinkABIPrp(link)->iaxi, &rkLinkABIPrp(rkLinkParent(link))->b );
}

/* update ABI of a link in backward computation. */
/* RENAME: _rkLinkABIUpdateBackward -> _rkLinkUpdateABIBackward */
static void _rkLinkABIUpdateBackward(rkLink *link)
{
  rkABIPrp *ap;

  ap = rkLinkABIPrp(link);
  /* b */
  zVec6DSubDRC( &ap->b, &ap->w );
  /* IsIs */
  rkJointABIAxisInertia( rkLinkJoint(link), &ap->i, ap->axi, ap->iaxi );
  rkJointABIDrivingTorque( rkLinkJoint(link) );
  if( !rkLinkParent(link) ) return;

  /* add ABI and bias acceleration to parent prp */
  rkJointABIAddABI( rkLinkJoint(link), &ap->i, rkLinkAdjFrame( link ), ap->iaxi, &rkLinkABIPrp(rkLinkParent(link))->i );
  _rkLinkABIAddBias( link );
}

/* backward computation to update ABI of a link. */
/* RENAME: rkLinkABIUpdateBackward -> rkLinkUpdateABIBackward */
void rkLinkABIUpdateBackward(rkLink *link)
{
  /* recursive update of ABI */
  if( rkLinkSibl(link) )
    rkLinkABIUpdateBackward( rkLinkSibl(link) );
  if( rkLinkChild(link) )
    rkLinkABIUpdateBackward( rkLinkChild(link) );

  _rkLinkABIUpdateBackward( link );
}

/* update acceleration from ABI of a link in forward computation. */
/* RENAME: _rkLinkABIUpdateForward -> _rkLinkUpdateABIForward */
static void _rkLinkABIUpdateForward(rkLink *link, zVec6D *pa)
{
  rkABIPrp *ap;
  zVec6D jac;

  ap = rkLinkABIPrp(link);
  /*J^Ta+c*/
  zVec6DLinShift( pa, rkLinkAdjPos(link), &jac );
  zMulMat3DTVec6DDRC( rkLinkAdjAtt(link), &jac );
  zVec6DAddDRC( &jac, &ap->c );
  /* update acceleration */
  rkJointABIQAcc( rkLinkJoint(link), &ap->i, &ap->b, &jac, ap->iaxi, rkLinkAcc(link) );
}

/* forward computation to update acceleration from ABI of a link. */
/* RENAME: rkLinkABIUpdateForward -> rkLinkUpdateABIForward */
void rkLinkABIUpdateForward(rkLink *link, zVec6D *pa)
{
  _rkLinkABIUpdateForward( link, pa );

  /* forward recursive computation of ABI */
  if( rkLinkSibl(link) )
    rkLinkABIUpdateForward( rkLinkSibl(link), pa );
  if( rkLinkChild(link) )
    rkLinkABIUpdateForward( rkLinkChild(link), rkLinkAcc(link) );
}

/* forward computation to update acceleration and wrench from ABI of a link. */
/* RENAME: rkLinkABIUpdateForwardGetWrench -> rkLinkUpdateABIWrenchForward */
void rkLinkABIUpdateForwardGetWrench(rkLink *link, zVec6D *pa)
{
  _rkLinkABIUpdateForward( link, pa );
  /* link wrench */
  rkJointUpdateWrench( rkLinkJoint(link), &rkLinkABIPrp(link)->i, &rkLinkABIPrp(link)->b, rkLinkAcc(link) );

  /* forward recursive computation of ABI */
  if( rkLinkSibl(link) )
    rkLinkABIUpdateForwardGetWrench( rkLinkSibl(link), pa );
  if( rkLinkChild(link) )
    rkLinkABIUpdateForwardGetWrench( rkLinkChild(link), rkLinkAcc(link) );
}

/* zero velocity and acceleration of links of a kinematic chain. */
static void _rkChainZeroLinkRate(rkChain *chain)
{
  uint i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkLinkZeroRate( rkChainLink(chain,i) );
}

/* update ABI and acceleration of a kinematic chain. */
/* RENAME: rkChainABIUpdate -> rkChainUpdateABI */
void rkChainABIUpdate(rkChain *chain)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainZeroLinkRate( chain );
    return;
  }
  rkChainABIUpdateInit( chain );
  rkChainABIUpdateBackward( chain );
  rkChainABIUpdateForward( chain );
}

/* update ABI, acceleration and wrench of a kinematic chain. */
/* RENAME: rkChainABIUpdateGetWrench -> rkChainUpdateABIWrench */
void rkChainABIUpdateGetWrench(rkChain *chain)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainZeroLinkRate( chain );
    return;
  }
  rkChainABIUpdateInit( chain );
  rkChainABIUpdateBackward( chain );
  rkChainABIUpdateForwardGetWrench( chain );
}

/* compute accleration of a kinematic chain based on ABI method. */
/* RENAME: rkChainABI -> rkChainFD_ABI */
/* QUESTION: joint motor input */
zVec rkChainABI(rkChain *chain, zVec dis, zVec vel, zVec acc)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainZeroLinkRate( chain );
    return NULL;
  }
  rkChainSetJointDisAll( chain, dis );
  rkChainSetJointVelAll( chain, vel );
  rkChainUpdateFK( chain );
  rkChainUpdateVel( chain );
  rkChainABIUpdateGetWrench( chain );
  rkChainGetJointAccAll( chain, acc );
  return acc;
}

/******************************************************************************/
/* for rigid contact force computation
 *
 * the ABI backward path follows only the links with rigid contact forces
 * and their all parent links when the ABI and ABbias with no rigid contact
 * forces have been computed.
 *
 * NOTE:
 * by the time of using this ABI version,
 * the regular ABI method must be run,
 * the ABI and ABbias must not be overwritten, and
 * the forces except rigid contact forces in the list 'wlist' must be removed.
 */
/* RENAME: _rkLinkABIFindBackwardPathAddExForce -> _rkLinkCheckABIExtWrench */
static void _rkLinkABIFindBackwardPathAddExForce(rkLink *link)
{
  if( rkLinkChild(link) )
    _rkLinkABIFindBackwardPathAddExForce( rkLinkChild(link) );
  if( rkLinkSibl(link) )
    _rkLinkABIFindBackwardPathAddExForce( rkLinkSibl(link) );

  if( !rkLinkParent(link) ) return;
  if( rkLinkABIPrp(link)->abi_backward_path || !zListIsEmpty(&rkLinkABIPrp(link)->wlist) )
    rkLinkABIPrp(rkLinkParent(link))->abi_backward_path = true;
}

/* RENAME: _rkLinkABISubBiasNetWrenchAddExForce -> _rkLinkSubABIExtWrench */
static void _rkLinkABISubBiasNetWrenchAddExForce(rkLink *link)
{
  zVec6D w;

  if( !zListIsEmpty(&rkLinkABIPrp(link)->wlist) ){
    rkWrenchListNet( &rkLinkABIPrp(link)->wlist, &w );
    zVec6DSubDRC( &rkLinkABIPrp(link)->b, &w );
  }
}

/* RENAME: _rkLinkABIUpdateBackwardAddExForce -> _rkLinkUpdateCachedABIBackward */
static void _rkLinkABIUpdateBackwardAddExForce(rkLink *link)
{
  if( rkLinkABIPrp(link)->abi_backward_path ){
    zVec6DSub( &rkLinkABIPrp(link)->f, &rkLinkABIPrp(link)->w, &rkLinkABIPrp(link)->b );
    _rkLinkABISubBiasNetWrenchAddExForce( link );
    if( rkLinkChild(link) )  _rkLinkABIUpdateBackwardAddExForce( rkLinkChild(link) );
    if( rkLinkParent(link) ) _rkLinkABIAddBias( link );
    if( rkLinkSibl(link) )   _rkLinkABIUpdateBackwardAddExForce( rkLinkSibl(link) );
  } else {
    _rkLinkABISubBiasNetWrenchAddExForce( link );
    if( rkLinkParent(link) ) _rkLinkABIAddBias( link );
    if( rkLinkSibl(link) )   _rkLinkABIUpdateBackwardAddExForce( rkLinkSibl(link) );
  }
}

/* RENAME: _rkChainABIUpdateBackwardAddExForce -> _rkChainUpdateCachedABIBackward */
static void _rkChainABIUpdateBackwardAddExForce(rkChain *chain)
{
  uint i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkLinkABIPrp(rkChainLink(chain,i))->abi_backward_path = false;
  _rkLinkABIFindBackwardPathAddExForce( rkChainRoot(chain) );
  _rkLinkABIUpdateBackwardAddExForce( rkChainRoot(chain) );
}

/* RENAME: rkChainABIUpdateAddExForce -> rkChainUpdateCachedABI */
void rkChainABIUpdateAddExForce(rkChain *chain)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainZeroLinkRate( chain );
    return;
  }
  _rkChainABIUpdateBackwardAddExForce( chain );
  rkChainABIUpdateForward( chain );
}

/* RENAME: rkChainABIUpdateAddExForceGetWrench -> rkChainUpdateCachedABIWrench */
void rkChainABIUpdateAddExForceGetWrench(rkChain *chain)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainZeroLinkRate( chain );
    return;
  }
  _rkChainABIUpdateBackwardAddExForce( chain );
  rkChainABIUpdateForwardGetWrench( chain );
}

/* in the case of only one or two additional external forces, to find the backward path is easier.
 * this case happens in the computation of the relation between acceleration and contact force
 *
 * NOTE:
 * before rkChainABIUpdateAddExForceTwo is called,
 * call rkChainABIPushPrpAccBias to store the ABIprp at no rigid contact forces.
 */
/* RENAME: rkChainABIPushPrpAccBias -> rkChainSaveABIAccBias */
void rkChainABIPushPrpAccBias(rkChain *chain)
{
  uint i;

  for( i=0; i<rkChainLinkNum(chain); i++ ){
    zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->b, &rkLinkABIPrp(rkChainLink(chain,i))->b0 );
    zVec6DCopy( rkChainLinkAcc(chain,i), &rkLinkABIPrp(rkChainLink(chain,i))->a0 );
  }
}

/* RENAME: rkChainABIPopPrpAccBias -> rkChainRestoreABIAccBias */
void rkChainABIPopPrpAccBias(rkChain *chain)
{
  uint i;

  for( i=0; i<rkChainLinkNum(chain); i++ ){
    zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->b0, &rkLinkABIPrp(rkChainLink(chain,i))->b );
    zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->a0, rkLinkAcc(rkChainLink(chain,i)) );
  }
}

/* RENAME: rkChainABIPopPrpAccBiasAddExForceTwo -> rkChainRestoreABIAccBiasPair */
void rkChainABIPopPrpAccBiasAddExForceTwo(rkChain *chain, rkLink *link, rkLink *link2)
{
  uint i;

  if( link )  zVec6DCopy( &rkLinkABIPrp(link)->b0, &rkLinkABIPrp(link)->b );
  if( link2 ) zVec6DCopy( &rkLinkABIPrp(link2)->b0, &rkLinkABIPrp(link2)->b );
  for( i=0; i<rkChainLinkNum(chain); i++ ){
    zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->a0, rkLinkAcc(rkChainLink(chain,i)) );
    if( rkLinkABIPrp(rkChainLink(chain,i))->abi_backward_path )
      zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->b0, &rkLinkABIPrp(rkChainLink(chain,i))->b );
  }
}

/* RENAME: _rkLinkABIFindBackwardPathAddExForceOne -> _rkLinkCheckABIBackwardPathSole */
static void _rkLinkABIFindBackwardPathAddExForceOne(rkChain *chain, rkLink *link)
{
  rkLink *l;

  for( l=rkLinkParent(link); l; l=rkLinkParent(l) )
    rkLinkABIPrp(l)->abi_backward_path = true;
}

/* RENAME: _rkChainABIUpdateBackwardAddExForceTwo -> _rkChainCheckBackwardPathPair */
static void _rkChainABIUpdateBackwardAddExForceTwo(rkChain *chain, rkLink *link, rkWrench *w, rkLink *link2, rkWrench *w2)
{
  uint i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkLinkABIPrp(rkChainLink(chain,i))->abi_backward_path = false;
  if( link && w ){
    _rkLinkABIFindBackwardPathAddExForceOne( chain, link );
    rkWrenchListPush( &rkLinkABIPrp(link)->wlist, w );
  }
  if( link2 && w2 ){
    _rkLinkABIFindBackwardPathAddExForceOne( chain, link2 );
    rkWrenchListPush( &rkLinkABIPrp(link2)->wlist, w2 );
  }
  _rkLinkABIUpdateBackwardAddExForce( rkChainRoot(chain) );
  /* clean-up wrench list */
  if( link && w )   zListPurge( &rkLinkABIPrp(link)->wlist, w );
  if( link2 && w2 ) zListPurge( &rkLinkABIPrp(link2)->wlist, w2 );
}

/* RENAME: rkChainABIUpdateAddExForceTwo -> _rkChainUpdateCachedABIPair */
void rkChainABIUpdateAddExForceTwo(rkChain *chain, rkLink *link, rkWrench *w, rkLink *link2, rkWrench *w2)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainZeroLinkRate( chain );
    return;
  }
  _rkChainABIUpdateBackwardAddExForceTwo( chain, link, w, link2, w2 );
  rkChainABIUpdateForward( chain );
}
