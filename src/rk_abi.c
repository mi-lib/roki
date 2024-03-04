/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_abi - Articulated Body Inertia Method.
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_abi.h>

/* initialize invariant mass properties. */
static void _rkLinkInitABIInertia(rkLink *link)
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
/* TODO: _abiprp -> *_abiprp */
rkLink *rkLinkAllocABI(rkLink *link)
{
  rkABIPrp *ap;
  bool result = true;

  ap = rkLinkABIPrp(link);
  memset( (void *)ap, 0, sizeof(rkABIPrp) );
  if( rkLinkJointDOF(link) == 0 ){
    ap->axi = ap->iaxi = NULL;
  } else{
    if( !( ap->axi  = zMatAllocSqr( rkLinkJointDOF(link) ) ) ||
        !( ap->iaxi = zMatAllocSqr( rkLinkJointDOF(link) ) ) ) result = false;
  }
  zListInit( &ap->wlist );
  _rkLinkInitABIInertia( link );
  if( !result ){
    rkLinkDestroyABI( link );
    return NULL;
  }
  return link;
}

/* allocate memory for ABI of a kinematic chain. */
rkChain *rkChainAllocABI(rkChain *chain)
{
  int i;
  bool result = true;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( !rkLinkAllocABI( rkChainLink(chain,i) ) ) result = false;
  if( !result ){
    rkChainDestroyABI( chain );
    return NULL;
  }
  return chain;
}

/* destroy ABI of a link. */
/* TODO: free _abiprp */
void rkLinkDestroyABI(rkLink *link)
{
  zMatFree( rkLinkABIPrp(link)->axi );
  zMatFree( rkLinkABIPrp(link)->iaxi );
  rkWrenchListDestroy( &rkLinkABIPrp(link)->wlist );
}

/* destroy ABI of a kinematic chain. */
void rkChainDestroyABI(rkChain *chain)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkLinkDestroyABI( rkChainLink(chain,i) );
}

/* initialize ABI of a link for recursive computation. */
void rkLinkInitABI(rkLink *link, zVec6D *pvel)
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

/* add bias acceleration term in backward computation to update ABI of a link. */
static void _rkLinkUpdateABIBias(rkLink *link)
{
  zVec6D icb;

  /* IH (HIH)^-1 (u - H^T (Ic + b)) */
  zMulMat6DVec6D( &rkLinkABIPrp(link)->i, &rkLinkABIPrp(link)->c, &icb );
  zVec6DAddDRC( &icb, &rkLinkABIPrp(link)->b );
  rkJointABIAddBias( rkLinkJoint(link), &rkLinkABIPrp(link)->i, &icb, rkLinkAdjFrame(link), rkLinkABIPrp(link)->iaxi, &rkLinkABIPrp(rkLinkParent(link))->b );
}

/* update ABI of a link in backward computation. */
static void _rkLinkUpdateABIBackward(rkLink *link)
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
  _rkLinkUpdateABIBias( link );
}

/* backward computation to update ABI of a link. */
void rkLinkUpdateABIBackward(rkLink *link)
{
  /* recursive update of ABI */
  if( rkLinkSibl(link) )
    rkLinkUpdateABIBackward( rkLinkSibl(link) );
  if( rkLinkChild(link) )
    rkLinkUpdateABIBackward( rkLinkChild(link) );

  _rkLinkUpdateABIBackward( link );
}

/* update acceleration from ABI of a link in forward computation. */
static void _rkLinkUpdateABIForward(rkLink *link, zVec6D *pa)
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
void rkLinkUpdateABIForward(rkLink *link, zVec6D *pa)
{
  _rkLinkUpdateABIForward( link, pa );

  /* forward recursive computation of ABI */
  if( rkLinkSibl(link) )
    rkLinkUpdateABIForward( rkLinkSibl(link), pa );
  if( rkLinkChild(link) )
    rkLinkUpdateABIForward( rkLinkChild(link), rkLinkAcc(link) );
}

/* forward computation to update acceleration and wrench from ABI of a link. */
void rkLinkUpdateABIWrenchForward(rkLink *link, zVec6D *pa)
{
  _rkLinkUpdateABIForward( link, pa );
  /* link wrench */
  rkJointUpdateWrench( rkLinkJoint(link), &rkLinkABIPrp(link)->i, &rkLinkABIPrp(link)->b, rkLinkAcc(link) );

  /* forward recursive computation of ABI */
  if( rkLinkSibl(link) )
    rkLinkUpdateABIWrenchForward( rkLinkSibl(link), pa );
  if( rkLinkChild(link) )
    rkLinkUpdateABIWrenchForward( rkLinkChild(link), rkLinkAcc(link) );
}

/* initialize ABI of a kinematic chain for recursive computation. */
void rkChainInitABI(rkChain *chain)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkLinkInitABI( rkChainLink(chain,i), rkChainLinkParent(chain,i) ?
      rkLinkVel(rkChainLinkParent(chain,i)) : ZVEC6DZERO );
}

/* zero velocity and acceleration of links of a kinematic chain. */
static void _rkChainZeroLinkRate(rkChain *chain)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkLinkZeroRate( rkChainLink(chain,i) );
}

/* update ABI and acceleration of a kinematic chain. */
void rkChainUpdateABI(rkChain *chain)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainZeroLinkRate( chain );
    return;
  }
  rkChainInitABI( chain );
  rkChainUpdateABIBackward( chain );
  rkChainUpdateABIForward( chain );
}

/* update ABI, acceleration and wrench of a kinematic chain. */
void rkChainUpdateABIWrench(rkChain *chain)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainZeroLinkRate( chain );
    return;
  }
  rkChainInitABI( chain );
  rkChainUpdateABIBackward( chain );
  rkChainUpdateABIWrenchForward( chain );
}

/* compute accleration of a kinematic chain based on ABI method. */
/* QUESTION: joint motor input */
zVec rkChainFD_ABI(rkChain *chain, zVec dis, zVec vel, zVec acc)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainZeroLinkRate( chain );
    return NULL;
  }
  rkChainSetJointDisAll( chain, dis );
  rkChainSetJointVelAll( chain, vel );
  rkChainUpdateFK( chain );
  rkChainUpdateVel( chain );
  rkChainUpdateABIWrench( chain );
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
static void _rkLinkCheckABIExtWrench(rkLink *link)
{
  if( rkLinkChild(link) )
    _rkLinkCheckABIExtWrench( rkLinkChild(link) );
  if( rkLinkSibl(link) )
    _rkLinkCheckABIExtWrench( rkLinkSibl(link) );

  if( !rkLinkParent(link) ) return;
  if( rkLinkABIPrp(link)->abi_backward_path || !zListIsEmpty(&rkLinkABIPrp(link)->wlist) )
    rkLinkABIPrp(rkLinkParent(link))->abi_backward_path = true;
}

static void _rkLinkSubABIExtWrench(rkLink *link)
{
  zVec6D w;

  if( !zListIsEmpty(&rkLinkABIPrp(link)->wlist) ){
    rkWrenchListNet( &rkLinkABIPrp(link)->wlist, &w );
    zVec6DSubDRC( &rkLinkABIPrp(link)->b, &w );
  }
}

static void _rkLinkUpdateCachedABIBackward(rkLink *link)
{
  if( rkLinkABIPrp(link)->abi_backward_path ){
    zVec6DSub( &rkLinkABIPrp(link)->f, &rkLinkABIPrp(link)->w, &rkLinkABIPrp(link)->b );
    _rkLinkSubABIExtWrench( link );
    if( rkLinkChild(link) )  _rkLinkUpdateCachedABIBackward( rkLinkChild(link) );
    if( rkLinkParent(link) ) _rkLinkUpdateABIBias( link );
    if( rkLinkSibl(link) )   _rkLinkUpdateCachedABIBackward( rkLinkSibl(link) );
  } else{
    _rkLinkSubABIExtWrench( link );
    if( rkLinkParent(link) ) _rkLinkUpdateABIBias( link );
    if( rkLinkSibl(link) )   _rkLinkUpdateCachedABIBackward( rkLinkSibl(link) );
  }
}

static void _rkChainUpdateCachedABIBackward(rkChain *chain)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkLinkABIPrp(rkChainLink(chain,i))->abi_backward_path = false;
  _rkLinkCheckABIExtWrench( rkChainRoot(chain) );
  _rkLinkUpdateCachedABIBackward( rkChainRoot(chain) );
}

void rkChainUpdateCachedABI(rkChain *chain)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainZeroLinkRate( chain );
    return;
  }
  _rkChainUpdateCachedABIBackward( chain );
  rkChainUpdateABIForward( chain );
}

void rkChainUpdateCachedABIWrench(rkChain *chain)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainZeroLinkRate( chain );
    return;
  }
  _rkChainUpdateCachedABIBackward( chain );
  rkChainUpdateABIWrenchForward( chain );
}

/* in the case of only one or two additional external forces, to find the backward path is easier.
 * this case happens in the computation of the relation between acceleration and contact force
 *
 * NOTE:
 * before rkChainABIUpdateAddExForceTwo is called,
 * call rkChainABIPushPrpAccBias to store the ABIprp at no rigid contact forces.
 */
void rkChainSaveABIAccBias(rkChain *chain)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ ){
    zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->b, &rkLinkABIPrp(rkChainLink(chain,i))->b0 );
    zVec6DCopy( rkChainLinkAcc(chain,i), &rkLinkABIPrp(rkChainLink(chain,i))->a0 );
  }
}

void rkChainRestoreABIAccBias(rkChain *chain)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ ){
    zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->b0, &rkLinkABIPrp(rkChainLink(chain,i))->b );
    zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->a0, rkLinkAcc(rkChainLink(chain,i)) );
  }
}

void rkChainRestoreABIAccBiasPair(rkChain *chain, rkLink *link, rkLink *link2)
{
  int i;

  if( link )  zVec6DCopy( &rkLinkABIPrp(link)->b0, &rkLinkABIPrp(link)->b );
  if( link2 ) zVec6DCopy( &rkLinkABIPrp(link2)->b0, &rkLinkABIPrp(link2)->b );
  for( i=0; i<rkChainLinkNum(chain); i++ ){
    zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->a0, rkLinkAcc(rkChainLink(chain,i)) );
    if( rkLinkABIPrp(rkChainLink(chain,i))->abi_backward_path )
      zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->b0, &rkLinkABIPrp(rkChainLink(chain,i))->b );
  }
}

static void _rkLinkCheckABIBackwardPathSole(rkChain *chain, rkLink *link)
{
  rkLink *l;

  for( l=rkLinkParent(link); l; l=rkLinkParent(l) )
    rkLinkABIPrp(l)->abi_backward_path = true;
}

static void _rkChainCheckBackwardPathPair(rkChain *chain, rkLink *link, rkWrench *w, rkLink *link2, rkWrench *w2)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkLinkABIPrp(rkChainLink(chain,i))->abi_backward_path = false;
  if( link && w ){
    _rkLinkCheckABIBackwardPathSole( chain, link );
    rkWrenchListPush( &rkLinkABIPrp(link)->wlist, w );
  }
  if( link2 && w2 ){
    _rkLinkCheckABIBackwardPathSole( chain, link2 );
    rkWrenchListPush( &rkLinkABIPrp(link2)->wlist, w2 );
  }
  _rkLinkUpdateCachedABIBackward( rkChainRoot(chain) );
  /* clean-up wrench list */
  if( link && w )   zListPurge( &rkLinkABIPrp(link)->wlist, w );
  if( link2 && w2 ) zListPurge( &rkLinkABIPrp(link2)->wlist, w2 );
}

void rkChainUpdateCachedABIPair(rkChain *chain, rkLink *link, rkWrench *w, rkLink *link2, rkWrench *w2)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainZeroLinkRate( chain );
    return;
  }
  _rkChainCheckBackwardPathPair( chain, link, w, link2, w2 );
  rkChainUpdateABIForward( chain );
}
