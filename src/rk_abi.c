/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_abi - Articulated Body Inertia Method.
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_abi.h>

/* (static)
 * _rkLinkABIInitInertia
 * - initialize invariant mass properties.
 */
void _rkLinkABIInitInertia(rkLink *link)
{
  rkABIPrp *ap;
  zVec3D pc;
  zMat3D mpcross2;

  ap = rkLinkABIPrp(link);
  zMat3DCreate( &ap->m.e[0][0], rkLinkMass(link), 0, 0, 0, rkLinkMass(link), 0, 0, 0, rkLinkMass(link) );
  zVec3DMul( rkLinkCOM(link), rkLinkMass(link), &pc );
  zVec3DOuterProd2Mat3D( &pc, &ap->m.e[1][0] );
  zMat3DRev( &ap->m.e[1][0], &ap->m.e[0][1] );
  zVec3DTripleProd2Mat3D( &pc, rkLinkCOM(link), &mpcross2 );
  zMat3DSub( rkLinkInertia(link), &mpcross2, &ap->m.e[1][1] );
}

void rkLinkABIInit(rkLink *link)
{
  rkABIPrp *ap;

  ap = rkLinkABIPrp(link);
  memset( ap, 0, sizeof(rkABIPrp) );
  if( rkLinkJointSize(link) == 0 ){
    ap->axi = ap->iaxi = NULL;
  } else{
    ap->axi  = zMatAllocSqr( rkLinkJointSize(link) );
    ap->iaxi = zMatAllocSqr( rkLinkJointSize(link) );
  }
  zListInit( &ap->wlist );
  _rkLinkABIInitInertia( link );
}

void rkChainABIInit(rkChain *chain)
{
  register int i;

  for( i=0; i<rkChainNum(chain); i++ )
    rkLinkABIInit( rkChainLink(chain,i) );
}

void rkLinkABIDestroy(rkLink *link)
{
  zMatFreeAO( 2, rkLinkABIPrp(link)->axi, rkLinkABIPrp(link)->iaxi );
  rkWrenchListDestroy( &rkLinkABIPrp(link)->wlist );
}

void rkChainABIDestroy(rkChain *chain)
{
  register int i;

  for( i=0; i<rkChainNum(chain); i++ )
    rkLinkABIDestroy( rkChainLink(chain,i) );
}

/******************************************************************************/
void rkLinkABIUpdateInit(rkLink *link, zVec6D *pvel)
{
  rkABIPrp *ap;
  zVec3D tempv;

  ap = rkLinkABIPrp(link);
  /*I*/
  zMat6DCopy( &ap->m, &ap->i );

  /*b*/
  zVec3DTripleProd( rkLinkAngVel(link), rkLinkAngVel(link), rkLinkCOM(link), &tempv);
  zVec3DMul( &tempv, rkLinkMass(link), zVec6DLin(&ap->f) );
  zMulMatVec3D( zMat6DMat3D(&ap->i,1,1), rkLinkAngVel(link), &tempv );
  zVec3DOuterProd( rkLinkAngVel(link), &tempv, zVec6DAng(&ap->f) );
  zVec6DCopy( &ap->f, &ap->b );

  /* total external forces */
  rkWrenchListNet( &ap->wlist, &ap->w ); /* temporary contact forces */
  rkLinkCalcExtWrench( link, &ap->w0 );       /* external forces */
  zVec3DCreate( &tempv,                  /* gravity force */
                0, 0, -RK_G * rkLinkMass(link) );
  zMulMatTVec3DDRC( rkLinkWldAtt(link), &tempv );
  zVec3DAddDRC( zVec6DLin(&ap->w0), &tempv );
  zVec3DOuterProd( rkLinkCOM(link), &tempv, &tempv );
  zVec3DAddDRC( zVec6DAng(&ap->w0), &tempv );
  zVec6DAddDRC( &ap->w, &ap->w0 );

  /*c*/
  zVec6DClear( &ap->c );
  zMulMatTVec3D( rkLinkAdjAtt(link), zVec6DAng(pvel), &tempv );
  rkJointIncAccOnVel( rkLinkJoint(link), &tempv, &ap->c );
  zVec3DTripleProd( zVec6DAng(pvel), zVec6DAng(pvel), rkLinkAdjPos(link), &tempv );
  zMulMatTVec3DDRC( rkLinkAdjAtt(link), &tempv );
  zVec3DAddDRC( zVec6DLin(&ap->c), &tempv );
}

void rkChainABIUpdateInit(rkChain *chain)
{
  register int i;

  for( i=0; i<rkChainNum(chain); i++ ){
    if( rkChainLinkParent(chain,i) == NULL )
      rkLinkABIUpdateInit( rkChainLink(chain,i), ZVEC6DZERO );
    else
      rkLinkABIUpdateInit( rkChainLink(chain,i), rkLinkVel(rkChainLinkParent(chain,i)) );
  }
}

void _rkLinkABIAddBias(rkLink *link)
{
  zVec6D icb;
  /* IH (HIH)-1 (u - H^T (Ic + b)) */
  zMulMat6DVec6D( &rkLinkABIPrp(link)->i, &rkLinkABIPrp(link)->c, &icb );
  zVec6DAddDRC( &icb, &rkLinkABIPrp(link)->b );
  rkJointABIAddBias( rkLinkJoint(link), &rkLinkABIPrp(link)->i, &icb, rkLinkAdjFrame(link), rkLinkABIPrp(link)->iaxi, &rkLinkABIPrp(rkLinkParent(link))->b );
}

void _rkLinkABIUpdateBackward(rkLink *link)
{
  rkABIPrp *ap;

  ap = rkLinkABIPrp(link);
  /* b */
  zVec6DSubDRC( &ap->b, &ap->w );
  /* IsIs */
  rkJointABIAxisInertia( rkLinkJoint(link), &ap->i, ap->axi, ap->iaxi );

  if( !rkLinkParent(link) ) return;

  /* add ABI and bias acceleration to parent prp */
  rkJointABIAddAbi( rkLinkJoint(link), &ap->i, rkLinkAdjFrame( link ), ap->iaxi, &rkLinkABIPrp(rkLinkParent(link))->i );
  rkJointABIDrivingTorque( rkLinkJoint(link) );
  _rkLinkABIAddBias( link );
}

void rkLinkABIUpdateBackward(rkLink *link)
{
  /* recursive update of ABI */
  if( rkLinkSibl(link) )
    rkLinkABIUpdateBackward( rkLinkSibl(link) );
  if( rkLinkChild(link) )
    rkLinkABIUpdateBackward( rkLinkChild(link) );

  _rkLinkABIUpdateBackward( link );
}

void _rkLinkABIUpdateForward(rkLink *link, zVec6D *pa)
{
  rkABIPrp *ap;
  zVec6D jac;
  zMat3D att;

  ap = rkLinkABIPrp(link);
  /*J^Ta+c*/
  zVec3DOuterProd( zVec6DAng( pa ), rkLinkAdjPos( link ), zVec6DLin( &jac ) );
  zVec3DAddDRC( zVec6DLin( &jac ), zVec6DLin( pa ) );
  zVec3DCopy( zVec6DAng( pa ), zVec6DAng( &jac ) );
  zMulMatTVec6DDRC( rkLinkAdjAtt( link ), &jac );

  zVec6DAddDRC( &jac, &ap->c );

  /* q, acc update */
  if( rkLinkJointType(link) >= RK_JOINT_SPHER ){
    zMulMatTMat3D(rkLinkOrgAtt(link), rkLinkAdjAtt(link), &att);
    rkJointABIQAcc( rkLinkJoint(link), &att, &ap->i, &ap->b, &jac, ap->iaxi, rkLinkAcc(link) );
  } else
    rkJointABIQAcc( rkLinkJoint(link), NULL, &ap->i, &ap->b, &jac, ap->iaxi, rkLinkAcc(link) );
}

void rkLinkABIUpdateForward(rkLink *link, zVec6D *pa)
{
  _rkLinkABIUpdateForward( link, pa );

  /* recursive forward computation of ABI */
  if( rkLinkSibl(link) )
    rkLinkABIUpdateForward( rkLinkSibl(link), pa );
  if( rkLinkChild(link) )
    rkLinkABIUpdateForward( rkLinkChild(link), rkLinkAcc(link) );
}

void rkLinkABIUpdateForwardGetWrench(rkLink *link, zVec6D *pa)
{
  _rkLinkABIUpdateForward( link, pa );
  /* link wrench */
  rkJointUpdateWrench( rkLinkJoint(link), &rkLinkABIPrp(link)->i, &rkLinkABIPrp(link)->b, rkLinkAcc(link) );

  /* recursive forward computation of ABI */
  if( rkLinkSibl(link) )
    rkLinkABIUpdateForwardGetWrench( rkLinkSibl(link), pa );
  if( rkLinkChild(link) )
    rkLinkABIUpdateForwardGetWrench( rkLinkChild(link), rkLinkAcc(link) );
}

/******************************************************************************/
void _rkLinkVelAccClear(rkLink *link)
{
  zVec6DClear( rkLinkVel(link) );
  zVec6DClear( rkLinkAcc(link) );
}

void _rkChainLinkVelAccClear(rkChain *chain)
{
  register int i;

  for( i=0; i<rkChainNum(chain); i++ )
    _rkLinkVelAccClear( rkChainLink(chain,i) );
}

void rkChainABIUpdate(rkChain *chain)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainLinkVelAccClear( chain );
    return;
  }
  rkChainABIUpdateInit( chain );
  rkChainABIUpdateBackward( chain );
  rkChainABIUpdateForward( chain );
}

void rkChainABIUpdateGetWrench(rkChain *chain)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainLinkVelAccClear( chain );
    return;
  }
  rkChainABIUpdateInit( chain );
  rkChainABIUpdateBackward( chain );
  rkChainABIUpdateForwardGetWrench( chain );
}

zVec rkChainABI(rkChain *chain, zVec dis, zVec vel, zVec acc)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainLinkVelAccClear( chain );
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
/* for rigid contact force computation */
/* *****
 * the ABI backward path follows only the links with rigid contact forces and their all parent links
 * when the ABI and ABbias with no rigid contact forces have been computed.
 *
 * NOTE:
 * by the time of using this ABI version,
 * the regular ABI method must be run,
 * the ABI and ABbios must not be overwritten, and
 * the forces except rigid contact forces in the list 'wlist' must be removed.
 */
void _rkLinkABIFindBackwardPathAddExForce(rkLink *link)
{
  if( rkLinkChild(link) )
    _rkLinkABIFindBackwardPathAddExForce( rkLinkChild(link) );
  if( rkLinkSibl(link) )
    _rkLinkABIFindBackwardPathAddExForce( rkLinkSibl(link) );

  if( !rkLinkParent(link) ) return;
  if( rkLinkABIPrp(link)->abi_backward_path || !zListIsEmpty(&rkLinkABIPrp(link)->wlist) )
    rkLinkABIPrp(rkLinkParent(link))->abi_backward_path = true;
}

void _rkLinkABISubBiasNetWrenchAddExForce(rkLink *link)
{
  zVec6D w;

  if( !zListIsEmpty(&rkLinkABIPrp(link)->wlist) ){
    rkWrenchListNet( &rkLinkABIPrp(link)->wlist, &w );
    zVec6DSubDRC( &rkLinkABIPrp(link)->b, &w );
  }
}

void _rkLinkABIUpdateBackwardAddExForce(rkLink *link)
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

void _rkChainABIUpdateBackwardAddExForce(rkChain *chain)
{
  register int i;

  for( i=0; i<rkChainNum(chain); i++ )
    rkLinkABIPrp(rkChainLink(chain,i))->abi_backward_path = false;
  _rkLinkABIFindBackwardPathAddExForce( rkChainRoot(chain) );
  _rkLinkABIUpdateBackwardAddExForce( rkChainRoot(chain) );
}

void rkChainABIUpdateAddExForce(rkChain *chain)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainLinkVelAccClear( chain );
    return;
  }
  _rkChainABIUpdateBackwardAddExForce( chain );
  rkChainABIUpdateForward( chain );
}

void rkChainABIUpdateAddExForceGetWrench(rkChain *chain)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainLinkVelAccClear( chain );
    return;
  }
  _rkChainABIUpdateBackwardAddExForce( chain );
  rkChainABIUpdateForwardGetWrench( chain );
}

/* *****
 * in the case of only one or two additional external forces, to find the backward path is easier.
 * this case happens in the computation of the relation between acceleration and contact force
 *
 * NOTE:
 * before rkChainABIUpdateAddExForceTwo is called,
 * call rkChainABIPushPrpAccBias to store the ABIprp at no rigid contact forces.
 */
void rkChainABIPushPrpAccBias(rkChain *chain)
{
  register int i;

  for( i=0; i<rkChainNum(chain); i++ ){
    zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->b, &rkLinkABIPrp(rkChainLink(chain,i))->b0 );
    zVec6DCopy( rkChainLinkAcc(chain,i), &rkLinkABIPrp(rkChainLink(chain,i))->a0 );
  }
}

void rkChainABIPopPrpAccBias(rkChain *chain)
{
  register int i;

  for( i=0; i<rkChainNum(chain); i++ ){
    zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->b0, &rkLinkABIPrp(rkChainLink(chain,i))->b );
    zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->a0, rkLinkAcc(rkChainLink(chain,i)) );
  }
}

void rkChainABIPopPrpAccBiasAddExForceTwo(rkChain *chain, rkLink *link, rkLink *link2)
{
  register int i;

  if( link )  zVec6DCopy( &rkLinkABIPrp(link)->b0, &rkLinkABIPrp(link)->b );
  if( link2 ) zVec6DCopy( &rkLinkABIPrp(link2)->b0, &rkLinkABIPrp(link2)->b );
  for( i=0; i<rkChainNum(chain); i++ ){
    zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->a0, rkLinkAcc(rkChainLink(chain,i)) );
    if( rkLinkABIPrp(rkChainLink(chain,i))->abi_backward_path )
      zVec6DCopy( &rkLinkABIPrp(rkChainLink(chain,i))->b0, &rkLinkABIPrp(rkChainLink(chain,i))->b );
  }
}

void _rkLinkABIFindBackwardPathAddExForceOne(rkChain *chain, rkLink *link)
{
  rkLink *l;

  for( l=rkLinkParent(link); l; l=rkLinkParent(l) )
    rkLinkABIPrp(l)->abi_backward_path = true;
}

void _rkChainABIUpdateBackwardAddExForceTwo(rkChain *chain, rkLink *link, rkWrench *w, rkLink *link2, rkWrench *w2)
{
  register int i;

  for( i=0; i<rkChainNum(chain); i++ )
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

  /* clear wlist */
  if( link && w )   zListPurge( &rkLinkABIPrp(link)->wlist, w );
  if( link2 && w2 ) zListPurge( &rkLinkABIPrp(link2)->wlist, w2 );
}

void rkChainABIUpdateAddExForceTwo(rkChain *chain, rkLink *link, rkWrench *w, rkLink *link2, rkWrench *w2)
{
  if( rkChainJointSize(chain) == 0 ){
    _rkChainLinkVelAccClear( chain );
    return;
  }
  _rkChainABIUpdateBackwardAddExForceTwo( chain, link, w, link2, w2 );
  rkChainABIUpdateForward( chain );
}
