#include <roki/rk_chain.h>

void create_chain(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "floatbody" );
  zArrayAlloc( &chain->link, rkLink, 1 );
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "body" );
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_FLOAT );

  rkChainSetMass( chain, 1.0 );
  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
}

#define STEP 100

int main(void)
{
  rkChain chain;
  double angle;
  zVec dis;
  zVec3D a;
  register int i;

  create_chain( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );

  for( i=0; i<=STEP; i++ ){
    angle = zPI*2*i/STEP;
    zVecSetElem( dis, 3, angle );
    rkChainFK( &chain, dis );
    zVec3DDataWrite( rkChainGravityDir( &chain, &a ) );
  }
  zVecClear( dis );
  for( i=0; i<=STEP; i++ ){
    angle = zPI*2*i/STEP;
    zVecSetElem( dis, 4, angle );
    rkChainFK( &chain, dis );
    zVec3DDataWrite( rkChainGravityDir( &chain, &a ) );
  }
  zVecClear( dis );
  for( i=0; i<=STEP; i++ ){
    angle = zPI*2*i/STEP;
    zVecSetElem( dis, 5, angle );
    rkChainFK( &chain, dis );
    zVec3DDataWrite( rkChainGravityDir( &chain, &a ) );
  }

  zVecFree( dis );
  rkChainDestroy( &chain );
  return 0;
}
