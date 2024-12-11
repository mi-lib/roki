#include <roki/rk_chain.h>

void create_chain(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "floatbody" );
  rkLinkArrayAlloc( rkChainLinkArray(chain), 1 );
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "body" );
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_float );

  rkChainSetMass( chain, 1.0 );
  rkChainSetJointIDOffset( chain );
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
  int i;

  create_chain( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );

  for( i=0; i<=STEP; i++ ){
    angle = zPI*2*i/STEP;
    zVecSetElem( dis, 3, angle );
    rkChainFK( &chain, dis );
    zVec3DValuePrint( rkChainGravityDir( &chain, &a ) ); zEndl();
  }
  zVecZero( dis );
  for( i=0; i<=STEP; i++ ){
    angle = zPI*2*i/STEP;
    zVecSetElem( dis, 4, angle );
    rkChainFK( &chain, dis );
    zVec3DValuePrint( rkChainGravityDir( &chain, &a ) ); zEndl();
  }
  zVecZero( dis );
  for( i=0; i<=STEP; i++ ){
    angle = zPI*2*i/STEP;
    zVecSetElem( dis, 5, angle );
    rkChainFK( &chain, dis );
    zVec3DValuePrint( rkChainGravityDir( &chain, &a ) ); zEndl();
  }

  zVecFree( dis );
  rkChainDestroy( &chain );
  return 0;
}
