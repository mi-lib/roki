#include <roki/rk_chain.h>

#define N    5
#define SIZE 7

void chain_init(rkChain *chain)
{
  register int i;
  char name[BUFSIZ];

  rkChainInit( chain );
  zArrayAlloc( &chain->link, rkLink, N );
  for( i=0; i<N; i++ ){
    sprintf( name, "link#%02d", i );
    rkLinkInit( rkChainLink(chain,i) );
    zNameSet( rkChainLink(chain,i), name );
    if( i > 0 )
      rkLinkAddChild( rkChainLink(chain,i-1), rkChainLink(chain,i) );
  }
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_FIXED );
  rkJointCreate( rkChainLinkJoint(chain,1), RK_JOINT_SPHER );
  rkJointCreate( rkChainLinkJoint(chain,2), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,3), RK_JOINT_CYLIN );
  rkJointCreate( rkChainLinkJoint(chain,4), RK_JOINT_FIXED );
  zVec3DCreate( rkChainLinkOrgPos(chain,2), 1, 0, 0 );
  zVec3DCreate( rkChainLinkOrgPos(chain,3), 0, 0, 1 );
  zFrame3DCopy( rkChainLinkOrgFrame(chain,2), rkChainLinkAdjFrame(chain,2) );
  zFrame3DCopy( rkChainLinkOrgFrame(chain,3), rkChainLinkAdjFrame(chain,3) );
  rkChainSetOffset( chain );
}

int main(void)
{
  rkChain chain;
  zVec dis;
  zIndex idx;

  chain_init( &chain );
  rkChainConnectionWrite( &chain );
  dis = zVecCreateList( SIZE, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7 );
  idx = rkChainCreateDefaultJointIndex( &chain );
  printf( "++ input joint ++\n" );
  zIndexWrite( idx );
  zVecWrite( dis );
  rkChainSetJointDis( &chain, idx, dis );
  rkChainUpdateFK( &chain );
  rkChainPostureWrite( &chain );
  rkChainGetJointDis( &chain, idx, dis );
  printf( "++ output joint ++\n" );
  zIndexWrite( idx );
  zVecWrite( dis );
  zIndexFree( idx );
  zVecFree( dis );
  rkChainDestroy( &chain );
  return 0;
}
