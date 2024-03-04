#include <roki/rk_chain.h>

#define N    5
#define SIZE 7

void chain_init(rkChain *chain)
{
  int i;
  char name[BUFSIZ];

  rkChainInit( chain );
  rkLinkArrayAlloc( rkChainLinkArray(chain), N );
  for( i=0; i<N; i++ ){
    sprintf( name, "link#%02d", i );
    rkLinkInit( rkChainLink(chain,i) );
    zNameSet( rkChainLink(chain,i), name );
    if( i > 0 )
      rkLinkAddChild( rkChainLink(chain,i-1), rkChainLink(chain,i) );
  }
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_fixed );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_spher );
  rkJointAssign( rkChainLinkJoint(chain,2), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,3), &rk_joint_cylin );
  rkJointAssign( rkChainLinkJoint(chain,4), &rk_joint_fixed );
  zVec3DCreate( rkChainLinkOrgPos(chain,2), 1, 0, 0 );
  zVec3DCreate( rkChainLinkOrgPos(chain,3), 0, 0, 1 );
  zFrame3DCopy( rkChainLinkOrgFrame(chain,2), rkChainLinkAdjFrame(chain,2) );
  zFrame3DCopy( rkChainLinkOrgFrame(chain,3), rkChainLinkAdjFrame(chain,3) );
  rkChainSetJointIDOffset( chain );
}

int main(void)
{
  rkChain chain;
  zVec dis;
  zIndex idx;

  chain_init( &chain );
  rkChainConnectionPrint( &chain );
  dis = zVecCreateList( SIZE, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7 );
  idx = rkChainCreateDefaultJointIndex( &chain );
  printf( "++ input joint ++\n" );
  zIndexPrint( idx );
  zVecPrint( dis );
  rkChainSetJointDis( &chain, idx, dis );
  rkChainUpdateFK( &chain );
  rkChainPosturePrint( &chain );
  rkChainGetJointDis( &chain, idx, dis );
  printf( "++ output joint ++\n" );
  zIndexPrint( idx );
  zVecPrint( dis );
  zIndexFree( idx );
  zVecFree( dis );
  rkChainDestroy( &chain );
  return 0;
}
