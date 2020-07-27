#include <roki/rk_chain.h>

#define N   7
#define SIZE 12

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
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_fixed );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_spher ); /* 0-2 */
  rkJointAssign( rkChainLinkJoint(chain,2), &rk_joint_revol ); /* 3 */
  rkJointAssign( rkChainLinkJoint(chain,3), &rk_joint_cylin ); /* 4-5 */
  rkJointAssign( rkChainLinkJoint(chain,4), &rk_joint_revol ); /* 6 */
  rkJointAssign( rkChainLinkJoint(chain,5), &rk_joint_spher ); /* 7-9 */
  rkJointAssign( rkChainLinkJoint(chain,6), &rk_joint_fixed );
  rkChainSetOffset( chain );
}

int main(void)
{
  rkChain chain;
  zVec dis, dis2;
  zIndex idx, idx2;

  chain_init( &chain );
  rkChainConnectionPrint( &chain );
  dis = zVecCreateList( SIZE, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2 );
  idx = rkChainCreateDefaultJointIndex( &chain );
  idx2 = zIndexCreateList( 3, 2, 4, 5 );
  dis2 = zVecAlloc( rkChainJointIndexSize(&chain,idx2) );
  printf( "++ input joint ++\n" );
  zIndexPrint( idx );
  zVecPrint( dis );
  rkChainSetJointDis( &chain, idx, dis );
  rkChainGetJointDis( &chain, idx2, dis2 );
  printf( "++ output joint ++\n" );
  zIndexPrint( idx );
  zVecPrint( dis );
  zIndexPrint( idx2 );
  zVecPrint( dis2 );

  zIndexFree( idx );
  zVecFree( dis );
  zIndexFree( idx2 );
  zVecFree( dis2 );
  rkChainDestroy( &chain );
  return 0;
}
