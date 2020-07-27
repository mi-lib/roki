#include <roki/rk_ik.h>

#define N 7

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
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_spher );
  rkJointAssign( rkChainLinkJoint(chain,2), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,3), &rk_joint_cylin );
  rkJointAssign( rkChainLinkJoint(chain,4), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,5), &rk_joint_spher );
  rkJointAssign( rkChainLinkJoint(chain,6), &rk_joint_fixed );
  rkChainSetOffset( chain );
}

int main(void)
{
  rkChain r;
  rkIK ik;

  chain_init( &r );
  rkChainConnectionPrint( &r );
  rkIKCreate( &ik, &r );

  rkIKJointReg( &ik, 0, 100 );
  rkIKJointReg( &ik, 2,  10 );
  rkIKJointReg( &ik, 3,   5 );
  rkIKJointReg( &ik, 5,  20 );

  zIndexPrint( ik._j_idx );
  zIndexPrint( ik._j_ofs );
  zVecPrint( ik._j_wn );

  rkIKDestroy( &ik );
  rkChainDestroy( &r );
  return 0;
}
