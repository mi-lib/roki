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
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_FIXED );
  rkJointCreate( rkChainLinkJoint(chain,1), RK_JOINT_SPHER );
  rkJointCreate( rkChainLinkJoint(chain,2), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,3), RK_JOINT_CYLIN );
  rkJointCreate( rkChainLinkJoint(chain,4), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,5), RK_JOINT_SPHER );
  rkJointCreate( rkChainLinkJoint(chain,6), RK_JOINT_FIXED );
  rkChainSetOffset( chain );
}

int main(void)
{
  rkChain r;
  rkIK ik;

  chain_init( &r );
  rkChainConnectionWrite( &r );
  rkIKCreate( &ik, &r );

  rkIKJointReg( &ik, 0, 100 );
  rkIKJointReg( &ik, 2,  10 );
  rkIKJointReg( &ik, 3,   5 );
  rkIKJointReg( &ik, 5,  20 );

  zIndexWrite( ik._j_idx );
  zIndexWrite( ik._j_ofs );
  zVecWrite( ik._j_wn );

  rkIKDestroy( &ik );
  rkChainDestroy( &r );
  return 0;
}
