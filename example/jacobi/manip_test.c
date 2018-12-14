#include <roki/rk_jacobi.h>

#define STEP 1000

#define N   5
#define TIP 4

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
  }
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );
  rkLinkAddChild( rkChainLink(chain,1), rkChainLink(chain,2) );
  rkLinkAddChild( rkChainLink(chain,2), rkChainLink(chain,3) );
  rkLinkAddChild( rkChainLink(chain,3), rkChainLink(chain,4) );
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_FIXED );
  rkJointCreate( rkChainLinkJoint(chain,1), RK_JOINT_SPHER );
  rkJointCreate( rkChainLinkJoint(chain,2), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,3), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,4), RK_JOINT_FIXED );
  zVec3DCreate( rkChainLinkOrgPos(chain,2), 1, 0, 0 );
  zVec3DCreate( rkChainLinkOrgPos(chain,3), 1, 0, 0 );
  zVec3DCreate( rkChainLinkOrgPos(chain,4), 1, 0, 0 );
  for( i=0; i<N; i++ )
    zFrame3DCopy( rkChainLinkOrgFrame(chain,i), rkChainLinkAdjFrame(chain,i) );
  rkChainSetMass( chain, 1.0 ); /* dummy */
  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
}

void set_dis(rkChain *chain, zVec dis, int st)
{
  register int i, j;
  double val;
  zMat3D r;

  val = 0.5*zPI*st/STEP;
  for( i=0; i<rkChainNum(chain); i++ ){
    if( rkChainLinkOffset(chain,i) < 0 ) continue;
    if( rkChainLinkJointType(chain,i) == RK_JOINT_SPHER ){
      zMat3DZYX( &r, val, val, val );
      zMat3DToEP( &r, (zEP*)&zVecElem(dis,rkChainLinkOffset(chain,i)) );
    } else
      for( j=0; j<rkChainLinkJointSize(chain,i); j++ )
        zVecSetElem( dis, rkChainLinkOffset(chain,i)+j, val );
  }
}

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec dis;
  zMat jacobi;
  register int i;

  /* initialization */
  zRandInit();
  chain_init( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  jacobi = zMatAlloc( 3, rkChainJointSize(&chain) );

  for( i=0; i<=STEP; i++ ){
    set_dis( &chain, dis, i );
    rkChainFK( &chain, dis );
    rkChainLinkTtlLinJacobi( &chain, TIP, Z_ZEROVEC3D, jacobi );
    printf( "%f\n", rkJacobiManip( jacobi ) );
  }

  /* termination */
  zVecFree( dis );
  zMatFree( jacobi );
  rkChainDestroy( &chain );
  return 0;
}
