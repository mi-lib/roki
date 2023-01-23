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
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_fixed );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_spher );
  rkJointAssign( rkChainLinkJoint(chain,2), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,3), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,4), &rk_joint_fixed );
  zVec3DCreate( rkChainLinkOrgPos(chain,2), 1, 0, 0 );
  zVec3DCreate( rkChainLinkOrgPos(chain,3), 1, 0, 0 );
  zVec3DCreate( rkChainLinkOrgPos(chain,4), 1, 0, 0 );
  for( i=0; i<N; i++ )
    zFrame3DCopy( rkChainLinkOrgFrame(chain,i), rkChainLinkAdjFrame(chain,i) );
  rkChainSetMass( chain, 1.0 ); /* dummy */
  rkChainSetJointIDOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
}

void set_dis(rkChain *chain, zVec dis, int st)
{
  register int i, j;
  double val;
  zMat3D r;

  val = 0.5*zPI*st/STEP;
  for( i=0; i<rkChainLinkNum(chain); i++ ){
    if( rkChainLinkJointIDOffset(chain,i) < 0 ) continue;
    if( rkChainLinkJoint(chain,i)->com == &rk_joint_spher ){
      zMat3DFromZYX( &r, val, val, val );
      zMat3DToEP( &r, (zEP*)&zVecElemNC(dis,rkChainLinkJointIDOffset(chain,i)) );
    } else
      for( j=0; j<rkChainLinkJointSize(chain,i); j++ )
        zVecSetElem( dis, rkChainLinkJointIDOffset(chain,i)+j, val );
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
    rkChainLinkWldLinJacobi( &chain, TIP, ZVEC3DZERO, jacobi );
    printf( "%f\n", rkJacobiManip( jacobi ) );
  }

  /* termination */
  zVecFree( dis );
  zMatFree( jacobi );
  rkChainDestroy( &chain );
  return 0;
}
