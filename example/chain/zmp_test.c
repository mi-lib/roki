#include <roki/rk_chain.h>

#define DT 0.001
#define DTHETA 0.001
#define STEP 1000

rkChain chain;
zVec dis;

void update_dis(int st)
{
  int i;

  for( i=7; i<zVecSizeNC(dis); i++ )
    zVecSetElem( dis, i, DTHETA*(1-cos(2*zPI*st/STEP)) );
}

void total_zmp_test(void)
{
  zVec3D zmp;

  zVec3DValuePrint( rkChainWldCOM(&chain) );
  rkChainZMP( &chain, 0, &zmp );
  zVec3DValueNLPrint( &zmp );
}

int main(void)
{
  int i;

  rkChainReadZTK( &chain, "../model/H5.ztk" );
  rkChainUpdateID( &chain );
  dis = zVecAlloc( rkChainJointSize( &chain ) );
  rkChainGetJointDisAll( &chain, dis );
  for( i=0; i<=STEP; i++ ){
    update_dis( i );
    rkChainFKCNT( &chain, dis, DT );
    total_zmp_test();
  }
  zVecFree( dis );
  rkChainDestroy( &chain );
  return 0;
}
