#include <roki/rk_chain.h>

#define DT 0.001
#define DTHETA zDeg2Rad(20.0)
#define STEP 100

rkChain chain;
zVec dis;

zVec3D com, vel, acc;

void update_dis(int st)
{
  int i;
  double val;
  zMat3D r;

  val = DTHETA * ( 1 - cos(2*zPI*st/STEP) );
  zMat3DFromZYX( &r, val, val, val );
  zMat3DToAA( &r, (zVec3D*)&zVecElemNC(dis,3) );
  for( i=6; i<zVecSizeNC(dis); i++ )
    zVecSetElem( dis, i, val );
}

void world_com_test(void)
{
  zVec3D v, a;

  zVec3DValuePrint( rkChainWldCOM(&chain) );
  zVec3DValuePrint( rkChainCOMVel(&chain) );
  zVec3DValuePrint( rkChainCOMAcc(&chain) );

  zVec3DSub( rkChainWldCOM(&chain), &com, &v );
  zVec3DDivDRC( &v, DT );
  zVec3DValuePrint( &v );
  zVec3DSub( &v, &vel, &a );
  zVec3DDivDRC( &a, DT );
  zVec3DValueNLPrint( &a );

  zVec3DCopy( rkChainWldCOM(&chain), &com );
  zVec3DCopy( &v, &vel );
}

int main(void)
{
  int i;

  rkChainReadZTK( &chain, "../model/H5.ztk" );
  rkChainUpdateID( &chain );
  zVec3DCopy( rkChainWldCOM(&chain), &com );
  zVec3DCopy( rkChainCOMVel(&chain), &vel );
  dis = zVecAlloc( rkChainJointSize( &chain ) );
  rkChainGetJointDisAll( &chain, dis );
  for( i=0; i<=STEP; i++ ){
    update_dis( i );
    rkChainFKCNT( &chain, dis, DT );
    world_com_test();
  }
  zVecFree( dis );
  rkChainDestroy( &chain );
  return 0;
}
