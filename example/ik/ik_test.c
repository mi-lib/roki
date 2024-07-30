#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec dis;
  rkIKCell *cell;
  rkIKAttr attr;
  double r;
  zVec3D pos, err;

  zRandInit();
  rkChainReadZTK( &chain, "../model/arm.ztk" );
  rkChainCreateIK( &chain );
  rkChainRegisterIKJointAll( &chain, 0.001 );
  dis = zVecAlloc( rkChainJointSize( &chain ) );

  rkChainFK( &chain, dis );

  attr.id = 5;
  cell = rkChainRegisterIKCellWldPos( &chain, NULL, 0, &attr, RK_IK_ATTR_MASK_ID );
  r = rkChainLinkWldPos(&chain,attr.id)->c.z - rkChainLinkWldPos(&chain,1)->c.z;

  zVec3DCreatePolar( &pos, zRandF(0.5,1.0)*r, zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
  rkIKCellSetRefVec( cell, &pos );
  zVec3DPrint( rkIKCellRefPos(cell) );
  rkChainIK( &chain, dis, zTOL, 0 );
  zVecPrint( dis );
  zVec3DPrint( rkChainLinkWldPos(&chain,attr.id) );
  zVec3DSub( rkIKCellRefPos(cell), rkChainLinkWldPos(&chain,attr.id), &err );
  printf( "error: " );
  zVec3DPrint( &err );

  rkChainDestroy( &chain );
  zVecFree( dis );
  return 0;
}
