#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec dis;
  rkIKCell *cell;
  rkIKAttr attr;

  rkChainReadZTK( &chain, "../model/arm.ztk" );
  rkChainCreateIK( &chain );
  rkChainRegIKJointAll( &chain, 0.001 );
  dis = zVecAlloc( rkChainJointSize( &chain ) );

  rkChainFK( &chain, dis );

  attr.id = 5;
  cell = rkChainRegIKCellWldPos( &chain, NULL, &attr, RK_IK_ATTR_ID );

  rkChainDisableIK( &chain );
  rkIKCellSetRef( cell, 0, -0.2, 0.1 );
  zVec3DPrint( rkIKCellRefPos(cell) );
  rkChainIK( &chain, dis, zTOL, 0 );
  zVecPrint( dis );
  zVec3DPrint( rkChainLinkWldPos(&chain,attr.id) );

  rkChainDestroy( &chain );
  zVecFree( dis );
  return 0;
}
