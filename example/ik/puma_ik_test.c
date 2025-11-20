#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec dis;
  zFrame3D goal;
  zVec6D err;
  rkIKCell *cell[2];
  rkIKAttr attr;

  zRandInit();
  if( !rkChainReadZTK( &chain, "../model/puma.ztk" ) )
    exit( 1 );
  dis = zVecAlloc( rkChainJointSize( &chain ) );
  rkChainRegisterIKJointAll( &chain, 0.001 );

  attr.id = 6;
  cell[0] = rkChainRegisterIKCellWldAtt( &chain, NULL, 0, &attr, RK_IK_ATTR_MASK_ID );
  cell[1] = rkChainRegisterIKCellWldPos( &chain, NULL, 0, &attr, RK_IK_ATTR_MASK_ID );

  rkChainBindIK( &chain );
  rkIKCellSetRef( cell[0],
    zDeg2Rad(zRandF(-30,30)), zDeg2Rad(zRandF(0,45)), zDeg2Rad(zRandF(-30,30)) );
  cell[1]->data.ref.pos.e[zZ] = zRandF(0.1,0.6);
  printf( "++ initial frame\n" );
  zFrame3DPrint( rkChainLinkWldFrame(&chain,6) );
  rkChainIK( &chain, dis, zTOL, 0 );
  zVecPrint( dis );

  printf( "++ goal frame\n" );
  zFrame3DCreate( &goal, &cell[1]->data.ref.pos, &cell[0]->data.ref.att );
  zFrame3DPrint( &goal );
  printf( "++ final frame\n" );
  zFrame3DPrint( rkChainLinkWldFrame(&chain,6) );
  printf( "++ error\n" );
  zFrame3DError( &goal, rkChainLinkWldFrame(&chain,6), &err );
  zVec6DPrint( &err );

  rkChainDestroy( &chain );
  zVecFree( dis );
  return 0;
}
