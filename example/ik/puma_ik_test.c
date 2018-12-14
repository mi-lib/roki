#include <roki/rk_ik.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  rkIK ik;
  zVec dis;
  zFrame3D goal;
  zVec6D err;
  rkIKCell *cell[2];
  rkIKCellAttr attr;

  zRandInit();
  if( !rkChainReadFile( &chain, "../model/puma.zkc" ) )
    exit( 1 );
  dis = zVecAlloc( rkChainJointSize( &chain ) );
  rkChainFK( &chain, dis );

  rkIKCreate( &ik, &chain );
  rkIKJointRegAll( &ik, 0.001 );

  attr.id = 6;
  cell[0] = rkIKCellRegWldAtt( &ik, &attr, RK_IK_CELL_ATTR_ID );
  cell[1] = rkIKCellRegWldPos( &ik, &attr, RK_IK_CELL_ATTR_ID );

  rkIKDeactivate( &ik );
  rkIKBind( &ik ); /* bind current status to the reference. */
  rkIKCellSetRef( cell[0],
    zDeg2Rad(zRandF(-30,30)), zDeg2Rad(zRandF(0,45)), zDeg2Rad(zRandF(-30,30)) );
  cell[1]->data.ref.pos.e[zZ] = zRandF(0.1,0.6);
  printf( "++ initial frame\n" );
  zFrame3DWrite( rkChainLinkWldFrame(ik.chain,6) );
  rkIKSolve( &ik, dis, zTOL, 0 );
  zVecWrite( dis );
  rkChainFK( ik.chain, dis );
  printf( "++ goal frame\n" );
  zFrame3DCreate( &goal, &cell[1]->data.ref.pos, &cell[0]->data.ref.att );
  zFrame3DWrite( &goal );
  printf( "++ final frame\n" );
  zFrame3DWrite( rkChainLinkWldFrame(ik.chain,6) );
  printf( "++ error\n" );
  zFrame3DError( &goal, rkChainLinkWldFrame(ik.chain,6), &err );
  zVec6DWrite( &err );

  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  zVecFree( dis );
  return 0;
}
