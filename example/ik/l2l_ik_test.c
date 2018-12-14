#include <roki/rk_ik.h>

/*
  <--o---o---o[_]o---o---o-->
     6   5   4 0 1   2   3
 */

#define EL1 3
#define EL2 6

int main(int argc, char *argv[])
{
  rkChain chain;
  rkIK ik;
  rkIKCellAttr attr;
  zVec dis;
  zVec3D err;
  double val[6];
  rkIKCell *cell[4];
  register int i;

  zRandInit();
  rkChainReadFile( &chain, "../model/dualarm.zkc" );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainSetJointDisAll( &chain, dis );
  rkChainGetJointDisAll( &chain, dis );

  rkIKCreate( &ik, &chain );
  rkIKJointRegAll( &ik, 0.001 );

  attr.id = EL1;
  cell[0] = rkIKCellRegWldPos( &ik, &attr, RK_IK_CELL_ATTR_ID );
  cell[1] = rkIKCellRegWldAtt( &ik, &attr, RK_IK_CELL_ATTR_ID );
  attr.id = EL2;
  attr.id_sub = EL1;
  cell[2] = rkIKCellRegL2LPos( &ik, &attr, RK_IK_CELL_ATTR_ID|RK_IK_CELL_ATTR_ID_SUB );
  cell[3] = rkIKCellRegL2LAtt( &ik, &attr, RK_IK_CELL_ATTR_ID|RK_IK_CELL_ATTR_ID_SUB );

  rkIKDeactivate( &ik );
  rkIKBind( &ik ); /* bind current status to the reference. */
  for( i=0; i<3; i++ ){
    val[i] = zRandF(-0.15,0.15);
    val[i+3] = zRandF(-1.0,1.0);
  }
  rkIKCellSetRef( cell[0], val[0], val[1], val[2] );
  rkIKCellSetRef( cell[1], val[3], val[4], val[5] );
  rkIKCellSetRef( cell[2], 0.01, 0.03, -0.01 );
  rkIKCellSetRef( cell[3], 0, 0, 0 );

  eprintf( "++ initial frame\n" );
  zFrame3DFWrite( stderr, rkChainLinkWldFrame(ik.chain,EL1) );
  zFrame3DFWrite( stderr, rkChainLinkWldFrame(ik.chain,EL2) );

  rkIKSolve( &ik, dis, zTOL, 0 );
  zVecFWrite( stderr, dis );
  rkChainFK( ik.chain, dis );
  eprintf( "++ goal frame\n" );
  zVec3DFWrite( stderr, &cell[0]->data.ref.pos );
  zMat3DFWrite( stderr, &cell[1]->data.ref.att );
  zVec3DFWrite( stderr, &cell[2]->data.ref.pos );
  zMat3DFWrite( stderr, &cell[3]->data.ref.att );

  eprintf( "++ final frame\n" );
  zFrame3DFWrite( stderr, rkChainLinkWldFrame(ik.chain,EL1) );
  zFrame3DFWrite( stderr, rkChainLinkWldFrame(ik.chain,EL2) );

  eprintf( "++ error\n" );
  zVec3DSub( &cell[0]->data.ref.pos, rkChainLinkWldPos(ik.chain,EL1), &err );
  zVec3DFWrite( stderr, &err );
  zMat3DError( &cell[1]->data.ref.att, rkChainLinkWldAtt(ik.chain,EL1), &err );
  zVec3DFWrite( stderr, &err );
  zVec3DSub( rkChainLinkWldPos(ik.chain,EL1), rkChainLinkWldPos(ik.chain,EL2), &err );
  zVec3DFWrite( stderr, &err );
  zMat3DError( rkChainLinkWldAtt(ik.chain,EL2), rkChainLinkWldAtt(ik.chain,EL2), &err );
  zVec3DFWrite( stderr, &err );

  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  zVecFree( dis );
  return 0;
}
