#include <roki/roki.h>

void assert_chain_loop_forward_kinematics(void)
{
  rkChain chain;
  zVec q;
  const int step = 100;
  int i;
  FILE *fp;
  bool result = true;
  const double tol = 1.0e-10;

  rkChainReadZTK( &chain, "../model/double_fourbar_linkage.ztk" );
  q = zVecAlloc( rkChainJointSize( &chain ) );
  fp = fopen( "loop_fk.zvs", "w" );
  for( i=0; i<=step; i++ ){
    zVecSetElem( q, 0, zDeg2Rad(30)*(1-cos(zPIx2*i/step)) );
    zVecSetElem( q, 4,-zDeg2Rad(30)*(1-cos(zPIx2*i/step)) );
    rkChainFK( &chain, q );
    fprintf( fp, "0.1 " ); zVecFPrint( fp, q );
    /* check */
    if( !zEqual( zVecElemNC(q,1),-zVecElemNC(q,0), tol ) ||
        !zEqual( zVecElemNC(q,2), zVecElemNC(q,0), tol ) ||
        !zEqual( zVecElemNC(q,3),-zVecElemNC(q,0), tol ) ||
        !zEqual( zVecElemNC(q,5),-zVecElemNC(q,4), tol ) ||
        !zEqual( zVecElemNC(q,6), zVecElemNC(q,4), tol ) ||
        !zEqual( zVecElemNC(q,7),-zVecElemNC(q,4), tol ) ) result = false;
  }
  fclose( fp );
  zVecFree( q );
  rkChainDestroy( &chain );
  zAssert( rkChainFK (closed-loop), result );
}

void assert_chain_loop_inverse_kinematics(void)
{
  rkChain chain;
  zVec q;
  rkIKAttr attr;
  rkIKCell *cell;
  const int step = 100;
  int i;
  FILE *fp;
  bool result = true;
  const double tol = 1.0e-8;

  rkChainReadZTK( &chain, "../model/double_fourbar_linkage.ztk" );
  q = zVecAlloc( rkChainJointSize( &chain ) );
  rkChainRegisterIKJointAll( &chain, 0.001 );
  attr.id = rkChainFindLinkID( &chain, "coupler_link2" );
  cell = rkChainRegisterIKCellWldPos( &chain, NULL, 0, &attr, RK_IK_ATTR_MASK_ID );
  zVecSetElem( q, 0, zDeg2Rad(1.0) );
  rkChainSetJointDisAll( &chain, q ); /* to escape from initial singular point */
  fp = fopen( "loop_ik.zvs", "w" );
  for( i=0; i<=step; i++ ){
    rkIKCellSetRef( cell, 0,-0.1, 0.6 - 0.1*(1-cos(zPIx2*i/step)) );
    rkChainIK( &chain, q, zTOL, 0 );
    fprintf( fp, "0.1 " ); zVecFPrint( fp, q );
    /* check */
    if( !zEqual( zVecElemNC(q,1),-zVecElemNC(q,0), tol ) ||
        !zEqual( zVecElemNC(q,2), zVecElemNC(q,0), tol ) ||
        !zEqual( zVecElemNC(q,3),-zVecElemNC(q,0), tol ) ||
        !zEqual( zVecElemNC(q,5),-zVecElemNC(q,4), tol ) ||
        !zEqual( zVecElemNC(q,6), zVecElemNC(q,4), tol ) ||
        !zEqual( zVecElemNC(q,7),-zVecElemNC(q,4), tol ) ) result = false;
  }
  fclose( fp );
  zVecFree( q );
  rkChainDestroy( &chain );
  zAssert( rkChainIK (closed-loop), result );
}

int main(int argc, char *argv[])
{
  assert_chain_loop_forward_kinematics();
  assert_chain_loop_inverse_kinematics();
  return EXIT_SUCCESS;
}
