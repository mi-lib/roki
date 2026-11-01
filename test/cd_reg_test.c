#include <roki/rk_cd.h>

static bool ph_vertices_equal(zPH3D *a, zPH3D *b)
{
  int i;

  if( zPH3DVertNum(a) != zPH3DVertNum(b) ) return false;
  for( i=0; i<zPH3DVertNum(a); i++ )
    if( !zVec3DEqual( zPH3DVert(a,i), zPH3DVert(b,i) ) ) return false;
  return true;
}

static void assert_chain_reg_fixed(void)
{
  rkChain chain;
  rkCD cd;

  rkChainReadZTK( &chain, "cd_fixed.ztk" );
  rkCDCreate( &cd );
  zAssert( rkCDChainReg (zero DOF), rkCDChainReg( &cd, &chain, RK_CD_CELL_STAT ) == &cd );
  zAssert( rkCDChainReg (fixed cell), zListSize(&cd.clist) == 1 );
  rkCDDestroy( &cd );
  rkChainDestroy( &chain );
}

static void assert_chain_reg_pose(rkCDCellType type)
{
  rkChain chain, fixed;
  rkCD cd;
  rkCDCell *cell;
  rkCDPair *pair;
  zVec dis, restored;
  zFrame3D frames[3];
  zPH3D expected;
  zBox3D obb;
  zAABox3D aabb;
  int i, size, external_count;
  bool frames_equal, pairs_valid, geometry_equal;

  rkChainReadZTK( &chain, "cd_chain.ztk" );
  size = rkChainJointSize( &chain );
  dis = zVecAlloc( size );
  restored = zVecAlloc( size );
  /* Swap which link pair overlaps, and rotate the root. */
  zVecElemNC(dis,0) = 1;
  zVecElemNC(dis,2) = 2;
  zVecElemNC(dis,5) = 0.4;
  zVecElemNC(dis,6) = -0.3;
  zVecElemNC(dis,12) = 0.5;
  rkChainFK( &chain, dis );
  for( i=0; i<3; i++ ) frames[i] = *rkChainLinkWldFrame(&chain,i);
  rkCDCreate( &cd );
  zAssert( rkCDChainReg (posed chain), rkCDChainReg( &cd, &chain, type ) == &cd );
  rkChainGetJointDisAll( &chain, restored );
  zAssert( rkCDChainReg (joint displacement), zVecEqual( dis, restored, zTOL ) );
  frames_equal = true;
  for( i=0; i<3; i++ )
    if( !zFrame3DEqual( &frames[i], rkChainLinkWldFrame(&chain,i) ) ) frames_equal = false;
  zAssert( rkCDChainReg (world frames), frames_equal );
  pairs_valid = zListSize(&cd.plist) == ( type == RK_CD_CELL_STAT ? 0 : 2 );
  zListForEach( &cd.plist, pair )
    if( pair->data.cell[0]->data.link != rkChainLink(&chain,1) &&
        pair->data.cell[1]->data.link != rkChainLink(&chain,1) ) pairs_valid = false;
  zAssert( rkCDChainReg (neutral self-collision pairs), pairs_valid );
  geometry_equal = true;
  zListForEach( &cd.clist, cell ){
    zPH3DClone( zShape3DPH(cell->data.shape), &expected );
    zPH3DXform( zShape3DPH(cell->data.shape), &frames[cell->data.link-rkChainLink(&chain,0)], &expected );
    zBox3DXform( &cell->data.bb, &frames[cell->data.link-rkChainLink(&chain,0)], &obb );
    zBox3DToAABox3D( &obb, &aabb );
    if( !ph_vertices_equal( &expected, &cell->data.ph ) ||
        !zVec3DEqual( &aabb.min, &cell->data.aabb.min ) ||
        !zVec3DEqual( &aabb.max, &cell->data.aabb.max ) ) geometry_equal = false;
    zPH3DDestroy( &expected );
  }
  zAssert( rkCDChainReg (world geometry), geometry_equal );
  rkChainReadZTK( &fixed, "cd_fixed.ztk" );
  rkCDChainReg( &cd, &fixed, RK_CD_CELL_MOVE );
  external_count = 0;
  zListForEach( &cd.plist, pair )
    if( pair->data.cell[0]->data.chain != pair->data.cell[1]->data.chain ) external_count++;
  zAssert( rkCDChainReg (inter-chain pairs), external_count == 3 );
  rkCDDestroy( &cd );
  rkChainDestroy( &fixed );
  rkChainDestroy( &chain );
  zVecFreeAtOnce( 2, dis, restored );
}

static void assert_colchk_boxes(double height, bool expected)
{
  rkChain a, b;
  rkCD cd;

  rkChainReadZTK( &a, "cd_fixed.ztk" );
  rkChainReadZTK( &b, "cd_fixed.ztk" );
  zVec3DCreate( rkLinkOrgPos(rkChainLink(&b,0)), 0, 0, height );
  rkChainUpdateFK( &b );
  rkCDCreate( &cd );
  rkCDChainReg( &cd, &a, RK_CD_CELL_STAT );
  rkCDChainReg( &cd, &b, RK_CD_CELL_MOVE );
  zAssert( rkCDChainReg (box pair), zListSize(&cd.plist) == 1 );
  zAssert( rkCDColChkAABB, rkCDColChkAABB( &cd ) == expected );
  zAssert( rkCDColChkOBB, rkCDColChkOBB( &cd ) == expected );
  zAssert( rkCDColChkGJK, rkCDColChkGJK( &cd ) == expected );
  rkCDDestroy( &cd );
  rkChainDestroy( &a );
  rkChainDestroy( &b );
}

static void assert_colchk_motion(void)
{
  rkChain a, b;
  rkCD cd;

  rkChainReadZTK( &a, "cd_fixed.ztk" );
  rkChainReadZTK( &b, "cd_fixed.ztk" );
  zVec3DCreate( rkLinkOrgPos(rkChainLink(&b,0)), 0, 0, 0.2 );
  rkChainUpdateFK( &b );
  rkCDCreate( &cd );
  rkCDChainReg( &cd, &a, RK_CD_CELL_STAT );
  rkCDChainReg( &cd, &b, RK_CD_CELL_MOVE );
  zAssert( rkCDColChkGJK (separated), !rkCDColChkGJK( &cd ) );
  zVec3DCreate( rkLinkOrgPos(rkChainLink(&b,0)), 0, 0, 0.05 );
  rkChainUpdateFK( &b );
  zAssert( rkCDColChkGJK (overlapping after motion), rkCDColChkGJK( &cd ) );
  zVec3DCreate( rkLinkOrgPos(rkChainLink(&b,0)), 0, 0, 0.2 );
  rkChainUpdateFK( &b );
  zAssert( rkCDColChkGJK (separated after motion), !rkCDColChkGJK( &cd ) );
  rkCDDestroy( &cd );
  rkChainDestroy( &a );
  rkChainDestroy( &b );
}

int main(void)
{
  assert_chain_reg_fixed();
  assert_colchk_motion();
  assert_colchk_boxes( 0.2, false );
  assert_colchk_boxes( 0.1, true );
  assert_colchk_boxes( 0.05, true );
  assert_chain_reg_pose( RK_CD_CELL_MOVE );
  assert_chain_reg_pose( RK_CD_CELL_STAT );
  return EXIT_SUCCESS;
}
