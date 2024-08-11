#include <roki/rk_chain.h>

/*
link0(revol)--link1(revol)
 --link2(revol)--link3(revol)--link4(revol)--link5(fixed)
 --link6(revol)--link7(revol)--link8(revol)--link9(fixed)
 --link10(revol)--link11(revol)--link12(revol)--link13(fixed)
 */

#define BRANCH_NUM 3
#define LINK_NUM   ( 4 * BRANCH_NUM + 2 )

void chain_init(rkChain *chain)
{
  int i;
  char name[BUFSIZ];

  rkChainInit( chain );
  rkLinkArrayAlloc( rkChainLinkArray(chain), LINK_NUM );
  for( i=0; i<LINK_NUM; i++ ){
    sprintf( name, "link#%02d", i );
    rkLinkInit( rkChainLink(chain,i) );
    zNameSet( rkChainLink(chain,i), name );
  }
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_revol );
  zVec3DCreate( rkChainOrgPos(chain), 0,-0.2, 0 );
  zMat3DCreate( rkChainOrgAtt(chain),
    0, 0, 1,
    0, 1, 0,
   -1, 0, 0 );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_revol );
  zVec3DCreate( rkChainLinkOrgPos(chain,1), 0, 0.1, 0 );
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );
  for( i=0; i<BRANCH_NUM; i++ ){
    rkJointAssign( rkChainLinkJoint(chain,i*4+2), &rk_joint_revol );
    rkJointAssign( rkChainLinkJoint(chain,i*4+3), &rk_joint_revol );
    rkJointAssign( rkChainLinkJoint(chain,i*4+4), &rk_joint_revol );
    rkJointAssign( rkChainLinkJoint(chain,i*4+5), &rk_joint_fixed );
    zVec3DCreate( rkChainLinkOrgPos(chain,i*4+2), 0, 0.1, 0 );
    zVec3DCreate( rkChainLinkOrgPos(chain,i*4+3), 0, 0.1, 0 );
    zVec3DCreate( rkChainLinkOrgPos(chain,i*4+4), 0, 0.1, 0 );
    zVec3DCreate( rkChainLinkOrgPos(chain,i*4+5), 0, 0.1, 0 );
    rkLinkAddChild( rkChainLink(chain,1), rkChainLink(chain,i*4+2) );
    rkLinkAddChild( rkChainLink(chain,i*4+2), rkChainLink(chain,i*4+3) );
    rkLinkAddChild( rkChainLink(chain,i*4+3), rkChainLink(chain,i*4+4) );
    rkLinkAddChild( rkChainLink(chain,i*4+4), rkChainLink(chain,i*4+5) );
  }
  for( i=0; i<LINK_NUM; i++ )
    zFrame3DCopy( rkChainLinkOrgFrame(chain,i), rkChainLinkAdjFrame(chain,i) );
  rkChainSetMass( chain, 1.0 ); /* dummy weight */
  rkChainSetJointIDOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  rkChainWriteZTK( chain, "branched.ztk" );
}

#define DIV 100

int main(int argc, char *argv[])
{
  rkChain chain;
  rkIKAttr attr;
  zVec dis;
  rkIKCell *cell[BRANCH_NUM];
  zVec3D ref[BRANCH_NUM];
  int i, j;
  FILE *fp;

  chain_init( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainFK( &chain, dis );
  rkChainRegisterIKJointAll( &chain, 0.01 );

  for( i=0; i<BRANCH_NUM; i++ ){
    attr.id = i*4+5;
    cell[i] = rkChainRegisterIKCellWldPos( &chain, NULL, BRANCH_NUM-i, &attr, RK_IK_ATTR_MASK_ID );
    rkIKCellSetActiveComponent( cell[i], RK_IK_CELL_MODE_Y | RK_IK_CELL_MODE_Z );
  }
  zVec3DCreate( &ref[0], 0, 0.2, 0.2 );
  zVec3DCreate( &ref[1], 0, 0.2,-0.2 );
  zVec3DCreate( &ref[2], 0, 0.2, 0.0 );
  rkIKCellSetRefVec( cell[0], &ref[0] );
  rkIKCellSetRefVec( cell[1], &ref[1] );
  rkIKCellSetRefVec( cell[2], &ref[2] );
  fp = fopen( "f.zvs", "w" );
  for( i=0; i<=DIV; i++ ){
    zVec3DCreate( &ref[1], 0, 0.2,-0.2-0.5*(double)i/DIV );
    rkIKCellSetRefVec( cell[1], &ref[1] );
    rkChainIK( &chain, dis, zTOL, 0 );
    rkChainFK( &chain, dis );
    for( j=0; j<BRANCH_NUM; j++ ){
      printf( "[#%d] %g %g ", j,
        ref[j].c.y - rkChainLinkWldPos(&chain,rkIKCellLinkID(cell[j]))->c.y,
        ref[j].c.z - rkChainLinkWldPos(&chain,rkIKCellLinkID(cell[j]))->c.z );
    }
    printf( "\n" );
    fprintf( fp, "0.05 " );
    zVecFPrint( fp, dis );
  }
  for( i=0; i<=DIV; i++ ){
    zVec3DCreate( &ref[2], 0, 0.2+0.2*(double)i/DIV, 0 );
    rkIKCellSetRefVec( cell[2], &ref[2] );
    rkChainIK( &chain, dis, zTOL, 0 );
    rkChainFK( &chain, dis );
    for( j=0; j<BRANCH_NUM; j++ ){
      printf( "[#%d] %g %g ", j,
        ref[j].c.y - rkChainLinkWldPos(&chain,rkIKCellLinkID(cell[j]))->c.y,
        ref[j].c.z - rkChainLinkWldPos(&chain,rkIKCellLinkID(cell[j]))->c.z );
    }
    printf( "\n" );
    fprintf( fp, "0.05 " );
    zVecFPrint( fp, dis );
  }
  fclose( fp );
  rkChainDestroy( &chain );
  zVecFree( dis );
  return 0;
}
