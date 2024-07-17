#include <roki/rk_chain.h>

/*
base(float)
 --link1(revol)--link2(revol)--link3(fixed)
 --link4(revol)--link5(revol)--link6(fixed)
 */

#define BRANCH_NUM 2
#define LINK_NUM   ( 3 * BRANCH_NUM + 1 )

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
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_float );
  for( i=0; i<BRANCH_NUM; i++ ){
    rkJointAssign( rkChainLinkJoint(chain,i*3+1), &rk_joint_revol );
    rkJointAssign( rkChainLinkJoint(chain,i*3+2), &rk_joint_revol );
    rkJointAssign( rkChainLinkJoint(chain,i*3+3), &rk_joint_fixed );
    zVec3DCreate( rkChainLinkOrgPos(chain,i*3+2), 0, 1, 0 );
    zVec3DCreate( rkChainLinkOrgPos(chain,i*3+3), 0, 1, 0 );
    rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,i*3+1) );
    rkLinkAddChild( rkChainLink(chain,i*3+1), rkChainLink(chain,i*3+2) );
    rkLinkAddChild( rkChainLink(chain,i*3+2), rkChainLink(chain,i*3+3) );
  }
  for( i=0; i<LINK_NUM; i++ )
    zFrame3DCopy( rkChainLinkOrgFrame(chain,i), rkChainLinkAdjFrame(chain,i) );
  rkChainSetMass( chain, 1.0 ); /* dummy weight */
  rkChainSetJointIDOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  rkChainWriteZTK( chain, "branched.ztk" );
}

#define DIV 200

int main(int argc, char *argv[])
{
  rkChain chain;
  rkIKAttr attr;
  zVec dis;
  rkIKCell *cell[BRANCH_NUM];
  int i;
  FILE *fp;

  chain_init( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainFK( &chain, dis );

  rkChainCreateIK( &chain );
  rkChainRegIKJointAll( &chain, 0.1 );

  for( i=0; i<BRANCH_NUM; i++ ){
    attr.id = i*3+3;
    cell[i] = rkChainRegIKCellWldPos( &chain, NULL, &attr, RK_IK_ATTR_MASK_ID );
    rkIKCellSetActiveComponent( cell[i], RK_IK_CELL_MODE_X | RK_IK_CELL_MODE_Y );
  }
  rkChainDisableIK( &chain );
  rkChainBindIK( &chain );

  zVec3DCreate( &cell[0]->data.ref.pos, 1, 1, 0 );
  rkIKCellForce( cell[0] );
  fp = fopen( "f.zvs", "w" );
  for( i=0; i<=DIV; i++ ){
    zVec3DCreate( &cell[1]->data.ref.pos, 1, 1-5*(double)i/DIV, 0 );
    rkChainIK( &chain, dis, zTOL, 0 );
    rkChainFK( &chain, dis );
    printf( "%g %g %g %g %g %g %g %g\n",
      cell[0]->data.ref.pos.c.x, cell[0]->data.ref.pos.c.y,
      rkChainLinkWldPos(&chain,3)->c.x, rkChainLinkWldPos(&chain,3)->c.y,
      cell[1]->data.ref.pos.c.x, cell[1]->data.ref.pos.c.y,
      rkChainLinkWldPos(&chain,6)->c.x, rkChainLinkWldPos(&chain,6)->c.y );
    fprintf( fp, "0.05 " );
    zVecFPrint( fp, dis );
  }
  fclose( fp );
  rkChainDestroy( &chain );
  zVecFree( dis );
  return 0;
}
