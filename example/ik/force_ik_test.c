#include <roki/rk_chain.h>

/*
base(float)
 --link1(revol)--link2(revol)--link3(fix)
 --link4(revol)--link5(revol)--link6(fix)
 --link7(revol)--link8(revol)--link9(fix)
 */

void chain_init(rkChain *chain)
{
  int i;
  char name[BUFSIZ];

  rkChainInit( chain );
  rkLinkArrayAlloc( rkChainLinkArray(chain), 10 );
  for( i=0; i<10; i++ ){
    sprintf( name, "link#%02d", i );
    rkLinkInit( rkChainLink(chain,i) );
    zNameSet( rkChainLink(chain,i), name );
  }
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_float );
  for( i=0; i<3; i++ ){
    rkJointAssign( rkChainLinkJoint(chain,i*3+1), &rk_joint_revol );
    rkJointAssign( rkChainLinkJoint(chain,i*3+2), &rk_joint_revol );
    rkJointAssign( rkChainLinkJoint(chain,i*3+3), &rk_joint_revol );
    zVec3DCreate( rkChainLinkOrgPos(chain,i*3+2), 0, 1, 0 );
    zVec3DCreate( rkChainLinkOrgPos(chain,i*3+3), 0, 1, 0 );
    rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,i*3+1) );
    rkLinkAddChild( rkChainLink(chain,i*3+1), rkChainLink(chain,i*3+2) );
    rkLinkAddChild( rkChainLink(chain,i*3+2), rkChainLink(chain,i*3+3) );
  }
  for( i=0; i<10; i++ )
    zFrame3DCopy( rkChainLinkOrgFrame(chain,i), rkChainLinkAdjFrame(chain,i) );
  rkChainSetMass( chain, 1.0 ); /* dummy weight */
  rkChainSetJointIDOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  rkChainWriteZTK( chain, "trident.ztk" );
}

#define DIV 200

int main(int argc, char *argv[])
{
  rkChain chain;
  rkIKAttr attr;
  zVec dis;
  zVec6D err;
  rkIKCell *cell[6];
  int i, j;

  chain_init( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainFK( &chain, dis );

  rkChainCreateIK( &chain );
  rkChainRegIKJointAll( &chain, 0.01 );

  for( i=0; i<3; i++ ){
    attr.id = i*3+3;
    attr.mode = RK_IK_CELL_FORCE;
    cell[i*2]   = rkChainRegIKCellWldPos( &chain, NULL, &attr, RK_IK_ATTR_ID | RK_IK_ATTR_FORCE );
    cell[i*2+1] = rkChainRegIKCellWldAtt( &chain, NULL, &attr, RK_IK_ATTR_ID | RK_IK_ATTR_FORCE );
  }
  rkChainDisableIK( &chain );
  rkChainBindIK( &chain );

  zVec3DCreate( &cell[0]->data.ref.pos, 1,  1, 0 );
  zVec3DCreate( &cell[2]->data.ref.pos, 1, -1, 0 );
#if 0 /* strong weight */
  rkIKCellSetWeight( cell[4], 0.1, 0.1, 0.1 );
#else
  rkIKCellForce( cell[0] );
  rkIKCellForce( cell[1] );
  rkIKCellForce( cell[2] );
  rkIKCellForce( cell[3] );
#endif

  for( i=0; i<=DIV; i++ ){
    zVec3DCreate( &cell[4]->data.ref.pos,-4.0*(double)i/DIV,  0, 0 );
    rkChainIK( &chain, dis, zTOL, 0 );
    rkChainFK( &chain, dis );
    for( j=0; j<3; j++ ){
      zVec3DSub( &cell[j*2]->data.ref.pos, zFrame3DPos(rkChainLinkWldFrame(&chain,j*3+3)), zVec6DLin(&err) );
      zMat3DError( &cell[j*2+1]->data.ref.att, rkChainLinkWldAtt(&chain,j*3+3), zVec6DAng(&err) );
      printf( "%g %g ", zVec3DNorm(zVec6DLin(&err)), zVec3DNorm(zVec6DAng(&err)) );
    }
    zEndl();
  }
  rkChainDestroy( &chain );
  zVecFree( dis );
  return 0;
}
