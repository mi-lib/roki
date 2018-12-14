#include <roki/rk_ik.h>

void chain_init(rkChain *chain)
{
  register int i;
  char name[BUFSIZ];

  rkChainInit( chain );
  zArrayAlloc( &chain->link, rkLink, 4 );
  for( i=0; i<4; i++ ){
    sprintf( name, "link#%02d", i );
    rkLinkInit( rkChainLink(chain,i) );
    zNameSet( rkChainLink(chain,i), name );
    if( i > 0 )
      rkLinkAddChild( rkChainLink(chain,i-1), rkChainLink(chain,i) );
  }
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,1), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,2), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,3), RK_JOINT_FIXED );
  zMat3DCreate( rkChainLinkOrgAtt(chain,1), 1, 0, 0, 0, 0, 1, 0,-1, 0 );
  zMat3DCreate( rkChainLinkOrgAtt(chain,2), 0, 0, 1,-1, 0, 0, 0,-1, 0 );
  zMat3DCreate( rkChainLinkOrgAtt(chain,3), 0, 0, 1, 0,-1, 0, 1, 0, 0 );
  for( i=0; i<4; i++ )
    zFrame3DCopy( rkChainLinkOrgFrame(chain,i), rkChainLinkAdjFrame(chain,i) );
  rkChainSetMass( chain, 1.0 ); /* dummy weight */
  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
}

int main(int argc, char *argv[])
{
  rkChain chain;
  rkIK ik;
  rkIKCellAttr attr;
  zVec dis;
  zVec3D err;
  rkIKCell *cell;

  zRandInit();
  chain_init( &chain );
  dis = zVecAlloc( 3 );
  /*
  zVecSetElemList( dis, 0.0, 0.0, zDeg2Rad(d) );
  */

  rkIKCreate( &ik, &chain );
  rkIKJointReg( &ik, 0, 0.0 );
  rkIKJointReg( &ik, 1, 0.0 );
  rkIKJointReg( &ik, 2, 0.0 );

  attr.id = 3;
  cell = rkIKCellRegWldAtt( &ik, &attr, RK_IK_CELL_ATTR_ID );

  rkIKDeactivate( &ik );
  rkIKBind( &ik ); /* bind current status to the reference. */
  rkIKCellSetRef( cell,
    zDeg2Rad(zRandF(-180,180)), zDeg2Rad(zRandF( 0,180)), zDeg2Rad(zRandF(-180,180)) );

  printf( "++ initial attitude\n" );
  zMat3DWrite( rkChainLinkWldAtt(ik.chain,3) );
  rkIKSolve( &ik, dis, zTOL, 0 );
  zVecWrite( dis );
  rkChainFK( ik.chain, dis );
  printf( "++ goal attitude\n" );
  zMat3DWrite( &cell->data.ref.att );
  printf( "++ final attitude\n" );
  zMat3DWrite( rkChainLinkWldAtt(ik.chain,3) );
  printf( "++ error\n" );
  zMat3DError( &cell->data.ref.att, rkChainLinkWldAtt(ik.chain,3), &err );
  zVec3DWrite( &err );

  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  zVecFree( dis );
  return 0;
}
