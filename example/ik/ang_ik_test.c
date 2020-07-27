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
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,2), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,3), &rk_joint_fixed );
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

  rkIKSolve( &ik, dis, zTOL, 0 );
  rkChainFK( ik.chain, dis );
  zMat3DError( &cell->data.ref.att, rkChainLinkWldAtt(ik.chain,3), &err );
  zVec3DPrint( &err );
  eprintf( "%s.\n", zVec3DIsTiny( &err ) ? "success" : "failure" );

  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  zVecFree( dis );
  return 0;
}
