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
  zVec3DCreate( rkChainLinkOrgPos(chain,1), 1, 0, 0 );
  zVec3DCreate( rkChainLinkOrgPos(chain,2), 1, 0, 0 );
  zVec3DCreate( rkChainLinkOrgPos(chain,3), 1, 0, 0 );
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
  zVec6D err;
  rkIKCell *cell[2];

  chain_init( &chain );
  dis = zVecCreateList( 3, zDeg2Rad(0.0), zDeg2Rad(0.0), zDeg2Rad(0.0) );
  rkChainFK( &chain, dis );

  rkIKCreate( &ik, &chain );
  rkIKJointReg( &ik, 0, 0.01 );
  rkIKJointReg( &ik, 1, 0.01 );
  rkIKJointReg( &ik, 2, 0.01 );

  attr.id = 3;
  zVec3DZero( &attr.ap );
  cell[0] = rkIKCellRegWldPos( &ik, &attr, RK_IK_CELL_ATTR_ID );
  cell[1] = rkIKCellRegWldAtt( &ik, &attr, RK_IK_CELL_ATTR_ID );

  rkIKDeactivate( &ik );
  rkIKBind( &ik ); /* bind current status to the reference. */
#if 0
  cell[0]->data.ref.pos.e[zX] -= 1.0;
  zMat3DRotYaw( ZMAT3DIDENT, zDeg2Rad(45), &cell[1]->data.ref.att );
#else
  zVec2DCreatePolar( (zVec2D*)&cell[0]->data.ref.pos, zRandF(0,2), zRandF(-zPI_2,zPI_2) );
  cell[0]->data.ref.pos.c.z = 0;
  zMat3DRotYaw( ZMAT3DIDENT, zRandF(-zPI_2,zPI_2), &cell[1]->data.ref.att );
#endif

  rkIKSolve( &ik, dis, zTOL, 0 );
  rkChainFK( ik.chain, dis );
  zVec3DSub( &cell[0]->data.ref.pos, zFrame3DPos(rkChainLinkWldFrame(ik.chain,3)), zVec6DLin(&err) );
  zMat3DError( &cell[1]->data.ref.att, rkChainLinkWldAtt(ik.chain,3), zVec6DAng(&err) );
  zVec6DPrint( &err );
  eprintf( "%s.\n", zVec6DIsTiny( &err ) ? "success" : "failure" );

  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  zVecFree( dis );
  return 0;
}
