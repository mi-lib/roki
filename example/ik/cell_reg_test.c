#include <roki/rk_ik.h>

#define N 7

void chain_init(rkChain *chain)
{
  register int i;
  char name[BUFSIZ];

  rkChainInit( chain );
  zArrayAlloc( &chain->link, rkLink, N );
  for( i=0; i<N; i++ ){
    sprintf( name, "link#%02d", i );
    rkLinkInit( rkChainLink(chain,i) );
    zNameSet( rkChainLink(chain,i), name );
    if( i > 0 )
      rkLinkAddChild( rkChainLink(chain,i-1), rkChainLink(chain,i) );
  }
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_FIXED );
  rkJointCreate( rkChainLinkJoint(chain,1), RK_JOINT_SPHER );
  rkJointCreate( rkChainLinkJoint(chain,2), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,3), RK_JOINT_CYLIN );
  rkJointCreate( rkChainLinkJoint(chain,4), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,5), RK_JOINT_SPHER );
  rkJointCreate( rkChainLinkJoint(chain,6), RK_JOINT_FIXED );
  rkChainSetOffset( chain );
}

rkIKCell *cell_reg(rkIK *ik, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKSRV_fp vf, rkIKBind_fp bf)
{
  rkIKCell *cell;

  cell = rkIKCellReg( ik, NULL, RK_IK_CELL_ATTR_NONE, rf, mf, vf, bf, NULL, NULL );
  printf( "registered cell #%d\n", cell->data.id );
  printf( "workspace for coefficient matrix = (%dx%d).\n", _zMatRowSize(ik->_c_mat), _zMatColSize(ik->_c_mat) );
  printf( "workspace for SRV vector = (%d).\n", _zVecSize(ik->_c_srv) );
  printf( "workspace for residual error vector = (%d).\n", _zVecSize(ik->_c_we) );
  return cell;
}

void cell_unreg(rkIK *ik, rkIKCell *cell)
{
  rkIKCellUnreg( ik, cell );
  printf( "unregistered cell #%d\n", cell->data.id );
  printf( "workspace for coefficient matrix = (%dx%d).\n", _zMatRowSize(ik->_c_mat), _zMatColSize(ik->_c_mat) );
  printf( "workspace for SRV vector = (%d).\n", _zVecSize(ik->_c_srv) );
  printf( "workspace for residual error vector = (%d).\n", _zVecSize(ik->_c_we) );
}

int main(void)
{
  rkChain r;
  rkIK ik;
  rkIKCell *cell[5];

  chain_init( &r );
  rkChainConnectionWrite( &r );
  rkIKCreate( &ik, &r );

  rkIKJointReg( &ik, 0, 100 );
  rkIKJointReg( &ik, 2,  10 );
  rkIKJointReg( &ik, 3,   5 );
  rkIKJointReg( &ik, 5,  20 );

  cell[0] = cell_reg( &ik, rkIKRefSetPos, rkIKJacobiLinkWldLin, rkIKLinkWldPosErr, rkIKBindLinkWldPos );
  cell[1] = cell_reg( &ik, rkIKRefSetZYX, rkIKJacobiLinkWldAng, rkIKLinkWldAttErr, rkIKBindLinkWldAtt );
  cell[2] = cell_reg( &ik, rkIKRefSetPos, rkIKJacobiLinkWldLin, rkIKLinkWldPosErr, rkIKBindLinkWldPos );
  cell_unreg( &ik, cell[1] );
  cell[3] = cell_reg( &ik, rkIKRefSetPos, rkIKJacobiLinkWldLin, rkIKLinkWldPosErr, rkIKBindLinkWldPos );
  cell_unreg( &ik, cell[0] );
  cell[4] = cell_reg( &ik, rkIKRefSetPos, rkIKJacobiLinkWldLin, rkIKLinkWldPosErr, rkIKBindLinkWldPos );
  rkIKDeactivate( &ik );

  zIndexWrite( ik._j_idx );
  zIndexWrite( ik._j_ofs );
  zVecWrite( ik._j_wn );

  rkIKDestroy( &ik );
  rkChainDestroy( &r );
  return 0;
}
