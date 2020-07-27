#include <roki/rk_ik.h>

#define TEST 0

void chain_init(rkChain *chain)
{
  char name[BUFSIZ];

  rkChainInit( chain );
  zArrayAlloc( &chain->link, rkLink, 3 );
  /* link #0 */
  sprintf( name, "link#0" );
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), name );
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_fixed );
  zMat3DCreate( rkChainLinkOrgAtt(chain,0), 1, 0, 0, 0, 0, 1, 0,-1, 0 );
  zFrame3DCopy( rkChainLinkOrgFrame(chain,0), rkChainLinkAdjFrame(chain,0) );
  /* link #1 */
  sprintf( name, "link#1" );
  rkLinkInit( rkChainLink(chain,1) );
  zNameSet( rkChainLink(chain,1), name );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_float );
  zFrame3DCopy( rkChainLinkOrgFrame(chain,1), rkChainLinkAdjFrame(chain,1) );
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );
  /* link #2 */
  sprintf( name, "link#2" );
  rkLinkInit( rkChainLink(chain,2) );
  zNameSet( rkChainLink(chain,2), name );
  rkJointAssign( rkChainLinkJoint(chain,2), &rk_joint_float );
  zFrame3DCopy( rkChainLinkOrgFrame(chain,2), rkChainLinkAdjFrame(chain,2) );
#if TEST == 1
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,2) );
#else
  rkLinkAddChild( rkChainLink(chain,1), rkChainLink(chain,2) );
#endif

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
  zVec6D err1, err2;
  rkIKCell *cl0, *ca0, *cl1, *ca1;

  zRandInit();
  chain_init( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainGetJointDisAll( &chain, dis );

  rkIKCreate( &ik, &chain );
  rkIKJointReg( &ik, 1, 0.001 );
  rkIKJointReg( &ik, 2, 0.001 );

  attr.id = 1;
  cl0 = rkIKCellRegWldPos( &ik, &attr, RK_IK_CELL_ATTR_ID );
  ca0 = rkIKCellRegWldAtt( &ik, &attr, RK_IK_CELL_ATTR_ID );
  attr.id = 2;
  cl1 = rkIKCellRegWldPos( &ik, &attr, RK_IK_CELL_ATTR_ID );
  ca1 = rkIKCellRegWldAtt( &ik, &attr, RK_IK_CELL_ATTR_ID );

  rkIKDeactivate( &ik );
  rkIKBind( &ik ); /* bind current status to the reference. */
  rkIKCellSetRef( cl0, 1.0, 2.0, 3.0 );
  rkIKCellSetRef( ca0, zDeg2Rad(30), zDeg2Rad(45), zDeg2Rad(-30) );
  rkIKCellSetRef( cl1, 4.0, 5.0, 6.0 );
  rkIKCellSetRef( ca1, zDeg2Rad(-30), zDeg2Rad(0), zDeg2Rad(45) );

  rkIKSolve( &ik, dis, zTOL, 0 );
  rkChainFK( ik.chain, dis );
  zVec3DSub( &cl0->data.ref.pos, rkChainLinkWldPos(ik.chain,1), zVec6DLin(&err1) );
  zMat3DError( &ca0->data.ref.att, rkChainLinkWldAtt(ik.chain,1), zVec6DAng(&err1) );
  zVec3DSub( &cl1->data.ref.pos, rkChainLinkWldPos(ik.chain,2), zVec6DLin(&err2) );
  zMat3DError( &ca1->data.ref.att, rkChainLinkWldAtt(ik.chain,2), zVec6DAng(&err2) );
  eprintf( "%s.\n", zVec6DIsTiny( &err1 ) && zVec6DIsTiny( &err2 ) ? "success" : "failure" );

  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  zVecFree( dis );
  return 0;
}
