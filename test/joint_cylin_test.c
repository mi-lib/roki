#include <roki/rk_chain.h>

void create_cylin1(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "cylindric1" );
  rkLinkArrayAlloc( rkChainLinkArray(chain), 2 );
  /* link 1 */
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "link" );
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_cylin );
  zMat3DCreate( rkChainLinkOrgAtt(chain,0), 0, 0, 1, 1, 0, 0, 0, 1, 0 );
  /* link E */
  rkLinkInit( rkChainLink(chain,1) );
  zNameSet( rkChainLink(chain,1), "endpoint" );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_fixed );
  zVec3DCreate( rkChainLinkOrgPos(chain,1), 1, 2, 3 );
  /* connect links */
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );

  rkChainSetMass( chain, 1.0 );
  rkChainSetJointIDOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
}

void create_cylin2(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "cylindric2" );
  rkLinkArrayAlloc( rkChainLinkArray(chain), 3 );
  /* link 1 */
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "link1" );
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_prism );
  zMat3DCreate( rkChainLinkOrgAtt(chain,0), 0, 0, 1, 1, 0, 0, 0, 1, 0 );
  /* link 2 */
  rkLinkInit( rkChainLink(chain,1) );
  zNameSet( rkChainLink(chain,1), "link2" );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_revol );
  /* link E */
  rkLinkInit( rkChainLink(chain,2) );
  zNameSet( rkChainLink(chain,2), "endpoint" );
  rkJointAssign( rkChainLinkJoint(chain,2), &rk_joint_fixed );
  zVec3DCreate( rkChainLinkOrgPos(chain,2), 1, 2, 3 );
  /* connect links */
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );
  rkLinkAddChild( rkChainLink(chain,1), rkChainLink(chain,2) );

  rkChainSetMass( chain, 1.0 );
  rkChainSetJointIDOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
}

int main(void)
{
  rkChain chain1, chain2;
  zVec dis, vel, acc, trq;
  zVec6D w, wj, err;
  double u1[2], u2[2];

  /* create chain */
  create_cylin1( &chain1 );
  create_cylin2( &chain2 );
  /* create joint configuration */
  zRandInit();
  dis = zVecAlloc( 2 );
  vel = zVecAlloc( 2 );
  acc = zVecAlloc( 2 );
  trq = zVecAlloc( 2 );
  zVecRandUniform( dis, -zPI, zPI );
  zVecRandUniform( vel,  -10,  10 );
  zVecRandUniform( acc, -100, 100 );
  /* create wrench */
  zVec6DCreate( &w, zRandF(-1,1), zRandF(-1,1), zRandF(-1,1), zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );

  rkChainID( &chain1, dis, vel, acc, trq );
  rkChainID( &chain2, dis, vel, acc, trq );
  /* output */
  zFrame3DError( rkChainLinkWldFrame(&chain1,1), rkChainLinkWldFrame(&chain2,2), &err );
  zAssert( rkChainFK (cylindrical joint), zVec6DIsTiny(&err) );
  zVec6DSub( rkChainLinkVel(&chain1,1), rkChainLinkVel(&chain2,2), &err );
  zAssert( rkChainID (cylindrical joint velocity), zVec6DIsTiny(&err) );
  zVec6DSub( rkChainLinkAcc(&chain1,1), rkChainLinkAcc(&chain2,2), &err );
  zAssert( rkChainID (cylindrical joint acceleration), zVec6DIsTiny(&err) );
  /* torque */
  zMulMat3DTVec6D( rkChainLinkWldAtt(&chain1,0), &w, &wj );
  rkJointCalcTrq( rkChainLinkJoint(&chain1,0), &wj );
  rkJointGetTrq( rkChainLinkJoint(&chain1,0), u1 );
  zMulMat3DTVec6D( rkChainLinkWldAtt(&chain2,0), &w, &wj );
  rkJointCalcTrq( rkChainLinkJoint(&chain2,0), &wj );
  rkJointGetTrq( rkChainLinkJoint(&chain2,0), &u2[0] );
  zMulMat3DTVec6D( rkChainLinkWldAtt(&chain2,1), &w, &wj );
  rkJointCalcTrq( rkChainLinkJoint(&chain2,1), &wj );
  rkJointGetTrq( rkChainLinkJoint(&chain2,1), &u2[1] );
  zAssert( rkJointCalcTrq (cylindrical joint), zIsTiny( u1[0] - u2[0] ) && zIsTiny( u1[1] - u2[1] ) );

  /* terminate */
  zVecFreeAtOnce( 4, dis, vel, acc, trq );
  rkChainDestroy( &chain1 );
  rkChainDestroy( &chain2 );
  return 0;
}
