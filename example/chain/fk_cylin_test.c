#include <roki/rk_chain.h>

void create_cylin1(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "cylindric1" );
  zArrayAlloc( &chain->link, rkLink, 2 );
  /* link 1 */
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "link" );
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_CYLIN );
  zMat3DCreate( rkChainLinkOrgAtt(chain,0), 0, 0, 1, 1, 0, 0, 0, 1, 0 );
  /* link E */
  rkLinkInit( rkChainLink(chain,1) );
  zNameSet( rkChainLink(chain,1), "endpoint" );
  rkJointCreate( rkChainLinkJoint(chain,1), RK_JOINT_FIXED );
  zVec3DCreate( rkChainLinkOrgPos(chain,1), 1, 2, 3 );
  /* connect links */
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );

  rkChainSetMass( chain, 1.0 );
  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
}

void create_cylin2(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "cylindric2" );
  zArrayAlloc( &chain->link, rkLink, 3 );
  /* link 1 */
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "link1" );
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_PRISM );
  zMat3DCreate( rkChainLinkOrgAtt(chain,0), 0, 0, 1, 1, 0, 0, 0, 1, 0 );
  /* link 2 */
  rkLinkInit( rkChainLink(chain,1) );
  zNameSet( rkChainLink(chain,1), "link2" );
  rkJointCreate( rkChainLinkJoint(chain,1), RK_JOINT_REVOL );
  /* link E */
  rkLinkInit( rkChainLink(chain,2) );
  zNameSet( rkChainLink(chain,2), "endpoint" );
  rkJointCreate( rkChainLinkJoint(chain,2), RK_JOINT_FIXED );
  zVec3DCreate( rkChainLinkOrgPos(chain,2), 1, 2, 3 );
  /* connect links */
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );
  rkLinkAddChild( rkChainLink(chain,1), rkChainLink(chain,2) );

  rkChainSetMass( chain, 1.0 );
  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
}

int main(void)
{
  rkChain chain1, chain2;
  zVec dis, vel, acc;
  zVec6D err;

  /* create chain */
  create_cylin1( &chain1 );
  create_cylin2( &chain2 );
  /* create joint configuration */
  zRandInit();
  dis = zVecAlloc( 2 );
  vel = zVecAlloc( 2 );
  acc = zVecAlloc( 2 );
  zVecRandUniform( dis, -zPI, zPI );
  zVecRandUniform( vel,  -10,  10 );
  zVecRandUniform( acc, -100, 100 );
  /* FK test */
  rkChainFK( &chain1, dis );
  rkChainFK( &chain2, dis );
  rkChainID( &chain1, vel, acc );
  rkChainID( &chain2, vel, acc );
  /* output */
  rkChainConnectionWrite( &chain1 );
  printf( " frame ... " );
  zFrame3DWrite( rkChainLinkWldFrame(&chain1,1) );
  printf( " velocity ...\n" );
  zVec6DWrite( rkChainLinkVel(&chain1,1) );
  printf( " acceleration ...\n" );
  zVec6DWrite( rkChainLinkAcc(&chain1,1) );

  printf( "\n" );
  rkChainConnectionWrite( &chain2 );
  printf( " frame ... " );
  zFrame3DWrite( rkChainLinkWldFrame(&chain2,2) );
  printf( " velocity ...\n" );
  zVec6DWrite( rkChainLinkVel(&chain2,2) );
  printf( " acceleration ...\n" );
  zVec6DWrite( rkChainLinkAcc(&chain2,2) );

  printf( "\n>> evaluation <<\n" );
  printf( " frame error ...\n" );
  zFrame3DError( rkChainLinkWldFrame(&chain1,1), rkChainLinkWldFrame(&chain2,2), &err );
  zVec6DWrite( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );
  printf( " velocity error ...\n" );
  zVec6DSub( rkChainLinkVel(&chain1,1), rkChainLinkVel(&chain2,2), &err );
  zVec6DWrite( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );
  printf( " acceleration error ...\n" );
  zVec6DSub( rkChainLinkAcc(&chain1,1), rkChainLinkAcc(&chain2,2), &err );
  zVec6DWrite( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );

  /* terminate */
  zVecFree( dis );
  zVecFree( vel );
  zVecFree( acc );
  rkChainDestroy( &chain1 );
  rkChainDestroy( &chain2 );
  return 0;
}
