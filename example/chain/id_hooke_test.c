#include <roki/rk_chain.h>

rkLink *le1, *le2;

void create_hooke1(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "hooke1" );
  zArrayAlloc( &chain->link, rkLink, 1 );
  /* link 1 */
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "link" );
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_hooke );

  rkChainSetMass( chain, 1.0 );
  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  le1 = rkChainLink(chain,0);
}

void create_hooke2(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "hooke2" );
  zArrayAlloc( &chain->link, rkLink, 3 );
  /* link 1 */
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "link1" );
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_revol );
  /* link 2 */
  rkLinkInit( rkChainLink(chain,1) );
  zNameSet( rkChainLink(chain,1), "link2" );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_revol );
  zMat3DCreate( rkChainLinkOrgAtt(chain,1), 0, 1, 0, 0, 0, 1, 1, 0, 0 );
  /* link 3 */
  rkLinkInit( rkChainLink(chain,2) );
  zNameSet( rkChainLink(chain,2), "link3(aligner)" );
  rkJointAssign( rkChainLinkJoint(chain,2), &rk_joint_fixed );
  zMat3DCreate( rkChainLinkOrgAtt(chain,2), 0, 0, 1, 1, 0, 0, 0, 1, 0 );
  /* connect links */
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );
  rkLinkAddChild( rkChainLink(chain,1), rkChainLink(chain,2) );

  rkChainSetMass( chain, 1.0 );
  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  le2 = rkChainLink(chain,2);
}

int main(void)
{
  rkChain c1, c2;
  zVec dis;
  zVec6D w, wj, err;
  double u1[2], u2[2];

  /* create chain */
  create_hooke1( &c1 );
  create_hooke2( &c2 );
  /* create joint configuration */
  zRandInit();
  dis = zVecAlloc( 2 );
  zVecRandUniform( dis, -zPI, zPI );
  /* create wrench */
  zVec6DCreate( &w, zRandF(-1,1), zRandF(-1,1), zRandF(-1,1), zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );

  /* FK test */
  rkChainFK( &c1, dis );
  rkChainFK( &c2, dis );

  /* output */
  printf( ">> frame test\n" );
  printf( " Hooke joint ..." );
  zFrame3DPrint( rkLinkWldFrame(le1) );
  printf( " Successive revolute joints ... " );
  zFrame3DPrint( rkLinkWldFrame(le2) );
  printf( " (error) ...\n" );
  zFrame3DError( rkLinkWldFrame(le1), rkLinkWldFrame(le2), &err );
  zVec6DPrint( &err );
  printf( "\n" );

  /* torque */
  zMulMat3DTVec6D( rkChainLinkWldAtt(&c1,0), &w, &wj );
  rkJointCalcTrq( rkChainLinkJoint(&c1,0), &wj );
  rkJointGetTrq( rkChainLinkJoint(&c1,0), u1 );
  printf( "trq(r1) = %.16g %.16g\n", u1[0], u1[1] );
  zMulMat3DTVec6D( rkChainLinkWldAtt(&c2,0), &w, &wj );
  rkJointCalcTrq( rkChainLinkJoint(&c2,0), &wj );
  rkJointGetTrq( rkChainLinkJoint(&c2,0), &u2[0] );
  zMulMat3DTVec6D( rkChainLinkWldAtt(&c2,1), &w, &wj );
  rkJointCalcTrq( rkChainLinkJoint(&c2,1), &wj );
  rkJointGetTrq( rkChainLinkJoint(&c2,1), &u2[1] );
  printf( "trq(r2) = %.16g %.16g\n", u2[0], u2[1] );
  eprintf( "%s.\n", zIsTiny( u1[0] - u2[0] ) && zIsTiny( u1[1] - u2[1] ) ? "success" : "failure" );

  /* terminate */
  zVecFree( dis );
  rkChainDestroy( &c1 );
  rkChainDestroy( &c2 );
  return 0;
}
