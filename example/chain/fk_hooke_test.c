#include <roki/rk_chain.h>

zVec3D p = { { 1.0, 2.0, 3.0 } };
rkLink *le1, *le2;

void truth(zVec dis, zVec vel, zVec acc, zFrame3D *f, zVec6D *v, zVec6D *a)
{
  double s1, c1, s2, c2, dq1, dq2, ddq1, ddq2;
  zVec3D w, dw, tmp;

  zSinCos( zVecElem(dis,0), &s1, &c1 );
  zSinCos( zVecElem(dis,1), &s2, &c2 );
  /* frame */
  zMat3DCreate( zFrame3DAtt(f),
    c1*c2,-s1, c1*s2,
    s1*c2, c1, s1*s2,
      -s2,  0,    c2 );
  zMulMatVec3D( zFrame3DAtt(f), &p, zFrame3DPos(f) );
  /* velocity */
  dq1 = zVecElem(vel,0);
  dq2 = zVecElem(vel,1);
  zVec3DCreate( &w,-dq2*s1, dq2*c1, dq1 );
  zMulMatTVec3D( zFrame3DAtt(f), &w, zVec6DAng(v) );
  zVec3DOuterProd( zVec6DAng(v), &p, zVec6DLin(v) );
  /* acceleration */
  ddq1 = zVecElem(acc,0);
  ddq2 = zVecElem(acc,1);
  zVec3DCreate( &dw, -ddq2*s1-dq1*dq2*c1, ddq2*c1-dq1*dq2*s1, ddq1 );
  zMulMatTVec3D( zFrame3DAtt(f), &dw, zVec6DAng(a) );

  zVec3DOuterProd( zVec6DAng(v), zVec6DLin(v), zVec6DLin(a) );
  zVec3DOuterProd( zVec6DAng(a), &p, &tmp );
  zVec3DAddDRC( zVec6DLin(a), &tmp );
  zMulMatTVec3D( zFrame3DAtt(f), RK_GRAVITY3D, &tmp );
  zVec3DAddDRC( zVec6DLin(a), &tmp );
}

void create_hooke1(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "hooke1" );
  zArrayAlloc( &chain->link, rkLink, 2 );
  /* link 1 */
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "link" );
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_HOOKE );
  /* link E */
  rkLinkInit( rkChainLink(chain,1) );
  zNameSet( rkChainLink(chain,1), "endpoint" );
  rkJointCreate( rkChainLinkJoint(chain,1), RK_JOINT_FIXED );
  zVec3DCopy( &p, rkChainLinkOrgPos(chain,1) );
  /* connect links */
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );

  rkChainSetMass( chain, 1.0 );
  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  le1 = rkChainLink(chain,1);
}

void create_hooke2(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "hooke2" );
  zArrayAlloc( &chain->link, rkLink, 4 );
  /* link 1 */
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "link1" );
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_REVOL );
  /* link 2 */
  rkLinkInit( rkChainLink(chain,1) );
  zNameSet( rkChainLink(chain,1), "link2" );
  rkJointCreate( rkChainLinkJoint(chain,1), RK_JOINT_REVOL );
  zMat3DCreate( rkChainLinkOrgAtt(chain,1), 0, 1, 0, 0, 0, 1, 1, 0, 0 );
  /* link 3 */
  rkLinkInit( rkChainLink(chain,2) );
  zNameSet( rkChainLink(chain,2), "link3(aligner)" );
  rkJointCreate( rkChainLinkJoint(chain,2), RK_JOINT_FIXED );
  zMat3DCreate( rkChainLinkOrgAtt(chain,2), 0, 0, 1, 1, 0, 0, 0, 1, 0 );
  /* link E */
  rkLinkInit( rkChainLink(chain,3) );
  zNameSet( rkChainLink(chain,3), "endpoint" );
  rkJointCreate( rkChainLinkJoint(chain,3), RK_JOINT_FIXED );
  zVec3DCopy( &p, rkChainLinkOrgPos(chain,3) );
  /* connect links */
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );
  rkLinkAddChild( rkChainLink(chain,1), rkChainLink(chain,2) );
  rkLinkAddChild( rkChainLink(chain,2), rkChainLink(chain,3) );

  rkChainSetMass( chain, 1.0 );
  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  le2 = rkChainLink(chain,3);
}

int main(void)
{
  rkChain chain1, chain2;
  zVec dis, vel, acc;
  zFrame3D f;
  zVec6D v, a, err;

  /* create chain */
  create_hooke1( &chain1 );
  create_hooke2( &chain2 );
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
  truth( dis, vel, acc, &f, &v, &a );

  /* output */
  printf( ">> frame test\n" );
  printf( " True answer ..." );
  zFrame3DWrite( &f );
  printf( " Hooke joint ..." );
  zFrame3DWrite( rkLinkWldFrame(le1) );
  printf( " (error) ...\n" );
  zFrame3DError( &f, rkLinkWldFrame(le1), &err );
  zVec6DWrite( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );
  printf( " Successive revolute joints ... " );
  zFrame3DWrite( rkLinkWldFrame(le2) );
  printf( " (error) ...\n" );
  zFrame3DError( &f, rkLinkWldFrame(le2), &err );
  zVec6DWrite( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );
  printf( "Hit enter key." );
  getchar();

  printf( ">> velocity test\n" );
  printf( " True answer ...\n" );
  zVec6DWrite( &v );
  printf( " Hooke joint ...\n" );
  zVec6DWrite( rkLinkVel(le1) );
  printf( " (error) ...\n" );
  zVec6DSub( &v, rkLinkVel(le1), &err );
  zVec6DWrite( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );
  printf( " Successive revolute joints ... " );
  zVec6DWrite( rkLinkVel(le2) );
  printf( " (error) ...\n" );
  zVec6DSub( &v, rkLinkVel(le2), &err );
  zVec6DWrite( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );
  printf( "Hit enter key." );
  getchar();

  printf( ">> acceleration test\n" );
  printf( " True answer ...\n" );
  zVec6DWrite( &a );
  printf( " Hooke joint ...\n" );
  zVec6DWrite( rkLinkAcc(le1) );
  printf( " (error) ...\n" );
  zVec6DSub( &a, rkLinkAcc(le1), &err );
  zVec6DWrite( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );
  printf( " Successive revolute joints ... " );
  zVec6DWrite( rkLinkAcc(le2) );
  printf( " (error) ...\n" );
  zVec6DSub( &a, rkLinkAcc(le2), &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );
  zVec6DWrite( &err );

  /* terminate */
  zVecFree( dis );
  zVecFree( vel );
  zVecFree( acc );
  rkChainDestroy( &chain1 );
  rkChainDestroy( &chain2 );
  return 0;
}
