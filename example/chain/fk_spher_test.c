#include <roki/rk_chain.h>

zVec3D p = { { 1.0, 2.0, 3.0 } };
rkLink *le;

void truth(zVec3D *aa1, zVec3D *w1, zVec3D *a1, zVec3D *aa2, zVec3D *w2, zVec3D *a2, zFrame3D *f, zVec6D *v, zVec6D *a)
{
  zMat3D m1, m2;
  zVec3D wp, ww, tmp1, tmp2;

  /* frame */
  zMat3DFromAA( &m1, aa1 );
  zMat3DFromAA( &m2, aa2 );
  zMulMat3DMat3D( &m1, &m2, zFrame3DAtt(f) );
  zMulMat3DVec3D( &m1, &p, zFrame3DPos(f) );
  /* lin. veloc. */
  zVec3DOuterProd( w1, zFrame3DPos(f), &wp );
  zMulMat3DTVec3D( zFrame3DAtt(f), &wp, zVec6DLin(v) );
  /* ang. veloc. */
  zMulMat3DVec3D( &m1, w2, &ww );
  zVec3DAdd( w1, &ww, &tmp1 );
  zMulMat3DTVec3D( zFrame3DAtt(f), &tmp1, zVec6DAng(v) );
  /* lin. accel. */
  zVec3DOuterProd( a1, zFrame3DPos(f), &tmp1 );
  zVec3DOuterProd( w1, &wp, &tmp2 );
  zVec3DAddDRC( &tmp1, &tmp2 );
  zMulMat3DTVec3D( zFrame3DAtt(f), &tmp1, zVec6DLin(a) );
  zMulMat3DTVec3D( zFrame3DAtt(f), RK_GRAVITY3D, &tmp1 );
  zVec3DAddDRC( zVec6DLin(a), &tmp1 );
  /* ang. accel. */
  zVec3DOuterProd( w1, &ww, &tmp1 );
  zVec3DAddDRC( &tmp1, a1 );
  zMulMat3DVec3D( &m1, a2, &tmp2 );
  zVec3DAddDRC( &tmp1, &tmp2 );
  zMulMat3DTVec3D( zFrame3DAtt(f), &tmp1, zVec6DAng(a) );
}

void create_sphere(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "sphere1" );
  zArrayAlloc( &chain->link, rkLink, 2 );
  /* link 1 */
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "link1" );
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_SPHER );
  /* link 2 */
  rkLinkInit( rkChainLink(chain,1) );
  zNameSet( rkChainLink(chain,1), "link2" );
  rkJointCreate( rkChainLinkJoint(chain,1), RK_JOINT_SPHER );
  zVec3DCopy( &p, rkChainLinkOrgPos(chain,1) );
  /* connect */
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );

  rkChainSetMass( chain, 1.0 );
  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  le = rkChainLink(chain,1);
}

int main(void)
{
  rkChain chain;
  zVec dis, vel, acc;
  zFrame3D f;
  zVec3D aa1, aa2, w1, w2, a1, a2;
  zVec6D v, a, err;

  /* create chain */
  create_sphere( &chain );
  /* create joint configuration */
  zRandInit();
  dis = zVecAlloc( rkChainJointSize(&chain) );
  vel = zVecAlloc( rkChainJointSize(&chain) );
  acc = zVecAlloc( rkChainJointSize(&chain) );
  zVecRandUniform( dis, -0.5*zPI, 0.5*zPI );
  zVecRandUniform( vel, -10, 10 );
  zVecRandUniform( acc, -100, 100 );

  zVec3DCreate( &aa1, zVecElem(dis,0), zVecElem(dis,1), zVecElem(dis,2) );
  zVec3DCreate( &aa2, zVecElem(dis,3), zVecElem(dis,4), zVecElem(dis,5) );
  zVec3DCreate( &w1, zVecElem(vel,0), zVecElem(vel,1), zVecElem(vel,2) );
  zVec3DCreate( &w2, zVecElem(vel,3), zVecElem(vel,4), zVecElem(vel,5) );
  zVec3DCreate( &a1, zVecElem(acc,0), zVecElem(acc,1), zVecElem(acc,2) );
  zVec3DCreate( &a2, zVecElem(acc,3), zVecElem(acc,4), zVecElem(acc,5) );

  /* FK test */
  rkChainFK( &chain, dis );
  rkChainID( &chain, vel, acc );
  truth( &aa1, &w1, &a1, &aa2, &w2, &a2, &f, &v, &a );

  /* output */
  printf( ">> frame test\n" );
  printf( " spherical joint ..." );
  zFrame3DPrint( rkLinkWldFrame(le) );
  printf( " answer ..." );
  zFrame3DPrint( &f );
  printf( " (error) ...\n" );
  zFrame3DError( &f, rkLinkWldFrame(le), &err );
  zVec6DPrint( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );

  printf( ">> velocity test\n" );
  printf( " spherical joint ...\n" );
  zVec6DPrint( rkLinkVel(le) );
  printf( " answer ...\n" );
  zVec6DPrint( &v );
  printf( " (error) ...\n" );
  zVec6DSub( &v, rkLinkVel(le), &err );
  zVec6DPrint( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );

  printf( ">> acceleration test\n" );
  printf( " spherical joint ...\n" );
  zVec6DPrint( rkLinkAcc(le) );
  printf( " answer ...\n" );
  zVec6DPrint( &a );
  printf( " (error) ...\n" );
  zVec6DSub( &a, rkLinkAcc(le), &err );
  zVec6DPrint( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );

  /* terminate */
  zVecFree( dis );
  zVecFree( vel );
  zVecFree( acc );
  rkChainDestroy( &chain );
  return 0;
}
