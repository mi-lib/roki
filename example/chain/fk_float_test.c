#include <roki/rk_chain.h>

zVec3D p = { { 1.0, 2.0, 3.0 } };
rkLink *le;

void truth(zMat3D *morg, zVec3D *pos, zVec3D *aa, zVec6D *vin, zVec6D *ain, zFrame3D *fout, zVec6D *vout, zVec6D *aout)
{
  zMat3D m;
  zVec3D dp, wp, tmp;

  /* frame */
  zMat3DFromAA( &m, aa );
  zMulMat3DMat3D( morg, &m, zFrame3DAtt(fout) );
  zMulMat3DVec3D( zFrame3DAtt(fout), &p, &dp );
  zMulMat3DVec3D( morg, pos, zFrame3DPos(fout) );
  zVec3DAddDRC( zFrame3DPos(fout), &dp );
  /* veloc. */
  zMulMat3DVec3D( &m, &p, &dp );
  zVec3DOuterProd( zVec6DAng(vin), &dp, &wp );
  zVec3DAdd( zVec6DLin(vin), &wp, zVec6DLin(vout) );
  zVec3DCopy( zVec6DAng(vin), zVec6DAng(vout) );
  zMulMat3DTVec6DDRC( &m, vout );
  /* accel. */
  zVec3DOuterProd( zVec6DAng(vin), &wp, zVec6DLin(aout) );
  zVec3DOuterProd( zVec6DAng(ain), &dp, &tmp );
  zVec3DAddDRC( zVec6DLin(aout), &tmp );
  zVec3DAddDRC( zVec6DLin(aout), zVec6DLin(ain) );
  zVec3DCopy( zVec6DAng(ain), zVec6DAng(aout) );
  zMulMat3DTVec6DDRC( &m, aout );
  zMulMat3DTVec3D( zFrame3DAtt(fout), RK_GRAVITY3D, &tmp );
  zVec3DAddDRC( zVec6DLin(aout), &tmp );
}

void create_float(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "float1" );
  zArrayAlloc( &chain->link, rkLink, 2 );
  /* link 1 */
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "link1" );
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_float );
  zMat3DCreate( rkChainLinkOrgAtt(chain,0),
    0, 0, 1,
    0, 1, 0,
   -1, 0, 0 );
  /* link 2 */
  rkLinkInit( rkChainLink(chain,1) );
  zNameSet( rkChainLink(chain,1), "link2" );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_fixed );
  zVec3DCopy( &p, rkChainLinkOrgPos(chain,1) );
  /* connect */
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );

  rkChainSetMass( chain, 1.0 );
  rkChainSetJointIDOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  le = rkChainLink(chain,1);
}

int main(void)
{
  rkChain chain;
  zVec dis, vel, acc;
  zVec3D pos, aa;
  zVec6D vin, ain, vout, aout, err;
  zFrame3D fout;

  /* create chain */
  create_float( &chain );
  /* create joint configuration */
  zRandInit();
  dis = zVecAlloc( rkChainJointSize(&chain) );
  vel = zVecAlloc( rkChainJointSize(&chain) );
  acc = zVecAlloc( rkChainJointSize(&chain) );
  zVecRandUniform( dis, -0.5*zPI, 0.5*zPI );
  zVecRandUniform( vel, -10, 10 );
  zVecRandUniform( acc, -100, 100 );

  zVec3DCreate( &pos, zVecElem(dis,0), zVecElem(dis,1), zVecElem(dis,2) );
  zVec3DCreate( &aa,  zVecElem(dis,3), zVecElem(dis,4), zVecElem(dis,5) );
  zVec6DCreate( &vin,
    zVecElem(vel,0), zVecElem(vel,1), zVecElem(vel,2),
    zVecElem(vel,3), zVecElem(vel,4), zVecElem(vel,5) );
  zVec6DCreate( &ain,
    zVecElem(acc,0), zVecElem(acc,1), zVecElem(acc,2),
    zVecElem(acc,3), zVecElem(acc,4), zVecElem(acc,5) );

  /* FK test */
  rkChainFK( &chain, dis );
  rkChainID( &chain, vel, acc );
  truth( rkChainLinkOrgAtt(&chain,0), &pos, &aa, &vin, &ain, &fout, &vout, &aout );

  /* output */
  printf( ">> frame test\n" );
  printf( " float joint ..." );
  zFrame3DPrint( rkLinkWldFrame(le) );
  printf( " answer ..." );
  zFrame3DPrint( &fout );
  printf( " (error) ...\n" );
  zFrame3DError( &fout, rkLinkWldFrame(le), &err );
  zVec6DPrint( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );

  printf( ">> velocity test\n" );
  printf( " float joint ...\n" );
  zVec6DPrint( rkLinkVel(le) );
  printf( " answer ...\n" );
  zVec6DPrint( &vout );
  printf( " (error) ...\n" );
  zVec6DSub( &vout, rkLinkVel(le), &err );
  zVec6DPrint( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );

  printf( ">> acceleration test\n" );
  printf( " float joint ...\n" );
  zVec6DPrint( rkLinkAcc(le) );
  printf( " answer ...\n" );
  zVec6DPrint( &aout );
  printf( " (error) ...\n" );
  zVec6DSub( &aout, rkLinkAcc(le), &err );
  zVec6DPrint( &err );
  printf( " ...%s.\n\n", zVec6DIsTiny(&err) ? "OK" : "may be a bug" );

  /* terminate */
  zVecFree( dis );
  zVecFree( vel );
  zVecFree( acc );
  rkChainDestroy( &chain );
  return 0;
}
