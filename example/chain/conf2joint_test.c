#include <roki/rk_chain.h>

int main(void)
{
  rkChain chain;
  zVec orgdis, orgconf, dis, conf;

  zRandInit();
  rkChainReadFile( &chain, "../model/arm.zkc" );
  orgdis = zVecAlloc( rkChainJointSize( &chain ) );
  dis = zVecAlloc( rkChainJointSize( &chain ) );
  orgconf = zVecAlloc( rkChainNum( &chain ) * 6 );
  conf = zVecAlloc( rkChainNum( &chain ) * 6 );

  /* displacement -> configuration -> displacement */
  zVecRandUniform( orgdis, -1.0, 1.0 );
  rkChainFK( &chain, orgdis );
  rkChainGetConf( &chain, conf );

  rkChainSetConf( &chain, conf );
  rkChainGetJointDisAll( &chain, dis );

  zVecSubDRC( dis, orgdis );

  printf( "error norm (dis) = %g\n", zVecNorm( dis ) );
  printf( "displacement -> configuration -> displacement test ... %s\n", zVecIsTiny(dis) ? "ok." : "failure." );

  /* configuration -> displacement -> configuration */
  zVecRandUniform( orgconf, -1.0, 1.0 );
  rkChainSetConf( &chain, orgconf );
  rkChainGetJointDisAll( &chain, orgdis );

  rkChainFK( &chain, orgdis );
  rkChainGetConf( &chain, conf );
  rkChainSetConf( &chain, conf );
  rkChainGetJointDisAll( &chain, dis );

  zVecSubDRC( conf, orgconf );
  zVecSubDRC( dis, orgdis );
  printf( "error norm (conf) = %g\n", zVecNorm( conf ) );
  printf( "error norm (dis) = %g\n", zVecNorm( dis ) );
  printf( "configuration -> displacement -> configuration test ... %s\n\n", zVecIsTiny(dis) ? "ok." : "failure." );

  zVecFree( orgconf );
  zVecFree( conf );
  zVecFree( orgdis );
  zVecFree( dis );
  rkChainDestroy( &chain );
  return 0;
}
