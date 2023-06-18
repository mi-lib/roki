#include <roki/rk_cd.h>

int main(int argc, char *argv[])
{
  rkChain chain, chain2;
  rkCD cd;

  if( !rkChainReadZTK( &chain, argv[1] ) ){
    ZOPENERROR( argv[1] );
    return EXIT_FAILURE;
  }
  if( !rkChainReadZTK( &chain2, argv[2] ) ){
    ZOPENERROR( argv[2] );
    return EXIT_FAILURE;
  }

  rkCDCreate( &cd );
  rkCDChainReg( &cd, &chain, RK_CD_CELL_MOVE );
  rkCDChainReg( &cd, &chain2, RK_CD_CELL_MOVE );
  rkCDPairChainUnreg( &cd, &chain);
  rkCDColChkVert( &cd );
  rkCDPairPrint( &cd );

  rkChainDestroy( &chain );
  rkChainDestroy( &chain2 );
  rkCDDestroy( &cd );
  return 0;
}
