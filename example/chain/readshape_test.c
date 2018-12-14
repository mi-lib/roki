#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  char filename[BUFSIZ];

  if( argc == 1 ){
    eprintf( ".z3d file unspecified.\n" );
    return 1;
  }
  rkChainMShape3DReadFile( &chain, argv[1] );
  zReplaceSuffix( argv[1], RK_CHAIN_SUFFIX, filename, BUFSIZ );
  rkChainWriteFile( &chain, filename );
  rkChainDestroy( &chain );
  return 0;
}
