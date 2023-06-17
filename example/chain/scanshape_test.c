#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  char filename[BUFSIZ];

  if( argc == 1 ){
    eprintf( ".ztk file unspecified.\n" );
    return 1;
  }
  rkChainReadZTK( &chain, argv[1] );
  zReplaceSuffix( argv[1], ZEDA_ZTK_SUFFIX, filename, BUFSIZ );
  rkChainWriteZTK( &chain, filename );
  rkChainDestroy( &chain );
  return 0;
}
