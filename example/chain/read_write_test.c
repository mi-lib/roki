#include <roki/roki.h>

int main(int argc, char *argv[])
{
  rkChain chain;

  if( argc < 2 ) return EXIT_FAILURE;
  rkChainReadZTK( &chain, argv[1] );
  rkChainWriteZTK( &chain, "copy.ztk" );
  rkChainDestroy( &chain );
  return EXIT_SUCCESS;
}
