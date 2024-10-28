#include <roki/roki.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  char filename[BUFSIZ];

  if( argc < 2 ) return EXIT_FAILURE;
  rkChainReadURDF( &chain, argv[1] );
  zReplaceSuffix( argv[1], "ztk", filename, BUFSIZ );
  rkChainWriteZTK( &chain, filename );
  rkChainDestroy( &chain );
  return EXIT_SUCCESS;
}
