#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;

  if( argc <= 1 ) return 0;
  rkChainReadZTK( &chain, argv[1] );
  if( argc > 2 ) rkChainInitReadZTK( &chain, argv[2] );
  rkChainFPrintZTK( stdout, &chain );
  rkChainDestroy( &chain );
  return 0;
}
