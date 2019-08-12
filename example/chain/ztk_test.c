#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;

  if( argc <= 1 ) return 0;
  rkChainScanZTK( &chain, argv[1] );
  rkChainScanInitZTK( &chain, argv[1] );
  rkChainFPrint( stdout, &chain );
  rkChainDestroy( &chain );
  return 0;
}
