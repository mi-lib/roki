#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;

  if( !rkChainReadZTK( &chain, argc > 1 ? argv[1] : "../model/mighty.ztk" ) )
    return 1;
  rkChainConnectivityFPrint( stdout, &chain );
  rkChainDestroy( &chain );
  return 0;
}
