#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  zIndex index;

  if( !rkChainReadZTK( &chain, argc > 1 ? argv[1] : "../model/H5.ztk" ) )
    return 1;
  index = rkChainCreateDefaultJointIndex( &chain );
  rkChainConnectionPrint( &chain );
  printf( "total joint size = %d\n", rkChainJointSize( &chain ) );
  zIndexPrint( index );
  rkChainDestroy( &chain );
  zIndexFree( index );
  return 0;
}
