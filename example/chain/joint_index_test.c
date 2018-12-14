#include <roki/rk_chain.h>

#define CHAIN_FILE "../model/humanoid.zkc"

int main(void)
{
  rkChain chain;
  zIndex index;

  rkChainReadFile( &chain, CHAIN_FILE );
  index = rkChainCreateDefaultJointIndex( &chain );
  rkChainConnectionWrite( &chain );
  printf( "total joint size = %d\n", rkChainJointSize( &chain ) );
  zIndexWrite( index );
  rkChainDestroy( &chain );
  zIndexFree( index );
  return 0;
}
