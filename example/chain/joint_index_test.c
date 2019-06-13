#include <roki/rk_chain.h>

#define CHAIN_FILE "../model/humanoid.zkc"

int main(void)
{
  rkChain chain;
  zIndex index;

  rkChainScanFile( &chain, CHAIN_FILE );
  index = rkChainCreateDefaultJointIndex( &chain );
  rkChainConnectionPrint( &chain );
  printf( "total joint size = %d\n", rkChainJointSize( &chain ) );
  zIndexPrint( index );
  rkChainDestroy( &chain );
  zIndexFree( index );
  return 0;
}
