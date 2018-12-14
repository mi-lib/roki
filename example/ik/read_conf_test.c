#include <roki/rk_ik.h>

int main(int argc, char *argv[])
{
  rkIK ik;
  rkChain chain;

  if( !rkChainReadFile( &chain, "../model/arm.zkc" ) )
    return 1;
  rkIKConfReadFile( &ik, &chain, "test.zik" );

  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  return 0;
}
