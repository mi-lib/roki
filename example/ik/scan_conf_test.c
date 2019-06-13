#include <roki/rk_ik.h>

int main(int argc, char *argv[])
{
  rkIK ik;
  rkChain chain;

  if( !rkChainScanFile( &chain, "../model/arm.ztk" ) )
    return 1;
  rkIKConfScanFile( &ik, &chain, "test.zik" );

  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  return 0;
}
