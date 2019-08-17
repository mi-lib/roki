#include <roki/rk_ik.h>

int main(int argc, char *argv[])
{
  rkIK ik;
  rkChain chain;

  if( !rkChainScanFile( &chain, "../model/arm.ztk" ) )
    return 1;
  rkIKConfScanZTK( &ik, &chain, "iktest.ztk" );

  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  return 0;
}
