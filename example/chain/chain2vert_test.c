#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec3DList vl;
  zSphere3D bb;

  rkChainScanFile( &chain, "../model/mighty.zkc" );
  rkChain2VertList( &chain, &vl );
  zVec3DListDataPrint( &vl );
  zBBallPL( &bb, &vl, NULL );
  zSphere3DFPrint( stderr, &bb );

  zVec3DListDestroy( &vl );
  rkChainDestroy( &chain );
  return 0;
}
