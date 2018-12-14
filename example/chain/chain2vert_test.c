#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec3DList vl;
  zSphere3D bb;

  rkChainReadFile( &chain, "../model/mighty.zkc" );
  rkChain2VertList( &chain, &vl );
  zVec3DListDataWrite( &vl );
  zBBallPL( &bb, &vl, NULL );
  zSphere3DFWrite( stderr, &bb );

  zVec3DListDestroy( &vl, true );
  rkChainDestroy( &chain );
  return 0;
}
