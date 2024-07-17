#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec3DList vl;
  zSphere3D bb;

  rkChainReadZTK( &chain, "../model/mighty.ztk" );
  rkChainVertList( &chain, &vl );
  zVec3DListDataPrint( &vl );
  zBoundingBall3DPL( &bb, &vl, NULL );
  zSphere3DFPrintZTK( stderr, &bb );

  zVec3DListDestroy( &vl );
  rkChainDestroy( &chain );
  return 0;
}
