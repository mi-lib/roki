#include <roki/rk_chain.h>

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec3DData point_data;
  zSphere3D bb;

  rkChainReadZTK( &chain, "../model/mighty.ztk" );
  rkChainVertData( &chain, &point_data );
  zVec3DDataValuePrint( &point_data );
  zVec3DDataBoundingBall( &point_data, &bb, NULL );
  zSphere3DFPrintZTK( stderr, &bb );

  zVec3DDataDestroy( &point_data );
  rkChainDestroy( &chain );
  return 0;
}
