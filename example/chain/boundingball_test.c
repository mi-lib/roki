#include <roki/rk_chain.h>

void output(rkChain *chain, zSphere3D *bb)
{
  zFrame3D f;

  rkChainFPrintZTK( stdout, chain );
  printf( "\n[optic]\n" );
  printf( "name: transparent\n" );
  printf( "diffuse: 1.0 1.0 1.0\n" );
  printf( "alpha: 0.2\n\n" );
  printf( "[shape]\n" );
  printf( "name: bounding_ball\n" );
  printf( "type: sphere\n" );
  printf( "optic: transparent\n" );
  zFrame3DInv( rkChainLinkWldFrame(chain,0), &f );
  printf( "frame: " );
  zFrame3DPrint( &f );
  printf( "center: " );
  zVec3DPrint( zSphere3DCenter(bb) );
  printf( "radius: %.10g\n\n", zSphere3DRadius(bb) );
  printf( "[link]\n" );
  printf( "name: bounding_ball\n" );
  printf( "jointtype: fixed\n" );
  printf( "shape: bounding_ball\n" );
  printf( "parent: %s\n", rkChainLinkName(chain,0) );
}

int main(int argc, char *argv[])
{
  rkChain chain;
  zSphere3D bb;

  if( argc < 2 ){
    eprintf( "Usage %s [.ztk file]\n", argv[0] );
    return 1;
  }
  if( !rkChainReadZTK( &chain, argv[1] ) ) return 1;
  rkChainBoundingBall( &chain, &bb );
  output( &chain, &bb );
  rkChainDestroy( &chain );
  return 0;
}
