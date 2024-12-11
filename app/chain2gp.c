#include <roki/rk_chain.h>

void usage(char *arg)
{
  eprintf( "Usage: %s <.ztk file>\n", arg );
  exit( 0 );
}

void output_shape(zShape3D *s)
{
  int i;

  for( i=0; i<zShape3DFaceNum(s); i++ ){
    zVec3DValuePrint( zShape3DFaceVert(s,i,0) );
    zVec3DValuePrint( zShape3DFaceVert(s,i,1) );
    printf( "\n" );
    zVec3DValuePrint( zShape3DFaceVert(s,i,2) );
    zVec3DValuePrint( zShape3DFaceVert(s,i,2) );
    printf( "\n\n" );
  }
}

void conv_chain(char *filename)
{
  int i;
  rkChain chain;
  rkLink *l;
  zShapeListCell *sp;
  zShape3D s;

  if( !rkChainReadZTK( &chain, filename ) )
    exit( EXIT_FAILURE );
  for( i=0; i<rkChainLinkNum(&chain); i++ ){
    l = rkChainLink(&chain,i);
    if( rkLinkShapeIsEmpty( l ) ) continue;
    zListForEach( rkLinkShapeList(l), sp ){
      zShape3DClone( zShapeListCellShape(sp), &s, NULL );
      zShape3DXform( zShapeListCellShape(sp), rkLinkWldFrame(l), &s );
      output_shape( &s );
      zShape3DDestroy( &s );
    }
  }
  rkChainDestroy( &chain );
}

int main(int argc, char *argv[])
{
  if( argc < 2 ) usage( argv[0] );
  conv_chain( argv[1] );
  return 0;
}
