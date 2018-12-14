#include <roki/rk_chain.h>

void usage(char *arg)
{
  eprintf( "Usage: %s <.z3d file>\n", arg );
  eprintf( "       %s <.zkc file>\n", arg );
  exit( 0 );
}

void output_shape(zShape3D *s)
{
  register int i;

  for( i=0; i<zShape3DFaceNum(s); i++ ){
    zVec3DDataWrite( zShape3DFaceVert(s,i,0) );
    zVec3DDataWrite( zShape3DFaceVert(s,i,1) );
    printf( "\n" );
    zVec3DDataWrite( zShape3DFaceVert(s,i,2) );
    zVec3DDataWrite( zShape3DFaceVert(s,i,2) );
    printf( "\n\n" );
  }
}

void conv_mshape(char *filename)
{
  register int i;
  zMShape3D ms;

  if( !zMShape3DReadFile( &ms, filename ) )
    exit( EXIT_FAILURE );
  for( i=0; i<zMShape3DShapeNum(&ms); i++ )
    output_shape( zMShape3DShape(&ms,i) );
  zMShape3DDestroy( &ms );
}

void conv_chain(char *filename)
{
  register int i;
  rkChain chain;
  rkLink *l;
  zShapeListCell *sp;
  zShape3D s;

  if( !rkChainReadFile( &chain, filename ) )
    exit( EXIT_FAILURE );
  for( i=0; i<rkChainNum(&chain); i++ ){
    l = rkChainLink(&chain,i);
    if( rkLinkShapeIsEmpty( l ) ) continue;
    zListForEach( rkLinkShapeList(l), sp ){
      zShape3DClone( zShapeListCellShape(sp), &s, NULL );
      zShape3DXfer( zShapeListCellShape(sp), rkLinkWldFrame(l), &s );
      output_shape( &s );
      zShape3DDestroy( &s );
    }
  }
  rkChainDestroy( &chain );
}

int main(int argc, char *argv[])
{
  if( argc < 2 ) usage( argv[0] );
  if( strcmp( zGetSuffix(argv[1]), "z3d" ) == 0 )
    conv_mshape( argv[1] );
  else
  if( strcmp( zGetSuffix(argv[1]), "zkc" ) == 0 )
    conv_chain( argv[1] );
  else
    ZRUNERROR( "unknown filetype" );
  return 0;
}
