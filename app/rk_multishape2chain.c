#include <roki/rk_chain.h>

void mshape2chainUsage(char *cmd)
{
  eprintf( "Usage: %s <.ztk file>\n", cmd );
  exit( EXIT_SUCCESS );
}

bool mshape2chainConvert(zMultiShape3D *ms, char basename[])
{
  FILE *fp;
  int i;

  if( !( fp = zOpenZTKFile( basename, "w" ) ) ){
    ZOPENERROR( basename );
    return false;
  }
  fprintf( fp, "[chain]\n" );
  fprintf( fp, "name: %s\n\n", basename );
  zMultiShape3DFPrintZTK( fp, ms );
  fprintf( fp, "[link]\n" );
  fprintf( fp, "name: base\n" );
  for( i=0; i<zMultiShape3DShapeNum(ms); i++ )
    fprintf( fp, "shape: %s\n", zName(zMultiShape3DShape(ms,i)) );
  fclose( fp );
  return true;
}

int main(int argc, char *argv[])
{
  zMultiShape3D ms;
  char basename[BUFSIZ];

  if( argc == 1 ) mshape2chainUsage( argv[0] );
  if( !zMultiShape3DReadZTK( &ms, argv[1] ) )
    return EXIT_FAILURE;
  zGetBasename( argv[1], basename, BUFSIZ );
  mshape2chainConvert( &ms, basename );
  zMultiShape3DDestroy( &ms );
  return EXIT_SUCCESS;
}
