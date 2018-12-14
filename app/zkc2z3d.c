#include <roki/rk_chain.h>

void zkc2z3dUsage(char *cmd)
{
  eprintf( "Usage: %s <.zkc file>\n", cmd );
  exit( EXIT_SUCCESS );
}

bool zkc2z3dConvert(rkChain *chain, char basename[])
{
  FILE *fp;
  char filename[BUFSIZ];
  register int i;
  rkLink *l;
  zShapeListCell *sc;
  zShape3D s;

  zAddSuffix( basename, ZMULTISHAPE3D_SUFFIX, filename, BUFSIZ );
  if( !( fp = fopen( filename, "w" ) ) ){
    ZOPENERROR( filename );
    return false;
  }
  for( i=0; i<zMShape3DOpticNum(rkChainShape(chain)); i++ ){
    fprintf( fp, "[optic]\n" );
    zOpticalInfoFWrite( fp, zMShape3DOptic(rkChainShape(chain),i) );
  }
  for( i=0; i<rkChainNum(chain); i++ ){
    l = rkChainLink(chain,i);
    zListForEach( rkLinkShapeList(l), sc ){
      zShape3DClone( sc->data, &s, zShape3DOptic(sc->data) );
      zShape3DXfer( sc->data, rkChainLinkWldFrame(chain,i), &s );
      fprintf( fp, "[shape]\n" );
      zShape3DFWrite( fp, &s );
      zShape3DDestroy( &s );
    }
  }
  fclose( fp );
  return true;
}

int main(int argc, char *argv[])
{
  rkChain chain;
  char basename[BUFSIZ];

  if( argc == 1 ) zkc2z3dUsage( argv[0] );
  if( !rkChainReadFile( &chain, argv[1] ) )
    return EXIT_FAILURE;
  zGetBasename( argv[1], basename, BUFSIZ );
  zkc2z3dConvert( &chain, basename );
  rkChainDestroy( &chain );
  return EXIT_SUCCESS;
}
