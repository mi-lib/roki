#include <roki/rk_chain.h>

void chain2mshapeUsage(char *cmd)
{
  eprintf( "Usage: %s <.zkc file>\n", cmd );
  exit( EXIT_SUCCESS );
}

bool chain2mshapeConvert(rkChain *chain, char basename[])
{
  FILE *fp;
  register int i;
  rkLink *l;
  zShapeListCell *sc;
  zShape3D s;

  if( !( fp = zOpenZTKFile( basename, "w" ) ) ){
    ZOPENERROR( basename );
    return false;
  }
  for( i=0; i<zMShape3DOpticNum(rkChainShape(chain)); i++ ){
    fprintf( fp, "[optic]\n" );
    zOpticalInfoFPrintZTK( fp, zMShape3DOptic(rkChainShape(chain),i) );
  }
  for( i=0; i<rkChainLinkNum(chain); i++ ){
    l = rkChainLink(chain,i);
    zListForEach( rkLinkShapeList(l), sc ){
      zShape3DClone( sc->data, &s, zShape3DOptic(sc->data) );
      zShape3DXform( sc->data, rkChainLinkWldFrame(chain,i), &s );
      fprintf( fp, "[shape]\n" );
      zShape3DFPrintZTK( fp, &s );
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

  if( argc == 1 ) chain2mshapeUsage( argv[0] );
  if( !rkChainReadZTK( &chain, argv[1] ) )
    return EXIT_FAILURE;
  zGetBasename( argv[1], basename, BUFSIZ );
  chain2mshapeConvert( &chain, basename );
  rkChainDestroy( &chain );
  return EXIT_SUCCESS;
}
