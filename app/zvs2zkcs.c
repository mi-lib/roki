/*
 * zvs2zkcs
 * .zvs -> .zkcs file
 *
 * 2011.12.14. Created.
 * 2011.12.14. Last updated.
 */

#include <roki/rk_chain.h>
#include <zm/zm_seq.h>

void usage(void)
{
  eprintf( "Usage: zvs2zkcs <.zkc file> <.zvs file> [.zkcs file]\n" );
  exit( 0 );
}

int main(int argc, char *argv[])
{
  rkChain chain;
  zSeq seq;
  zSeqListCell *cp;
  zVec conf;
  FILE *fp;

  if( argc < 3 ) usage();

  rkChainReadFile( &chain, argv[1] );
  if( !( conf = zVecAlloc( rkChainNum(&chain) * 6 ) ) ){
    ZALLOCERROR();
    return 1;
  }
  zSeqReadFile( &seq, argv[2] );
  if( argc > 3 ){
    if( !( fp = fopen( argv[3], "r" ) ) ){
      ZOPENERROR( argv[3] );
      return 1;
    }
  } else{
    fp = stdout;
  }
  while( !zListIsEmpty(&seq) ){
    cp = zSeqDequeue( &seq );
    rkChainFK( &chain, cp->data.v );
    rkChainGetConf( &chain, conf );
    fprintf( fp, "%g ", cp->data.dt );
    zVecFWrite( fp, conf );
    zSeqListCellFree( cp );
  }
  if( fp != stdout ) fclose( fp );
  zVecFree( conf );
  rkChainDestroy( &chain );
  return 0;
}
