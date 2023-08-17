/* zvs2zkcs
 * .zvs -> .zkcs file
 */

#include <roki/rk_chain.h>
#include <zm/zm_seq.h>

void usage(char *argv)
{
  eprintf( "Usage: %s <.ztk file> <.zvs file> [.zkcs file]\n", argv );
  exit( 0 );
}

int main(int argc, char *argv[])
{
  rkChain chain;
  zSeq seq;
  zSeqCell *cp;
  zVec conf;
  FILE *fp;

  if( argc < 3 ) usage( argv[0] );

  rkChainReadZTK( &chain, argv[1] );
  if( !( conf = zVecAlloc( rkChainLinkNum(&chain) * 6 ) ) ){
    ZALLOCERROR();
    return 1;
  }
  zSeqScanFile( &seq, argv[2] );
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
    zVecFPrint( fp, conf );
    zSeqCellFree( cp );
  }
  if( fp != stdout ) fclose( fp );
  zVecFree( conf );
  rkChainDestroy( &chain );
  return 0;
}
