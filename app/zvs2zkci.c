/* zvs2zkci
 * end posture in .zvs -> .zkci file
 */

#include <roki/rk_chain.h>
#include <zm/zm_seq.h>

void usage(char *argv)
{
  eprintf( "Usage: %s <.ztk file> <.zvs file> [0-/last] [.zkci file]\n", argv );
  exit( 0 );
}

int main(int argc, char *argv[])
{
  rkChain chain;
  zSeq seq;
  zSeqCell *cp;

  if( argc < 3 ) usage( argv[0] );
  rkChainReadZTK( &chain, argv[1] );
  zSeqScanFile( &seq, argv[2] );
  if( argc > 3 ){
    if( !strcmp( argv[3], "last" ) )
      cp = zListTail( &seq );
    else
      cp = zSeqJump( &seq, atoi( argv[3] ) );
  } else
    cp = zListHead( &seq );
  if( argc > 4 )
    rkChainInitReadZTK( &chain, argv[4] );
  rkChainFK( &chain, cp->data.v );
  rkChainInitFPrintZTK( stdout, &chain );
  return 0;
}
