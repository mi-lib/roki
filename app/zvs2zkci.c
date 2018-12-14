/*
 * zvs2zkci
 * end posture in .zvs -> .zkci file
 *
 * 2005. 5. 2. Created.
 * 2007.11.29. Last updated.
 */

#include <roki/rk_chain.h>
#include <zm/zm_seq.h>

void usage(void)
{
  eprintf( "Usage: zvs2zkci <.zkc file> <.zvs file> [0-/last] [.zkci file]\n" );
  exit( 0 );
}

int main(int argc, char *argv[])
{
  rkChain chain;
  zSeq seq;
  zSeqListCell *cp;

  if( argc < 3 ) usage();
  rkChainReadFile( &chain, argv[1] );
  zSeqReadFile( &seq, argv[2] );
  if( argc > 3 ){
    if( !strcmp( argv[3], "last" ) )
      cp = zListTail( &seq );
    else
      cp = zSeqJump( &seq, atoi( argv[3] ) );
  } else
    cp = zListHead( &seq );
  if( argc > 4 )
    rkChainInitReadFile( &chain, argv[4] );
  rkChainFK( &chain, cp->data.v );
  rkChainInitWrite( &chain );
  return 0;
}
