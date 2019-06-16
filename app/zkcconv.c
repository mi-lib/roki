/* convert .zkc file in which all parameters are denoted
 * with respect to the world frame to that with a standard notation.
 */

#include <roki/rk_chain.h>

rkChain chain;

void zkcconv_usage(void)
{
  ZECHO( "Usage: zkcconv <input .zkc file> [output .zkc file]" );
  ZECHO( " all parameters are supposed to be denoted with respect to" );
  ZECHO( " the world frame in input .zkc file." );
  ZECHO( " when output .zkc file is omitted, the result is output" );
  ZECHO( " to the standard output." );
  exit( 0 );
}

void zkcconv_mass(rkLink *l)
{
  zFrame3D *wf; /* world frame temporarily stored in link_base_frame ) */

  wf = rkLinkAdjFrame( l );
  zXform3DInvDRC( wf, rkLinkCOM(l) );
  zRotMat3DInv( zFrame3DAtt(wf), rkLinkInertia(l), rkLinkInertia(l) );

  if( rkLinkChild(l) )
    zkcconv_mass( rkLinkChild(l) );
  if( rkLinkSibl(l) )
    zkcconv_mass( rkLinkSibl(l) );
}

void zkcconv_frame(rkLink *l)
{
  zFrame3D frm;

  if( rkLinkChild(l) )
    zkcconv_frame( rkLinkChild(l) );

  if( rkLinkParent(l) ){
    zFrame3DXform( rkLinkAdjFrame(rkLinkParent(l)), rkLinkAdjFrame(l), &frm );
    zFrame3DCopy( &frm, rkLinkAdjFrame(l) );
  }
  if( rkLinkSibl(l) )
    zkcconv_frame( rkLinkSibl(l) );
}

int main(int argc, char *argv[])
{
  if( argc < 2 ) zkcconv_usage();

  if( !rkChainScanFile( &chain, argv[1] ) ) exit( 1 );

  zkcconv_mass( rkChainRoot( &chain ) );
  zkcconv_frame( rkChainRoot( &chain ) );

  if( argc > 3 )
    rkChainPrintFile( &chain, argv[2] );
  else
    rkChainPrint( &chain );
  rkChainDestroy( &chain );
  return 0;
}
