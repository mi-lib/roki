#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <zm/zm_misc.h>
#include <roki/rk_g.h>
#include <roki/rk_contact.h>

void zelmat_usage(void)
{
  eprintf( "Usage: zelmat [mass] [depth] [damping coeff.] [stat. fric. coeff.] [kine. fric. coeff.]\n" );
  exit( 0 );
}

int main(int argc, char *argv[])
{
  double mass, depth, zeta, ms, mu;
  double k, c;
  rkContactInfo ci;

  if( argc < 5 ) zelmat_usage();
  mass  = atof( argv[1] );
  depth = atof( argv[2] );
  zeta  = atof( argv[3] );
  ms    = atof( argv[4] );
  mu    = atof( argv[5] );

  if( zIsTiny( depth ) ){
    ZRUNERROR( "almost rigid contact model required" );
    exit( 1 );
  }

  k = mass * RK_G / depth;
  c = 2 * zeta * sqrt( k * mass );

  rkContactInfoElasticCreate( &ci, k, c, ms, mu, NULL, NULL );
  rkContactInfoWrite( &ci );
  return 0;
}
