#include <roki/rk_force.h>

#define N

void assert_wrenchlist(void)
{
  rkWrenchList wl;
  rkWrench w[4];
  zVec6D rw;
  zVec6D netw( 0, 0, 0, 0, 0, 4 );

  zListInit( &wl );
  rkWrenchW(&w[0])->create( -1, 0, 0, 0, 0, 0 );
  rkWrenchPos(&w[0])->create( 1, 1, 0 );
  rkWrenchW(&w[1])->create( 0,-1, 0, 0, 0, 0 );
  rkWrenchPos(&w[1])->create( -1, 1, 0 );
  rkWrenchW(&w[2])->create( 1, 0, 0, 0, 0, 0 );
  rkWrenchPos(&w[2])->create( -1,-1, 0 );
  rkWrenchW(&w[3])->create( 0, 1, 0, 0, 0, 0 );
  rkWrenchPos(&w[3])->create( 1,-1, 0 );

  wl.insertTail( &w[0] );
  wl.insertTail( &w[1] );
  wl.insertTail( &w[2] );
  wl.insertTail( &w[3] );

  rkWrenchListNet( &wl, &rw );
  zAssert( rkWrenchListNet, ( rw == netw ) );
}

int main(void)
{
  assert_wrenchlist();
  return EXIT_SUCCESS;
}
