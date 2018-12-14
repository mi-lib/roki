#include <roki/rk_force.h>

#define N

void assert_wrenchlist(void)
{
  rkWrenchList wl;
  rkWrench w[4];
  zVec6D rw;
  zVec6D netw = { { 0, 0, 0, 0, 0, 4 } };

  zListInit( &wl );
  zVec6DCreate( rkWrenchW(&w[0]),-1, 0, 0, 0, 0, 0 );
  zVec3DCreate( rkWrenchPos(&w[0]), 1, 1, 0 );
  zVec6DCreate( rkWrenchW(&w[1]), 0,-1, 0, 0, 0, 0 );
  zVec3DCreate( rkWrenchPos(&w[1]),-1, 1, 0 );
  zVec6DCreate( rkWrenchW(&w[2]), 1, 0, 0, 0, 0, 0 );
  zVec3DCreate( rkWrenchPos(&w[2]),-1,-1, 0 );
  zVec6DCreate( rkWrenchW(&w[3]), 0, 1, 0, 0, 0, 0 );
  zVec3DCreate( rkWrenchPos(&w[3]), 1,-1, 0 );

  zListInsertTail( &wl, &w[0] );
  zListInsertTail( &wl, &w[1] );
  zListInsertTail( &wl, &w[2] );
  zListInsertTail( &wl, &w[3] );

  rkWrenchListNet( &wl, &rw );
  zAssert( rkWrenchListNet, zVec6DEqual( &rw, &netw ) );
}

int main(void)
{
  assert_wrenchlist();
  return EXIT_SUCCESS;
}
