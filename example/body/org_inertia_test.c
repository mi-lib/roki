#include <roki/rk_body.h>

void create_mp(rkMP *mp, double x, double y)
{
  rkMPSetMass( mp, 1.0 );
  zVec3DCreate( rkMPCOM(mp), x, y, 0 );
  zMat3DCreate( rkMPInertia(mp),
    1.0/6.0, 0, 0,
    0, 1.0/6.0, 0,
    0, 0, 1.0/6.0 );
}

void test(int id, rkMP *mp)
{
  zMat3D io;

  printf( "*** test %d\n", id );
  rkMPWrite( mp );
  rkMPOrgInertia( mp, &io );
  printf( "(inertia about origin)\n" );
  zMat3DWrite( &io );
}

int main(int argc, char *argv[])
{
  rkMP mp;

  create_mp( &mp, 0.5, 0.0 ); test( 0, &mp );
  create_mp( &mp, 0.0, 0.5 ); test( 1, &mp );
  create_mp( &mp,-0.5, 0.0 ); test( 2, &mp );
  create_mp( &mp, 0.0,-0.5 ); test( 3, &mp );
  return 0;
}
