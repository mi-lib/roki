#include <roki/rk_body.h>

int main(void)
{
  printf( "+++ gravity +++\n" );
  zVec6DWrite( RK_GRAVITY6D );
  zVec3DWrite( RK_GRAVITY3D );

  printf( "+++ clear gravity +++\n" );
  rkGravitySet( 0 );
  zVec6DWrite( RK_GRAVITY6D );
  zVec3DWrite( RK_GRAVITY3D );

  printf( "+++ reset gravity +++\n" );
  rkGravityReset();
  zVec6DWrite( RK_GRAVITY6D );
  zVec3DWrite( RK_GRAVITY3D );
  return 0;
}
