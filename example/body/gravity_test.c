#include <roki/rk_body.h>

int main(void)
{
  printf( "+++ gravity +++\n" );
  zVec6DPrint( RK_GRAVITY6D );
  zVec3DPrint( RK_GRAVITY3D );

  printf( "+++ clear gravity +++\n" );
  rkGravitySet( 0 );
  zVec6DPrint( RK_GRAVITY6D );
  zVec3DPrint( RK_GRAVITY3D );

  printf( "+++ reset gravity +++\n" );
  rkGravityReset();
  zVec6DPrint( RK_GRAVITY6D );
  zVec3DPrint( RK_GRAVITY3D );
  return 0;
}
