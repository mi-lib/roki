#include <roki/rk_body.h>

int main(void)
{
  printf( "+++ gravity +++\n" );
  zVec6DPrint( RK_GRAVITY6D );
  zVec3DPrint( RK_GRAVITY3D );

  return 0;
}
