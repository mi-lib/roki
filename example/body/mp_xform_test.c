#include <roki/rk_body.h>

int main(void)
{
  rkMP src, dest;
  zFrame3D f;

  rkMPSetMass( &src, 1.0 );
  zVec3DCreate( rkMPCOM(&src), 0.5, 0.5, 0.5 );
  zMat3DCreate( rkMPInertia(&src),
    1.0/6.0, 0, 0,
    0, 1.0/6.0, 0,
    0, 0, 1.0/6.0 );
  zFrame3DIdent( &f );
  zVec3DCreate( zFrame3DPos(&f),-0.5,-0.5,-0.5 );
  rkMPXform( &src, &f, &dest );

  printf( "*** before transformation ***\n" );
  rkMPPrint( &src );
  printf( "*** after transformation ***\n" );
  rkMPPrint( &dest );
  return 0;
}
