#include <roki/rk_body.h>

void create_body(rkBody *b, double x, double y, double yaw)
{
  rkBodyInit( b );
  rkBodySetMass( b, 1.0 );
  zMat3DCreate( rkBodyInertia(b),
    1.0/6.0, 0, 0,
    0, 1.0/6.0, 0,
    0, 0, 1.0/6.0 );
  zVec3DCreate( rkBodyPos(b), x, y, 0 );
  zMat3DRotYaw( Z_IDENTMAT3D, yaw, rkBodyAtt(b) );
}

int main(int argc, char *argv[])
{
  rkBody src[4], dst[3];
  zFrame3D f;

  create_body( &src[0], 0.5, 0.0, zDeg2Rad(  0) );
  create_body( &src[1], 0.0, 0.5, zDeg2Rad( 90) );
  create_body( &src[2],-0.5, 0.0, zDeg2Rad(180) );
  create_body( &src[3], 0.0,-0.5, zDeg2Rad(270) );
  zFrame3DIdent( &f );

  printf( "*** before combined ***\n" );
  rkMPWrite( rkBodyMP(&src[0]) );
  rkMPWrite( rkBodyMP(&src[1]) );
  rkMPWrite( rkBodyMP(&src[2]) );
  rkMPWrite( rkBodyMP(&src[3]) );
  printf( "*** combining test 1 ***\n" );
  rkBodyCombine( &src[0], &src[2], &f, &dst[0] );
  rkBodyCombine( &src[1], &src[3], &f, &dst[1] );
  rkBodyCombine( &dst[0], &dst[1], &f, &dst[2] );
  rkMPWrite( rkBodyMP(&dst[2]) );
  printf( "*** combining test 2 ***\n" );
  rkBodyCombine( &src[0], &src[1], &f, &dst[0] );
  rkBodyCombine( &src[2], &src[3], &f, &dst[1] );
  rkBodyCombine( &dst[0], &dst[1], &f, &dst[2] );
  rkMPWrite( rkBodyMP(&dst[2]) );
  printf( "*** combining test 3 ***\n" );
  rkBodyInit( &dst[2] );
  rkBodyCombineDRC( &dst[2], &src[0] );
  rkBodyCombineDRC( &dst[2], &src[1] );
  rkBodyCombineDRC( &dst[2], &src[2] );
  rkBodyCombineDRC( &dst[2], &src[3] );
  rkMPWrite( rkBodyMP(&dst[2]) );
  return 0;
}
