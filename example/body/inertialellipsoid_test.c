#include <roki/rk_body.h>

void output(zEllips3D *e, zVec3D *c, zMat3D *r)
{
  rkMP mp;
  zEllips3D ie;
  zMat3D rt;
  zVec3D v;

  zEllips3DCreate( e, c, &r->v[0], &r->v[1], &r->v[2], 2, 8, 17, 0 );
  rkMPSetMass( &mp, zEllips3DVolume(e) );
  rkMPSetCOM( &mp, zEllips3DCenter(e) );
  zEllips3DInertia( e, rkMPInertia(&mp) );
  rkMPInertiaEllips( &mp, &ie );

  printf( "center: " ); zVec3DWrite( c );
  zMat3DT( r, &rt );
  printf( "rotation^T: " ); zMat3DWrite( &rt );
  rkMPWrite( &mp );
  zEllips3DWrite( &ie );
  zVec3DOuterProd( zEllips3DAxis(&ie,0), zEllips3DAxis(&ie,1), &v );
  printf( "(assertion of the right-hand system ... %s)\n",
    zVec3DEqual( &v, zEllips3DAxis(&ie,2) ) ? "ok" : "failure" );
}

int main(void)
{
  zEllips3D e;
  zMat3D r;
  zVec3D c;

  zRandInit();
  zMat3DZYX( &r, zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
  zVec3DCreate( &c, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
  output( &e, &c, &r );
  return 0;
}
