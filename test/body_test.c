#include <roki/rk_body.h>

#define N 1000
#define TOL (1.0e-7)

typedef struct{
  double m;
  zVec3D p;
} point_t;
zArrayClass( point_array_t, point_t );

void create_point_rand(point_array_t *pa, int n)
{
  register int i;

  zArrayAlloc( pa, point_t, n );
  for( i=0; i<n; i++ ){
    zArrayElem(pa,i)->m = zRandF(0.1,10.0);
    zVec3DCreate( &zArrayElem(pa,i)->p, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
  }
}

void create_mp(point_array_t *pa, rkMP mp[])
{
  register int i;
  rkMP *mpp;
  zVec3D r;

  rkMPZero( &mp[0] );
  rkMPZero( &mp[1] );
  rkMPZero( &mp[2] );
  /* mass & center of mass */
  for( i=0; i<zArraySize(pa); i++ ){
    mpp = &mp[i%2];
    rkMPMass(mpp) += zArrayElem(pa,i)->m;
    rkMPMass(&mp[2]) += zArrayElem(pa,i)->m;
    zVec3DCatDRC( rkMPCOM(mpp), zArrayElem(pa,i)->m, &zArrayElem(pa,i)->p );
    zVec3DCatDRC( rkMPCOM(&mp[2]), zArrayElem(pa,i)->m, &zArrayElem(pa,i)->p );
  }
  zVec3DDivDRC( rkMPCOM(&mp[0]), rkMPMass(&mp[0]) );
  zVec3DDivDRC( rkMPCOM(&mp[1]), rkMPMass(&mp[1]) );
  zVec3DDivDRC( rkMPCOM(&mp[2]), rkMPMass(&mp[2]) );
  /* inertia tensor */
  for( i=0; i<zArraySize(pa); i++ ){
    mpp = &mp[i%2];
    zVec3DSub( &zArrayElem(pa,i)->p, rkMPCOM(mpp), &r );
    zMat3DCatVec3DDoubleOuterProdDRC( rkMPInertia(mpp), -zArrayElem(pa,i)->m, &r );
    zVec3DSub( &zArrayElem(pa,i)->p, rkMPCOM(&mp[2]), &r );
    zMat3DCatVec3DDoubleOuterProdDRC( rkMPInertia(&mp[2]), -zArrayElem(pa,i)->m, &r );
  }
}

void assert_combine(void)
{
  point_array_t pa;
  rkMP mp[3], mpc;
  zVec3D ec;
  zMat3D ei;

  create_point_rand( &pa, N );
  create_mp( &pa, mp );
  zArrayFree( &pa );
  rkMPCombine( &mp[0], &mp[1], &mpc );
  zVec3DSub( rkMPCOM(&mp[2]), rkMPCOM(&mpc), &ec );
  zMat3DSub( rkMPInertia(&mp[2]), rkMPInertia(&mpc), &ei );
  zVec3DDivDRC( &ec, zVec3DNorm(rkMPCOM(&mpc) ) );
  zMat3DDivDRC( &ei, zMat3DNorm(rkMPInertia(&mpc) ) );
  zAssert( rkMPCombine, zVec3DIsTiny( &ec ) && zMat3DIsTiny( &ei ) );
}

bool assert_inertialellipsoid_axis(zVec3D *axis, zMat3D *r)
{
  zVec3D tmp;

  return zVec3DIsTol( zVec3DOuterProd( axis, &r->v[0], &tmp ), TOL ) ||
         zVec3DIsTol( zVec3DOuterProd( axis, &r->v[1], &tmp ), TOL ) ||
         zVec3DIsTol( zVec3DOuterProd( axis, &r->v[2], &tmp ), TOL );
}

void assert_inertialellipsoid(void)
{
  zEllips3D e, ie;
  zMat3D r;
  zVec3D c;
  rkMP mp;
  int i;
  bool result = true;

  for( i=0; i<N; i++ ){
    zMat3DFromZYX( &r, zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
    zVec3DCreate( &c, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
    zEllips3DCreate( &e, &c, &r.v[0], &r.v[1], &r.v[2], 2, 8, 17, 0 );
    rkMPSetMass( &mp, zEllips3DVolume(&e) );
    rkMPSetCOM( &mp, zEllips3DCenter(&e) );
    zEllips3DBaryInertia( &e, 1.0, rkMPInertia(&mp) );
    rkMPInertiaEllips( &mp, &ie );
    if( !assert_inertialellipsoid_axis( zEllips3DAxis(&ie,0), &r ) ||
        !assert_inertialellipsoid_axis( zEllips3DAxis(&ie,1), &r ) ||
        !assert_inertialellipsoid_axis( zEllips3DAxis(&ie,2), &r ) ) result = false;
  }
  zAssert( rkMPInertiaEllips, result );
}

int main(int argc, char *argv[])
{
  zRandInit();
  assert_combine();
  assert_inertialellipsoid();
  return 0;
}
