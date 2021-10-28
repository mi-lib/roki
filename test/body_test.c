#include <roki/rk_body.h>

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

#define N 1000

void combine_test(void)
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

int main(int argc, char *argv[])
{
  zRandInit();
  combine_test();
  return 0;
}
