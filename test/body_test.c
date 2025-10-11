#include <roki/rk_body.h>

#define TOL (1.0e-7)

void test_mp_org_inertia(double x, double y, zMat3D *io)
{
  rkMP mp;

  rkMPSetMass( &mp, 6.0 );
  zVec3DCreate( rkMPCOM(&mp), x, y, 0 );
  zMat3DIdent( rkMPInertia(&mp) );
  rkMPOrgInertia( &mp, io );
}

void assert_mp_org_inertia(void)
{
  zMat3D io;

  test_mp_org_inertia( 1, 0, &io );
  zAssert( rkMPOrgInertia (trivial case 1),
    io.c.xx == 1 && io.c.yx == 0 && io.c.zx == 0 &&
    io.c.xy == 0 && io.c.yy == 7 && io.c.zy == 0 &&
    io.c.xz == 0 && io.c.yz == 0 && io.c.zz == 7 );
  test_mp_org_inertia( 0, 1, &io );
  zAssert( rkMPOrgInertia (trivial case 2),
    io.c.xx == 7 && io.c.yx == 0 && io.c.zx == 0 &&
    io.c.xy == 0 && io.c.yy == 1 && io.c.zy == 0 &&
    io.c.xz == 0 && io.c.yz == 0 && io.c.zz == 7 );
  test_mp_org_inertia(-1, 0, &io );
  zAssert( rkMPOrgInertia (trivial case 3),
    io.c.xx == 1 && io.c.yx == 0 && io.c.zx == 0 &&
    io.c.xy == 0 && io.c.yy == 7 && io.c.zy == 0 &&
    io.c.xz == 0 && io.c.yz == 0 && io.c.zz == 7 );
  test_mp_org_inertia( 0,-1, &io );
  zAssert( rkMPOrgInertia (trivial case 4),
    io.c.xx == 7 && io.c.yx == 0 && io.c.zx == 0 &&
    io.c.xy == 0 && io.c.yy == 1 && io.c.zy == 0 &&
    io.c.xz == 0 && io.c.yz == 0 && io.c.zz == 7 );
}

typedef struct{
  double m;
  zVec3D p;
} point_t;
zArrayClass( point_array_t, point_t );

void create_point_rand(point_array_t *pa, int n)
{
  int i;

  zArrayAlloc( pa, point_t, n );
  for( i=0; i<n; i++ ){
    zArrayElem(pa,i)->m = zRandF(0.1,10.0);
    zVec3DCreate( &zArrayElem(pa,i)->p, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
  }
}

void create_mp(point_array_t *pa, rkMP mp[])
{
  int i;
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

void assert_mp_combine_pointmass(void)
{
  point_array_t pa;
  rkMP mp[3], mpc;
  zVec3D ec;
  zMat3D ei;
  const int n = 1000;

  create_point_rand( &pa, n );
  create_mp( &pa, mp );
  zArrayFree( &pa );
  rkMPCombine( &mp[0], &mp[1], &mpc );
  zVec3DSub( rkMPCOM(&mp[2]), rkMPCOM(&mpc), &ec );
  zMat3DSub( rkMPInertia(&mp[2]), rkMPInertia(&mpc), &ei );
  zVec3DDivDRC( &ec, zVec3DNorm(rkMPCOM(&mpc) ) );
  zMat3DDivDRC( &ei, zMat3DNorm(rkMPInertia(&mpc) ) );
  zAssert( rkMPCombine (point-mass test), zVec3DIsTiny( &ec ) && zMat3DIsTiny( &ei ) );
}

bool assert_mp_inertialellipsoid_axis(zVec3D *axis, zMat3D *r)
{
  zVec3D tmp;

  return zVec3DIsTol( zVec3DOuterProd( axis, &r->v[0], &tmp ), TOL ) ||
         zVec3DIsTol( zVec3DOuterProd( axis, &r->v[1], &tmp ), TOL ) ||
         zVec3DIsTol( zVec3DOuterProd( axis, &r->v[2], &tmp ), TOL );
}

void assert_mp_inertialellipsoid(void)
{
  zEllips3D e, ie;
  zMat3D r;
  zVec3D c;
  rkMP mp;
  const int n = 1000;
  int i;
  bool result = true;

  for( i=0; i<n; i++ ){
    zMat3DFromZYX( &r, zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
    zVec3DCreate( &c, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
    zEllips3DCreate( &e, &c, &r.v[0], &r.v[1], &r.v[2], 2, 8, 17, 0 );
    rkMPSetMass( &mp, zEllips3DVolume(&e) );
    rkMPSetCOM( &mp, zEllips3DCenter(&e) );
    zEllips3DBaryInertia( &e, 1.0, rkMPInertia(&mp) );
    rkMPInertiaEllips( &mp, &ie );
    if( !assert_mp_inertialellipsoid_axis( zEllips3DAxis(&ie,0), &r ) ||
        !assert_mp_inertialellipsoid_axis( zEllips3DAxis(&ie,1), &r ) ||
        !assert_mp_inertialellipsoid_axis( zEllips3DAxis(&ie,2), &r ) ) result = false;
  }
  zAssert( rkMPInertiaEllips, result );
}

void create_body_rand(rkBody *body)
{
  zVec3D iv[3];

  rkBodyInit( body );
  rkBodySetMass( body, zRandF(0.1,5.0) );
  zVec3DCreate( &iv[0], zRandF(0.1,1.0), zRandF(0.1,1.0), zRandF(0.1,1.0) );
  zVec3DCreate( &iv[1], zRandF(0.1,1.0), zRandF(0.1,1.0), zRandF(0.1,1.0) );
  zVec3DCreate( &iv[2], zRandF(0.1,1.0), zRandF(0.1,1.0), zRandF(0.1,1.0) );
  zMat3DZero( rkBodyInertia(body) );
  zMat3DAddDyad( rkBodyInertia(body), &iv[0], &iv[0] );
  zMat3DAddDyad( rkBodyInertia(body), &iv[1], &iv[1] );
  zMat3DAddDyad( rkBodyInertia(body), &iv[2], &iv[2] );
  zVec3DCreate( rkBodyPos(body), zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
}

void assert_body_combine(void)
{
  rkBody src[4], tmp[2], dst[3];

  create_body_rand( &src[0] );
  create_body_rand( &src[1] );
  create_body_rand( &src[2] );
  create_body_rand( &src[3] );
  /* combined body 0 */
  rkBodyCombine( &src[0], &src[2], ZFRAME3DIDENT, &tmp[0] );
  rkBodyCombine( &src[1], &src[3], ZFRAME3DIDENT, &tmp[1] );
  rkBodyCombine( &tmp[0], &tmp[1], ZFRAME3DIDENT, &dst[0] );
  /* combined body 1 */
  rkBodyCombine( &src[0], &src[1], ZFRAME3DIDENT, &tmp[0] );
  rkBodyCombine( &src[2], &src[3], ZFRAME3DIDENT, &tmp[1] );
  rkBodyCombine( &tmp[0], &tmp[1], ZFRAME3DIDENT, &dst[1] );
  /* combined body 2 */
  rkBodyInit( &dst[2] );
  rkBodyCombineDRC( &dst[2], &src[0] );
  rkBodyCombineDRC( &dst[2], &src[1] );
  rkBodyCombineDRC( &dst[2], &src[2] );
  rkBodyCombineDRC( &dst[2], &src[3] );

  zAssert( rkBodyCombine & rkBodyCombineDRC,
    rkMPEqual( rkBodyMP(&dst[0]), rkBodyMP(&dst[1]) ) &&
    rkMPEqual( rkBodyMP(&dst[0]), rkBodyMP(&dst[2]) ) );
}

void assert_body_contig_vert(void)
{
  rkBody body;
  zShape3D shape, shape_clone;
  zVec3D point;
  const zVec3D *cv1, *cv2;
  double dist1, dist2;
  const int n = 100;
  int i;
  bool result = true;

  zShape3DBoxCreateAlign( &shape, ZVEC3DZERO, 2.0, 2.0, 2.0 );
  zShape3DToPH( &shape );
  rkBodyInit( &body );
  rkBodyShapePush( &body, &shape );
  zFrame3DFromPosZYX( rkBodyFrame(&body),
    zRandF(-1,1), zRandF(-1,1), zRandF(-1,1),
    zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
  zShape3DClone( &shape, &shape_clone, NULL );
  zShape3DXform( &shape, rkBodyFrame(&body), &shape_clone );
  for( i=0; i<n; i++ ){
    zVec3DCreate( &point, zRandF(-2,2), zRandF(-2,2), zRandF(-2,2) );
    cv1 = rkBodyContigVert( &body, &point, &dist1 );
    cv2 = zShape3DContigVert( &shape_clone, &point, &dist2 );
    if( cv1 - zShape3DVertBuf(&shape) != cv2 - zShape3DVertBuf(&shape_clone) ) result = false;
  }
  rkBodyDestroy( &body );
  zShape3DDestroy( &shape );
  zShape3DDestroy( &shape_clone );
  zAssert( rkBodyContigVert, result );
}

int main(int argc, char *argv[])
{
  zRandInit();
  assert_mp_org_inertia();
  assert_mp_combine_pointmass();
  assert_mp_inertialellipsoid();
  assert_body_combine();
  assert_body_contig_vert();
  return 0;
}
