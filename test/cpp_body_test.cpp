#include <roki/rk_body.h>

struct point_t{
  double m;
  zVec3D p;
};
ZEDA_DEF_ARRAY_CLASS( point_array_t, point_t );

void create_mp3(point_array_t *pa, rkMP mp[])
{
  rkMP *mpp;

  mp[0].zero();
  mp[1].zero();
  mp[2].zero();
  /* mass & center of mass */
  for(int i=0; i<pa->size; i++ ){
    mpp = &mp[i%2];
    mpp->mass  += pa->get(i)->m;
    mp[2].mass += pa->get(i)->m;
    mpp->com  += pa->get(i)->m * pa->get(i)->p;
    mp[2].com += pa->get(i)->m * pa->get(i)->p;
  }
  mp[0].com /= mp[0].mass;
  mp[1].com /= mp[1].mass;
  mp[2].com /= mp[2].mass;
  /* inertia tensor */
  for(int i=0; i<pa->size; i++ ){
    zMat3D rr;
    mpp = &mp[i%2];
    mpp->inertia -= pa->get(i)->m * rr.createDoubleOuterprod( pa->get(i)->p - mpp->com );
    mp[2].inertia -= pa->get(i)->m * rr.createDoubleOuterprod( pa->get(i)->p - mp[2].com );
  }
}

void assert_mp_combine_pointmass(void)
{
  point_array_t pa;
  const int n = 1000;
  pa.alloc( n );
  for(int i=0; i<n; i++ ){
    pa.get(i)->m = zRandF(0.1,10.0);
    pa.get(i)->p.create( zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
  }

  rkMP mp[3];
  create_mp3( &pa, mp );
  pa._free();
  rkMP mpc = mp[0] + mp[1];
  zVec3D ec = ( mp[2].com - mpc.com ) / mpc.com.norm();
  zMat3D ei = ( mp[2].inertia - mpc.inertia ) / mpc.inertia.norm();
  zAssert( C++::rkMP.operator+ (point-mass test), ec.isTol() && ei.isTol() );
}

void create_body_rand(rkBody *body)
{
  body->init();
  body->mp.mass = zRandF( 1.0, 5.0 );
  body->mp.com.create( zRandF(-1.0,1.0), zRandF(-1.0,1.0), zRandF(-1.0,1.0) );
  body->mp.inertia.zero();

  const int n = 10;
  for(int i=0; i<n; i++ ){
    zVec3D distributed_mass{ zRandF(-1.0,1.0), zRandF(-1.0,1.0), zRandF(-1.0,1.0) };
    zMat3D rr;
    body->mp.inertia += rr.createDyad( distributed_mass, distributed_mass );
  }
}

void assert_body_combine(void)
{
  rkBody src[4], tmp[2], dst[3];

  create_body_rand( &src[0] );
  create_body_rand( &src[1] );
  create_body_rand( &src[2] );
  create_body_rand( &src[3] );
  // combined body 0
  dst[0].combine( tmp[0].combine( src[0], src[2] ), tmp[1].combine( src[1], src[3] ) );
  // combined body 1
  dst[1].combine( tmp[0].combine( src[0], src[1] ), tmp[1].combine( src[2], src[3] ) );
  // combined body 2
  dst[2].init();
  dst[2].merge( src[0] );
  dst[2].merge( src[1] );
  dst[2].merge( src[2] );
  dst[2].merge( src[3] );

  zAssert( C++::rkBody.combine & rkBody.merge, dst[0].mp == dst[1].mp && dst[0].mp == dst[2].mp );
}

int main(int argc, char *argv[])
{
  zRandInit();
  assert_mp_combine_pointmass();
  assert_body_combine();
  return 0;
}
