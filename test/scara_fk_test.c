#include <roki/rk_link.h>

#define L1 1.2
#define L2 0.8

void link_init(rkLink *l, int id, char *name, byte jtype)
{
  rkLinkInit( l );
  zNameSet( l, name );
  rkJointCreate( rkLinkJoint(l), jtype );
}

void scara_build(rkLink l[])
{
  link_init( &l[0], 1, "l1", RK_JOINT_REVOL );
  link_init( &l[1], 2, "l2", RK_JOINT_REVOL );
  link_init( &l[2], 3, "tip", RK_JOINT_FIXED );
  zVec3DCreate( rkLinkOrgPos(&l[1]), 0, L1, 0 );
  zVec3DCreate( rkLinkOrgPos(&l[2]), 0, L2, 0 );
  rkLinkAddChild( &l[0], &l[1] );
  rkLinkAddChild( &l[1], &l[2] );
  zFrame3DCopy( rkLinkOrgFrame(&l[0]), rkLinkAdjFrame(&l[0]) );
  zFrame3DCopy( rkLinkOrgFrame(&l[1]), rkLinkAdjFrame(&l[1]) );
  zFrame3DCopy( rkLinkOrgFrame(&l[2]), rkLinkAdjFrame(&l[2]) );
}

bool check_one(rkLink l[])
{
  double s1, c1, s12, c12;
  double a1, a2;

  a1 = zRandF(-zPI,zPI);
  a2 = zRandF(-zPI,zPI);

  rkLinkSetJointDis( &l[0], &a1 );
  rkLinkSetJointDis( &l[1], &a2 );
  rkLinkUpdateFrame( &l[0], ZFRAME3DIDENT );

  zSinCos( a1, &s1, &c1 );
  zSinCos( a1+a2, &s12, &c12 );
  return zIsTiny(rkLinkWldPos(&l[2])->e[zX]+L1*s1+L2*s12) && zIsTiny(rkLinkWldPos(&l[2])->e[zY]-L1*c1-L2*c12);
}

bool check(rkLink l[], int n)
{
  register int i;

  for( i=0; i<n; i++ ){
    if( !check_one( l ) ) return false;
  }
  return true;
}

#define N 100

int main(void)
{
  rkLink l[3];

  zRandInit();
  scara_build( l );

  zAssert( SCARA FK, check( l, N ) );

  rkLinkDestroy( &l[0] );
  rkLinkDestroy( &l[1] );
  rkLinkDestroy( &l[2] );
  return EXIT_SUCCESS;
}
