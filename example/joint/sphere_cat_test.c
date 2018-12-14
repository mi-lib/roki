#include <roki/rk_joint.h>

#define DT 0.01

int main(void)
{
  rkJoint j;
  zVec3D aa, v, a, aao, vo, ao, err;

  zRandInit();
  rkJointCreate( &j, RK_JOINT_SPHER );
  /* original displacement, velocity, acceleration */
  zVec3DCreate( &aa,zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
  zVec3DCreate( &v, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
  rkJointSetDis( &j, aa.e );
  rkJointSetVel( &j, v.e );
  printf( "original attitude:     " ); zVec3DWrite( &aa );
  printf( "original velocity:     " ); zVec3DWrite( &v );

  /* rot. acc. and vel. */
  zVec3DCreate( &a, zRandF(-100,100), zRandF(-100,100), zRandF(-100,100) );
  rkJointSetAcc( &j, a.e );
  printf( "original acceleration: " ); zVec3DWrite( &a );
  zVec3DCatDRC( &v, DT, &a );
  zEndl();
  printf( "updated velocity:      " ); zVec3DWrite( &v );
  /* update attitude */
  zVec3DCopy( &aa, &aao );
  rkJointCatDis( &j, aao.e, DT, v.e );
  printf( "updated attitude:      " ); zVec3DWrite( &aao );
  zEndl();
  /* discrete differentiation */
  rkJointSetDisCNT( &j, aao.e, DT );
  /* validation */
  rkJointGetDis( &j, aa.e );
  rkJointGetVel( &j, vo.e );
  rkJointGetAcc( &j, ao.e );
  printf( "sampled attitude:      " ); zVec3DWrite( &aa );
  printf( "sampled velocity:      " ); zVec3DWrite( &vo );
  printf( "sampled acceleration:  " ); zVec3DWrite( &ao );
  zEndl();
  printf( "velocity error:        " ); zVec3DWrite( zVec3DSub( &vo, &v, &err ) );
  printf( "acceleration error:    " ); zVec3DWrite( zVec3DSub( &ao, &a, &err ) );

  rkJointDestroy( &j );
  return 0;
}
