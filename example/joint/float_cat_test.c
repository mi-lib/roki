#include <roki/rk_joint.h>

#define DT 0.01

int main(void)
{
  rkJoint j;
  zVec6D d, v, a, dn, vn, an, err;

  zRandInit();
  rkJointCreate( &j, RK_JOINT_FLOAT );
  /* original displacement, velocity, acceleration */
  zVec6DCreate( &d, zRandF(-1,1), zRandF(-1,1), zRandF(-1,1), zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
  zVec6DCreate( &v, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10), zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
  rkJointSetDis( &j, d.e );
  rkJointSetVel( &j, v.e );
  printf( "original displacement: " ); zVec6DDataWrite( &d );
  printf( "original velocity:     " ); zVec6DDataWrite( &v );

  /* rot. acc. and vel. */
  zVec6DCreate( &a, zRandF(-100,100), zRandF(-100,100), zRandF(-100,100), zRandF(-100,100), zRandF(-100,100), zRandF(-100,100) );
  rkJointSetAcc( &j, a.e );
  printf( "original acceleration: " ); zVec6DDataWrite( &a );
  zVec6DCatDRC( &v, DT, &a );
  zEndl();
  printf( "updated velocity:      " ); zVec6DDataWrite( &v );
  /* update attitude */
  zVec6DCopy( &d, &dn );
  rkJointCatDis( &j, dn.e, DT, v.e );
  printf( "updated displacement:  " ); zVec6DDataWrite( &dn );
  zEndl();
  /* discrete differentiation */
  rkJointSetDisCNT( &j, dn.e, DT );
  /* validation */
  rkJointGetDis( &j, dn.e );
  rkJointGetVel( &j, vn.e );
  rkJointGetAcc( &j, an.e );
  printf( "sampled displacement:  " ); zVec6DDataWrite( &dn );
  printf( "sampled velocity:      " ); zVec6DDataWrite( &vn );
  printf( "sampled acceleration:  " ); zVec6DDataWrite( &an );
  zEndl();
  printf( "velocity error:        " ); zVec6DDataWrite( zVec6DSub( &vn, &v, &err ) );
  printf( "acceleration error:    " ); zVec6DDataWrite( zVec6DSub( &an, &a, &err ) );

  rkJointDestroy( &j );
  return 0;
}
