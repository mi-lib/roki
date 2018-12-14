#include <roki/rk_joint.h>

#define STEP 100
#define DT 0.01

int main(void)
{
  rkJoint j;
  zVec6D d, v, a, dn, vn, an, ve, ae;
  register int i;

  zRandInit();
  rkJointCreate( &j, RK_JOINT_FLOAT );
  /* original displacement, velocity, acceleration */
  zVec6DCreate( &d, zRandF(-1,1), zRandF(-1,1), zRandF(-1,1), zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
  zVec6DCreate( &v, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10), zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
  rkJointSetDis( &j, d.e );
  rkJointSetVel( &j, v.e );

  for( i=0; i<=STEP; i++ ){
    /* rot. acc. and vel. */
    zVec6DCreate( &a, zRandF(-100,100), zRandF(-100,100), zRandF(-100,100), zRandF(-100,100), zRandF(-100,100), zRandF(-100,100) );
    rkJointSetAcc( &j, a.e );
    zVec6DCatDRC( &v, DT, &a );
    /* update attitude */
    zVec6DCopy( &d, &dn );
    rkJointCatDis( &j, dn.e, DT, v.e );
    /* discrete differentiation */
    rkJointSetDisCNT( &j, dn.e, DT );
    /* validation */
    rkJointGetDis( &j, d.e );
    rkJointGetVel( &j, vn.e );
    rkJointGetAcc( &j, an.e );
    zVec6DSub( &vn, &v, &ve );
    zVec6DSub( &an, &a, &ae );
    printf( "%g %g %g %g\n", zVec3DNorm(zVec6DLin(&ve)), zVec3DNorm(zVec6DAng(&ve)), zVec3DNorm(zVec6DLin(&ae)), zVec3DNorm(zVec6DAng(&ae)) );
  }
  rkJointDestroy( &j );
  return 0;
}
