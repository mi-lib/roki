#include <roki/rk_joint.h>

#define STEP 100
#define DT 0.01

int main(void)
{
  rkJoint j;
  zVec3D aa, v, a, aao, vo, ao, err;
  register int i;

  zRandInit();
  rkJointCreate( &j, RK_JOINT_SPHER );
  /* original displacement, velocity, acceleration */
  zVec3DCreate( &aa,zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
  zVec3DCreate( &v, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
  rkJointSetDis( &j, aa.e );
  rkJointSetVel( &j, v.e );

  for( i=0; i<=STEP; i++ ){
    /* rot. acc. and vel. */
    zVec3DCreate( &a, zRandF(-100,100), zRandF(-100,100), zRandF(-100,100) );
    zVec3DCatDRC( &v, DT, &a );
    /* update attitude */
    zVec3DCopy( &aa, &aao );
    rkJointCatDis( &j, aao.e, DT, v.e );
    /* discrete differentiation */
    rkJointSetDisCNT( &j, aao.e, DT );
    /* validation */
    rkJointGetDis( &j, aa.e );
    rkJointGetVel( &j, vo.e );
    rkJointGetAcc( &j, ao.e );
    printf( "%g %g\n", zVec3DNorm( zVec3DSub( &vo, &v, &err ) ), zVec3DNorm( zVec3DSub( &ao, &a, &err ) ) );
  }
  rkJointDestroy( &j );
  return 0;
}
