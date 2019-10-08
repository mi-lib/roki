/* this code is to check if the forward dynamics computation
   are correct, comparing with the analytical solutions on
   a simple pendulum.
 */
#include <roki/roki.h>

typedef struct{
  double dis, vel, trq;
} Test;

Test tests[] = {
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 10.0},
  {zDeg2Rad(30), 0.0, 0.0},
  {zDeg2Rad(30), 0.0, 10.0},
  {zDeg2Rad(60), 0.0, 0.0},
  {zDeg2Rad(60), 0.0, 10.0},
  {zDeg2Rad(30), zDeg2Rad(-30)*100.0, 0.0},
  {zDeg2Rad(30), zDeg2Rad(-30)*100.0, 10.0}
};

void check_test(rkChain *pend, zVec dis, zVec vel, zVec acc)
{
  rkLink *link;
  rkJoint *joint;
  register int i;
  int num;
  Test *test;
  zVec3D grav, tmp;
  double inertia, ans;

  num = sizeof(tests) / sizeof(Test);
  link = rkChainLink( pend, 0 );
  joint = rkChainLinkJoint( pend, 0 );
  /* gravity vector */
  zVec3DMul( RK_GRAVITY3D, -rkLinkMass(link), &grav );
  /* inertia along the joint axis at the joint origin*/
  inertia = rkLinkInertia(link)->e[2][2] + rkLinkMass(link) * zSqr( rkLinkCOM(link)->e[0] );

  for( i=0; i<num; i++ ){
    test = &tests[i];

    /* set test states */
    zVecSetElem( dis, 0, test->dis );
    rkChainFK( pend, dis );

    zVecSetElem( vel, 0, test->vel );
    rkChainSetJointVelAll( pend, vel );
    rkChainUpdateVel( pend );

    /* set test driving torque */
    rkJointMotorSetInput( joint, &test->trq );

    /* forward dynamics computation */
    rkChainABIUpdate( pend );

    /* analytical solution */
    zMulMat3DTVec3D( rkLinkWldAtt(link), &grav, &tmp );
    zVec3DOuterProd( rkLinkCOM(link), &tmp, &tmp );
    ans = ( test->trq + zVec3DInnerProd( &tmp, ZVEC3DZ )) / inertia;

    /* get answer */
    rkChainGetJointAccAll( pend, acc );
    zAssert( acceleration test, zIsTiny( ans - zVecElemNC(acc,0) ) );
  }
}

int main(void)
{
  rkChain pend;
  zVec dis, vel, acc;

  rkChainReadZTK( &pend, "simple_pend.ztk" );
  rkChainABIAlloc( &pend );
  dis = zVecAlloc( rkChainJointSize(&pend) );
  vel = zVecAlloc( rkChainJointSize(&pend) );
  acc = zVecAlloc( rkChainJointSize(&pend) );

  check_test( &pend, dis, vel, acc );

  rkChainDestroy( &pend );
  zVecFreeAO( 3, dis, vel, acc );
  return 0;
}
