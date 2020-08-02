#include <roki/rk_joint.h>

#define STEP 2000
#define DT 0.001
/* inertia */
#define I  0.1

void intg(rkJointRevolPrp *prp, double acc, double dt)
{
  prp->acc = acc;
  prp->vel += prp->acc * dt;
  prp->dis += prp->vel * dt;
}

int main(void)
{
  rkJoint j;
  register int i;
  rkJointRevolPrp *prp;

  rkJointAssign( &j, &rk_joint_revol );
  prp = j.prp;
  prp->dis = 0.2;
  prp->stiffness = 1.0;
  prp->viscosity = 0.5;

  for( i=0; i<=STEP; i++ ){
    rkJointGetKFriction( &j, &prp->trq );
    printf( "%g %.10g %.10g %.10g %.10g\n", DT*i, prp->dis, prp->vel, prp->acc, prp->trq );
    intg( prp, prp->trq / I, DT );
  }
  rkJointDestroy( &j );
  return 0;
}
