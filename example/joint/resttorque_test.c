#include <roki/rk_joint.h>

#define STEP 2000
#define DT 0.001
/* inertia */
#define I  0.1

void intg(rkJointRevolState *stat, double acc, double dt)
{
  stat->acc = acc;
  stat->vel += stat->acc * dt;
  stat->dis += stat->vel * dt;
}

int main(void)
{
  rkJoint j;
  rkJointRevolState *stat;
  rkJointRevolPrp *prp;
  int i;

  rkJointAssign( &j, &rk_joint_revol );
  stat = j.state;
  prp = j.prp;
  stat->dis = 0.2;
  prp->stiffness = 1.0;
  prp->viscosity = 0.5;

  for( i=0; i<=STEP; i++ ){
    rkJointGetKFriction( &j, &stat->trq );
    printf( "%g %.10g %.10g %.10g %.10g\n", DT*i, stat->dis, stat->vel, stat->acc, stat->trq );
    intg( stat, stat->trq / I, DT );
  }
  rkJointDestroy( &j );
  return 0;
}
