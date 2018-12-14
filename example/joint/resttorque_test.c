#include <roki/rk_joint.h>

#define STEP 2000
#define DT 0.001
/* inertia */
#define I  0.1

void intg(rkJointPrpRevol *prp, double acc, double dt)
{
  prp->acc = acc;
  prp->vel += prp->acc * dt;
  prp->dis += prp->vel * dt;
}

int main(void)
{
  rkJoint j;
  double t;
  register int i;
  rkJointPrpRevol *prp;

  rkJointCreate( &j, RK_JOINT_REVOL );
  prp = j.prp;
  prp->dis = 0.2;
  prp->stiff = 1.0;
  prp->viscos = 0.5;

  for( i=0; i<=STEP; i++ ){
    t = DT * i;
    rkJointCalcTrq( &j, Z_ZEROVEC6D );
    printf( "%f %f %f %f\n", prp->dis, prp->vel, prp->acc, prp->trq );
    intg( prp, prp->trq / I, DT );
  }
  rkJointDestroy( &j );
  return 0;
}
