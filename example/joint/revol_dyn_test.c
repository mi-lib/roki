#include <roki/rk_joint.h>

double dis_test(double t){ return t*t*t - 2*t; }
double vel_test(double t){ return 3*t*t - 2; }
double acc_test(double t){ return 6*t; }

#define STEP 100
#define DT   0.01

int main(void)
{
  rkJoint joint;
  double t, dis, vel, acc;
  register int i;

  rkJointCreate( &joint, RK_JOINT_REVOL );
  for( i=0; i<=STEP; i++ ){
    t = DT * i;
    dis = dis_test(t);
    rkJointSetDisCNT( &joint, &dis, DT );
    rkJointGetVel( &joint, &vel );
    rkJointGetAcc( &joint, &acc );
    printf( "%g %g %g %g %g %g %g\n", t, dis, dis_test(t), vel, vel_test(t), acc, acc_test(t) );
  }
  rkJointDestroy( &joint );
  return 0;
}
