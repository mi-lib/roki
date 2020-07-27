#include <roki/rk_joint.h>

int main(void)
{
  rkJoint joint;
  double din, dout;
  register int i;

  rkJointAssign( &joint, &rk_joint_revol );
  ((rkJointRevolPrp *)joint.prp)->min = zDeg2Rad(-45);
  ((rkJointRevolPrp *)joint.prp)->max = zDeg2Rad( 45);
  for( i=-90; i<=90; i++ ){
    din = zDeg2Rad(i);
    rkJointSetDis( &joint, &din );
    rkJointGetDis( &joint, &dout );
    printf( "%f %f\n", din, dout );
  }
  rkJointDestroy( &joint );
  return 0;
}
