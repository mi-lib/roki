#include <roki/rk_joint.h>

int main(void)
{
  rkJoint joint;
  double din, dout;
  register int i;
  FILE *fp;

  fp = fopen( "revol.conf", "r" );
  rkJointCreate( &joint, RK_JOINT_REVOL );
  rkJointQueryFRead( fp, "min", &joint );
  rkJointQueryFRead( fp, "max", &joint );
  fclose( fp );

  for( i=-90; i<=90; i++ ){
    din = zDeg2Rad(i);
    rkJointSetDis( &joint, &din );
    rkJointGetDis( &joint, &dout );
    printf( "%f %f\n", din, dout );
  }
  rkJointDestroy( &joint );
  return 0;
}
