#include <roki/rk_joint.h>

int main(void)
{
  rkJoint joint;
  double dis[6];
  register int i, j;

  zRandInit();
  for( i=RK_JOINT_FIXED; i<=RK_JOINT_FLOAT; i++ ){
    rkJointCreate( &joint, i );
    printf( "[%s]\n", rkJointTypeExpr( i ) );
    printf( "(  input displacement) =" );
    for( j=0; j<rkJointSize(&joint); j++ )
      printf( " %g", ( dis[j] = zRandF(-1,1) ) );
    printf( "\n" );
    rkJointSetDis( &joint, dis );

    rkJointGetDis( &joint, dis );
    printf( "( stored displacement) =" );
    for( j=0; j<rkJointSize(&joint); j++ )
      printf( " %g", dis[j] );
    printf( "\n" );
    printf( " ... %s\n", rkJointIsNeutral(&joint) ? "neutral" : "not neutral" );

    rkJointNeutral( &joint );
    rkJointGetDis( &joint, dis );
    printf( "(neutral displacement) =" );
    for( j=0; j<rkJointSize(&joint); j++ )
      printf( " %g", dis[j] );
    printf( "\n" );
    printf( " ... %s\n", rkJointIsNeutral(&joint) ? "neutral" : "not neutral" );

    rkJointDestroy( &joint );
  }
  return 0;
}
