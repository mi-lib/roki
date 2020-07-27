#include <roki/rk_joint.h>

int main(void)
{
  rkJoint joint;
  double dis[6];
  register int i, j;

  zRandInit();
  for( i=0; rk_joint_com[i]; i++ ){
    rkJointAssign( &joint, rk_joint_com[i] );
    printf( "[%s]\n", rkJointTypeStr( &joint ) );
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
