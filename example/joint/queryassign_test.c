#include <roki/rk_joint.h>

void test(char *str)
{
  rkJoint j;

  if( rkJointQueryAssign( &j, str ) ){
    eprintf( "(original str=%s)\t %s\t... size=%d\n", str, rkJointTypeStr(&j), rkJointSize(&j) );
    rkJointDestroy( &j );
  } else{
    eprintf( "unknown joint type %s.\n", str );
  }
}

int main(void)
{
  test( "fix" );
  test( "revolute" );
  test( "prismatic" );
  test( "cylindrical" );
  test( "hooke" );
  test( "spherical" );
  test( "float" );
  test( "invalid" );
  return 0;
}
