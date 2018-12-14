#include <roki/rk_joint.h>

void test(char *str)
{
  byte type;
  rkJoint j;

  rkJointCreate( &j, ( type = rkJointTypeFromStr( str ) ) );
  printf( "(original str=%s)\t<%d> %s\t... size=%d\n", str, type, rkJointTypeExpr( type ), rkJointSize(&j) );
  rkJointDestroy( &j );
}

int main(void)
{
  test( "fix" );
  test( "revolute" );
  test( "prism" );
  test( "cylinder" );
  test( "hooke" );
  test( "sphere" );
  test( "float" );
  test( "invalid" );
  return 0;
}
