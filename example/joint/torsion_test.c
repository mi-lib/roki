#include <roki/rk_joint.h>

void check(rkJoint *joint, zVec6D *t, double dis[], zFrame3D *f)
{
  zFrame3D fo, fc;
  zVec6D tc, err;
  register int i;

  printf( "[%s]\n", rkJointTypeExpr(rkJointType(joint)) );
  printf( " torsion:\n" ); zVec6DWrite( t );
  printf( " displacement:" );
  for( i=0; i<6; i++ ) printf( " %g", dis[i] );
  printf( "\n" );

  zMulMatVec6D( zFrame3DAtt(f), t, &tc );
  zVec6DToFrame3DAA( &tc, &fo );
  rkJointSetDis( joint, dis );
  rkJointXfer( joint, &fo, &fc );
  zFrame3DError( &fc, f, &err );
  printf( "(error)=" );
  zVec6DDataNLWrite( &err );
  printf( "hit enter key." ); fflush( stdout );
  getchar();
}

int main(int argc, char *argv[])
{
  rkJoint joint;
  zVec6D d, t;
  zFrame3D f;
  double dis[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  int type;

  zRandInit();
  /* deviation */
  zVec3DCreate( zVec6DLin(&d),
    zRandF(-0.01,0.01), zRandF(-0.01,0.01), zRandF(-0.01,0.01) );
  zVec3DCreate( zVec6DAng(&d),
    zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
  zVec3DMulDRC( zVec6DAng(&d), zRandF(0,zPI)/zVec3DNorm(zVec6DAng(&d)) );
  /* distorted frame */
  zVec6DToFrame3DAA( &d, &f );

  printf( "+++ deviation +++\n" );
  zVec6DWrite( &d );

  for( type=RK_JOINT_FIXED; type<=RK_JOINT_FLOAT; type++ ){
    rkJointCreate( &joint, type );
    rkJointTorsion( &joint, &f, &t, dis );
    check( &joint, &t, dis, &f );
    rkJointDestroy( &joint );
  }
  return 0;
}
