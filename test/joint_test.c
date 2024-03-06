#include <roki/rk_joint.h>

#define N 1000

#define DT 0.01
#define TOL_VEL (1.0e-9)
#define TOL_ACC (1.0e-6)

bool assert_joint_assign_one(char *str, int size)
{
  rkJoint j;
  bool ret;

  ret = rkJointAssignByStr( &j, str ) != NULL;
  if( size >= 0 ){
    ret = ret && strcmp( str, rkJointTypeStr(&j) ) == 0 && rkJointDOF(&j) == size;
  } else{
    ret = ret == false;
  }
  rkJointDestroy( &j );
  return ret;
}

void assert_joint_assign(void)
{
  zAssert( rkJointAssignByStr,
    assert_joint_assign_one( "fixed", 0 ) &&
    assert_joint_assign_one( "revolute", 1 ) &&
    assert_joint_assign_one( "prismatic", 1 ) &&
    assert_joint_assign_one( "cylindrical", 2 ) &&
    assert_joint_assign_one( "hooke", 2 ) &&
    assert_joint_assign_one( "spherical", 3 ) &&
    assert_joint_assign_one( "float", 6 ) &&
    assert_joint_assign_one( "invalid", -1 ) );
}

void assert_limit_dis(void)
{
  rkJoint joint;
  double dmin, dmax;
  double din, dout;
  register int i;
  bool result = true;

  rkJointAssign( &joint, &rk_joint_revol );
  ((rkJointRevolPrp *)joint.prp)->min = dmin = zDeg2Rad(-45);
  ((rkJointRevolPrp *)joint.prp)->max = dmax = zDeg2Rad( 45);
  for( i=0; i<N; i++ ){
    din = zRandF(-10*zPI,10*zPI);
    rkJointSetDis( &joint, &din );
    rkJointGetDis( &joint, &dout );
    if( dout != zLimit( zPhaseNormalize(din), dmin, dmax ) ) result = false;
  }
  rkJointDestroy( &joint );
  zAssert( rkJointSetDis + rkJointGetDis, result );
}

void assert_joint_neutral(void)
{
  rkJoint joint;
  double dis[6];
  register int i, j;
  bool result = true;

  for( i=0; rk_joint_com[i]; i++ ){
    rkJointAssign( &joint, rk_joint_com[i] );
    for( j=0; j<rkJointDOF(&joint); j++ )
      dis[j] = zRandF(-1,1);
    rkJointSetDis( &joint, dis );
    rkJointGetDis( &joint, dis );

    rkJointNeutral( &joint );
    rkJointGetDis( &joint, dis );
    if( !rkJointIsNeutral(&joint) ){
      eprintf( "joint type = %s, joint displacement =", rkJointTypeStr(&joint) );
      for( j=0; j<rkJointDOF(&joint); j++ )
        eprintf( " %.10g", dis[j] );
      eprintf( "\n" );
      result = false;
    }
    rkJointDestroy( &joint );
  }
  zAssert( rkJointNeutral + rkJointIsNeutral, result );
}

void assert_spher_cat(void)
{
  rkJoint j;
  zVec3D aa, v, a, aao, vo, ao, err;
  register int i;
  bool result = true;

  rkJointAssign( &j, &rk_joint_spher );
  for( i=0; i<N; i++ ){
    /* original displacement, velocity, acceleration */
    zVec3DCreate( &aa,zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
    zVec3DCreate( &v, zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
    rkJointSetDis( &j, aa.e );
    rkJointSetVel( &j, v.e );
    /* rot. acc. and vel. */
    zVec3DCreate( &a, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
    rkJointSetAcc( &j, a.e );
    zVec3DCatDRC( &v, DT, &a );
    /* update attitude */
    zVec3DCopy( &aa, &aao );
    rkJointCatDis( &j, aao.e, DT, v.e );
    /* discrete differentiation */
    rkJointSetDisCNT( &j, aao.e, DT );
    /* validation */
    rkJointGetDis( &j, aa.e );
    rkJointGetVel( &j, vo.e );
    rkJointGetAcc( &j, ao.e );
    if( !zVec3DIsTol( zVec3DSub( &vo, &v, &err ), TOL_VEL ) ){
      eprintf( "velocity error = " );
      zVec3DDataFPrint( stderr, &err );
      eprintf( "\n" );
      result = false;
    }
    if( !zVec3DIsTol( zVec3DSub( &ao, &a, &err ), TOL_ACC ) ){
      eprintf( "acceleration error = " );
      zVec3DDataFPrint( stderr, &err );
      eprintf( "\n" );
      result = false;
    }
  }
  rkJointDestroy( &j );
  zAssert( rkJointSpherCatDis, result );
}

void assert_float_cat(void)
{
  rkJoint j;
  zVec6D d, v, a, dn, vn, an, err;
  register int i;
  bool result = true;

  rkJointAssign( &j, &rk_joint_float );
  for( i=0; i<N; i++ ){
    /* original displacement, velocity, acceleration */
    zVec6DCreate( &d, zRandF(-1,1), zRandF(-1,1), zRandF(-1,1), zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
    zVec6DCreate( &v, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10), zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
    rkJointSetDis( &j, d.e );
    rkJointSetVel( &j, v.e );
    /* rot. acc. and vel. */
    zVec6DCreate( &a, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10), zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
    rkJointSetAcc( &j, a.e );
    zVec6DCatDRC( &v, DT, &a );
    /* update attitude */
    zVec6DCopy( &d, &dn );
    rkJointCatDis( &j, dn.e, DT, v.e );
    /* discrete differentiation */
    rkJointSetDisCNT( &j, dn.e, DT );
    /* validation */
    rkJointGetDis( &j, dn.e );
    rkJointGetVel( &j, vn.e );
    rkJointGetAcc( &j, an.e );
    if( !zVec6DIsTol( zVec6DSub( &vn, &v, &err ), TOL_VEL ) ){
      eprintf( "velocity error = " );
      zVec6DDataFPrint( stderr, &err );
      eprintf( "\n" );
      result = false;
    }
    if( !zVec6DIsTol( zVec6DSub( &an, &a, &err ), TOL_ACC ) ){
      eprintf( "acceleration error = " );
      zVec6DDataFPrint( stderr, &err );
      eprintf( "\n" );
      result = false;
    }
  }
  rkJointDestroy( &j );
  zAssert( rkJointFloatCatDis, result );
}

void assert_joint_sub(void)
{
  RK_JOINT_COM_ARRAY;
  rkJoint joint;
  int i, k;
  zVec6D dis1, dis2, ddis;
  bool result = true;

  for( k=0; k<N; k++ ){
    for( i=0; i<6; i++ ){
      dis1.e[i] = zRandF( -zPI, zPI );
      ddis.e[i] = zRandF( -zPI, zPI );
    }
    for( i=0; rk_joint_com[i]; i++ ){
      rkJointAssign( &joint, rk_joint_com[i] );
      zVec6DCopy( &dis1, &dis2 );
      rkJointSubDis( &joint, dis2.e, ddis.e );
      rkJointCatDis( &joint, dis2.e, 1, ddis.e );
      rkJointSubDis( &joint, dis2.e, dis1.e );
      zVec6DZero( &ddis );
      memcpy( dis2.e, ddis.e, sizeof(double)*rkJointDOF(&joint) );
      if( !zVec6DIsTiny( &ddis ) ) result = false;
      rkJointDestroy( &joint );
    }
  }
  zAssert( rkJointSubDis, result );
}

bool assert_joint_torsion_check(rkJoint *joint, zVec6D *t, double dis[], zFrame3D *f)
{
  zFrame3D fo, fc;
  zVec6D tc, err;

  zMulMat3DVec6D( zFrame3DAtt(f), t, &tc );
  zVec6DToFrame3DAA( &tc, &fo );
  rkJointSetDis( joint, dis );
  rkJointXform( joint, &fo, &fc );
  zFrame3DError( &fc, f, &err );
  if( !zVec6DIsTol( &err, zTOL*10 ) ){
    eprintf( "joint type = %s, error = ", rkJointTypeStr(joint) );
    zVec6DDataFPrint( stderr, &err );
    eprintf( "\n" );
    return false;
  }
  return true;
}

void assert_joint_torsion(void)
{
  rkJoint joint;
  zVec6D d, t;
  zFrame3D f;
  double dis[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  int type;
  register int i;
  bool result = true;

  for( type=0; rk_joint_com[type]; type++ ){
    rkJointAssign( &joint, rk_joint_com[type] );
    for( i=0; i<N; i++ ){
      /* deviation */
      zVec3DCreate( zVec6DLin(&d), zRandF(-0.01,0.01), zRandF(-0.01,0.01), zRandF(-0.01,0.01) );
      zVec3DCreate( zVec6DAng(&d), zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
      zVec3DMulDRC( zVec6DAng(&d), zRandF(0,zPI)/zVec3DNorm(zVec6DAng(&d)) );
      /* distorted frame */
      zVec6DToFrame3DAA( &d, &f );

      memset( dis, 0, sizeof(dis) );
      rkJointTorsion( &joint, &f, &t, dis );
      if( !assert_joint_torsion_check( &joint, &t, dis, &f ) ) result = false;
    }
    rkJointDestroy( &joint );
  }
  zAssert( rkJointTorsion, result );
}

int main(void)
{
  zRandInit();
  assert_joint_assign();
  assert_limit_dis();
  assert_joint_neutral();
  assert_spher_cat();
  assert_float_cat();
  assert_joint_sub();
  assert_joint_torsion();
  return 0;
}
