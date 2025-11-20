#include <roki/rk_chain.h>
#include <roki/rk_abi.h>

#ifndef __WINDOWS__
#include <unistd.h>
#endif
void assert_chain_clone(void)
{
  const char *file_org = "org.ztk";
  const char *file_cln = "cln.ztk";
  rkChain chain_org, chain_cln;
  long ret;

  rkChainReadZTK( &chain_org, "../example/model/mighty.ztk" );
  rkChainClone( &chain_org, &chain_cln );
  zNameFree( &chain_cln );
  zNameSet( &chain_cln, zNamePtr(&chain_org) );
  rkChainWriteZTK( &chain_org, file_org );
  rkChainWriteZTK( &chain_cln, file_cln );
  ret = zFileCompare( file_org, file_cln );
  rkChainDestroy( &chain_org );
  rkChainDestroy( &chain_cln );
  zAssert( rkChainClone, ret == 0 );
#ifndef __WINDOWS__
  unlink( file_org );
  unlink( file_cln );
#else
  _unlink( file_org );
  _unlink( file_cln );
#endif
}

void assert_chain_clone_irregular(void)
{
  rkChain org, cln;
  rkIKAttr attr;
  rkIKCell *result;

  rkChainReadZTK( &org, "../example/model/puma.ztk" );
  rkChainRegisterIKJointAll( &org, 0.001 );
  rkChainClone( &org, &cln );
  attr.id = 6;
  result = rkChainRegisterIKCellWldAtt( &cln, NULL, 0, &attr, RK_IK_ATTR_MASK_ID );
  rkChainDestroy( &cln );
  rkChainDestroy( &org );
  zAssert( rkChainClone (null IK constraints), result );
}

bool check_inertia_matrix(rkChain *chain, zMat inertia, double tol)
{
  zMat h;
  bool ret;

  h = zMatAllocSqr( rkChainJointSize(chain) );
  rkChainInertiaMatMJ( chain, h );
  ret = zMatEqual( h, inertia, tol );
  zMatFree( h );
  return ret;
}

bool check_kinetic_energy(rkChain *chain, zMat inertia, zVec vel, double tol)
{
  zVec tmp;
  double ke;

  tmp = zVecAlloc( rkChainJointSize( chain ) );
  zMulMatVec( inertia, vel, tmp );
  ke = 0.5 * zVecInnerProd( vel, tmp );
  zVecFree( tmp );
  return zEqual( rkChainKineticEnergy( chain ), ke, tol );
}

bool check_fd(rkChain *chain, zMat inertia, zVec bias, zVec dis, zVec vel, double tol)
{
  zVec acc, trq_fd, trq_id;
  int n;
  bool ret;

  n = rkChainJointSize( chain );
  /* calc trq */
  acc = zVecAlloc( n );
  trq_fd = zVecAlloc( n );
  trq_id = zVecAlloc( n );
  zVecRandUniform( acc, -1.0, 1.0 );
  /* inverse dynamics */
  rkChainID( chain, dis, vel, acc, trq_id );
  /* forward dynamics */
  zMulMatVec( inertia, acc, trq_fd );
  zVecAddDRC( trq_fd, bias );
  /* check */
  ret = zVecEqual( trq_fd, trq_id, tol );
  zVecFreeAtOnce( 3, acc, trq_fd, trq_id );
  return ret;
}

void link_mp_rand(rkLink *l)
{
  double i11, i12, i13, i22, i23, i33;
  zVec3D com;
  zMat3D inertia;

  rkLinkSetMass( l, zRandF(0.1,1.0) );
  zVec3DCreate( &com, zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
  rkLinkSetCOM( l, &com );
  i11 = zRandF(0.01,0.1);
  i12 =-zRandF(0.0001,0.001);
  i13 =-zRandF(0.0001,0.001);
  i22 = zRandF(0.01,0.1);
  i23 =-zRandF(0.00001,0.0001);
  i33 = zRandF(0.01,0.1);
  zMat3DCreate( &inertia, i11, i12, i13, i12, i22, i23, i13, i23, i33 );
  rkLinkSetInertia( l, &inertia );
}

#define LINK_NUM 9
int chain_init(rkChain *chain)
{
  int i;
  char name[BUFSIZ];
  zVec3D aa;

  rkChainInit( chain );
  rkLinkArrayAlloc( rkChainLinkArray(chain), LINK_NUM );
  for( i=0; i<LINK_NUM; i++ ){
    sprintf( name, "link#%02d", i );
    rkLinkInit( rkChainLink(chain,i) );
    link_mp_rand( rkChainLink(chain,i) );
    zVec3DCreate( &aa, zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
    zVec3DCreate( rkChainLinkOrgPos(chain,i), zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
    zMat3DFromAA( rkChainLinkOrgAtt(chain,i), &aa );
    zNameSet( rkChainLink(chain,i), name );
  }
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );
  rkLinkAddChild( rkChainLink(chain,1), rkChainLink(chain,2) );
  rkLinkAddChild( rkChainLink(chain,2), rkChainLink(chain,3) );
  rkLinkAddChild( rkChainLink(chain,3), rkChainLink(chain,4) );
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,5) );
  rkLinkAddChild( rkChainLink(chain,5), rkChainLink(chain,6) );
  rkLinkAddChild( rkChainLink(chain,6), rkChainLink(chain,7) );
  rkLinkAddChild( rkChainLink(chain,7), rkChainLink(chain,8) );
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_float ); /* 0-5 */
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_spher ); /* 6-8 */
  rkJointAssign( rkChainLinkJoint(chain,2), &rk_joint_revol ); /* 9 */
  rkJointAssign( rkChainLinkJoint(chain,3), &rk_joint_cylin ); /* 10-11 */
  rkJointAssign( rkChainLinkJoint(chain,4), &rk_joint_plana ); /* 12-14 */
  rkJointAssign( rkChainLinkJoint(chain,5), &rk_joint_revol ); /* 15 */
  rkJointAssign( rkChainLinkJoint(chain,6), &rk_joint_prism ); /* 16 */
  rkJointAssign( rkChainLinkJoint(chain,7), &rk_joint_hooke ); /* 17-18 */
  rkJointAssign( rkChainLinkJoint(chain,8), &rk_joint_fixed );

  rkChainSetJointIDOffset( chain );
  rkChainUpdateCRBMass( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  return rkChainJointSize( chain );
}

void assert_chain_joint_index(void)
{
  rkChain chain;
  zVec dis_src, dis_dest, dis_selected;
  zIndex index, index_selected;
  int jointsize, i;
  bool result1, result2, result3;

  jointsize = chain_init( &chain );
  dis_src  = zVecAlloc( jointsize );
  dis_dest = zVecAlloc( jointsize );
  for( i=0; i<zVecSizeNC(dis_src); i++ )
    zVecSetElemNC( dis_src, i, 0.1 * i );

  index = rkChainCreateDefaultJointIndex( &chain );
  index_selected = zIndexCreateList( 4, 5, 6, 7, 8 );
  dis_selected = zVecAlloc( rkChainJointIndexSize(&chain,index_selected) );
  result1 = zVecSizeNC(dis_selected) == 4;
  rkChainSetJointDis( &chain, index, dis_src );
  rkChainUpdateFK( &chain );
  rkChainGetJointDis( &chain, index, dis_dest );
  result2 = zVecEqual( dis_src, dis_dest, zTOL );
  rkChainGetJointDis( &chain, index_selected, dis_selected );
  result3 = zEqual( zVecElemNC(dis_selected,0), 1.5, zTOL ) &&
            zEqual( zVecElemNC(dis_selected,1), 1.6, zTOL ) &&
            zEqual( zVecElemNC(dis_selected,2), 1.7, zTOL ) &&
            zEqual( zVecElemNC(dis_selected,3), 1.8, zTOL );
  zIndexFree( index );
  zIndexFree( index_selected );
  zVecFree( dis_src );
  zVecFree( dis_dest );
  zVecFree( dis_selected );
  rkChainDestroy( &chain );

  zAssert( rkChainJointIndexSize, result1 );
  zAssert( rkChainSetJointDis + rkChainGetJointDis (all), result2 );
  zAssert( rkChainSetJointDis + rkChainGetJointDis (selected), result3 );
}

#define N 1000
#define TOL (1.0e-10)

bool test_chain_state(rkChain *src, rkChain *dest)
{
  int tip;

  tip = rkChainLinkNum(src) - 1;
  /* frame */
  if( !zFrame3DMatch( rkChainLinkWldFrame(src,tip), rkChainLinkWldFrame(dest,tip) ) ) return false;
  /* velocity */
  if( !zVec6DMatch( rkChainLinkVel(src,tip), rkChainLinkVel(dest,tip) ) ) return false;
  /* acceleration */
  if( !zVec6DMatch( rkChainLinkAcc(src,tip), rkChainLinkAcc(dest,tip) ) ) return false;
  /* wrench */
  if( !zVec6DMatch( rkChainRootWrench(src), rkChainRootWrench(dest) ) ) return false;
  /* COM */
  if( !zVec3DMatch( rkChainLinkWldCOM(src,tip), rkChainLinkWldCOM(dest,tip) ) ) return false;
  /* COM velocity */
  if( !zVec3DMatch( rkChainLinkCOMVel(src,tip), rkChainLinkCOMVel(dest,tip) ) ) return false;
  /* COM acceleration */
  if( !zVec3DMatch( rkChainLinkCOMAcc(src,tip), rkChainLinkCOMAcc(dest,tip) ) ) return false;
  /* chain COM */
  if( !zVec3DMatch( rkChainWldCOM(src), rkChainWldCOM(dest) ) ) return false;
  /* chain COM velocity */
  if( !zVec3DMatch( rkChainCOMVel(src), rkChainCOMVel(dest) ) ) return false;
  /* chain COM acceleration */
  if( !zVec3DMatch( rkChainCOMAcc(src), rkChainCOMAcc(dest) ) ) return false;
  return true;
}

void assert_chain_copy_state(void)
{
  rkChain chain, chain_copy;
  zVec dis, vel;
  bool result;
  const double dt = 0.001;

  chain_init( &chain );
  chain_init( &chain_copy );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  vel = zVecAlloc( rkChainJointSize(&chain) );

  zVecRandUniform( dis, -1.0, 1.0 );
  zVecRandUniform( vel, -0.1, 0.1 );
  rkChainFK( &chain, dis );
  zVecCatNCDRC( dis, dt, vel );
  rkChainFKCNT( &chain, dis, dt );
  rkChainCopyState( &chain, &chain_copy );
  result = test_chain_state( &chain, &chain_copy );

  zVecFree( dis );
  zVecFree( vel );
  rkChainDestroy( &chain );
  rkChainDestroy( &chain_copy );
  zAssert( rkChainCopyState, result );
}

void assert_chain_getsetconf(void)
{
  rkChain chain;
  zVec orgdis, orgconf, dis, conf;
  int i;
  bool result1 = true, result2 = true;

  chain_init( &chain );
  orgdis = zVecAlloc( rkChainJointSize( &chain ) );
  dis = zVecAlloc( rkChainJointSize( &chain ) );
  orgconf = zVecAlloc( rkChainLinkNum( &chain ) * 6 );
  conf = zVecAlloc( rkChainLinkNum( &chain ) * 6 );
  for( i=0; i<N; i++ ){
    /* displacement -> configuration -> displacement */
    zVecRandUniform( orgdis, -1.0, 1.0 );
    rkChainFK( &chain, orgdis );
    rkChainGetConf( &chain, conf );
    rkChainSetConf( &chain, conf );
    rkChainGetJointDisAll( &chain, dis );
    zVecSubDRC( dis, orgdis );
    if( !zVecIsTol( dis, TOL ) ){
      eprintf( "(rkChainFK + rkChainGetConf + rkChainSetConf + rkChainGetJointDisAll) error abs max = %.10g\n", zVecElemAbsMax( dis, NULL ) );
      result1 = false;
    }
    /* configuration -> displacement -> configuration */
    zVecRandUniform( orgconf, -1.0, 1.0 );
    rkChainSetConf( &chain, orgconf );
    rkChainGetJointDisAll( &chain, orgdis );
    rkChainFK( &chain, orgdis );
    rkChainGetConf( &chain, orgconf );
    rkChainSetConf( &chain, orgconf );
    rkChainGetJointDisAll( &chain, dis );
    rkChainGetConf( &chain, conf );
    zVecSubDRC( conf, orgconf );
    if( !zVecIsTol( conf, TOL ) ){
      eprintf( "(rkChainSetConf + rkChainGetJointDisAll + rkChainFK + rkChainGetConf) error abs max = %.10g\n", zVecElemAbsMax( conf, NULL ) );
      result2 = false;
    }
  }
  zAssert( rkChainFK + rkChainGetConf + rkChainSetConf + rkChainGetJointDisAll, result1 );
  zAssert( rkChainSetConf + rkChainGetJointDisAll + rkChainFK + rkChainGetConf, result2 );

  zVecFree( orgconf );
  zVecFree( conf );
  zVecFree( orgdis );
  zVecFree( dis );
  rkChainDestroy( &chain );
}

bool check_chain_net_inertia(rkChain *chain, zMat3D *inertia_net)
{
  zMat3D inertia;
  zVec3D r;
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ ){
    zRotMat3D( rkChainLinkWldAtt(chain,i), rkChainLinkInertia(chain,i), &inertia );
    zVec3DSub( rkChainLinkWldCOM(chain,i), rkChainWldCOM(chain), &r );
    zMat3DCatVec3DDoubleOuterProdDRC( &inertia, -rkChainLinkMass(chain,i), &r );
    zMat3DSubDRC( inertia_net, &inertia );
  }
  return zMat3DIsTiny( inertia_net );
}

void assert_chain_com_inertia(void)
{
  rkChain chain;
  zVec dis;
  rkMP mp1, mp2;

  chain_init( &chain );
  dis = zVecAlloc( rkChainJointSize( &chain ) );
  zVecRandUniform( dis, -zPI, zPI );
  rkChainFK( &chain, dis );
  rkChainUpdateCRB( &chain );
  rkMPXform( rkLinkCRB(rkChainRoot(&chain)), rkChainRootFrame(&chain), &mp1 );
  rkChainCombineMP( &chain, &mp2 );
  zVecFree( dis );
  rkChainDestroy( &chain );
  zAssert( rkChainCombineMP,
    zEqual( rkMPMass(&mp1), rkMPMass(&mp2), zTOL ) &&
    zVec3DEqual( rkMPCOM(&mp1), rkMPCOM(&mp2) ) &&
    zMat3DEqual( rkMPInertia(&mp1), rkMPInertia(&mp2) ) );
}

void assert_chain_momentum(void)
{
  rkChain chain;
  zVec dis, vel;
  zVec3D momentum, momentum_recursive, momentum_gt;
  zVec3D angularmomentum, angularmomentum_recursive;

  chain_init( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  vel = zVecAlloc( rkChainJointSize(&chain) );
  zVecRandUniform( dis, -1.0, 1.0 );
  zVecRandUniform( vel, -0.1, 0.1 );
  rkChainFK( &chain, dis );
  rkChainSetJointVelAll( &chain, vel );
  rkChainUpdateVel( &chain );

  rkChainLinearMomentum( &chain, &momentum );
  rkChainLinearMomentumRecursive( &chain, &momentum_recursive );
  rkChainUpdateCOMVel( &chain );
  zVec3DMul( rkChainCOMVel(&chain), rkChainMass(&chain), &momentum_gt );
  rkChainAngularMomentum( &chain, ZVEC3DZERO, &angularmomentum );
  rkChainAngularMomentumRecursive( &chain, ZVEC3DZERO, &angularmomentum_recursive );

  zVecFree( dis );
  zVecFree( vel );
  rkChainDestroy( &chain );

  zAssert( rkChainLinearMomentum & rkChainLinearMomentumRecursive, zVec3DEqual( &momentum, &momentum_recursive ) );
  zAssert( rkChainLinearMomentum & rkChainUpdateCOMVel, zVec3DEqual( &momentum, &momentum_gt ) );
  zAssert( rkChainAngularMomentum & rkChainAngularMomentumRecursive, zVec3DEqual( &angularmomentum, &angularmomentum_recursive ) );
}

int simple_chain_create(rkChain *chain)
{
  char name[BUFSIZ];
  double mass;
  zVec3D com;
  zMat3D inertia;
  zBox3D box;

  rkChainInit( chain );
  rkLinkArrayAlloc( rkChainLinkArray(chain), 1 );
  sprintf( name, "link" );
  rkLinkInit( rkChainRoot(chain) );
  mass = 2.0;
  zVec3DCreate( &com, 1.0, 2.0, 1.0 );
  zBox3DCreateAlign( &box, &com, fabs(com.c.x), fabs(com.c.y), fabs(com.c.z) );
  zBox3DBaryInertiaMass( &box, mass, &inertia );
  rkLinkSetMass( rkChainRoot(chain), mass );
  rkLinkSetCOM( rkChainRoot(chain), &com );
  rkLinkSetInertia( rkChainRoot(chain), &inertia );
  zVec3DCreate( rkChainLinkOrgPos(chain,0), 0.0, 0.0, 0.0 );
  zNameSet( rkChainRoot(chain), name );
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_float );

  rkChainSetJointIDOffset( chain );
  rkChainUpdateCRBMass( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  return rkChainJointSize( chain );
}

void assert_chain_gravitydir(void)
{
  rkChain chain;
  double angle;
  zVec dis;
  zVec3D a, a_ans;
  const int step = 100;
  int i;
  bool result1 = true, result2 = true, result3 = true;

  rkChainInit( &chain );
  zNameSet( &chain, "floatbody" );
  rkLinkArrayAlloc( rkChainLinkArray(&chain), 1 );
  rkLinkInit( rkChainLink(&chain,0) );
  zNameSet( rkChainLink(&chain,0), "body" );
  rkJointAssign( rkChainLinkJoint(&chain,0), &rk_joint_float );
  rkChainSetMass( &chain, 1.0 );
  rkChainSetJointIDOffset( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );

  for( i=0; i<=step; i++ ){
    angle = zPI*2*i/step;
    zVecSetElem( dis, 3, angle );
    zVec3DCreate( &a_ans, 0, sin(angle), cos(angle) );
    rkChainFK( &chain, dis );
    if( !zVec3DEqual( rkChainGravityDir( &chain, &a ), &a_ans ) ) result1 = false;
  }
  zVecZero( dis );
  for( i=0; i<=step; i++ ){
    angle = zPI*2*i/step;
    zVecSetElem( dis, 4, angle );
    zVec3DCreate( &a_ans, -sin(angle), 0, cos(angle) );
    rkChainFK( &chain, dis );
    if( !zVec3DEqual( rkChainGravityDir( &chain, &a ), &a_ans ) ) result2 = false;
  }
  zVecZero( dis );
  for( i=0; i<=step; i++ ){
    angle = zPI*2*i/step;
    zVecSetElem( dis, 5, angle );
    zVec3DCreate( &a_ans, 0, 0, 1 );
    rkChainFK( &chain, dis );
    if( !zVec3DEqual( rkChainGravityDir( &chain, &a ), &a_ans ) ) result3 = false;
  }
  zVecFree( dis );
  rkChainDestroy( &chain );

  zAssert( rkChainGravityDir (rolling),  result1 );
  zAssert( rkChainGravityDir (pitching), result2 );
  zAssert( rkChainGravityDir (yawing),   result3 );
}

void assert_chain_zmp_simple(void)
{
  rkChain box;
  zVec3D zmp;
  zVec jointdis, jointtrq;
  bool result = true;
  int i;
  const int n = 100;

  simple_chain_create( &box );
  jointdis = zVecAlloc( rkChainJointSize( &box ) );
  jointtrq = zVecAlloc( rkChainJointSize( &box ) );
  for( i=0; i<n; i++ ){
    zVecSetElemList( jointdis, zRandF(-5,5), zRandF(-5,5), zRandF(-5,5), zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
    rkChainFK( &box, jointdis );
    rkChainUpdateID( &box );
    rkChainGetJointTrqAll( &box, jointtrq );
    rkChainZMP( &box, 0, &zmp );
    zVec3DSubDRC( &zmp, rkChainLinkWldCOM(&box,0) );
    if( !zIsTiny( zmp.c.x ) || !zIsTiny( zmp.c.y ) ||
        !zIsTiny( zVecElemNC(jointtrq,0) ) ||
        !zIsTiny( zVecElemNC(jointtrq,1) ) ||
        !zIsTiny( zVecElemNC(jointtrq,2) - rkChainMass(&box)*RK_G ) ||
        !zIsTiny( zVecElemNC(jointtrq,5) ) ) result = false;
  }
  zVecFree( jointdis );
  zVecFree( jointtrq );
  rkChainDestroy( &box );
  zAssert( rkChainZMP (a simple body), result );
}

void assert_chain_crb(void)
{
  rkChain chain;
  zVec dis;
  zVec3D com;
  zMat3D inertia;

  chain_init( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  zVecRandUniform( dis, -10, 10 );
  rkChainFK( &chain, dis );
  rkChainUpdateCRB( &chain );
  zXform3D( rkLinkAdjFrame(rkChainRoot(&chain)), rkMPCOM(rkLinkCRB(rkChainRoot(&chain))), &com );
  zRotMat3D( rkLinkAdjAtt(rkChainRoot(&chain)), rkMPInertia(rkLinkCRB(rkChainRoot(&chain))), &inertia );
  zAssert( rkChainUpdateCRB,
    zIsTiny( rkChainMass(&chain) - rkMPMass(rkLinkCRB(rkChainRoot(&chain))) ) &&
    zVec3DEqual( rkChainWldCOM(&chain), &com ) &&
    check_chain_net_inertia( &chain, &inertia ) );
  zVecFree( dis );
  rkChainDestroy( &chain );
}

void assert_chain_inertia_mat(void)
{
  rkChain chain;
  zMat h;
  zVec b, dis, vel;
  int i, n, count_iuv, count_icrb, count_ke, count_fd;
  clock_t c1, c2;
  long l_mj, l_uv, l_crb;

  n = chain_init( &chain );
  h = zMatAllocSqr( n );
  dis = zVecAlloc( n );
  vel = zVecAlloc( n );
  b = zVecAlloc( n );
  count_iuv = count_icrb = count_ke = count_fd = 0;
  for( i=0; i<N; i++ ){
    /* generate posture and velocity randomly */
    zVecRandUniform( dis, -10, 10 );
    zVecRandUniform( vel, -10, 10 );
    rkChainFK( &chain, dis );
    rkChainSetJointVelAll( &chain, vel );
    /* verifications */
    rkChainInertiaMatBiasVec( &chain, h, b );
    /* count success */
    if( check_inertia_matrix( &chain, h, TOL ) ) count_icrb++;
    if( check_kinetic_energy( &chain, h, vel, TOL ) ) count_ke++;
    if( check_fd( &chain, h, b, dis, vel, TOL ) ) count_fd++;
  }
  zAssert( rkChainInertiaMatBiasVec, count_icrb == N );
  zAssert( rkChainInertiaMatBiasVec + rkChainKE, count_ke == N );
  zAssert( rkChainInertiaMatBiasVec (FD-ID), count_fd == N );
  count_iuv = count_icrb = 0;
  l_mj = l_uv = l_crb = 0;
  for( i=0; i<N; i++ ){
    /* generate posture and velocity randomly */
    zVecRandUniform( dis, -10, 10 );
    rkChainFK( &chain, dis );
    c1 = clock();
    rkChainInertiaMatMJ( &chain, h );
    c2 = clock();
    l_mj += c2 - c1;
    c1 = clock();
    rkChainInertiaMatUV( &chain, h );
    c2 = clock();
    l_uv += c2 - c1;
    if( check_inertia_matrix( &chain, h, TOL ) ) count_iuv++;
    c1 = clock();
    rkChainInertiaMatCRB( &chain, h );
    c2 = clock();
    l_crb += c2 - c1;
    if( check_inertia_matrix( &chain, h, TOL ) ) count_icrb++;
  }
  eprintf( "clock (MJ/UV/CRB): %ld %ld %ld\n", l_mj, l_uv, l_crb );
  zAssert( rkChainInertiaMatUV, count_iuv == N );
  zAssert( rkChainInertiaMatCRB, count_icrb == N );

  zMatFree( h );
  zVecFreeAtOnce( 3, b, dis, vel );
  rkChainDestroy( &chain );
}

void assert_chain_fd_id(void)
{
  rkChain chain;
  zVec dis, vel, acc, trq, trq_id;
  int i, size, fd_id_count_success = 0, com_acc_count_success = 0;
  zVec3D f1, f2;

  size = chain_init( &chain );
  dis = zVecAlloc( size );
  vel = zVecAlloc( size );
  acc = zVecAlloc( size );
  trq = zVecAlloc( size );
  trq_id = zVecAlloc( size );

  for( i=0; i<N; i++ ){
    zVecRandUniform( dis, -1.0, 1.0 );
    zVecRandUniform( vel, -1.0, 1.0 );
    zVecRandUniform( trq, -1.0, 1.0 );
    rkChainFD( &chain, dis, vel, trq, acc );
    rkChainID( &chain, dis, vel, acc, trq_id );
    if( zVecEqual( trq, trq_id, zTOL ) ){
      fd_id_count_success++;
    } else{
      eprintf( "Failure case : RMSE = %.10g\n", zVecDist( trq, trq_id ) );
      eprintf( " (error) = " ); zVecPrint( zVecSubDRC( trq_id, trq ) );
    }
    zVec3DMul( rkChainCOMAcc(&chain), rkChainMass(&chain), &f1 );
    zMulMat3DVec3D( rkChainRootAtt(&chain), rkChainRootForce(&chain), &f2 );
    if( zVec3DEqual( &f1, &f2 ) ){
      com_acc_count_success++;
    } else{
      eprintf( " (error) = " ); zVec3DPrint( zVec3DSubDRC( &f1, &f2 ) );
    }
  }
  zVecFreeAtOnce( 5, dis, vel, acc, trq, trq_id );
  rkChainDestroy( &chain );
  eprintf( "Success rate = %d / %d\n", fd_id_count_success, N );
  zAssert( rkChainFD + rkChainID, fd_id_count_success == N );
  eprintf( "Success rate = %d / %d\n", com_acc_count_success, N );
  zAssert( rkChainUpdateCOMAcc + rkChainID, com_acc_count_success == N );
}

/* only works with torque-controlled robot models. */
void assert_chain_fd_id_abi(void)
{
  rkChain chain;
  zVec dis, vel, acc, expected, actual, err;
  int n, i, count_success = 0;

  rkChainReadZTK( &chain, "../example/model/arm_2DoF_trq.ztk" ); /* torque-controlled robot */
  rkChainAllocABI( &chain );
  n = rkChainJointSize( &chain );
  dis = zVecAlloc( n );
  vel = zVecAlloc( n );
  acc = zVecAlloc( n );
  expected = zVecAlloc( n );
  actual = zVecAlloc( n );
  err = zVecAlloc( n );
  for( i=0; i<N; i++ ){
    zVecRandUniform( dis, 10, -10 );
    zVecRandUniform( vel, 10, -10 );
    zVecRandUniform( expected, 10, -10 );

    rkChainSetMotorInputAll( &chain, expected );
    rkChainFD_ABI( &chain, dis, vel, acc ); /* forward dynamics (ABI method) */
    rkChainID( &chain, dis, vel, acc, actual ); /* inverse dynamics (Newton-Euler method) */
    if( zVecEqual( actual, expected, zTOL ) ){
      count_success++;
    } else{
      eprintf( "Failure case : RMSE = %.10g\n", zVecDist( expected, actual ) );
      eprintf( " (error) = " ); zVecPrint( zVecSub( expected, actual, err ) );
    }
  }
  eprintf( "Success rate = %d / %d\n", count_success, N );
  rkChainDestroyABI( &chain );
  rkChainDestroy( &chain );
  zVecFreeAtOnce( 6, dis, vel, acc, expected, actual, err );
  zAssert( rkChainFD_ABI + rkChainID, count_success == N );
}

int main(int argc, char *argv[])
{
  zRandInit();
  assert_chain_joint_index();
#ifndef __WINDOWS__
  assert_chain_clone();
#endif /* __WINDOWS__ */
  assert_chain_clone_irregular();
  assert_chain_copy_state();
  assert_chain_getsetconf();
  assert_chain_momentum();
  assert_chain_com_inertia();
  assert_chain_gravitydir();
  assert_chain_zmp_simple();
  assert_chain_crb();
  assert_chain_inertia_mat();
  assert_chain_fd_id();
  assert_chain_fd_id_abi();
  return EXIT_SUCCESS;
}
