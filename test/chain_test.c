#include <roki/rk_jacobi.h>

bool check_inertia_matrix(rkChain *chain, zMat inertia, double tol)
{
  zMat h;
  bool ret;

  h = zMatAllocSqr( rkChainJointSize(chain) );
  rkChainInertiaMatMJ( chain, h );
  ret = zMatIsEqual( h, inertia, tol );
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
  return zIsEqual( rkChainKE( chain ), ke, tol );
}

bool check_fd(rkChain *chain, zMat inertia, zVec bias, zVec vel, double tol)
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
  rkChainID( chain, vel, acc );
  rkChainGetJointTrqAll( chain, trq_id );
  /* forward dynamics */
  zMulMatVec( inertia, acc, trq_fd );
  zVecAddDRC( trq_fd, bias );
  /* check */
  ret = zVecIsEqual( trq_fd, trq_id, tol );
  zVecFreeAO( 3, acc, trq_fd, trq_id );
  return ret;
}

#define LINK_NUM 8

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

int chain_init(rkChain *chain)
{
  int i;
  char name[BUFSIZ];
  zVec3D aa;

  rkChainInit( chain );
  zArrayAlloc( &chain->link, rkLink, LINK_NUM );
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
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_float );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_spher );
  rkJointAssign( rkChainLinkJoint(chain,2), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,3), &rk_joint_cylin );
  rkJointAssign( rkChainLinkJoint(chain,4), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,5), &rk_joint_prism );
  rkJointAssign( rkChainLinkJoint(chain,6), &rk_joint_hooke );
  rkJointAssign( rkChainLinkJoint(chain,7), &rk_joint_fixed );

  rkChainSetJointIDOffset( chain );
  rkChainUpdateCRBMass( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  return rkChainJointSize( chain );
}

#define N 1000
#define TOL (1.0e-10)

void assert_getsetconf(rkChain *chain)
{
  zVec orgdis, orgconf, dis, conf;
  int i;
  bool result1 = true, result2 = true;

  orgdis = zVecAlloc( rkChainJointSize( chain ) );
  dis = zVecAlloc( rkChainJointSize( chain ) );
  orgconf = zVecAlloc( rkChainLinkNum( chain ) * 6 );
  conf = zVecAlloc( rkChainLinkNum( chain ) * 6 );
  for( i=0; i<N; i++ ){
    /* displacement -> configuration -> displacement */
    zVecRandUniform( orgdis, -1.0, 1.0 );
    rkChainFK( chain, orgdis );
    rkChainGetConf( chain, conf );
    rkChainSetConf( chain, conf );
    rkChainGetJointDisAll( chain, dis );
    zVecSubDRC( dis, orgdis );
    if( !zVecIsTol( dis, TOL ) ) result1 = false;
    /* configuration -> displacement -> configuration */
    zVecRandUniform( orgconf, -1.0, 1.0 );
    rkChainSetConf( chain, orgconf );
    rkChainGetJointDisAll( chain, orgdis );
    rkChainFK( chain, orgdis );
    rkChainGetConf( chain, orgconf );
    rkChainSetConf( chain, orgconf );
    rkChainGetJointDisAll( chain, dis );
    rkChainGetConf( chain, conf );
    zVecSubDRC( conf, orgconf );
    if( !zVecIsTol( conf, TOL ) ) result2 = false;
  }
  zAssert( rkChainFK + rkChainGetConf + rkChainSetConf + rkChainGetJointDisAll, result1 );
  zAssert( rkChainSetConf + rkChainGetJointDisAll + rkChainFK + rkChainGetConf, result2 );

  zVecFree( orgconf );
  zVecFree( conf );
  zVecFree( orgdis );
  zVecFree( dis );
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

void assert_crb(rkChain *chain, int n)
{
  zVec dis;
  zVec3D com;
  zMat3D inertia;

  dis = zVecAlloc( n );
  zVecRandUniform( dis, -10, 10 );
  rkChainFK( chain, dis );
  rkChainUpdateCRB( chain );
  zXform3D( rkLinkAdjFrame(rkChainRoot(chain)), rkMPCOM(rkLinkCRB(rkChainRoot(chain))), &com );
  zRotMat3D( rkLinkAdjAtt(rkChainRoot(chain)), rkMPInertia(rkLinkCRB(rkChainRoot(chain))), &inertia );
  zAssert( rkChainUpdateCRB,
    zIsTiny( rkChainMass(chain) - rkMPMass(rkLinkCRB(rkChainRoot(chain))) ) &&
    zVec3DEqual( rkChainWldCOM(chain), &com ) &&
    check_chain_net_inertia( chain, &inertia ) );
  zVecFree( dis );
}

void assert_inertia_mat(rkChain *chain, int n)
{
  zMat h;
  zVec b, dis, vel;
  int i, count_iuv, count_icrb, count_ke, count_fd;

  h = zMatAllocSqr( n );
  dis = zVecAlloc( n );
  vel = zVecAlloc( n );
  b = zVecAlloc( n );
  count_iuv = count_icrb = count_ke = count_fd = 0;
  for( i=0; i<N; i++ ){
    /* generate posture and velocity randomly */
    zVecRandUniform( dis, -10, 10 );
    zVecRandUniform( vel, -10, 10 );
    rkChainFK( chain, dis );
    rkChainSetJointVelAll( chain, vel );
    /* verifications */
    rkChainInertiaMatBiasVec( chain, h, b );
    /* count success */
    if( check_inertia_matrix( chain, h, TOL ) ) count_icrb++;
    if( check_kinetic_energy( chain, h, vel, TOL ) ) count_ke++;
    if( check_fd( chain, h, b, vel, TOL ) ) count_fd++;
  }
  zAssert( rkChainInertiaMatBiasVec, count_icrb == N );
  zAssert( rkChainInertiaMatBiasVec + rkChainKE, count_ke == N );
  zAssert( rkChainInertiaMatBiasVec (FD-ID), count_fd == N );
  count_iuv = count_icrb = 0;
  for( i=0; i<N; i++ ){
    /* generate posture and velocity randomly */
    zVecRandUniform( dis, -10, 10 );
    rkChainFK( chain, dis );
    rkChainInertiaMatUV( chain, h );
    if( check_inertia_matrix( chain, h, TOL ) ) count_iuv++;
    rkChainInertiaMatCRB( chain, h );
    if( check_inertia_matrix( chain, h, TOL ) ) count_icrb++;
  }
  zAssert( rkChainInertiaMatUV, count_iuv == N );
  zAssert( rkChainInertiaMatCRB, count_icrb == N );

  zMatFree( h );
  zVecFreeAO( 3, b, dis, vel );
}

int main(int argc, char *argv[])
{
  rkChain chain;
  int n;

  /* initialization */
  zRandInit();
  n = chain_init( &chain );
  assert_getsetconf( &chain );
  assert_crb( &chain, n );
  assert_inertia_mat( &chain, n );
  /* termination */
  rkChainDestroy( &chain );
  return EXIT_SUCCESS;
}
