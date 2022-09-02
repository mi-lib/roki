#include <roki/rk_jacobi.h>

/* inertia matrix */
zMat chain_inertia_matrix(rkChain *chain, zMat inertia)
{
  zMat jacobi;
  zMat inertia_tmp;
  zMat tmp;
  zMat mp;
  register int i;
  int n;

  n = rkChainJointSize( chain );
  inertia_tmp = zMatAllocSqr( n );
  jacobi = zMatAlloc( 3, n );
  tmp = zMatAlloc( 3, n );
  mp = zMatAllocSqr( 3 );
  for( i=rkChainLinkNum(chain)-1; i>=0; i-- ){
    /* linear component */
    rkChainLinkWldLinJacobi( chain, i, rkChainLinkCOM(chain,i), jacobi );
    zMulMatTMat( jacobi, jacobi, inertia_tmp );
    zMatCatDRC( inertia, rkChainLinkMass(chain,i), inertia_tmp );
    /* angular component */
    rkChainLinkWldAngJacobi(chain, i, jacobi);
    rkLinkWldInertia(rkChainLink(chain, i), (zMat3D*)zMatBufNC(mp) );
    zMulMatMat(mp, jacobi, tmp );
    zMulMatTMat( jacobi, tmp, inertia_tmp );
    zMatAddDRC( inertia, inertia_tmp );
  }
  zMatFreeAO( 4, inertia_tmp, jacobi, tmp, mp );
  return inertia;
}

bool check_inertia_matrix(rkChain *chain, zMat inertia, double tol)
{
  zMat h;
  bool ret;

  h = zMatAllocSqr( rkChainJointSize(chain) );
  chain_inertia_matrix( chain, h );
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

  rkLinkSetMass( l, zRandF(0.1,1.0) );
  zVec3DCreate( rkLinkCOM(l), zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
  i11 = zRandF(0.01,0.1);
  i12 =-zRandF(0.0001,0.001);
  i13 =-zRandF(0.0001,0.001);
  i22 = zRandF(0.01,0.1);
  i23 =-zRandF(0.00001,0.0001);
  i33 = zRandF(0.01,0.1);
  zMat3DCreate( rkLinkInertia(l), i11, i12, i13, i12, i22, i23, i13, i23, i33 );
}

int chain_init(rkChain *chain)
{
  register int i;
  char name[BUFSIZ];
  zVec3D aa;

  rkChainInit( chain );
  zArrayAlloc( &chain->link, rkLink, LINK_NUM );
  for( i=0; i<LINK_NUM; i++ ){
    sprintf( name, "link#%02d", i );
    rkLinkInit( rkChainLink(chain,i) );
    link_mp_rand( rkChainLink(chain,i) );
    rkChainMass( chain ) += rkChainLinkMass( chain, i );
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

  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  return rkChainJointSize( chain );
}

#define N 1000
#define TOL (1.0e-10)

int main(int argc, char *argv[])
{
  rkChain chain;
  zMat h, hs;
  zVec b, bs, dis, vel;
  int i, count_im, count_ke, count_fd, count_sep;
  int n;

  /* initialization */
  zRandInit();
  n = chain_init( &chain );
  h = zMatAllocSqr( n );
  hs= zMatAllocSqr( n );
  dis = zVecAlloc( n );
  vel = zVecAlloc( n );
  b = zVecAlloc( n );
  bs = zVecAlloc( n );

  count_im = count_ke = count_fd = count_sep = 0;
  for( i=0; i<N; i++ ){
    /* generate posture and velocity randomly */
    zVecRandUniform( dis, -10, 10 );
    zVecRandUniform( vel, -10, 10 );
    rkChainFK( &chain, dis );
    rkChainSetJointVelAll( &chain, vel );
    /* verifications */
    rkChainInertiaMatBiasVec( &chain, h, b );
    rkChainBiasVec( &chain, bs );
    rkChainInertiaMat( &chain, hs );/* joint vel to be zero */
    /* restore joint velocity */
    rkChainSetJointVelAll( &chain, vel );
    rkChainUpdateRate(&chain);
    /* count success */
    if( check_inertia_matrix( &chain, h, TOL ) ) count_im++;
    if( check_kinetic_energy( &chain, h, vel, TOL ) ) count_ke++;
    if( check_fd( &chain, h, b, vel, TOL ) ) count_fd++;
    if( zMatIsEqual(h, hs, TOL) && zVecIsEqual(b, bs, TOL)) count_sep++;
  }
  zAssert( rkChainInertiaMatBiasVec, count_im == N );
  zAssert( rkChainInertiaMatBiasVec + rkChainKE, count_ke == N );
  zAssert( rkChainInertiaMatBiasVec (FD-ID), count_fd == N );
  zAssert( rkChainBiasVec/rkChainInertiaMat, count_sep == N );

  /* termination */
  zMatFreeAO( 2, h, hs);
  zVecFreeAO( 4, b, bs, dis, vel );
  rkChainDestroy( &chain );
  return EXIT_SUCCESS;
}
