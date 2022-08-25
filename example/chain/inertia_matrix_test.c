#include <roki/roki.h>

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
  zMatSubDRC( h, inertia );
  ret = zMatIsTol( h, tol );
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
  return zIsTol( rkChainKE( chain ) - ke, tol );
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

#define N 10000
#define TOL (1.0e-10)

int main(int argc, char *argv[])
{
  rkChain chain;
  int n;
  zMat h;
  zVec b, dis, vel;
  int i, count_im, count_ke, count_fd;
  double tol_im, tol_ke, tol_fd;

  tol_im = argc > 1 ? atof( argv[1] ) : TOL;
  tol_ke = argc > 2 ? atof( argv[2] ) : TOL;
  tol_fd = argc > 3 ? atof( argv[3] ) : TOL;
  zRandInit();
  rkChainReadZTK(&chain, "../model/mighty.ztk");
  n = rkChainJointSize( &chain );
  h = zMatAllocSqr( n );
  dis = zVecAlloc( n );
  vel = zVecAlloc( n );
  b = zVecAlloc( n );

  count_im = count_ke = count_fd = 0;
  for( i=0; i<N; i++ ){
    printf( "test %d / %d\r", i, N );
    /* generate posture and velocity randomly */
    zVecRandUniform( dis, -10, 10 );
    zVecRandUniform( vel, -10, 10 );
    rkChainFK( &chain, dis );
    rkChainSetJointVelAll( &chain, vel );
    /* verifications */
    rkChainInertiaMatBiasVec( &chain, h, b );
    if( check_inertia_matrix( &chain, h, tol_im ) ) count_im++;
    if( check_kinetic_energy( &chain, h, vel, tol_ke ) ) count_ke++;
    if( check_fd( &chain, h, b, vel, tol_fd ) ) count_fd++;
  }
  printf( "rkChainInertiaMatBiasVec             success rate = %d / %d\n", count_im, N );
  printf( "rkChainInertiaMatBiasVec + rkChainKE success rate = %d / %d\n", count_ke, N );
  printf( "rkChainInertiaMatBiasVec (FD-ID)     success rate = %d / %d\n", count_fd, N );

  zMatFree( h );
  zVecFreeAO( 3, b, dis, vel );
  rkChainDestroy( &chain );
  return 0;
}
