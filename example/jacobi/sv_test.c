#include <roki/rk_jacobi.h>

#define N 1

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec dis;
  zMat jl, ja, jacobi;
  int i;

  /* initialization */
  zRandInit();
  rkChainReadZTK( &chain, argc > 1 ? argv[1] : "../model/arm.ztk" );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  jl = zMatAlloc( 3, rkChainJointSize(&chain) );
  ja = zMatAlloc( 3, rkChainJointSize(&chain) );
  jacobi = zMatAlloc( 6, rkChainJointSize(&chain) );

  for( i=0; i<N; i++ ){
    zVecRandUniform( dis, -10.0, 10.0 );
    rkChainFK( &chain, dis );
    rkChainLinkWldAngJacobi( &chain, rkChainLinkNum(&chain)-1, ja );
    rkChainLinkWldLinJacobi( &chain, rkChainLinkNum(&chain)-1, ZVEC3DZERO, jl );
    zMatPut( jacobi, 0, 0, ja );
    zMatPut( jacobi, 3, 0, jl );
    printf( "%.10f %.10f %.10f\n", rkJacobiManip(jacobi), zMatSingularValueMax(jacobi), zMatSingularValueMin(jacobi) );
  }

  /* termination */
  zVecFree( dis );
  zMatFree( ja );
  zMatFree( jl );
  zMatFree( jacobi );
  rkChainDestroy( &chain );
  return 0;
}
