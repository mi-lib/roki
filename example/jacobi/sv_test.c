#include <roki/rk_jacobi.h>

#define N 1000

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec dis;
  zMat jl, ja, jacobi;
  int i;
  char *name;

  name = argc > 1 ? argv[1] : "../model/arm.zkc";
  /* initialization */
  zRandInit();
  rkChainReadFile( &chain, name );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  jl = zMatAlloc( 3, rkChainJointSize(&chain) );
  ja = zMatAlloc( 3, rkChainJointSize(&chain) );
  jacobi = zMatAlloc( 12, rkChainJointSize(&chain) );

  for( i=0; i<N; i++ ){
    zVecRandUniform( dis, -10.0, 10.0 );
    rkChainFK( &chain, dis );
    rkChainLinkWldAngJacobi( &chain, rkChainNum(&chain)-1, ja );
    rkChainLinkWldLinJacobi( &chain, rkChainNum(&chain)-1, Z_ZEROVEC3D, jl );
    zMatPut( jacobi, 0, 0, ja );
    zMatPut( jacobi, 3, 0, jl );
    zMatPut( jacobi, 6, 0, ja );
    zMatPut( jacobi, 9, 0, jl );
    printf( "%.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n",
      rkJacobiManip(jacobi), zSVMax(jacobi), zSVMin(jacobi),
      rkJacobiManip(ja), zSVMax(ja), zSVMin(ja),
      rkJacobiManip(jl), zSVMax(jl), zSVMin(jl) );
  }

  /* termination */
  zVecFree( dis );
  zMatFree( ja );
  zMatFree( jl );
  zMatFree( jacobi );
  rkChainDestroy( &chain );
  return 0;
}
