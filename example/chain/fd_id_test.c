#include <roki/roki.h>

#define N 10000

/* only works with torque-controlled robot models. */
int main(int argc, char *argv[])
{
  rkChain chain;
  zVec dis, vel, acc, expected, actual, err;
  int n, i, count_success = 0;

  rkChainReadZTK( &chain, "../model/arm_2DoF_trq.ztk" ); /* torque-controlled robot */
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
    rkChainID( &chain, vel, acc ); /* inverse dynamics (Newton-Euler method) */
    rkChainGetJointTrqAll( &chain, actual );
    if( zVecIsEqual( actual, expected, zTOL ) ){
      count_success++;
    } else{
      printf( "Failure case : RMSE = %.10g\n", zVecDist( expected, actual ) );
      printf( " (error) = " ); zVecPrint( zVecSub( expected, actual, err ) );
    }
  }
  printf( "Success rate = %d / %d\n", count_success, N );
  rkChainDestroyABI( &chain );
  rkChainDestroy( &chain );
  zAssert( rkChainFD_ABI + rkChainID, count_success == N );
  return 0;
}
