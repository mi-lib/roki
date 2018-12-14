#include <roki/rk_chain.h>

#define DT 0.001
#define STEP 1000

#define N   8
#define TIP 7

void chain_init(rkChain *chain)
{
  register int i;
  char name[BUFSIZ];

  rkChainInit( chain );
  zArrayAlloc( &chain->link, rkLink, N );
  for( i=0; i<N; i++ ){
    sprintf( name, "link#%02d", i );
    rkLinkInit( rkChainLink(chain,i) );
    zVec3DCreate( rkChainLinkCOM(chain,i), 0.5, 0.3, 0.2 );
    zNameSet( rkChainLink(chain,i), name );
    rkLinkSetMass( rkChainLink(chain,i), 0.1 );
    rkChainMass(chain) += rkChainLinkMass(chain,i);
  }
  for( i=1; i<N; i++ )
    rkLinkAddChild( rkChainLink(chain,i-1), rkChainLink(chain,i) );
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_FLOAT );
  rkJointCreate( rkChainLinkJoint(chain,1), RK_JOINT_SPHER );
  rkJointCreate( rkChainLinkJoint(chain,2), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,3), RK_JOINT_CYLIN );
  rkJointCreate( rkChainLinkJoint(chain,4), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,5), RK_JOINT_PRISM );
  rkJointCreate( rkChainLinkJoint(chain,6), RK_JOINT_HOOKE );
  rkJointCreate( rkChainLinkJoint(chain,7), RK_JOINT_FIXED );
  zVec3DCreate( rkChainLinkOrgPos(chain,2), 1, 0, 0 );
  zVec3DCreate( rkChainLinkOrgPos(chain,3), 0, 0, 1 );
  zVec3DCreate( rkChainLinkOrgPos(chain,5), 0, 1, 0 );
  zVec3DCreate( rkChainLinkOrgPos(chain,6), 0, 0, 1 );
  zVec3DCreate( rkChainLinkOrgPos(chain,7), 1, 0, 0 );
  for( i=0; i<N; i++ )
    zFrame3DCopy( rkChainLinkOrgFrame(chain,i), rkChainLinkAdjFrame(chain,i) );
  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
}

void set_vel(zVec vel)
{
  register int i;

  for( i=0; i<_zVecSize(vel); i++ )
    zVecSetElem( vel, i, zRandF(-1.0,1.0) );
}

void err_assert(zVec6D *err)
{
  if( zVec3DNorm(zVec6DLin(err)) != 0 || zVec3DNorm(zVec6DAng(err)) != 0 ){
    ZRUNERROR( "arienai!" );
    exit( 1 );
  }
}

void err_test(rkChain *src, rkChain *dst, zVec6D *err)
{
  /* frame */
  err_assert( zFrame3DError( rkChainLinkWldFrame(src,TIP), rkChainLinkWldFrame(dst,TIP), err ) );
  /* velocity */
  err_assert( zVec6DSub( rkChainLinkVel(src,TIP), rkChainLinkVel(dst,TIP), err ) );
  /* acceleration */
  err_assert( zVec6DSub( rkChainLinkAcc(src,TIP), rkChainLinkAcc(dst,TIP), err ) );
  /* wrench */
  err_assert( zVec6DSub( rkChainLinkWrench(src,0), rkChainLinkWrench(dst,0), err ) );
  /* COM */
  zVec3DSub( rkChainLinkWldCOM(src,TIP), rkChainLinkWldCOM(dst,TIP), zVec6DLin(err) );
  err_assert( err );
  /* COM velocity */
  zVec3DSub( rkChainLinkCOMVel(src,TIP), rkChainLinkCOMVel(dst,TIP), zVec6DLin(err) );
  err_assert( err );
  /* COM acceleration */
  zVec3DSub( rkChainLinkCOMAcc(src,TIP), rkChainLinkCOMAcc(dst,TIP), zVec6DLin(err) );
  err_assert( err );
  /* chain COM */
  zVec3DSub( rkChainWldCOM(src), rkChainWldCOM(dst), zVec6DLin(err) );
  err_assert( err );
  /* chain COM velocity */
  zVec3DSub( rkChainCOMVel(src), rkChainCOMVel(dst), zVec6DLin(err) );
  err_assert( err );
  /* chain COM acceleration */
  zVec3DSub( rkChainCOMAcc(src), rkChainCOMAcc(dst), zVec6DLin(err) );
  err_assert( err );
}

int main(int argc, char *argv[])
{
  rkChain chain, chain_copy;
  zVec dis, vel;
  zVec6D err;
  register int i;

  zRandInit();
  chain_init( &chain );
  chain_init( &chain_copy );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  vel = zVecAlloc( rkChainJointSize(&chain) );

  for( i=0; i<=STEP; i++ ){
    set_vel( vel );
    zVecCatNCDRC( dis, DT, vel );
    rkChainFKCNT( &chain, dis, DT );
    rkChainCopyState( &chain, &chain_copy );
    err_test( &chain, &chain_copy, &err );
  }

  zVecFree( dis );
  zVecFree( vel );
  rkChainDestroy( &chain );
  rkChainDestroy( &chain_copy );
  return 0;
}
