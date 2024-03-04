#include <roki/rk_chain.h>

#define DT 0.001
#define STEP 1000

#define N   8
#define TIP 7

void chain_init(rkChain *chain)
{
  int i;
  char name[BUFSIZ];

  rkChainInit( chain );
  rkLinkArrayAlloc( rkChainLinkArray(chain), N );
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
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_float );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_spher );
  rkJointAssign( rkChainLinkJoint(chain,2), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,3), &rk_joint_cylin );
  rkJointAssign( rkChainLinkJoint(chain,4), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,5), &rk_joint_prism );
  rkJointAssign( rkChainLinkJoint(chain,6), &rk_joint_hooke );
  rkJointAssign( rkChainLinkJoint(chain,7), &rk_joint_fixed );
  zVec3DCreate( rkChainLinkOrgPos(chain,2), 1, 0, 0 );
  zVec3DCreate( rkChainLinkOrgPos(chain,3), 0, 0, 1 );
  zVec3DCreate( rkChainLinkOrgPos(chain,5), 0, 1, 0 );
  zVec3DCreate( rkChainLinkOrgPos(chain,6), 0, 0, 1 );
  zVec3DCreate( rkChainLinkOrgPos(chain,7), 1, 0, 0 );
  for( i=0; i<N; i++ )
    zFrame3DCopy( rkChainLinkOrgFrame(chain,i), rkChainLinkAdjFrame(chain,i) );
  rkChainSetJointIDOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
}

void set_vel(zVec vel)
{
  int i;

  for( i=0; i<zVecSizeNC(vel); i++ )
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
  int i;

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
