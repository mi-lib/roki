#include <roki/rk_chain.h>

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

#define LINK_NUM 8

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

  rkChainSetJointIDOffset( chain );
  rkChainUpdateCRBMass( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  return rkChainJointSize( chain );
}

#define N 1000
#define TOL (1.0e-10)

void test_inertia_mat(rkChain *chain, int n)
{
  zVec dis;
  zMat h, hc;
  int i, count_im, count_ij;
  clock_t c1, c2;
  long l1, l2, l3;

  h = zMatAllocSqr( n );
  hc = zMatAllocSqr( n );
  dis = zVecAlloc( n );
  count_im = count_ij = 0;
  l1 = l2 = l3 = 0;
  for( i=0; i<N; i++ ){
    zVecRandUniform( dis, -10, 10 );
    rkChainFK( chain, dis );
    c1 = clock();
    rkChainInertiaMatCRB( chain, h );
    c2 = clock();
    l1 += c2 - c1;
    c1 = clock();
    rkChainInertiaMatUV( chain, hc );
    c2 = clock();
    l2 += c2 - c1;
    if( zMatIsEqual( h, hc, TOL ) ) count_im++;
    c1 = clock();
    rkChainInertiaMatMJ( chain, hc );
    c2 = clock();
    l3 += c2 - c1;
    if( zMatIsEqual( h, hc, TOL ) ) count_ij++;
  }
  eprintf( "clock (CRB/UV/MJ): %ld %ld %ld\n", l1, l2, l3 );
  eprintf( "success (CRB/UV/MJ) %d/%d/%d\n", count_im, count_ij, N );
  zMatFreeAO( 2, h, hc );
  zVecFree( dis );
  zAssert( rkChainInertiaMatCRB, count_im == N );
  zAssert( rkChainInertiaMatMJ, count_ij == N );
}

int main(int argc, char *argv[])
{
  rkChain chain;
  int n;

  zRandInit();
  n = chain_init( &chain );
  test_inertia_mat( &chain, n );
  rkChainDestroy( &chain );
  return 0;
}
