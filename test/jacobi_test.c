#include <roki/rk_jacobi.h>

#define N   8
#define TIP 4
#define ANO 7

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

void chain_init(rkChain *chain)
{
  register int i;
  char name[BUFSIZ];
  zVec3D aa;

  rkChainInit( chain );
  zArrayAlloc( &chain->link, rkLink, N );
  for( i=0; i<N; i++ ){
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
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_FLOAT );
  rkJointCreate( rkChainLinkJoint(chain,1), RK_JOINT_SPHER );
  rkJointCreate( rkChainLinkJoint(chain,2), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,3), RK_JOINT_CYLIN );
  rkJointCreate( rkChainLinkJoint(chain,4), RK_JOINT_REVOL );
  rkJointCreate( rkChainLinkJoint(chain,5), RK_JOINT_PRISM );
  rkJointCreate( rkChainLinkJoint(chain,6), RK_JOINT_HOOKE );
  rkJointCreate( rkChainLinkJoint(chain,7), RK_JOINT_FIXED );

  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
}

void world_ang_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  rkChainLinkWldAngJacobi( chain, TIP, jacobi );
  zMulMatVec3D( rkChainLinkWldAtt(chain,TIP), rkChainLinkAngVel(chain,TIP), v );
}

void world_lin_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  rkChainLinkWldLinJacobi( chain, TIP, ZVEC3DZERO, jacobi );
  zMulMatVec3D( rkChainLinkWldAtt(chain,TIP), rkChainLinkLinVel(chain,TIP), v );
}

void world_com_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  rkChainLinkWldLinJacobi( chain, TIP, rkChainLinkCOM(chain,TIP), jacobi );
  zMulMatVec3D( rkChainLinkWldAtt(chain,TIP), rkChainLinkCOMVel(chain,TIP), v );
}

void l2l_ang_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  zVec3D av;

  rkChainLinkToLinkAngJacobi( chain, ANO, TIP, jacobi );
  zMulMatVec3D( rkChainLinkWldAtt(chain,TIP), rkChainLinkAngVel(chain,TIP), v );
  zMulMatVec3D( rkChainLinkWldAtt(chain,ANO), rkChainLinkAngVel(chain,ANO), &av );
  zVec3DSubDRC( v, &av );
}

void l2l_lin_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  zVec3D av, vr, tmp;

  rkChainLinkToLinkLinJacobi( chain, ANO, TIP, ZVEC3DZERO, jacobi );
  zMulMatVec3D( rkChainLinkWldAtt(chain,TIP), rkChainLinkLinVel(chain,TIP), v );
  zMulMatVec3D( rkChainLinkWldAtt(chain,ANO), rkChainLinkLinVel(chain,ANO), &av );
  zVec3DSubDRC( v, &av );

  zMulMatVec3D( rkChainLinkWldAtt(chain,ANO), rkChainLinkAngVel(chain,ANO), &vr );
  zVec3DSub( rkChainLinkWldPos(chain,TIP), rkChainLinkWldPos(chain,ANO), &av );
  zVec3DOuterProd( &vr, &av, &tmp );
  zVec3DSubDRC( v, &tmp );
}

void com_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  rkChainCOMJacobi( chain, jacobi );
  zVec3DCopy( rkChainCOMVel(chain), v );
}

void link_am_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  zVec3D tp;

  rkChainLinkAMJacobi( chain, TIP, ZVEC3DZERO, jacobi );
  zXfer3DInv( rkChainLinkWldFrame(chain,TIP), ZVEC3DZERO, &tp );
  rkLinkAM( rkChainLink(chain,TIP), &tp, v );
  zMulMatVec3DDRC( rkChainLinkWldAtt(chain,TIP), v );
}

void am_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  rkChainAMJacobi( chain, ZVEC3DZERO, jacobi );
  rkChainAM( chain, ZVEC3DZERO, v );
}

bool assert_jacobi(rkChain *chain, zMat jacobi, zVec dis, zVec vel, zVec acc, zVec ev, void (*test_f)(rkChain*,zMat,zVec3D*))
{
  zVec3D v, err;

  test_f( chain, jacobi, &v );
  zMulMatVec( jacobi, vel, ev );
  zVec3DSub( (zVec3D*)zVecBuf(ev), &v, &err );
  return zVec3DIsTiny( &err );
}

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec dis, vel, acc, ev;
  zMat jacobi;

  /* initialization */
  zRandInit();
  chain_init( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  vel = zVecAlloc( rkChainJointSize(&chain) );
  acc = zVecAlloc( rkChainJointSize(&chain) ); /* dummy */
  ev = zVecAlloc( 3 );
  jacobi = zMatAlloc( 3, rkChainJointSize(&chain) );

  zVecRandUniform( dis, -10.0, 10.0 );
  zVecRandUniform( vel, -10.0, 10.0 );
  rkChainFK( &chain, dis );
  rkChainID( &chain, vel, acc );

  zAssert( rkChainLinkWldAngJacobi, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, world_ang_test ) );
  zAssert( rkChainLinkWldLinJacobi, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, world_lin_test ) );
  zAssert( rkChainLinkWldLinJacobi(COM), assert_jacobi( &chain, jacobi, dis, vel, acc, ev, world_com_test ) );
  zAssert( rkChainLinkToLinkAngJacobi, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, l2l_ang_test ) );
  zAssert( rkChainLinkToLinkLinJacobi, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, l2l_lin_test ) );
  zAssert( rkChainCOMJacobi, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, com_test ) );
  zAssert( rkChainLinkAMJacobi, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, link_am_test ) );
  zAssert( rkChainAMJacobi, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, am_test ) );

  /* termination */
  zVecFree( dis );
  zVecFree( vel );
  zVecFree( acc );
  zVecFree( ev );
  zMatFree( jacobi );
  rkChainDestroy( &chain );
  return EXIT_SUCCESS;
}
