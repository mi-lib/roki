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
  int i;
  char name[BUFSIZ];
  zVec3D aa;

  rkChainInit( chain );
  rkLinkArrayAlloc( rkChainLinkArray(chain), N );
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
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_float );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_spher );
  rkJointAssign( rkChainLinkJoint(chain,2), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,3), &rk_joint_cylin );
  rkJointAssign( rkChainLinkJoint(chain,4), &rk_joint_revol );
  rkJointAssign( rkChainLinkJoint(chain,5), &rk_joint_prism );
  rkJointAssign( rkChainLinkJoint(chain,6), &rk_joint_hooke );
  rkJointAssign( rkChainLinkJoint(chain,7), &rk_joint_fixed );

  rkChainSetJointIDOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
}

void world_ang_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  rkChainLinkWldAngJacobi( chain, TIP, jacobi );
  zMulMat3DVec3D( rkChainLinkWldAtt(chain,TIP), rkChainLinkAngVel(chain,TIP), v );
}

void world_lin_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  rkChainLinkWldLinJacobi( chain, TIP, ZVEC3DZERO, jacobi );
  zMulMat3DVec3D( rkChainLinkWldAtt(chain,TIP), rkChainLinkLinVel(chain,TIP), v );
}

void world_com_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  rkChainLinkWldLinJacobi( chain, TIP, rkChainLinkCOM(chain,TIP), jacobi );
  zMulMat3DVec3D( rkChainLinkWldAtt(chain,TIP), rkChainLinkCOMVel(chain,TIP), v );
}

void l2l_ang_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  zVec3D av;

  rkChainLinkToLinkAngJacobi( chain, ANO, TIP, jacobi );
  zMulMat3DVec3D( rkChainLinkWldAtt(chain,TIP), rkChainLinkAngVel(chain,TIP), &av );
  zMulMat3DTVec3D( rkChainLinkWldAtt(chain,ANO), &av, v );
  zVec3DSubDRC( v, rkChainLinkAngVel(chain,ANO) );
}

void l2l_lin_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  zVec3D tmp;

  rkChainLinkToLinkLinJacobi( chain, ANO, TIP, ZVEC3DZERO, jacobi );
  rkLinkPointVel( rkChainLink(chain,TIP), ZVEC3DZERO, v );
  zMulMat3DVec3DDRC( rkChainLinkWldAtt(chain,TIP), v );
  zMulMat3DTVec3DDRC( rkChainLinkWldAtt(chain,ANO), v );
  zVec3DSubDRC( v, rkChainLinkLinVel(chain,ANO) );

  zXform3D( rkChainLinkWldFrame(chain,TIP), ZVEC3DZERO, &tmp );
  zXform3DInvDRC( rkChainLinkWldFrame(chain,ANO), &tmp );
  zVec3DOuterProd( rkChainLinkAngVel(chain,ANO), &tmp, &tmp );
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

  rkChainLinkAMMat( chain, TIP, ZVEC3DZERO, jacobi );
  zXform3DInv( rkChainLinkWldFrame(chain,TIP), ZVEC3DZERO, &tp );
  rkLinkAngularMomentum( rkChainLink(chain,TIP), &tp, v );
  zMulMat3DVec3DDRC( rkChainLinkWldAtt(chain,TIP), v );
}

void am_test(rkChain *chain, zMat jacobi, zVec3D *v)
{
  rkChainAMMat( chain, ZVEC3DZERO, jacobi );
  rkChainAngularMomentum( chain, ZVEC3DZERO, v );
}

bool assert_jacobi(rkChain *chain, zMat jacobi, zVec dis, zVec vel, zVec acc, zVec ev, void (*test_f)(rkChain*,zMat,zVec3D*))
{
  zVec3D v, err;

  test_f( chain, jacobi, &v );
  zMulMatVec( jacobi, vel, ev );
  zVec3DSub( (zVec3D*)zVecBuf(ev), &v, &err );
  if( !zVec3DIsTiny( &err ) ){
    zVec3DFPrint( stderr, &err );
    return false;
  }
  return true;
}

bool assert_zeroacc(rkChain *chain, int id, zVec dis, zVec vel, zVec acc, zMat jacobi, zVec a)
{
  zVec6D a0, av;
  zVec3D p, tmp;

  zVecRandUniform( dis, -zPI, zPI );
  zVecRandUniform( vel, -1, 1 );
  zVecRandUniform( acc, -1, 1 );
  zVec3DCreate( &p, zRandF(-0.1,0.1), zRandF(-0.1,0.1), zRandF(-0.1,0.1) );
  /* forward kinematics */
  rkChainSetJointDisAll( chain, dis );
  rkChainUpdateFK( chain );
  /* link acceleration */
  rkChainSetJointVelAll( chain, vel );
  rkChainLinkZeroAcc0G( chain, id, &p, &a0 );
  rkChainSetJointAccAll( chain, acc );
  rkChainUpdateRate0G( chain );
  /* check */
  rkChainLinkPointAcc( chain, id, &p, &tmp );
  zMulMat3DVec3D( rkChainLinkWldAtt(chain,id), &tmp, zVec6DLin(&av) );
  zMulMat3DVec3D( rkChainLinkWldAtt(chain,id), rkChainLinkAngAcc(chain,id), zVec6DAng(&av) );
  zVec6DSubDRC( &av, &a0 );
  /* linear acceleration */
  rkChainLinkWldLinJacobi( chain, id, &p, jacobi );
  zMulMatVec( jacobi, acc, a );
  zVec3DSubDRC( zVec6DLin(&av), (zVec3D*)zVecBufNC(a) );
  /* angular acceleration */
  rkChainLinkWldAngJacobi( chain, id, jacobi );
  zMulMatVec( jacobi, acc, a );
  zVec3DSubDRC( zVec6DAng(&av), (zVec3D*)zVecBufNC(a) );

  return zVec6DIsTiny( &av );
}

int main(int argc, char *argv[])
{
  rkChain chain;
  zVec dis, vel, acc, trq, ev;
  zMat jacobi;

  /* initialization */
  zRandInit();
  chain_init( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  vel = zVecAlloc( rkChainJointSize(&chain) );
  acc = zVecAlloc( rkChainJointSize(&chain) ); /* dummy */
  trq = zVecAlloc( rkChainJointSize(&chain) ); /* dummy */
  ev = zVecAlloc( 3 );
  jacobi = zMatAlloc( 3, rkChainJointSize(&chain) );

  zVecRandUniform( dis, -10.0, 10.0 );
  zVecRandUniform( vel, -10.0, 10.0 );
  rkChainID( &chain, dis, vel, acc, trq );

  zAssert( rkChainLinkWldAngJacobi, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, world_ang_test ) );
  zAssert( rkChainLinkWldLinJacobi, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, world_lin_test ) );
  zAssert( rkChainLinkWldLinJacobi(COM), assert_jacobi( &chain, jacobi, dis, vel, acc, ev, world_com_test ) );
  zAssert( rkChainLinkToLinkAngJacobi, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, l2l_ang_test ) );
  zAssert( rkChainLinkToLinkLinJacobi, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, l2l_lin_test ) );
  zAssert( rkChainCOMJacobi, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, com_test ) );
  zAssert( rkChainLinkAMMat, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, link_am_test ) );
  zAssert( rkChainAMMat, assert_jacobi( &chain, jacobi, dis, vel, acc, ev, am_test ) );
  zAssert( rkChainLinkZeroAcc, assert_zeroacc( &chain, TIP, dis, vel, acc, jacobi, ev ) );

  /* termination */
  zVecFreeAtOnce( 5, dis, vel, acc, trq, ev );
  zMatFree( jacobi );
  rkChainDestroy( &chain );
  return EXIT_SUCCESS;
}
