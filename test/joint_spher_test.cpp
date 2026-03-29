#include <roki/rk_chain.h>

zVec3D p( 1.0, 2.0, 3.0 );
rkLink *le;

void truth(const zVec3D &aa1, const zVec3D &w1, const zVec3D &a1, const zVec3D &aa2, const zVec3D &w2, const zVec3D &a2, zFrame3D &f, zVec6D &v, zVec6D &a)
{
  zMat3D m1, m2;
  zVec3D wp, ww, tmp1, tmp2;

  // frame
  m1.createFromAA( aa1 );
  m2.createFromAA( aa2 );
  *zFrame3DAtt(&f) = m1 * m2;
  *zFrame3DPos(&f) = m1 * p;
  // lin. veloc.
  zVec3DOuterProd( &w1, zFrame3DPos(&f), &wp );
  *zVec6DLin(&v) = zFrame3DAtt(&f)->mulT( wp );
  // ang. veloc.
  ww = m1 * w2;
  tmp1 = w1 + ww;
  *zVec6DAng(&v) = zFrame3DAtt(&f)->mulT( tmp1 );
  // lin. accel.
  zVec3DOuterProd( &a1, zFrame3DPos(&f), &tmp1 );
  zVec3DOuterProd( &w1, &wp, &tmp2 );
  tmp1 += tmp2;
  *zVec6DLin(&a) = zFrame3DAtt(&f)->mulT( tmp1 );
  tmp1 = zFrame3DAtt(&f)->mulT( *RK_GRAVITY3D );
  *zVec6DLin(&a) = *zVec6DLin(&a) + tmp1;
  // ang. accel.
  zVec3DOuterProd( &w1, &ww, &tmp1 );
  tmp1 += a1;
  tmp2 = m1 * a2;
  tmp1 += tmp2;
  *zVec6DAng(&a) = zFrame3DAtt(&f)->mulT( tmp1 );
}

void create_sphere(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "sphere1" );
  rkLinkArrayAlloc( rkChainLinkArray(chain), 2 );
  // link 1
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "link1" );
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_spher );
  // link 2
  rkLinkInit( rkChainLink(chain,1) );
  zNameSet( rkChainLink(chain,1), "link2" );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_spher );
  rkChainLinkOrgPos(chain,1)->copy( p );
  // connect
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );

  rkChainSetMass( chain, 1.0 );
  rkChainSetJointIDOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  le = rkChainLink(chain,1);
}

int main(void)
{
  rkChain chain;
  zVec dis, vel, acc, trq;
  zFrame3D f;
  zVec3D aa1, aa2, w1, w2, a1, a2;
  zVec6D v, a, err;

  // create chain
  create_sphere( &chain );
  // create joint configuration
  zRandInit();
  dis = zVecAlloc( rkChainJointSize(&chain) );
  vel = zVecAlloc( rkChainJointSize(&chain) );
  acc = zVecAlloc( rkChainJointSize(&chain) );
  trq = zVecAlloc( rkChainJointSize(&chain) );
  zVecRandUniform( dis, -0.5*zPI, 0.5*zPI );
  zVecRandUniform( vel, -10, 10 );
  zVecRandUniform( acc, -100, 100 );

  aa1.create( zVecElem(dis,0), zVecElem(dis,1), zVecElem(dis,2) );
  aa2.create( zVecElem(dis,3), zVecElem(dis,4), zVecElem(dis,5) );
  w1.create( zVecElem(vel,0), zVecElem(vel,1), zVecElem(vel,2) );
  w2.create( zVecElem(vel,3), zVecElem(vel,4), zVecElem(vel,5) );
  a1.create( zVecElem(acc,0), zVecElem(acc,1), zVecElem(acc,2) );
  a2.create( zVecElem(acc,3), zVecElem(acc,4), zVecElem(acc,5) );

  rkChainID( &chain, dis, vel, acc, trq );
  truth( aa1, w1, a1, aa2, w2, a2, f, v, a );
  err = f - *rkLinkWldFrame(le);
  zAssert( rkChainFK (spherical joint), zVec6DIsTiny(&err) );
  err = v - *rkLinkVel(le);
  zAssert( rkChainID (spherical joint velocity), zVec6DIsTiny(&err) );
  err = a - *rkLinkAcc(le);
  zAssert( rkChainID (spherical joint acceleration), zVec6DIsTiny(&err) );

  // terminate
  zVecFreeAtOnce( 4, dis, vel, acc, trq );
  rkChainDestroy( &chain );
  return 0;
}
