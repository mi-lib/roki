#include <roki/rk_chain.h>

zVec3D p( 1.0, 2.0, 3.0 );
rkLink *le;

void truth(const zMat3D &morg, const zVec3D &pos, const zVec3D &aa, const zVec6D &vin, const zVec6D &ain, zFrame3D &fout, zVec6D &vout, zVec6D &aout)
{
  zMat3D m, mtmp;
  zVec3D wp, tmp;

  // frame
  m.createFromAA( aa );
  mtmp = morg * m;
  fout.setAtt( mtmp );
  tmp = *zFrame3DAtt(&fout) * p;
  zVec3D dp( tmp );
  tmp = morg * pos;
  fout.setPos( tmp );
  *zFrame3DPos(&fout) += dp;
  // velocity
  tmp = m * p;
  dp.copy( tmp );
  zVec3DOuterProd( zVec6DAng(&vin), &dp, &wp );
  zVec6DLin(&vout)->copy( *zVec6DLin(&vin) + wp );
  zVec6DAng(&vout)->copy( *zVec6DAng(&vin) );
  zMulMat3DTVec6DDRC( &m, &vout );
  // acceleration
  zVec3DOuterProd( zVec6DAng(&vin), &wp, zVec6DLin(&aout) );
  zVec3DOuterProd( zVec6DAng(&vin), zVec6DLin(&vin), &tmp );
  *zVec6DLin(&aout) = *zVec6DLin(&aout) + 2 * tmp;
  zVec3DOuterProd( zVec6DAng(&ain), &dp, &tmp );
  *zVec6DLin(&aout) += tmp;
  *zVec6DLin(&aout) += *zVec6DLin(&ain);
  zVec6DAng(&aout)->copy( *zVec6DAng(&ain) );
  zMulMat3DTVec6DDRC( &m, &aout );
}

void create_float(rkChain *chain)
{
  rkChainInit( chain );
  zNameSet( chain, "float1" );
  rkLinkArrayAlloc( rkChainLinkArray(chain), 2 );
  // link 1
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), "link1" );
  rkJointAssign( rkChainLinkJoint(chain,0), &rk_joint_float );
  rkChainLinkOrgAtt(chain,0)->create(
    0, 0, 1,
    0, 1, 0,
   -1, 0, 0 );
  // link 2
  rkLinkInit( rkChainLink(chain,1) );
  zNameSet( rkChainLink(chain,1), "link2" );
  rkJointAssign( rkChainLinkJoint(chain,1), &rk_joint_fixed );
  rkChainLinkOrgPos(chain,1)->copy( p );
  // connect
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );

  rkChainSetMass( chain, 1.0 );
  rkChainSetJointIDOffset( chain );
  le = rkChainLink(chain,1);
}

int main(void)
{
  rkChain chain;
  zVec dis, vel, acc, trq;
  zVec3D pos, aa;
  zVec6D vin, ain, vout, aout, err;
  zFrame3D fout;
  int n;

  // create chain
  create_float( &chain );
  n = rkChainJointSize( &chain );
  // create joint configuration
  zRandInit();
  dis = zVecAlloc( n );
  vel = zVecAlloc( n );
  acc = zVecAlloc( n );
  trq = zVecAlloc( n );
  zVecRandUniform( dis, -0.5*zPI, 0.5*zPI );
  zVecRandUniform( vel, -10, 10 );
  zVecRandUniform( acc, -100, 100 );

  pos.create( zVecElem(dis,0), zVecElem(dis,1), zVecElem(dis,2) );
  aa.create(  zVecElem(dis,3), zVecElem(dis,4), zVecElem(dis,5) );
  vin.create(
    zVecElem(vel,0), zVecElem(vel,1), zVecElem(vel,2),
    zVecElem(vel,3), zVecElem(vel,4), zVecElem(vel,5) );
  ain.create(
    zVecElem(acc,0), zVecElem(acc,1), zVecElem(acc,2),
    zVecElem(acc,3), zVecElem(acc,4), zVecElem(acc,5) );

  rkChainID0G( &chain, dis, vel, acc, trq );
  truth( *rkChainLinkOrgAtt(&chain,0), pos, aa, vin, ain, fout, vout, aout );

  // output
  err = fout - *rkChainLinkWldFrame(&chain,1);
  zAssert( rkChainFK (float joint), zVec6DIsTiny(&err) );

  err = vout - *rkChainLinkVel(&chain,1);
  zAssert( rkChainID0G (float joint velocity), zVec6DIsTiny(&err) );

  err = aout - *rkLinkAcc(le);
  zAssert( rkChainID0G (float joint acceleration), zVec6DIsTiny(&err) );

  // terminate
  zVecFreeAtOnce( 4, dis, vel, acc, trq );
  rkChainDestroy( &chain );
  return 0;
}
