#include <roki/rk_chain.h>

#define DT 0.001
#define DTHETA zDeg2Rad(20.0)
#define STEP 100

rkChain chain;
zVec dis;

zVec3D com, vel, acc;

void update_dis(int st)
{
  double val;
  zMat3D r;

  val = DTHETA * ( 1 - cos(2*zPI*st/STEP) );
  zMat3DZYX( &r, val, val, val );
  zMat3DToAA( &r, (zVec3D*)&zVecElem(dis,0) );
}

void world_com_test(void)
{
  zVec3D v, a;

  zVec3DDataWrite( rkChainLinkWldPos(&chain,1) );
  zMulMatVec3D( rkChainLinkWldAtt(&chain,1), rkChainLinkCOMVel(&chain,1), &v );
  zVec3DDataWrite( &v );
  zMulMatVec3D( rkChainLinkWldAtt(&chain,1), rkChainLinkCOMAcc(&chain,1), &a );
  zVec3DDataWrite( &a );

  zVec3DDataWrite( rkChainWldCOM(&chain) );
  zVec3DDataWrite( rkChainCOMVel(&chain) );
  zVec3DDataWrite( rkChainCOMAcc(&chain) );

  zVec3DSub( rkChainWldCOM(&chain), &com, &v );
  zVec3DDivDRC( &v, DT );
  zVec3DDataWrite( &v );
  zVec3DSub( &v, &vel, &a );
  zVec3DDivDRC( &a, DT );
  zVec3DDataNLWrite( &a );

  zVec3DCopy( rkChainWldCOM(&chain), &com );
  zVec3DCopy( &v, &vel );
}




void chain_init(rkChain *chain)
{
  char name[BUFSIZ];

  rkChainInit( chain );
  zArrayAlloc( &chain->link, rkLink, 2 );
  sprintf( name, "root" );
  rkLinkInit( rkChainLink(chain,0) );
  zNameSet( rkChainLink(chain,0), name );
  sprintf( name, "tip" );
  rkLinkInit( rkChainLink(chain,1) );
  rkChainSetMass( chain, rkLinkSetMass( rkChainLink(chain,1), 1.0 ) );
  zNameSet( rkChainLink(chain,1), name );
  rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,1) );

  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_SPHER );
  rkJointCreate( rkChainLinkJoint(chain,1), RK_JOINT_FIXED );
  zVec3DCreate( rkChainLinkOrgPos(chain,1), 1, 0, 0 );
  zFrame3DCopy( rkChainLinkOrgFrame(chain,1), rkChainLinkAdjFrame(chain,1) );
  rkChainSetOffset( chain );
}

int main(void)
{
  register int i;

  chain_init( &chain );
  rkChainConnectionFWrite( stderr, &chain );
  rkChainUpdateID( &chain );
  zVec3DCopy( rkChainWldCOM(&chain), &com );
  zVec3DCopy( rkChainCOMVel(&chain), &vel );
  dis = zVecAlloc( rkChainJointSize( &chain ) );
  rkChainGetJointDisAll( &chain, dis );
  for( i=0; i<=STEP; i++ ){
    update_dis( i );
    rkChainFKCNT( &chain, dis, DT );
    world_com_test();
  }
  zVecFree( dis );
  rkChainDestroy( &chain );
  return 0;
}
