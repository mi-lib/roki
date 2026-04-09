#include <roki/roki.h>

void link_mp_rand(rkLink *link)
{
  zVec3D com{ zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) };
  double i11, i12, i13, i22, i23, i33;
  i11 = zRandF(0.01,0.1);
  i12 =-zRandF(0.0001,0.001);
  i13 =-zRandF(0.0001,0.001);
  i22 = zRandF(0.01,0.1);
  i23 =-zRandF(0.00001,0.0001);
  i33 = zRandF(0.01,0.1);
  zMat3D inertia{ i11, i12, i13, i12, i22, i23, i13, i23, i33 };
  link->body.mp.create( zRandF(0.1,1.0), com, inertia );
}

#define LINK_NUM 9

int chain_init(rkChain *chain)
{
  chain->init();
  chain->linkarray.alloc( LINK_NUM );
  for(int i=0; i<LINK_NUM; i++ ){
    char name[BUFSIZ];
    sprintf( name, "link#%02d", i );
    chain->link(i)->init();
    link_mp_rand( chain->link(i) );
    chain->link(i)->orgframe.pos.create( zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
    chain->link(i)->orgframe.att.createFromAA( zVec3D{ zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) } );
    zNameSet( chain->link(i), name );
  }
  chain->link(0)->addChild( chain->link(1) );
  chain->link(1)->addChild( chain->link(2) );
  chain->link(2)->addChild( chain->link(3) );
  chain->link(3)->addChild( chain->link(4) );
  chain->link(0)->addChild( chain->link(5) );
  chain->link(5)->addChild( chain->link(6) );
  chain->link(6)->addChild( chain->link(7) );
  chain->link(7)->addChild( chain->link(8) );

  chain->link(0)->joint.assign( "float" );       // 0-5
  chain->link(1)->joint.assign( "spherical" );   // 6-8
  chain->link(2)->joint.assign( "revolute" );    // 9
  chain->link(3)->joint.assign( "cylindrical" ); // 10-11
  chain->link(4)->joint.assign( "planar" );      // 12-14
  chain->link(5)->joint.assign( "revolute" );    // 15
  chain->link(6)->joint.assign( "prismatic" );   // 16
  chain->link(7)->joint.assign( "hooke" );       // 17-18
  chain->link(8)->joint.assign( "fixed" );

  chain->setJointIDOffset();
  chain->updateCRBMass();
  chain->updateForwardKinematics();
  chain->updateInverseDynamics();
  return chain->jointSize();
}

void assert_chain_link()
{
  rkChain chain;
  chain_init( &chain );
  zAssert( C++::rkChain.link (valid cases), chain.link(0) && chain.link(8) );
  zAssert( C++::rkChain.link (invalid cases), !chain.link(-1) && !chain.link(9) );
  chain.destroy();
}

int main()
{
  zRandInit();
  assert_chain_link();
  return 0;
}
