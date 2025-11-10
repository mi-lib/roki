#include <roki/rk_chain.h>

zShape3D *generate_box_rand(void)
{
  zVec3D center, ax, ay, az, tmp;
  zShape3D *box;

  if( !( box = zAlloc( zShape3D, 1 ) ) ) return NULL;
  zVec3DCreate( &center, zRandF(-3,3), zRandF(-3,3), zRandF(-3,3) );
  zVec3DCreate( &ax, zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
  zVec3DCreate( &tmp, zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
  zVec3DOrthogonalize( &tmp, &ax, &ay );
  zVec3DOuterProd( &ax, &ay, &az );
  zVec3DNormalizeDRC( &ax );
  zVec3DNormalizeDRC( &ay );
  zVec3DNormalizeDRC( &az );
  return zShape3DBoxCreate( box, &center, &ax, &ay, &az, zRandF(0.1,5), zRandF(0.1,5), zRandF(0.1,5) );
}

void assert_link_shape(void)
{
  rkLink l;
  zShape3D *s;
  const int n = 10;
  int i;
  rkMP mp1, mp2;
  double v;
  zMat3D inertia;

  rkLinkInit( &l );
  rkMPZero( &mp1 );
  for( i=0; i<n;i ++ ){
    if( !( s = generate_box_rand() ) ) return;
    rkLinkShapePush( &l, s );
    rkMPMass(&mp1) += ( v = zBox3DVolume( zShape3DBox(s) ) );
    zVec3DCatDRC( rkMPCOM(&mp1), v, zBox3DCenter( zShape3DBox(s) ) );
    zBox3DBaryInertia( zShape3DBox(s), 1, &inertia );
    zMat3DCatVec3DDoubleOuterProdDRC( &inertia,-v, zBox3DCenter( zShape3DBox(s) ) );
    zMat3DAddDRC( rkMPInertia(&mp1), &inertia );
  }
  zVec3DDivDRC( rkMPCOM(&mp1), rkMPMass(&mp1) );
  zMat3DCatVec3DDoubleOuterProdDRC( rkMPInertia(&mp1), rkMPMass(&mp1), rkMPCOM(&mp1) );
  rkLinkShapeMP( &l, 1, &mp2 );
  zAssert( rkLinkShapeMP,
    zIsTiny( rkMPMass(&mp1) - rkMPMass(&mp2) ) &&
    zVec3DEqual( rkMPCOM(&mp1), rkMPCOM(&mp2) ) &&
    zMat3DEqual( rkMPInertia(&mp1), rkMPInertia(&mp2) ) );
  while( ( s = rkLinkShapePop( &l ) ) ){
    zShape3DDestroy( s );
    free( s );
  }
}

void assert_link_is_included(void)
{
  rkChain puma1, puma2;
  int i;
  bool result1, result2;

  rkChainReadZTK( &puma1, "../example/model/puma.ztk" );
  rkChainClone( &puma1, &puma2 );
  /* positive case */
  result1 = true;
  for( i=0; i<rkChainLinkNum(&puma1); i++ ){
    if( !rkChainLinkIsIncluded( &puma1, rkChainLink(&puma1,i) ) ) result1 = false;
    if( !rkChainLinkIsIncluded( &puma2, rkChainLink(&puma2,i) ) ) result1 = false;
  }
  /* negative case */
  result2 = true;
  for( i=0; i<rkChainLinkNum(&puma1); i++ ){
    if( rkChainLinkIsIncluded( &puma1, rkChainLink(&puma2,i) ) ) result2 = false;
    if( rkChainLinkIsIncluded( &puma2, rkChainLink(&puma1,i) ) ) result2 = false;
  }
  rkChainDestroy( &puma1 );
  rkChainDestroy( &puma2 );
  zAssert( rkChainLinkIsIncluded (positive case), result1 );
  zAssert( rkChainLinkIsIncluded (negative case), result2 );
}

void assert_link_detach(void)
{
  rkChain puma, box;
  rkLink *link6;
  zVec joint_dis;
  zVec6D hand_pose, box_pose;
  const int n = 100;
  int i;
  bool result1 = true, result2 = true;

  rkChainReadZTK( &puma, "../example/model/puma.ztk" );
  rkChainReadZTK( &box, "../example/model/box.ztk" );
  joint_dis = zVecAlloc( rkChainJointSize( &puma ) );

  link6 = rkChainFindLink( &puma, "link6" );
  rkLinkAttach( rkChainRoot(&box), link6 );
  for( i=0; i<n; i++ ){
    zVecRandUniform( joint_dis, -zPI, zPI );
    rkChainFK( &puma, joint_dis );
    zFrame3DToVec6DAA( rkLinkWldFrame(link6), &hand_pose );
    zFrame3DToVec6DAA( rkChainRootFrame(&box), &box_pose );
    if( !zVec6DEqual( &hand_pose, &box_pose ) ) result1 = false;
  }
  rkLinkDetach( rkChainRoot(&box) );
  for( i=0; i<n; i++ ){
    zVecRandUniform( joint_dis, -zPI, zPI );
    rkChainFK( &puma, joint_dis );
    zFrame3DToVec6DAA( rkLinkWldFrame(link6), &hand_pose );
    zFrame3DToVec6DAA( rkChainRootFrame(&box), &box_pose );
    if( zVec6DEqual( &hand_pose, &box_pose ) ) result1 = false;
  }
  zVecFree( joint_dis );
  rkChainDestroy( &puma );
  rkChainDestroy( &box );
  zAssert( rkLinkAttach & rkLinkDetach, result1 && result2 );
}

int main(int argc, char *argv[])
{
  zRandInit();
  assert_link_shape();
  assert_link_is_included();
  assert_link_detach();
  return 0;
}
