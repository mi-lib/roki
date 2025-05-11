#include <roki/rk_link.h>

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

#define N 10

void assert_link_shape(void)
{
  rkLink l;
  zShape3D *s;
  int i;
  rkMP mp1, mp2;
  double v;
  zMat3D inertia;

  rkLinkInit( &l );
  rkMPZero( &mp1 );
  for( i=0; i<N;i ++ ){
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

int main(int argc, char *argv[])
{
  assert_link_shape();
  return 0;
}
