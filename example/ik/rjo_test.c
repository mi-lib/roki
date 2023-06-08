#include <roki/rk_chain.h>

#define WN 0.001

void ik_test(rkChain *chain, double px, double py, double pz, double ax, double ay, double az)
{
  rkIKAttr attr;
  rkIKCell *entry[2];
  zVec q;
  zVec6D ref, err;
  zFrame3D fd;

  rkChainCreateIK( chain );
  rkChainRegIKJoint( chain, "left_hip_rotation",    WN );
  rkChainRegIKJoint( chain, "left_hip_abduction",   WN );
  rkChainRegIKJoint( chain, "left_hip_flexion",     WN );
  rkChainRegIKJoint( chain, "left_knee_flexion",    WN );
  rkChainRegIKJoint( chain, "left_ankle_flexion",   WN );
  rkChainRegIKJoint( chain, "left_ankle_abduction", WN );
  zVec3DZero( &attr.ap );
  rkIKAttrSetLink( &attr, chain, "left_foot" );
  entry[0] = rkChainRegIKCellWldPos( chain, &attr, RK_IK_ATTR_ID | RK_IK_ATTR_AP );
  entry[1] = rkChainRegIKCellWldAtt( chain, &attr, RK_IK_ATTR_ID );

  q = zVecAlloc( zArraySize( rkChainIKJointIndex(chain) ) );
  rkChainDeactivateIK( chain );
  rkChainBindIK( chain );

  zVec6DCreate( &ref, px, py, pz, ax, ay, az );
  zVec6DToFrame3DZYX( &ref, &fd );
  rkIKCellSetRefVec( entry[0], zVec6DLin(&ref) );
  rkIKCellSetRefVec( entry[1], zVec6DAng(&ref) );
  rkChainIK_RJO( chain, q, zTOL, 0 );
  zFrame3DError( &fd, rkChainLinkWldFrame(chain,rkIKCellAttr(entry[0])->id), &err );
  zVec6DPrint( &err );
  zVecFree( q );
}

int main(int argc, char *argv[])
{
  rkChain chain;

  rkChainReadZTK( &chain, "../model/H5.ztk" );
  ik_test( &chain, 0.02, 0.05, 0.05, 0, 0, 0 );
  ik_test( &chain, 0.02, 0.05, 0.05, 0, -zDeg2Rad(45), 0 );
  rkChainDestroy( &chain );
  return 0;
}
