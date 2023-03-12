#include <roki/rk_chain.h>

#define WN 0.001

int main(int argc, char *argv[])
{
  rkChain robot;
  rkIKCellAttr attr;
  rkIKCell *entry[2];
  zVec q;
  zVec3D pl = { { 0.01, 0.02, 0.05 } };

  rkChainReadZTK( &robot, "../model/H5.ztk" );
  rkChainCreateIK( &robot );
  rkChainRegIKJoint( &robot, "left_hip_rotation",    WN );
  rkChainRegIKJoint( &robot, "left_hip_abduction",   WN );
  rkChainRegIKJoint( &robot, "left_hip_flexion",     WN );
  rkChainRegIKJoint( &robot, "left_knee_flexion",    WN );
  rkChainRegIKJoint( &robot, "left_ankle_flexion",   WN );
  rkChainRegIKJoint( &robot, "left_ankle_abduction", WN );
  rkIKCellAttrSetLink( &attr, &robot, "left_foot" );
  entry[0] = rkChainRegIKCellWldPos( &robot, &attr, RK_IK_CELL_ATTR_ID );
  entry[1] = rkChainRegIKCellWldAtt( &robot, &attr, RK_IK_CELL_ATTR_ID );
  q = zVecAlloc( zArraySize( rkChainIKJointIndex(&robot) ) );
  rkChainBindIK( &robot );
  rkIKCellSetRefVec( entry[0], &pl );
  rkIKCellSetRefVec( entry[1], ZVEC3DZERO );

  rkChainIK_RJO( &robot, q, zTOL, 0 );
  zVecPrint( q );
  zFrame3DPrint( rkChainLinkWldFrame(&robot,attr.id) );

  zVecFree( q );
  rkChainDestroy( &robot );
  return 0;
}
