#include <roki/rk_chain.h>

#define WN 0.001

int main(int argc, char *argv[])
{
  rkChain robot;
  rkIKCellAttr attr;
  rkIKCell *entry[2];
  zVec q;
  zVec6D ref = { { 0.01, 0.02, 0.05, 0.0, -zDeg2Rad(45), 0.0 } };

  rkChainReadZTK( &robot, "../model/H5.ztk" );
  rkChainCreateIK( &robot );
  rkChainRegIKJoint( &robot, "left_hip_rotation",    WN );
  rkChainRegIKJoint( &robot, "left_hip_abduction",   WN );
  rkChainRegIKJoint( &robot, "left_hip_flexion",     WN );
  rkChainRegIKJoint( &robot, "left_knee_flexion",    WN );
  rkChainRegIKJoint( &robot, "left_ankle_flexion",   WN );
  rkChainRegIKJoint( &robot, "left_ankle_abduction", WN );
  zVec3DZero( &attr.ap );
  rkIKCellAttrSetLink( &attr, &robot, "left_foot" );
  entry[0] = rkChainRegIKCellWldPos( &robot, &attr, RK_IK_CELL_ATTR_ID | RK_IK_CELL_ATTR_AP );
  entry[1] = rkChainRegIKCellWldAtt( &robot, &attr, RK_IK_CELL_ATTR_ID );
#if 0
  q = zVecAlloc( zArraySize( rkChainIKJointIndex(&robot) ) );
#else
  q = zVecAlloc( rkChainJointSize(&robot) );
#endif
  rkChainDeactivateIK( &robot );
  rkChainBindIK( &robot );

  zVec6DPrint( &ref );
  rkIKCellSetRefVec( entry[0], zVec6DLin(&ref) );
  rkIKCellSetRefVec( entry[1], zVec6DAng(&ref) );

#if 0
  rkChainIK_RJO( &robot, q, zTOL, 0 );
#else
  rkChainIK( &robot, q, zTOL, 0 );
#endif
  zVecPrint( q );
  zFrame3DPrint( rkChainLinkWldFrame(&robot,attr.id) );

  zVecFree( q );
  rkChainDestroy( &robot );
  return 0;
}
