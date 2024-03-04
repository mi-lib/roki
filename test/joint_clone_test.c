#include <roki/rk_joint.h>

void assert_fixed_set(rkJoint *joint)
{
}

bool assert_fixed_comp(rkJoint *joint1, rkJoint *joint2)
{
  return true;
}

void assert_revol_set(rkJoint *joint)
{
  rkJointRevolPrp *prp;
  double dis[1];

  prp = (rkJointRevolPrp *)joint->prp;
  prp->min = zDeg2Rad(-90);
  prp->max = zDeg2Rad(90);
  prp->stiffness = 0.1;
  prp->viscosity = 0.2;
  prp->coulomb = 0.3;
  prp->sf = 0.4;

  dis[0] = zDeg2Rad(45);
  rkJointSetDis( joint, dis );
}

bool assert_revol_comp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointRevolState *stat1, *stat2;
  rkJointRevolPrp *prp1, *prp2;

  stat1 = (rkJointRevolState *)joint1->state;
  stat2 = (rkJointRevolState *)joint2->state;
  prp1 = (rkJointRevolPrp *)joint1->prp;
  prp2 = (rkJointRevolPrp *)joint2->prp;
  return 
    zIsEqual( stat1->dis, stat2->dis, zTOL ) &&
    zIsEqual( prp1->min, prp2->min, zTOL ) &&
    zIsEqual( prp1->max, prp2->max, zTOL ) &&
    zIsEqual( prp1->stiffness, prp2->stiffness, zTOL ) &&
    zIsEqual( prp1->viscosity, prp2->viscosity, zTOL ) &&
    zIsEqual( prp1->coulomb, prp2->coulomb, zTOL ) &&
    zIsEqual( prp1->sf, prp2->sf, zTOL );
}

void assert_prism_set(rkJoint *joint)
{
  rkJointPrismPrp *prp;
  double dis[1];

  prp = (rkJointPrismPrp *)joint->prp;
  prp->min = -2.0;
  prp->max = 2.0;
  prp->stiffness = 0.1;
  prp->viscosity = 0.2;
  prp->coulomb = 0.3;
  prp->sf = 0.4;

  dis[0] = 1.0;
  rkJointSetDis( joint, dis );
}

bool assert_prism_comp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointPrismState *stat1, *stat2;
  rkJointPrismPrp *prp1, *prp2;

  stat1 = (rkJointPrismState *)joint1->state;
  stat2 = (rkJointPrismState *)joint2->state;
  prp1 = (rkJointPrismPrp *)joint1->prp;
  prp2 = (rkJointPrismPrp *)joint2->prp;
  return 
    zIsEqual( stat1->dis, stat2->dis, zTOL ) &&
    zIsEqual( prp1->min, prp2->min, zTOL ) &&
    zIsEqual( prp1->max, prp2->max, zTOL ) &&
    zIsEqual( prp1->stiffness, prp2->stiffness, zTOL ) &&
    zIsEqual( prp1->viscosity, prp2->viscosity, zTOL ) &&
    zIsEqual( prp1->coulomb, prp2->coulomb, zTOL ) &&
    zIsEqual( prp1->sf, prp2->sf, zTOL );
}

void assert_cylin_set(rkJoint *joint)
{
  rkJointCylinPrp *prp;
  double dis[2];

  prp = (rkJointCylinPrp *)joint->prp;

  prp->min[0] = -2.0;
  prp->max[0] = 2.0;
  prp->stiffness[0] = 0.1;
  prp->viscosity[0] = 0.2;
  prp->coulomb[0] = 0.3;
  prp->sf[0] = 0.4;

  prp->min[1] = zDeg2Rad(-90);
  prp->max[1] = zDeg2Rad(90);
  prp->stiffness[1] = 0.5;
  prp->viscosity[1] = 0.6;
  prp->coulomb[1] = 0.7;
  prp->sf[1] = 0.8;

  dis[0] = 1.0;
  dis[1] = zDeg2Rad(45);
  rkJointSetDis( joint, dis );
}

bool assert_cylin_comp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointCylinState *stat1, *stat2;
  rkJointCylinPrp *prp1, *prp2;

  stat1 = (rkJointCylinState *)joint1->state;
  stat2 = (rkJointCylinState *)joint2->state;
  prp1 = (rkJointCylinPrp *)joint1->prp;
  prp2 = (rkJointCylinPrp *)joint2->prp;
  return
    zIsEqual( stat1->dis[0], stat2->dis[0], zTOL ) &&
    zIsEqual( prp1->min[0], prp2->min[0], zTOL ) &&
    zIsEqual( prp1->max[0], prp2->max[0], zTOL ) &&
    zIsEqual( prp1->stiffness[0], prp2->stiffness[0], zTOL ) &&
    zIsEqual( prp1->viscosity[0], prp2->viscosity[0], zTOL ) &&
    zIsEqual( prp1->coulomb[0], prp2->coulomb[0], zTOL ) &&
    zIsEqual( prp1->sf[0], prp2->sf[0], zTOL ) &&
    zIsEqual( stat1->dis[1], stat2->dis[1], zTOL ) &&
    zIsEqual( prp1->min[1], prp2->min[1], zTOL ) &&
    zIsEqual( prp1->max[1], prp2->max[1], zTOL ) &&
    zIsEqual( prp1->stiffness[1], prp2->stiffness[1], zTOL ) &&
    zIsEqual( prp1->viscosity[1], prp2->viscosity[1], zTOL ) &&
    zIsEqual( prp1->coulomb[1], prp2->coulomb[1], zTOL ) &&
    zIsEqual( prp1->sf[1], prp2->sf[1], zTOL );
}

void assert_hooke_set(rkJoint *joint)
{
  rkJointHookePrp *prp;
  double dis[2];

  prp = (rkJointHookePrp *)joint->prp;

  prp->min[0] = zDeg2Rad(-90);
  prp->max[0] = zDeg2Rad(90);
  prp->stiffness[0] = 0.1;
  prp->viscosity[0] = 0.2;
  prp->coulomb[0] = 0.3;
  prp->sf[0] = 0.4;

  prp->min[1] = zDeg2Rad(-60);
  prp->max[1] = zDeg2Rad(60);
  prp->stiffness[1] = 0.5;
  prp->viscosity[1] = 0.6;
  prp->coulomb[1] = 0.7;
  prp->sf[1] = 0.8;

  dis[0] = zDeg2Rad(45);
  dis[1] = zDeg2Rad(30);
  rkJointSetDis( joint, dis );
}

bool assert_hooke_comp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointHookeState *stat1, *stat2;
  rkJointHookePrp *prp1, *prp2;

  stat1 = (rkJointHookeState *)joint1->state;
  stat2 = (rkJointHookeState *)joint2->state;
  prp1 = (rkJointHookePrp *)joint1->prp;
  prp2 = (rkJointHookePrp *)joint2->prp;
  return
    zIsEqual( stat1->dis[0], stat2->dis[0], zTOL ) &&
    zIsEqual( prp1->min[0], prp2->min[0], zTOL ) &&
    zIsEqual( prp1->max[0], prp2->max[0], zTOL ) &&
    zIsEqual( prp1->stiffness[0], prp2->stiffness[0], zTOL ) &&
    zIsEqual( prp1->viscosity[0], prp2->viscosity[0], zTOL ) &&
    zIsEqual( prp1->coulomb[0], prp2->coulomb[0], zTOL ) &&
    zIsEqual( prp1->sf[0], prp2->sf[0], zTOL ) &&
    zIsEqual( stat1->dis[1], stat2->dis[1], zTOL ) &&
    zIsEqual( prp1->min[1], prp2->min[1], zTOL ) &&
    zIsEqual( prp1->max[1], prp2->max[1], zTOL ) &&
    zIsEqual( prp1->stiffness[1], prp2->stiffness[1], zTOL ) &&
    zIsEqual( prp1->viscosity[1], prp2->viscosity[1], zTOL ) &&
    zIsEqual( prp1->coulomb[1], prp2->coulomb[1], zTOL ) &&
    zIsEqual( prp1->sf[1], prp2->sf[1], zTOL );
}

void assert_spher_set(rkJoint *joint)
{
  zVec3D axis;

  zVec3DCreate( &axis, 1, 2, 0 );
  zVec3DNormalizeDRC( &axis );
  rkJointSetDis( joint, axis.e );
}

bool assert_spher_comp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointSpherState *stat1, *stat2;

  stat1 = (rkJointSpherState *)joint1->state;
  stat2 = (rkJointSpherState *)joint2->state;
  return zVec3DEqual( &stat1->aa, &stat2->aa );
}

void assert_float_set(rkJoint *joint)
{
  zVec6D dis;

  zVec3DCreate( zVec6DLin(&dis), 0, 1, 2 );
  zVec3DCreate( zVec6DAng(&dis), 1, 2, 0 );
  zVec3DNormalizeDRC( zVec6DAng(&dis) );
  rkJointSetDis( joint, dis.e );
}

bool assert_float_comp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointFloatState *stat1, *stat2;

  stat1 = (rkJointFloatState *)joint1->state;
  stat2 = (rkJointFloatState *)joint2->state;
  return zVec6DEqual( &stat1->dis, &stat2->dis );
}

void assert_brfloat_set(rkJoint *joint)
{
  rkJointBrFloatPrp *prp;
  zVec6D dis;

  prp = (rkJointBrFloatPrp *)joint->prp;
  prp->ep_f = 1.0;
  prp->ep_t = 2.0;

  zVec3DCreate( zVec6DLin(&dis), 0, 1, 2 );
  zVec3DCreate( zVec6DAng(&dis), 1, 2, 0 );
  zVec3DNormalizeDRC( zVec6DAng(&dis) );
  rkJointSetDis( joint, dis.e );
}

bool assert_brfloat_comp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointBrFloatState *stat1, *stat2;
  rkJointBrFloatPrp *prp1, *prp2;

  stat1 = (rkJointBrFloatState *)joint1->state;
  stat2 = (rkJointBrFloatState *)joint2->state;
  prp1 = (rkJointBrFloatPrp *)joint1->prp;
  prp2 = (rkJointBrFloatPrp *)joint2->prp;
  return 
    zIsEqual( prp1->ep_f, prp2->ep_f, zTOL ) &&
    zIsEqual( prp1->ep_t, prp2->ep_t, zTOL ) &&
    zVec6DEqual( &stat1->dis, &stat2->dis );
}

bool assert_clone_one(char *str, void (*set_func)(rkJoint*), bool (*comp_func)(rkJoint*,rkJoint*))
{
  rkJoint jorg, jcln;

  if( rkJointQueryAssign( &jorg, str ) == NULL ) return false;
  set_func( &jorg );
  if( rkJointClone( &jorg, &jcln, NULL, NULL ) == NULL ) return false;
  return comp_func( &jorg, &jcln );
}

void assert_clone(void)
{
  zAssert( rkJointClone (fixed joint),
    assert_clone_one( "fixed", assert_fixed_set, assert_fixed_comp ) );
  zAssert( rkJointClone (revolute joint),
    assert_clone_one( "revolute", assert_revol_set, assert_revol_comp ) );
  zAssert( rkJointClone (prismatic joint),
    assert_clone_one( "prismatic", assert_prism_set, assert_prism_comp ) );
  zAssert( rkJointClone (cylindrical joint),
    assert_clone_one( "cylindrical", assert_cylin_set, assert_cylin_comp ) );
  zAssert( rkJointClone (universal joint),
    assert_clone_one( "hooke", assert_hooke_set, assert_hooke_comp ) );
  zAssert( rkJointClone (spherical joint),
    assert_clone_one( "spherical", assert_spher_set, assert_spher_comp ) );
  zAssert( rkJointClone (floating joint),
    assert_clone_one( "float", assert_float_set, assert_float_comp ) );
  zAssert( rkJointClone (breakable floating joint),
    assert_clone_one( "breakablefloat", assert_brfloat_set, assert_brfloat_comp ) );
}

int main(void)
{
  assert_clone();
  return 0;
}
