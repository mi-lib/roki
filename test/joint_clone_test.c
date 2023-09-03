#include <roki/rk_joint.h>

void assert_fixed_set_prp(rkJoint *joint)
{
}

bool assert_fixed_comp_prp(rkJoint *joint1, rkJoint *joint2)
{
  return true;
}

void assert_revol_set_prp(rkJoint *joint)
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

bool assert_revol_comp_prp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointRevolPrp *prp1, *prp2;

  prp1 = (rkJointRevolPrp *)joint1->prp;
  prp2 = (rkJointRevolPrp *)joint2->prp;
  return 
    zIsEqual( prp1->dis, prp2->dis, zTOL ) &&
    zIsEqual( prp1->min, prp2->min, zTOL ) &&
    zIsEqual( prp1->max, prp2->max, zTOL ) &&
    zIsEqual( prp1->stiffness, prp2->stiffness, zTOL ) &&
    zIsEqual( prp1->viscosity, prp2->viscosity, zTOL ) &&
    zIsEqual( prp1->coulomb, prp2->coulomb, zTOL ) &&
    zIsEqual( prp1->sf, prp2->sf, zTOL );
}

void assert_prism_set_prp(rkJoint *joint)
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

bool assert_prism_comp_prp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointPrismPrp *prp1, *prp2;

  prp1 = (rkJointPrismPrp *)joint1->prp;
  prp2 = (rkJointPrismPrp *)joint2->prp;
  return 
    zIsEqual( prp1->dis, prp2->dis, zTOL ) &&
    zIsEqual( prp1->min, prp2->min, zTOL ) &&
    zIsEqual( prp1->max, prp2->max, zTOL ) &&
    zIsEqual( prp1->stiffness, prp2->stiffness, zTOL ) &&
    zIsEqual( prp1->viscosity, prp2->viscosity, zTOL ) &&
    zIsEqual( prp1->coulomb, prp2->coulomb, zTOL ) &&
    zIsEqual( prp1->sf, prp2->sf, zTOL );
}

void assert_cylin_set_prp(rkJoint *joint)
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

bool assert_cylin_comp_prp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointCylinPrp *prp1, *prp2;

  prp1 = (rkJointCylinPrp *)joint1->prp;
  prp2 = (rkJointCylinPrp *)joint2->prp;
  return
    zIsEqual( prp1->dis[0], prp2->dis[0], zTOL ) &&
    zIsEqual( prp1->min[0], prp2->min[0], zTOL ) &&
    zIsEqual( prp1->max[0], prp2->max[0], zTOL ) &&
    zIsEqual( prp1->stiffness[0], prp2->stiffness[0], zTOL ) &&
    zIsEqual( prp1->viscosity[0], prp2->viscosity[0], zTOL ) &&
    zIsEqual( prp1->coulomb[0], prp2->coulomb[0], zTOL ) &&
    zIsEqual( prp1->sf[0], prp2->sf[0], zTOL ) &&
    zIsEqual( prp1->dis[1], prp2->dis[1], zTOL ) &&
    zIsEqual( prp1->min[1], prp2->min[1], zTOL ) &&
    zIsEqual( prp1->max[1], prp2->max[1], zTOL ) &&
    zIsEqual( prp1->stiffness[1], prp2->stiffness[1], zTOL ) &&
    zIsEqual( prp1->viscosity[1], prp2->viscosity[1], zTOL ) &&
    zIsEqual( prp1->coulomb[1], prp2->coulomb[1], zTOL ) &&
    zIsEqual( prp1->sf[1], prp2->sf[1], zTOL );
}

void assert_hooke_set_prp(rkJoint *joint)
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

bool assert_hooke_comp_prp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointHookePrp *prp1, *prp2;

  prp1 = (rkJointHookePrp *)joint1->prp;
  prp2 = (rkJointHookePrp *)joint2->prp;
  return
    zIsEqual( prp1->dis[0], prp2->dis[0], zTOL ) &&
    zIsEqual( prp1->min[0], prp2->min[0], zTOL ) &&
    zIsEqual( prp1->max[0], prp2->max[0], zTOL ) &&
    zIsEqual( prp1->stiffness[0], prp2->stiffness[0], zTOL ) &&
    zIsEqual( prp1->viscosity[0], prp2->viscosity[0], zTOL ) &&
    zIsEqual( prp1->coulomb[0], prp2->coulomb[0], zTOL ) &&
    zIsEqual( prp1->sf[0], prp2->sf[0], zTOL ) &&
    zIsEqual( prp1->dis[1], prp2->dis[1], zTOL ) &&
    zIsEqual( prp1->min[1], prp2->min[1], zTOL ) &&
    zIsEqual( prp1->max[1], prp2->max[1], zTOL ) &&
    zIsEqual( prp1->stiffness[1], prp2->stiffness[1], zTOL ) &&
    zIsEqual( prp1->viscosity[1], prp2->viscosity[1], zTOL ) &&
    zIsEqual( prp1->coulomb[1], prp2->coulomb[1], zTOL ) &&
    zIsEqual( prp1->sf[1], prp2->sf[1], zTOL );
}

void assert_spher_set_prp(rkJoint *joint)
{
  zVec3D axis;

  zVec3DCreate( &axis, 1, 2, 0 );
  zVec3DNormalizeDRC( &axis );
  rkJointSetDis( joint, axis.e );
}

bool assert_spher_comp_prp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointSpherPrp *prp1, *prp2;

  prp1 = (rkJointSpherPrp *)joint1->prp;
  prp2 = (rkJointSpherPrp *)joint2->prp;
  return zVec3DEqual( &prp1->aa, &prp2->aa );
}

void assert_float_set_prp(rkJoint *joint)
{
  zVec6D dis;

  zVec3DCreate( zVec6DLin(&dis), 0, 1, 2 );
  zVec3DCreate( zVec6DAng(&dis), 1, 2, 0 );
  zVec3DNormalizeDRC( zVec6DAng(&dis) );
  rkJointSetDis( joint, dis.e );
}

bool assert_float_comp_prp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointFloatPrp *prp1, *prp2;

  prp1 = (rkJointFloatPrp *)joint1->prp;
  prp2 = (rkJointFloatPrp *)joint2->prp;
  return zVec6DEqual( &prp1->dis, &prp2->dis );
}

void assert_brfloat_set_prp(rkJoint *joint)
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

bool assert_brfloat_comp_prp(rkJoint *joint1, rkJoint *joint2)
{
  rkJointBrFloatPrp *prp1, *prp2;

  prp1 = (rkJointBrFloatPrp *)joint1->prp;
  prp2 = (rkJointBrFloatPrp *)joint2->prp;
  return 
    zIsEqual( prp1->ep_f, prp2->ep_f, zTOL ) &&
    zIsEqual( prp1->ep_t, prp2->ep_t, zTOL ) &&
    zVec6DEqual( &prp1->dis, &prp2->dis );
}

bool assert_clone_one(char *str, void (*set_prp)(rkJoint*), bool (*comp_prp)(rkJoint*,rkJoint*))
{
  rkJoint jorg, jcln;

  if( rkJointQueryAssign( &jorg, str ) == NULL ) return false;
  set_prp( &jorg );
  if( rkJointClone( &jorg, &jcln ) == NULL ) return false;
  return comp_prp( &jorg, &jcln );
}

void assert_clone(void)
{
  zAssert( rkJointClone, assert_clone_one("fixed", assert_fixed_set_prp, assert_fixed_comp_prp) );
  zAssert( rkJointClone, assert_clone_one("revolute", assert_revol_set_prp, assert_revol_comp_prp) );
  zAssert( rkJointClone, assert_clone_one("prismatic", assert_prism_set_prp, assert_prism_comp_prp) );
  zAssert( rkJointClone, assert_clone_one("cylindrical", assert_cylin_set_prp, assert_cylin_comp_prp) );
  zAssert( rkJointClone, assert_clone_one("hooke", assert_hooke_set_prp, assert_hooke_comp_prp) );
  zAssert( rkJointClone, assert_clone_one("spherical", assert_spher_set_prp, assert_spher_comp_prp) );
  zAssert( rkJointClone, assert_clone_one("float", assert_float_set_prp, assert_float_comp_prp) );
  zAssert( rkJointClone, assert_clone_one("breakablefloat", assert_brfloat_set_prp, assert_brfloat_comp_prp) );
}

int main(void)
{
  assert_clone();
  return 0;
}
