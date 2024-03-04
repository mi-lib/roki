/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_cylin - joint structure: cylindrical joint
 */

#ifndef __RK_JOINT_CYLIN_H__
#define __RK_JOINT_CYLIN_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* 0: prismatic translation, 1: rotation */

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointCylinPrp ){
  double min[2], max[2]; /* joint displacement limiter */
  double stiffness[2];   /* joint stiffness */
  double viscosity[2];   /* joint viscosity */
  double coulomb[2];     /* joint coulomb friction */
  /* friction */
  double sf[2];
  double tf[2];
};

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointCylinState ){
  double dis[2]; /* joint displacement */
  double vel[2]; /* joint velocity */
  double acc[2]; /* joint acceleration */
  double trq[2]; /* joint torque */
  /* trigonometric values */
  double _s, _c;
  /* forward dynamics */
  rkJointFrictionPivot _fp[2];
  double _u[2];
};

__ROKI_EXPORT rkJointCom rk_joint_cylin;

__END_DECLS

#endif /* __RK_JOINT_CYLIN_H__ */
