/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_cylin - joint structure: cylindrical joint
 */

#ifndef __RK_JOINT_CYLIN_H__
#define __RK_JOINT_CYLIN_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

typedef struct{
  /* 0: prismatic, 1: revolutional */
  /* joint displacement, velocity, acceleration and torque */
  double dis[2], vel[2], acc[2], trq[2];
  double min[2], max[2]; /* limiter */
  /* joint stiffness, viscosity and coulomb friction */
  double stiff[2], viscos[2], coulomb[2];
  /* trigonometric values */
  double _s, _c;
  /* friction */
  double sf[2];
  double tf[2];

  /* motor */
  rkMotor m;

  /* forward dynamics */
  rkJointRef _ref[2];
  double _u[2];
} rkJointPrpCylin;

__EXPORT rkJoint *rkJointCreateCylin(rkJoint *joint);

__END_DECLS

#endif /* __RK_JOINT_CYLIN_H__ */
