/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_hooke - joint structure: universal joint
 */

#ifndef __RK_JOINT_HOOKE_H__
#define __RK_JOINT_HOOKE_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

typedef struct{
  /* 0: rotation about z-axis, 1: rotation about y-axis */
  /* joint displacement, velocity, acceleration and torque */
  double dis[2], vel[2], acc[2], trq[2];
  double min[2], max[2]; /* limiter */
  /* joint stiffness, viscosity and coulomb friction */
  double stiffness[2];
  double viscosity[2];
  double coulomb[2];
  /* trigonometric values */
  double _s[2], _c[2];
  /* friction */
  double sf[2];
  double tf[2];

  /* motor */
  rkMotor m;

  /* for forward dynamics */
  rkJointFrictionPivot _fp[2];
  double _u[2];
} rkJointHookePrp;

__EXPORT rkJointCom rk_joint_hooke;

__END_DECLS

#endif /* __RK_JOINT_HOOKE_H__ */
