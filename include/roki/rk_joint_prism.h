/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_prism - joint structure: prismatic joint
 */

#ifndef __RK_JOINT_PRISM_H__
#define __RK_JOINT_PRISM_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointPrismPrp ){
  /* joint displacement, velocity, acceleration and torque */
  double dis, vel, acc, trq;
  double min, max; /* limiter */
  /* joint stiffness, viscosity and coulomb friction */
  double stiffness;
  double viscosity;
  double coulomb;
  /* friction */
  double tf;
  /* static friction */
  double sf;

  /* motor */
  rkMotor m;

  /* for forward dynamics */
  rkJointFrictionPivot _fp;
  double _u;
};

__ROKI_EXPORT rkJointCom rk_joint_prism;

__END_DECLS

#endif /* __RK_JOINT_PRISM_H__ */
