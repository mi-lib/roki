/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_revol - joint structure: revolutional joint
 */

#ifndef __RK_JOINT_REVOL_H__
#define __RK_JOINT_REVOL_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

typedef struct{
  /* joint displacement, velocity, acceleration and torque */
  double dis, vel, acc, trq;
  double min, max; /* limiter */
  /* joint stiffness, viscosity and coulomb friction */
  double stiff, viscos, coulomb;
  /* friction */
  double tf;
  /* static friction */
  double sf;
  /* trigonometric values */
  double _s, _c;

  /* motor */
  rkMotor m;

  /* for forward dynamics */
  rkJointRef _ref;
  double _u;
} rkJointPrpRevol;

__EXPORT rkJoint *rkJointCreateRevol(rkJoint *joint);

__END_DECLS

#endif /* __RK_JOINT_REVOL_H__ */
