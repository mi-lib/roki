/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_revol - joint structure: revolutional joint
 */

#ifndef __RK_JOINT_REVOL_H__
#define __RK_JOINT_REVOL_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointRevolPrp ){
  /* joint displacement, velocity, acceleration and torque */
  double dis, vel, acc, trq;
  /* trigonometric values */
  double _s, _c;

  double min, max;  /* joint displacement limiter */
  double stiffness; /* joint stiffness */
  double viscosity; /* joint viscosity */
  double coulomb;   /* joint coulomb friction */
  double tf;  /* friction */
  double sf;  /* static friction */

  /* motor */
  rkMotor m;

  /* for forward dynamics */
  rkJointFrictionPivot _fp;
  double _u;
};

__ROKI_EXPORT rkJointCom rk_joint_revol;

__END_DECLS

#endif /* __RK_JOINT_REVOL_H__ */
