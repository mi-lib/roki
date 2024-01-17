/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_prism - joint structure: prismatic joint
 */

#ifndef __RK_JOINT_PRISM_H__
#define __RK_JOINT_PRISM_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointPrismState ){
  double dis; /* joint displacement */
  double vel; /* joint velocity */
  double acc; /* joint acceleration */
  double trq; /* joint torque */
  /* for forward dynamics */
  rkJointFrictionPivot _fp;
  double _u;
};

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointPrismPrp ){
  double min, max;  /* joint displacement limiter */
  double stiffness; /* joint stiffness */
  double viscosity; /* joint viscosity */
  double coulomb;   /* joint coulomb friction */
  double tf;  /* friction */
  double sf;  /* static friction */

  /* motor */
  rkMotor m;
};

__ROKI_EXPORT rkJointCom rk_joint_prism;

__END_DECLS

#endif /* __RK_JOINT_PRISM_H__ */
