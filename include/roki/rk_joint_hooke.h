/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_hooke - joint structure: universal joint
 */

#ifndef __RK_JOINT_HOOKE_H__
#define __RK_JOINT_HOOKE_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* 0: rotation about z-axis, 1: rotation about y-axis */

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointHookeState ){
  double dis[2]; /* joint displacement */
  double vel[2]; /* joint velocity */
  double acc[2]; /* joint acceleration */
  double trq[2]; /* joint torque */
  /* trigonometric values */
  double _s[2], _c[2];
  /* for forward dynamics */
  rkJointFrictionPivot _fp[2];
  double _u[2];
};

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointHookePrp ){
  double min[2], max[2]; /* joint displacement limiter */
  double stiffness[2];   /* joint stiffness */
  double viscosity[2];   /* joint viscosity */
  double coulomb[2];     /* joint coulomb friction */
  /* friction */
  double sf[2];
  double tf[2];

  /* motor */
  rkMotor m;
};

__ROKI_EXPORT rkJointCom rk_joint_hooke;

__END_DECLS

#endif /* __RK_JOINT_HOOKE_H__ */
