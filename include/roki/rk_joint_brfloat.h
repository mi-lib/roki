/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_brfloat - joint structure: breakable free-floating joint
 */

#ifndef __RK_JOINT_BRFLOAT_H__
#define __RK_JOINT_BRFLOAT_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointBrFloatPrp ){
  double ep_f; /* threshold force to break joint */
  double ep_t; /* threshold torque to break joint */
};

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointBrFloatState ){
  zVec6D dis; /* joint displacement (translational vector + angle-axis vector) */
  zVec6D vel; /* velocity (translational vector + angular vector) */
  zVec6D acc; /* acceleration (translational vector + angular vector) */
  zVec6D trq; /* torque (translational vector + angular vector) */
  /* internal matrix */
  zMat3D _att;
};

__ROKI_EXPORT rkJointCom rk_joint_brfloat;

__END_DECLS

#endif /* __RK_JOINT_BRFLOAT_H__ */
