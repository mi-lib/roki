/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_spher - joint structure: spherical joint
 */

#ifndef __RK_JOINT_SPHER_H__
#define __RK_JOINT_SPHER_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointSpherState ){
  zVec3D aa;  /* joint displacement (angle-axis vector) */
  zVec3D vel; /* velocity (angular vector) */
  zVec3D acc; /* acceleration (angular vector) */
  zVec3D trq; /* torque (angular vector) */
  /* internal matrix */
  zMat3D _att;
  /* for forward dynamics */
  double _u[3];
};

__ROKI_EXPORT rkJointCom rk_joint_spher;

__END_DECLS

#endif /* __RK_JOINT_SPHER_H__ */
