/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_spher - joint structure: spherical joint
 */

#ifndef __RK_JOINT_SPHER_H__
#define __RK_JOINT_SPHER_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

typedef struct{
  /* joint displacement: angle-axis vector */
  /* velocity, acceleration and torque: angular vector */
  zVec3D aa, vel, acc, trq;

  zMat3D _att; /* internal matrix */

  /* motor */
  rkMotor m;

  /* for forward dynamics */
  double _u[3];
} rkJointPrpSpher;

__EXPORT rkJoint *rkJointCreateSpher(rkJoint *joint);

__END_DECLS

#endif /* __RK_JOINT_SPHER_H__ */
