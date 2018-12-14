/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_brfloat - joint structure: breakable free-floating joint
 */

#ifndef __RK_JOINT_BRFLOAT_H__
#define __RK_JOINT_BRFLOAT_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

typedef struct{
  /* joint displacement
     0-2: translational displacement
     3-5: rotational displacement by angle-axis vector */
  /* velocity, acceleration and torque
     0-2: translational vector
     3-5: rotational vector */
  zVec6D dis, vel, acc, trq;

  zMat3D _att; /* internal matrix */

  double ep_f, ep_t; /* threshold to break joint */
} rkJointPrpBrFloat;

__EXPORT rkJoint *rkJointCreateBrFloat(rkJoint *joint);

__END_DECLS

#endif /* __RK_JOINT_BRFLOAT_H__ */
