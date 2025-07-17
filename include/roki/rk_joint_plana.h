/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint_plana - joint structure: planar joint (for wheeled mobile base)
 */

#ifndef __RK_JOINT_PLANA_H__
#define __RK_JOINT_PLANA_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* 0, 1: planar translation, 2: rotation */

/* this class is particularly for modelling wheeled mobile bases. */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointPlanaState ){
  zCoord2D dis; /* joint displacement */
  zCoord2D vel; /* joint velocity */
  zCoord2D acc; /* joint acceleration */
  zCoord2D trq; /* joint torque */
  /* trigonometric values */
  double _s, _c;
};

__ROKI_EXPORT rkJointCom rk_joint_plana;

__END_DECLS

#endif /* __RK_JOINT_PLANA_H__ */
