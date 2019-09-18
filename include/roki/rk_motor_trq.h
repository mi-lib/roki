/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_trq - motor model: direct torque motor
 * contributer: 2014-2015 Naoki Wakisaka
 */

#ifndef __RK_MOTOR_TRQ_H__
#define __RK_MOTOR_TRQ_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

typedef struct {
  /* value */
  double input;
  /* properties */
  double min; /*!< \brief minimum input torque */
  double max; /*!< \brief maximum input torque */
} rkMotorTrqPrp;

extern rkMotorCom rk_motor_trq;

__END_DECLS

#endif /* __RK_MOTOR_TRQ_H__ */
