/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor_dc - motor model: DC motor
 * contributer: 2014-2015 Naoki Wakisaka
 */

#ifndef __RK_MOTOR_DC_H__
#define __RK_MOTOR_DC_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

typedef struct {
  /* value */
  double e;  /* applied voltage */
  double tf; /* friction torque */

  /* properties */
  double k;              /* motor constant */
  double admit;          /* inner admitance */
  double maxvol, minvol; /* maximum & minimum input voltage */
  double decratio;       /* deceleration ratio */
  double inertia;        /* motor inertia */
  double inertia_gear;   /* gear inertia */

  /* friction model: TO BE ADDED */
  /*
   double fric_s;
   double fric_k;
   double fric_inc;
   double fric_eta;
  */
  /* for forward dynamics computation */
  double _comp_k;
  double _comp_l;
} rkMotorPrpDC;

__EXPORT rkMotor *rkMotorCreateDC(rkMotor *motor);

__END_DECLS

#endif /* __RK_MOTOR_DC_H__ */
