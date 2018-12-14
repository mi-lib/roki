/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor - motor model
 * contributer: 2014-2015 Naoki Wakisaka
 */

#ifndef __RK_MOTOR_H__
#define __RK_MOTOR_H__

#include <zm/zm.h>

__BEGIN_DECLS

/*! \brief motor type */
enum{
  RK_MOTOR_INVALID=-1,
  RK_MOTOR_NONE=0,     /* not actuated */
  RK_MOTOR_TRQ,        /* torque motor */
  RK_MOTOR_DC,         /* DC motor */
};

__EXPORT char *rkMotorTypeExpr(byte type);
__EXPORT byte rkMotorTypeFromStr(char *str);

/* ********************************************************** */
/* CLASS: rkMotor
 * geared DC motor model class
 * ********************************************************** */

typedef struct{
  byte size;
  void (*_setinput)(void*,double*);

  void (*_inertia)(void*,double*);
  void (*_inputtrq)(void*,double*);
  void (*_regist)(void*,double*,double*,double*);
  void (*_dtrq)(void*,double*,double*,double*,double*);

  void (*_statecopy)(void*,void*);

  bool (*_query)(FILE*,char*,void*);
  void (*_write)(FILE*,void*);
} rkMotorCom;

typedef struct{
  Z_NAMED_CLASS
  byte type;
  void *prp;
  rkMotorCom *com;
} rkMotor;

#define rkMotorType(m) (m)->type
#define rkMotorSize(m) (m)->com->size

#define rkMotorInit(m) do{\
  (m)->type = RK_MOTOR_INVALID;\
  (m)->prp = NULL;\
  (m)->com = NULL;\
} while(0)
__EXPORT rkMotor *rkMotorCreate(rkMotor *m, byte type);
__EXPORT void rkMotorDestroy(rkMotor *m);

__EXPORT rkMotor *rkMotorClone(rkMotor *org, rkMotor *cln);

/* __EXPORT rkMotor *rkMotorCopyState(rkMotor *src, rkMotor *dst); */

/* ********************************************************** */
/* method
 * ********************************************************** */
#define rkMotorSetInput(m,i)           (m)->com->_setinput( (m)->prp, (i) )

#define rkMotorInertia(m,i)            (m)->com->_inertia( (m)->prp, (i) )
#define rkMotorInputTrq(m,i)           (m)->com->_inputtrq( (m)->prp, (i) )
#define rkMotorRegistance(m,d,v,r)     (m)->com->_regist( (m)->prp, (d), (v), (r) )
#define rkMotorDrivingTrq(m,d,v,a,t)   (m)->com->_dtrq( (m)->prp, (d), (v), (a), (t) )

#define rkMotorStateCopy(src,dst)      ( (src)->type == (dst)->type ? (src)->com->_statecopy( (src)->prp, (dst)->prp ) : false )

#define rkMotorQueryFRead(f,k,m)       (m)->com->_query( f, k, (m)->prp )
__EXPORT void rkMotorFWrite(FILE *fp, rkMotor *m);
#define rkMotorWrite(m)                rkMotorFWrite( stdout, m )

/* ********************************************************** */
/* CLASS: rkMotorArray
 * ********************************************************** */

#define RK_MOTOR_TAG "motor"
zArrayClass( rkMotorArray, rkMotor );

__EXPORT rkMotorArray *rkMotorArrayClone(rkMotorArray *org);
__EXPORT rkMotorArray *rkMotorArrayFRead(FILE *fp, rkMotorArray *m);
__EXPORT void rkMotorArrayFWrite(FILE *fp, rkMotorArray *m);

__END_DECLS

#include <roki/rk_motor_none.h> /* for unactuated joints */
#include <roki/rk_motor_trq.h>  /* direct torque motor */
#include <roki/rk_motor_dc.h>   /* DC motor */

#endif /* __RK_MOTOR_H__ */
