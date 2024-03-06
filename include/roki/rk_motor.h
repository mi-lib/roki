/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor - motor model
 * contributer: 2014-2015 Naoki Wakisaka
 */

#ifndef __RK_MOTOR_H__
#define __RK_MOTOR_H__

#include <roki/rk_misc.h>

__BEGIN_DECLS

/* ********************************************************** */
/*! \struct rkMotorSpec
 * \brief motor specification class
 * ********************************************************** */
struct _rkMotor;
typedef struct _rkMotor rkMotor;
struct _rkMotorSpec;
typedef struct _rkMotorSpec rkMotorSpec;

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkMotorCom ){
  const char *typestr; /*!< \brief a string for type identification */
  byte dof;            /*!< \brief degree-of-freedom of motor motion */
  /* methods for motor specification */
  void (* _init_prp)(rkMotorSpec*);
  void *(* _alloc_prp)(void);
  void (* _copy_prp)(rkMotorSpec*,rkMotorSpec*);
  rkMotorSpec *(* _fromZTK)(rkMotorSpec*,ZTK*);
  void (* _fprintZTK)(FILE*,rkMotorSpec*);
  /* methods for motor instance */
  void (* _init_state)(rkMotor*);
  void *(* _alloc_state)(void);
  void (* _copy_state)(rkMotor*,rkMotor*);
  void (* _set_input)(rkMotor*,double*); /*!< \brief set feasible motor input */
  void (* _inertia)(rkMotor*,double*); /*!< \brief rotor inertia of motor + gear */
  void (* _inputtrq)(rkMotor*,double*); /*!< \brief actuation torque from motor input signal */
  void (* _regist)(rkMotor*,double*,double*,double*); /*!< \brief registance torque (e.g. counter electromotive) */
  void (* _dtrq)(rkMotor*,double*,double*,double*,double*); /*!< \brief driving torque */
};

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkMotorSpec ){
  Z_NAMED_CLASS;
  void *prp;
  rkMotorCom *com;
};

#define rkMotorSpecTypeStr(ms) (ms)->com->typestr
#define rkMotorSpecDOF(ms)     (ms)->com->dof

#define rkMotorSpecInit(ms) do{\
  zNameSetPtr( ms, NULL );\
  (ms)->prp = NULL;\
  (ms)->com = NULL;\
} while(0)

/*! \brief assign a motor type to motor specification instance. */
__ROKI_EXPORT rkMotorSpec *rkMotorSpecAssign(rkMotorSpec *ms, rkMotorCom *com);
/*! \brief assign motor specification by a string. */
__ROKI_EXPORT rkMotorSpec *rkMotorSpecAssignByStr(rkMotorSpec *ms, const char *str);
/*! \brief destroy a motor specification instance. */
__ROKI_EXPORT void rkMotorSpecDestroy(rkMotorSpec *ms);

/*! \brief clone a motor specification instance. */
__ROKI_EXPORT rkMotorSpec *rkMotorSpecClone(rkMotorSpec *org, rkMotorSpec *cln);

/* ZTK */

__ROKI_EXPORT rkMotorSpec *rkMotorSpecFromZTK(rkMotorSpec *ms, ZTK *ztk);
__ROKI_EXPORT void rkMotorSpecFPrintZTK(FILE *fp, rkMotorSpec *ms);

/* ********************************************************** */
/*! \struct rkMotorSpecArray
 * \brief array of motor specifications
 * ********************************************************** */

#define ZTK_TAG_RKMOTOR "motor"
zArrayClass( rkMotorSpecArray, rkMotorSpec );

__ROKI_EXPORT rkMotorSpecArray *rkMotorSpecArrayAlloc(rkMotorSpecArray *msarray, int size);

__ROKI_EXPORT void rkMotorSpecArrayDestroy(rkMotorSpecArray *msarray);

__ROKI_EXPORT rkMotorSpecArray *rkMotorSpecArrayClone(rkMotorSpecArray *org, rkMotorSpecArray *cln);

__ROKI_EXPORT rkMotorSpec *rkMotorSpecArrayFind(rkMotorSpecArray *msarray, const char *name);

__ROKI_EXPORT void rkMotorSpecArrayFPrintZTK(FILE *fp, rkMotorSpecArray *msarray);

__END_DECLS

#include <roki/rk_motor_none.h> /* for unactuated joints */
#include <roki/rk_motor_trq.h>  /* direct torque motor */
#include <roki/rk_motor_dc.h>   /* DC motor */

__BEGIN_DECLS

/* add the handle to the following list when you create a new motor class. */
#define RK_MOTOR_COM_ARRAY \
rkMotorCom *_rk_motor_com[] = {\
  &rk_motor_none,\
  &rk_motor_dc,\
  &rk_motor_trq,\
  NULL,\
}

/* ********************************************************** */
/*! \struct rkMotor
 * \brief motor class
 * ********************************************************** */

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkMotor ){
  rkMotorSpec *spec;
  void *state;
};

#define RK_MOTOR_COM_DEF_ALLOC_COPY_FUNC( name ) \
  static void *_rkMotorSpec##name##AllocPrp(void){ return zAlloc( rkMotor##name##Prp, 1 ); } \
  static void _rkMotorSpec##name##CopyPrp(rkMotorSpec *src, rkMotorSpec *dst){ \
    memcpy( dst->prp, src->prp, sizeof(rkMotor##name##Prp) ); } \
  static void *_rkMotor##name##AllocState(void){ return zAlloc( rkMotor##name##State, 1 ); } \
  static void _rkMotor##name##CopyState(rkMotor *src, rkMotor *dst){ \
    memcpy( dst->state, src->state, sizeof(rkMotor##name##State) ); }

#define rkMotorName(motor) zName( (motor)->spec )
#define rkMotorDOF(motor)  rkMotorSpecDOF( (motor)->spec )

#define rkMotorInit(m) do{\
  rkMotorSpecInit( (m)->spec );\
  (m)->state = NULL;\
} while(0)

__ROKI_EXPORT rkMotor *rkMotorCreate(rkMotor *motor, rkMotorSpec *ms);

__ROKI_EXPORT void rkMotorDestroy(rkMotor *motor);

__ROKI_EXPORT rkMotor *rkMotorClone(rkMotor *org, rkMotor *cln, rkMotorSpecArray *msarray_org, rkMotorSpecArray *msarray_cln);

#define rkMotorSetInput(m,i)           (m)->spec->com->_set_input( (m), (i) )
#define rkMotorInertia(m,i)            (m)->spec->com->_inertia( (m), (i) )
#define rkMotorInputTrq(m,i)           (m)->spec->com->_inputtrq( (m), (i) )
#define rkMotorRegistance(m,d,v,r)     (m)->spec->com->_regist( (m), (d), (v), (r) )
#define rkMotorDrivingTrq(m,d,v,a,t)   (m)->spec->com->_dtrq( (m), (d), (v), (a), (t) )

__END_DECLS

#endif /* __RK_MOTOR_H__ */
