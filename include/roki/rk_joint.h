/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint - joint structure
 */

#ifndef __RK_JOINT_H__
#define __RK_JOINT_H__

#include <roki/rk_misc.h>

#include <roki/rk_motor.h>
#include <roki/rk_body.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkJoint
 * joint class
 * ********************************************************** */
ZDECL_STRUCT( rkJointCom );

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJoint ){
  void *prp;       /*!< joint properties */
  void *state;     /*!< joint state */
  rkMotor *motor;  /*!< motor */
  zVec6D wrench;   /*!< joint wrench */
  rkJointCom *com; /*!< common methods for joint class */
};

/* for forward dynamics */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointFrictionPivot ){
  byte type;
  double ref_dis;
  double prev_trq;
};

/*! \brief common methods for joint class. */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointCom ){
  const char *typestr; /*!< a string to identify the type of joint */
  byte dof;            /*!< degree-of-freedom of joint movement */
  void (*_init)(rkJoint*);
  void *(*_alloc_prp)(void);
  void *(*_alloc_state)(void);
  void (*_copy_prp)(rkJoint*,rkJoint*);
  void (*_copy_state)(rkJoint*,rkJoint*);

  /* joint value manipulation function */
  void (*_lim_dis)(rkJoint*,double*,double*); /* limit displacements */
  void (*_set_dis)(rkJoint*,double*);     /* set displacements */
  void (*_set_min)(rkJoint*,double*);     /* set minimum displacements */
  void (*_set_max)(rkJoint*,double*);     /* set maximum displacements */
  void (*_set_vel)(rkJoint*,double*);     /* set velocity */
  void (*_set_acc)(rkJoint*,double*);     /* set acceleration */
  void (*_set_trq)(rkJoint*,double*);     /* set torque */
  void (*_get_dis)(rkJoint*,double*);     /* get displacements */
  void (*_get_min)(rkJoint*,double*);     /* get minimum displacements */
  void (*_get_max)(rkJoint*,double*);     /* get maximum displacements */
  void (*_get_vel)(rkJoint*,double*);     /* get velocity */
  void (*_get_acc)(rkJoint*,double*);     /* get acceleration */
  void (*_get_trq)(rkJoint*,double*);     /* get torques */
  void (*_cat_dis)(rkJoint*,double*,double,double*); /* concatenate displacements */
  void (*_sub_dis)(rkJoint*,double*,double*); /* subtract displacements */
  void (*_cnt_dis)(rkJoint*,double*,double); /* continuous update */

  zFrame3D *(*_xform)(rkJoint*,zFrame3D*,zFrame3D*); /* frame transformation */
  void (*_inc_vel)(rkJoint*,zVec6D*); /* motion rate transformation */
  void (*_inc_acc_vel)(rkJoint*,zVec3D*,zVec6D*);
  void (*_inc_acc)(rkJoint*,zVec6D*);
  void (*_trq)(rkJoint*,zVec6D*);        /* joint torque transformation */
  void (*_torsion)(zFrame3D*,zVec6D*,double*); /* inverse computation of torsion and displacement */

  /* axis vector */
  zVec3D* (**_angaxis)(rkJoint*,zFrame3D*,zVec3D*); /* angular */
  zVec3D* (**_linaxis)(rkJoint*,zFrame3D*,zVec3D*); /* linear */

  /* composite rigid body method */
  void (*_crb_wrench)(rkJoint*,rkMP*,zVec6D[]);
  void (*_crb_xform)(rkJoint*,zFrame3D*,zVec6D[]);

  /* for forward dynamics */
  void (*_set_frictionpivot)(rkJoint*,rkJointFrictionPivot*); /* set referential displacement of friction */
  void (*_get_frictionpivot)(rkJoint*,rkJointFrictionPivot*); /* get referential displacement of friction */
  void (*_set_friction)(rkJoint*,double*);  /* set joint friction force/torque */
  void (*_get_friction)(rkJoint*,double*);  /* get joint friction force/torque */
  void (*_get_sfriction)(rkJoint*,double*); /* set static friction force/torque */
  void (*_get_kfriction)(rkJoint*,double*); /* get kinetic friction force/torque */

  void (*_set_motor_input)(rkJoint*,double*);
  void (*_motor_inertia)(rkJoint*,double*);
  void (*_motor_inputtrq)(rkJoint*,double*);
  void (*_motor_regist)(rkJoint*,double*);
  void (*_motor_destrq)(rkJoint*,double*);

  void (*_axisinertia)(rkJoint*,zMat6D*,zMat,zMat);
  void (*_add_abi)(rkJoint*,zMat6D*,zFrame3D*,zMat,zMat6D*);
  void (*_add_bias)(rkJoint*,zMat6D*,zVec6D*,zFrame3D*,zMat,zVec6D*);
  void (*_dtrq)(rkJoint*);
  void (*_qacc)(rkJoint*,zMat6D*,zVec6D*,zVec6D*,zMat,zVec6D*);
  void (*_wrench)(rkJoint*,zMat6D*,zVec6D*,zVec6D*);

  /* I/O */
  void *(*_dis_fromZTK)(void*,int,void*,ZTK*);
  rkJoint *(*_fromZTK)(rkJoint*,rkMotorSpecArray*,ZTK*);
  bool (*_dis_fprintZTK)(FILE*,int,void*);
  void (*_fprintZTK)(FILE*,rkJoint*,char*);  /* print */
};

#define RK_JOINT_COM_DEF_PRP_FUNC( name ) \
  static void *_rkJoint##name##AllocPrp(void){ return zAlloc( rkJoint##name##Prp, 1 ); } \
  static void _rkJoint##name##CopyPrp(rkJoint *src, rkJoint *dst){ \
    memcpy( dst->prp, src->prp, sizeof(rkJoint##name##Prp) ); }

#define RK_JOINT_COM_DEF_STATE_FUNC( name ) \
  static void *_rkJoint##name##AllocState(void){ return zAlloc( rkJoint##name##State, 1 ); } \
  static void _rkJoint##name##CopyState(rkJoint *src, rkJoint *dst){ \
    memcpy( dst->state, src->state, sizeof(rkJoint##name##State) ); }

#define rkJointDOF(joint)      (joint)->com->dof
#define rkJointTypeStr(joint)  (joint)->com->typestr
#define rkJointMotor(joint)    (joint)->motor
#define rkJointWrench(joint)   ( &(joint)->wrench )

#define rkJointSetMotor( joint, _motor ) ( rkJointMotor(joint) = _motor )

/*! \brief initialize a joint object.
 *
 * rkJointInit() initializes a joint object \a joint, cleaning
 * up all properties.
 * \return
 * rkJointInit() returns a pointer \a joint if it succeeds.
 * If \a type is invalid, or it fails to allocate the internal
 * working memory, the null pointer is returned.
 */
#define rkJointInit(joint) do{\
  (joint)->prp   = NULL;\
  (joint)->state = NULL;\
  (joint)->motor = NULL;\
  (joint)->com   = NULL;\
} while(0)

__ROKI_EXPORT rkJoint *rkJointAssign(rkJoint *joint, rkJointCom *com);
__ROKI_EXPORT rkJoint *rkJointAssignByStr(rkJoint *joint, const char *str);
__ROKI_EXPORT void rkJointDestroy(rkJoint *joint);

__ROKI_EXPORT rkJoint *rkJointClone(rkJoint *org, rkJoint *cln, rkMotorSpecArray *msarray_org, rkMotorSpecArray *msarray_cln);
__ROKI_EXPORT rkJoint *rkJointCopyPrp(rkJoint *src, rkJoint *dst);
__ROKI_EXPORT rkJoint *rkJointCopyState(rkJoint *src, rkJoint *dst);

/*! \brief set and get joint status.
 *
 * rkJointSetDis(), rkJointSetMin(), rJointSetMax(), rkJointSetVel(),
 * rkJointSetAcc() and rkJointSetTrq() set joint displacements, minimum
 * joint displacements, maximum joint displacements, velocities,
 * accelerations and torques \a val to a joint \a joint.
 *
 * rkJointGetDis(), rkJointGetMin(), rkJointGetMax(), rkJointGetVel(),
 * rkJointGetAcc() and rkJointGetTrq() get joint displacements, minimum
 * joint displacements, maximum joint displacements, velocities,
 * accelerations and torques of \a joint to \a val.
 *
 * rkJointSetDisCNT() continuously updates the joint
 * displacement of a joint \a joint to \a val over a time step
 * \a dt, calculating joint velocity and acceleration in
 * accordance with a simple differentiation.
 *
 * The size of the array \a val depends on the type of \a joint, basically
 * coinciding with its degree of freedom.
 * \notes
 * They do not check the size consistency. Mismatched array size might
 * cause anything.
 * \return
 * None of those functions return any value.
 */
#define rkJointLimDis(joint,t,v)        (joint)->com->_lim_dis( (joint), t, v )
#define rkJointSetDis(joint,v)          (joint)->com->_set_dis( (joint), v )
#define rkJointSetMin(joint,v)          (joint)->com->_set_min( (joint), v )
#define rkJointSetMax(joint,v)          (joint)->com->_set_max( (joint), v )
#define rkJointSetVel(joint,v)          (joint)->com->_set_vel( (joint), v )
#define rkJointSetAcc(joint,v)          (joint)->com->_set_acc( (joint), v )
#define rkJointSetTrq(joint,v)          (joint)->com->_set_trq( (joint), v )
#define rkJointGetDis(joint,v)          (joint)->com->_get_dis( (joint), v )
#define rkJointGetMin(joint,v)          (joint)->com->_get_min( (joint), v )
#define rkJointGetMax(joint,v)          (joint)->com->_get_max( (joint), v )
#define rkJointGetVel(joint,v)          (joint)->com->_get_vel( (joint), v )
#define rkJointGetAcc(joint,v)          (joint)->com->_get_acc( (joint), v )
#define rkJointGetTrq(joint,v)          (joint)->com->_get_trq( (joint), v )

#define rkJointCatDis(joint,d,k,v)      (joint)->com->_cat_dis( (joint), d, k, v )
#define rkJointSubDis(joint,d,sd)       (joint)->com->_sub_dis( (joint), d, sd )
#define rkJointSetDisCNT(joint,v,t)     (joint)->com->_cnt_dis( (joint), v, t )

#define rkJointSetFrictionPivot(joint,r) (joint)->com->_set_frictionpivot( (joint), r )
#define rkJointGetFrictionPivot(joint,r) (joint)->com->_get_frictionpivot( (joint), r )
#define rkJointSetFriction(joint,f)      (joint)->com->_set_friction( (joint), f )
#define rkJointGetFriction(joint,f)      (joint)->com->_get_friction( (joint), f )
#define rkJointGetSFriction(joint,f)     (joint)->com->_get_sfriction( (joint), f )
#define rkJointGetKFriction(joint,f)     (joint)->com->_get_kfriction( (joint), f )

/* motor */
#define rkJointMotorSetInput(joint,i)   (joint)->com->_set_motor_input( (joint), i )
#define rkJointMotorInertia(joint,i)    (joint)->com->_motor_inertia( (joint), i )
#define rkJointMotorInputTrq(joint,t)   (joint)->com->_motor_inputtrq( (joint), t )
#define rkJointMotorRegistance(joint,r) (joint)->com->_motor_regist( (joint), r )
#define rkJointMotorDrivingTrq(joint,t) (joint)->com->_motor_destrq( (joint), t )

/*! \brief neutral configuration of joint.
 *
 * rkJointNeutralize() sets displacement, velocity and acceleration of a joint
 * \a joint for zero.
 *
 * rkJointIsNeutral() checks if the joint \a joint is in neutral configuration
 * where all the components are zero.
 * \return
 * rkJointNeutralize() returns no value.
 *
 * rkJointIsNeutral() returns the true value if all (i.e. the number of
 * degrees-of-freedom) the components of joint displacement of \a joint are zero.
 * Otherwise, the false value is returned.
 */
void rkJointNeutralize(rkJoint *joint);
bool rkJointIsNeutral(rkJoint *joint);

/*! \brief transform a joint frame, velocity, acceleration and torque.
 *
 * rkJointXform() transforms a 3D frame \a fo in accordance with
 * the displacement of a joint \a joint. The result is put into \a f.
 *
 * Transformation rule depends on the joint type.
 * Fixed joint just copies \a fo to \a f.
 * Revolutional joint revolves \a fo about the z-axis in radian.
 * Prismatic joint translates \a fo along the z-axis.
 * Cylindric joint revolves \a fo about the z-axis with the first
 * displacement in radian, and then translates it along the z-axis
 * with the second displacement.
 * Universal (hooke) joint revolves \a fo with the displacements
 * about z-axis and y-axis in radian.
 * Spherical joint revolves \a fo with the displacements as z-y-x
 * Eulerian angles.
 * Free-floating joint translates \a fo three-dimensionaly with
 * the first three displacements, and revolves it with the rest
 * three as z-y-x Eulerian angles.
 *
 * rkJointIncRate() increments the given spatial velocity \a vel
 * and acceleration vector \a acc according to the local joint
 * movement. It is a particular function called in
 * rkLinkUpdateTtlRate() and rkLinkUpdateWldRate() internally.
 * Although it is exported, it is not a good idea to call this
 * function in user programs.
 *
 * rkJointCalcTrq() calculates joint torque from the six-axis force
 * \a f exerted at joint and the restitution force originating from
 * joint impedance.
 * \return
 * rkJointXform() returns a pointer \a f.
 * \sa
 * rkJointCreate
 */
#define rkJointXform(joint,fo,f)      (joint)->com->_xform( (joint), fo, f )
#define rkJointIncVel(joint,v)        (joint)->com->_inc_vel( (joint), v)
#define rkJointIncAccOnVel(joint,w,a) (joint)->com->_inc_acc_vel( (joint), w, a )
#define rkJointIncAcc(joint,a)        (joint)->com->_inc_acc( (joint), a )
__ROKI_EXPORT void rkJointIncRate(rkJoint *joint, zVec3D *w, zVec6D *vel, zVec6D *acc);
#define rkJointCalcTrq(joint,f)       (joint)->com->_trq( (joint), f )

#define rkJointTorsion(joint,dev,t,d) (joint)->com->_torsion( dev, t, d )

__ROKI_EXPORT double rkJointRevolTorsionDis(zFrame3D *dev, zVec6D *t);
__ROKI_EXPORT double rkJointPrismTorsionDis(zFrame3D *dev, zVec6D *t);

/*! \brief joint axis vector.
 *
 * rkJointAngAxis() and rkJointLinAxis() calculates axis vector of
 * the \a i'th component of a joint \a joint about angular and linear
 * motion, respectively.
 * \a f is a frame attached to the link with \a joint.
 * The result is put into \a a.
 * \return
 * rkJointAngJacobiCol() and rkJointLinJacobiCol() return a pointer
 * \a a if the axis vector is non-zero. Otherwise, they return the
 * null pointer.
 * \notes
 * Neither rkJointAngJacobiCol() nor rkJointLinJacobiCol() check if
 * \a i is valid. \a i which is larger than the the degree of
 * freedom of \a joint might cause anything.
 */
#define rkJointAngAxis(joint,i,f,a) (joint)->com->_angaxis[i]( (joint), f, a )
#define rkJointLinAxis(joint,i,f,a) (joint)->com->_linaxis[i]( (joint), f, a )

__ROKI_EXPORT zVec3D *_rkJointAxisNull(rkJoint *joint, zFrame3D *f, zVec3D *a);
__ROKI_EXPORT zVec3D *_rkJointAxisZ(rkJoint *joint, zFrame3D *f, zVec3D *a);

/* composite rigid body method */
#define rkJointCRBWrench(joint,m,w) (joint)->com->_crb_wrench( (joint), m, w )
#define rkJointCRBXform(joint,f,s)  (joint)->com->_crb_xform( (joint), f, s )

/* NOTE: The following macros and functions are for sharing some operation codes.
 * Do not use them in users programs. */
#define _rkJointRestTrq(stiffness,viscosity,coulomb,dis,vel) ( -(stiffness)*(dis) -(viscosity)*(vel) -(coulomb)*zSgn(vel) )

/* Composite Rigid Body */
/* The following macros are supposed to be used only in internal methods for
   the composite rigid body method computations. Never use them in user programs.
 */
#define _rkJointCRBWrenchLinZ(crb,w) _zVec6DCreate( w, \
  0, 0, rkMPMass(crb), \
  rkMPMass(crb)*rkMPCOM(crb)->c.y, \
 -rkMPMass(crb)*rkMPCOM(crb)->c.x, \
  0 )
#define _rkJointCRBWrenchAngX(crb,w) _zVec6DCreate( w, \
  0, \
 -rkMPMass(crb)*rkMPCOM(crb)->c.z, \
  rkMPMass(crb)*rkMPCOM(crb)->c.y, \
  rkMPInertia(crb)->c.xx+rkMPMass(crb)*(zSqr(rkMPCOM(crb)->c.y)+zSqr(rkMPCOM(crb)->c.z)), \
  rkMPInertia(crb)->c.xy-rkMPMass(crb)*rkMPCOM(crb)->c.x*rkMPCOM(crb)->c.y, \
  rkMPInertia(crb)->c.xz-rkMPMass(crb)*rkMPCOM(crb)->c.z*rkMPCOM(crb)->c.x )
#define _rkJointCRBWrenchAngY(crb,w) _zVec6DCreate( w, \
  rkMPMass(crb)*rkMPCOM(crb)->c.z, \
  0, \
 -rkMPMass(crb)*rkMPCOM(crb)->c.x, \
  rkMPInertia(crb)->c.yx-rkMPMass(crb)*rkMPCOM(crb)->c.x*rkMPCOM(crb)->c.y, \
  rkMPInertia(crb)->c.yy+rkMPMass(crb)*(zSqr(rkMPCOM(crb)->c.z)+zSqr(rkMPCOM(crb)->c.x)), \
  rkMPInertia(crb)->c.yz-rkMPMass(crb)*rkMPCOM(crb)->c.y*rkMPCOM(crb)->c.z )
#define _rkJointCRBWrenchAngZ(crb,w) _zVec6DCreate( w, \
 -rkMPMass(crb)*rkMPCOM(crb)->c.y, \
  rkMPMass(crb)*rkMPCOM(crb)->c.x, \
  0, \
  rkMPInertia(crb)->c.zx-rkMPMass(crb)*rkMPCOM(crb)->c.z*rkMPCOM(crb)->c.x, \
  rkMPInertia(crb)->c.zy-rkMPMass(crb)*rkMPCOM(crb)->c.y*rkMPCOM(crb)->c.z, \
  rkMPInertia(crb)->c.zz+rkMPMass(crb)*(zSqr(rkMPCOM(crb)->c.x)+zSqr(rkMPCOM(crb)->c.y)) )

#define _rkJointCRBXformLin(f,a,s) do{ \
  zVec3DCopy( &zFrame3DAtt(f)->v[a], zVec6DLin(s) ); \
  _zVec3DZero( zVec6DAng(s) ); \
} while(0)
#define _rkJointCRBXformAng(f,a,s) do{ \
  _zVec3DOuterProd( zFrame3DPos(f), &zFrame3DAtt(f)->v[a], zVec6DLin(s) ); \
  zVec3DCopy( &zFrame3DAtt(f)->v[a], zVec6DAng(s) ); \
} while(0)

/* Articulated Body Inertia */

#define rkJointABIAxisInertia(joint,m,h,ih) (joint)->com->_axisinertia( (joint), m, h, ih )
#define rkJointABIAddABI(joint,i,f,h,pi)    (joint)->com->_add_abi( (joint), i, f, h, pi )
#define rkJointABIAddBias(joint,i,b,f,h,pb) (joint)->com->_add_bias( (joint), i, b, f, h, pb )
#define rkJointABIDrivingTorque(joint)      (joint)->com->_dtrq( (joint) )
#define rkJointABIQAcc(joint,i,b,c,h,a)     (joint)->com->_qacc( (joint), i, b, c, h, a )
#define rkJointUpdateWrench(joint,i,b,a)    (joint)->com->_wrench( (joint), i, b, a )

__ROKI_EXPORT zMat6D *rkJointXformMat6D(zFrame3D *frame, zMat6D *i, zMat6D *m);
__ROKI_EXPORT void _rkJointUpdateWrench(rkJoint *joint, zMat6D *i, zVec6D *b, zVec6D *acc);

/* dummy functions for motorless joints */
__ROKI_EXPORT void rkJointMotorSetValDummy(rkJoint *joint, double *val);
__ROKI_EXPORT void rkJointMotorGetValDummy(rkJoint *joint, double *val);

__ROKI_EXPORT rkJoint *rkJointAssignMotorByStr(rkJoint *joint, rkMotorSpecArray *msarray, const char *str);

/* ZTK */

#define ZTK_KEY_ROKI_JOINT_DIS            "dis"
#define ZTK_KEY_ROKI_JOINT_MIN            "min"
#define ZTK_KEY_ROKI_JOINT_MAX            "max"
#define ZTK_KEY_ROKI_JOINT_STIFFNESS      "stiffness"
#define ZTK_KEY_ROKI_JOINT_VISCOSITY      "viscosity"
#define ZTK_KEY_ROKI_JOINT_COULOMB        "coulomb"
#define ZTK_KEY_ROKI_JOINT_STATICFRICTION "staticfriction"
#define ZTK_KEY_ROKI_JOINT_MOTOR          "motor"
#define ZTK_KEY_ROKI_JOINT_TH_FORCE       "forcethreshold"
#define ZTK_KEY_ROKI_JOINT_TH_TORQUE      "torquethreshold"

/*! \brief scan and print out joint displacement and properties.
 *
 * The candidates of \a key are listed as follows:
 *  "jointtype" for joint type. Refer rkJointTypeByStr().
 *  "dis" for the initial joint displacement.
 *  "min" for the minimum joint displacement.
 *  "max" for the maximum joint displacement.
 *  "stiffness" for the joint stiffness.
 *  "viscosity" for the joint viscosity.
 *  "coulomb" for Coulomb's joint friction.
 *  "staticfriction" for static joint friction.
 * The number of components of each property is according to the
 * joint type, and hence, the joint type has to be identified
 * first of all.
 *
 * rkJointFPrintZTK() prints the joint properties out to a file
 * \a fp in accordance with the above rule. The field "dis" is
 * output only when any components of the joint displacement
 * are non-zero values.
 * \a name is used instead of the key "dis", unless it is the
 * null pointer.
 * \return
 * rkJointFPrintZTK() returns no value.
 */

#define rkJointPrpFromZTK(joint,motorspecarray,ztk,ztkprp) \
  ( ZTKEvalKey( joint, motorspecarray, ztk, ztkprp ) ? joint : NULL )

__ROKI_EXPORT rkJoint *rkJointFromZTK(rkJoint *joint, rkMotorSpecArray *motorspecarray, ZTK *ztk);

#define rkJointDisFPrintZTK(fp,joint)   (joint)->com->_dis_fprintZTK( fp, 0, joint )
#define rkJointFPrintZTK(fp,joint,name) ( (name) ? (joint)->com->_fprintZTK( fp, joint, name ) : (joint)->com->_fprintZTK( fp, joint, (char *)"dis" ) )

__END_DECLS

#include <roki/rk_joint_fixed.h>   /* fixed joint */
#include <roki/rk_joint_revol.h>   /* revolutional joint */
#include <roki/rk_joint_prism.h>   /* prismatic joint */
#include <roki/rk_joint_cylin.h>   /* cylindrical joint */
#include <roki/rk_joint_hooke.h>   /* universal joint */
#include <roki/rk_joint_spher.h>   /* spherical joint */
#include <roki/rk_joint_float.h>   /* free-floating joint */
#include <roki/rk_joint_brfloat.h> /* breakable free-floating joint */

__BEGIN_DECLS

__ROKI_EXPORT rkJointCom *rk_joint_com[];

/* add the handle to the following list when you create a new joint class. */
#define RK_JOINT_COM_ARRAY \
rkJointCom *rk_joint_com[] = {\
  &rk_joint_fixed,\
  &rk_joint_revol,\
  &rk_joint_prism,\
  &rk_joint_cylin,\
  &rk_joint_hooke,\
  &rk_joint_spher,\
  &rk_joint_float,\
  &rk_joint_brfloat,\
  NULL,\
}

__END_DECLS

#endif /* __RK_JOINT_H__ */
