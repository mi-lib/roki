/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_joint - joint structure
 */

#ifndef __RK_JOINT_H__
#define __RK_JOINT_H__

#include <zm/zm.h>
#include <zeo/zeo_frame.h>
#include <zeo/zeo_ep.h>
#include <zeo/zeo_mat6d.h>

#include <roki/rk_motor.h>

__BEGIN_DECLS

/* ********************************************************** */
/* joint type
 * ********************************************************** */
enum{
  RK_JOINT_INVALID=-1,
  RK_JOINT_FIXED=0, /* DOF = 0, fixed */
  RK_JOINT_REVOL,   /* DOF = 1, revolutional */
  RK_JOINT_PRISM,   /* DOF = 1, prismatic */
  RK_JOINT_CYLIN,   /* DOF = 2, cylindric */
  RK_JOINT_HOOKE,   /* DOF = 2, universal */
  RK_JOINT_SPHER,   /* DOF = 3, spherical */
  RK_JOINT_FLOAT,   /* DOF = 6, free-floating */
  RK_JOINT_BRFLOAT, /* DOF = 6, breakable free-floating */
};

/*! \brief convert from/to joint type and to/from string.
 *
 * rkJointTypeExpr() converts the joint type \a type to the
 * corresnponding string expression, while rkJointTypeByStr()
 * converts a string \a str to the corresponding joint type
 * identifier.
 *
 * The correspondence is as follows:
 *   joint type          type identifier    string expression
 *   fixed joint         RK_JOINT_FIXED <-> "fix"
 *   revolutional joint  RK_JOINT_REVOL <-> "revolute"
 *   prismatic joint     RK_JOINT_PRISM <-> "prism"
 *   cylindric joint     RK_JOINT_CYLIN <-> "cylinder"
 *   universal joint     RK_JOINT_HOOKE <-> "hooke"
 *   spherical joint     RK_JOINT_SPHER <-> "sphere"
 *   free-floating joint RK_JOINT_FLOAT <-> "free"
 * \return
 * rkJointTypeExpr() returns a pointer to the head of
 * corresponding string.
 * rkJointTypeByStr() returns the corresponding joint type.
 */
__EXPORT char *rkJointTypeExpr(byte type);
__EXPORT byte rkJointTypeFromStr(char *str);

/* ********************************************************** */
/* CLASS: rkJoint
 * joint class
 * ********************************************************** */
struct _rkJoint;

/* for forward dynamics (used in rk_fd) */
typedef struct{
  byte type;
  double ref_dis, ref_trq;
} rkJointRef;

typedef struct{
  byte size; /* number of joint components */
  /* joint value manipulation function */
  void (*_limdis)(void*,double*,double*); /* limit displacements */
  void (*_setdis)(void*,double*);     /* set displacements */
  void (*_setvel)(void*,double*);     /* set velocity */
  void (*_setacc)(void*,double*);     /* set acceleration */
  void (*_settrq)(void*,double*);     /* set torque */
  void (*_getdis)(void*,double*);     /* get displacements */
  void (*_getvel)(void*,double*);     /* get velocity */
  void (*_getacc)(void*,double*);     /* get acceleration */
  void (*_gettrq)(void*,double*);     /* get torques */
  void (*_getmotor)(void*,rkMotor**); /* get motor */
  void (*_catdis)(void*,double*,double,double*); /* concatenate displacements */
  void (*_subdis)(void*,double*,double*); /* subtract displacements */
  void (*_cntdis)(void*,double*,double); /* continuous update */

  zFrame3D *(*_xfer)(void*,zFrame3D*,zFrame3D*); /* frame transfer */
  void (*_incvel)(void*,zVec6D*); /* motion rate transfer */
  void (*_incaccvel)(void*,zVec3D*,zVec6D*);
  void (*_incacc)(void*,zVec6D*);
  void (*_trq)(void*,zVec6D*);        /* joint torque transfer */
  void (*_torsion)(zFrame3D*,zVec6D*,double*); /* inverse computation of torsion and displacement */

  void (*_setfric)(void*,double*);  /* set friction torque */
  void (*_getfric)(void*,double*);  /* get friction torque */
  void (*_getsfric)(void*,double*); /* set static friction torque */
  void (*_getkfric)(void*,double*); /* get static friction torque */

  /* for rkFD */
  void (*_setref)(void*,rkJointRef*); /* set referential displacement of friction */
  void (*_getref)(void*,rkJointRef*); /* get referential displacement of friction */

  /* axis vector */
  zVec3D* (**_angaxis)(void*,zFrame3D*,zVec3D*); /* angular */
  zVec3D* (**_linaxis)(void*,zFrame3D*,zVec3D*); /* linear */

  bool (*_query)(FILE*,char*,void*,rkMotor*,int);  /* query */
  void (*_write)(FILE*,void*,char*);  /* write */
} rkJointCom;

typedef struct{
  byte (*_type)(void*);
  void (*_setinput)(void*,double*);
  void (*_inertia)(void*,double*);
  void (*_inputtrq)(void*,double*);
  void (*_regist)(void*,double*);
  void (*_dtrq)(void*,double*);
} rkJointMotorCom;

typedef struct{
  void (*_axinertia)(void*,zMat6D*,zMat,zMat);
  void (*_addabi)(void*,zMat6D*,zFrame3D*,zMat,zMat6D*);
  void (*_addbias)(void*,zMat6D*,zVec6D*,zFrame3D*,zMat,zVec6D*);
  void (*_dtrq)(void*);
  void (*_qacc)(void*,zMat3D*,zMat6D*,zVec6D*,zVec6D*,zMat,zVec6D*);
  void (*_wrench)(struct _rkJoint*,zMat6D*,zVec6D*,zVec6D*);
} rkJointABICom;

typedef struct _rkJoint{
  byte type; /* joint type */
  void *prp;
  zVec6D wrench; /* joint wrench */
  rkJointCom *com;
  rkJointMotorCom *mcom;
  rkJointABICom *acom;
} rkJoint;

#define rkJointType(j)     (j)->type
#define rkJointSize(j)     (j)->com->size
#define rkJointWrench(j)   &(j)->wrench

/*! \brief initialize, create and destroy a joint object.
 *
 * rkJointInit() initializes a joint object \a j, cleaning
 * up all properties.
 *
 * rkJointCreate() creates \a j.
 * \a type is a joint type to be chosen from the followings.
 *  RK_JOINT_FIXED for fixed joint
 *  RK_JOINT_REVOL for revolutional joint
 *  RK_JOINT_PRISM for prismatic joint
 *  RK_JOINT_CYLIN for cylindric joint
 *  RK_JOINT_HOOKE for universal joint
 *  RK_JOINT_SPHER for spherical joint
 *  RK_JOINT_FLOAT for free-floating joint
 * The internal storage for joint properties is allocated
 * according to the above joint types.
 * \return
 * rkJointInit() returns a pointer \a j if it succeeds.
 * If \a type is invalid, or it fails to allocate the internal
 * working memory, the null pointer is returned.
 */
#define rkJointInit(j) do{\
  (j)->type = RK_JOINT_INVALID;\
  (j)->prp = NULL;\
  (j)->com = NULL;\
  (j)->mcom = NULL;\
  (j)->acom = NULL;\
} while(0)
__EXPORT rkJoint *rkJointCreate(rkJoint *j, byte type);
__EXPORT void rkJointDestroy(rkJoint *j);

__EXPORT rkJoint *rkJointClone(rkJoint *org, rkJoint *cln);
__EXPORT rkJoint *rkJointCopyState(rkJoint *src, rkJoint *dst);

/*! \brief set and get joint status.
 *
 * rkJointSetDis(), rkJointSetVel(), rkJointSetAcc() and
 * rkJointSetTrq() set joint displacements, velocities,
 * accelerations and torques \a val to a joint \a j.
 *
 * rkJointGetDis(), rkJointGetVel(), rkJointGetAcc() and
 * rkJointGetTrq() get joint displacements, velocities,
 * accelerations and torques of \a j to \a val.
 *
 * rkJointSetDisCNT() continuously updates the joint
 * displacement of a joint \a j to \a val over a time step
 * \a dt, calculating joint velocity and acceleration in
 * accordance with a simple differentiation.
 *
 * The size of the array \a val depends on the type of \a j,
 * basically coinciding with its degree of freedom.
 * \notes
 * They do not check the size consistency. Unmatching array
 * size might cause anything.
 * \return
 * None of those functions return any value.
 */
#define rkJointLimDis(j,t,v)    (j)->com->_limdis( (j)->prp, t, v )

#define rkJointSetDis(j,v)      (j)->com->_setdis( (j)->prp, v )
#define rkJointSetVel(j,v)      (j)->com->_setvel( (j)->prp, v )
#define rkJointSetAcc(j,v)      (j)->com->_setacc( (j)->prp, v )
#define rkJointSetTrq(j,v)      (j)->com->_settrq( (j)->prp, v )
#define rkJointGetDis(j,v)      (j)->com->_getdis( (j)->prp, v )
#define rkJointGetVel(j,v)      (j)->com->_getvel( (j)->prp, v )
#define rkJointGetAcc(j,v)      (j)->com->_getacc( (j)->prp, v )
#define rkJointGetTrq(j,v)      (j)->com->_gettrq( (j)->prp, v )

#define rkJointCatDis(j,d,k,v)  (j)->com->_catdis( (j)->prp, d, k, v )
#define rkJointSubDis(j,d,sd)   (j)->com->_subdis( (j)->prp, d, sd )
#define rkJointSetDisCNT(j,v,t) (j)->com->_cntdis( (j)->prp, v, t )

#define rkJointSetFric(j,f)  (j)->com->_setfric( (j)->prp, f )
#define rkJointGetFric(j,f)  (j)->com->_getfric( (j)->prp, f )
#define rkJointGetSFric(j,f) (j)->com->_getsfric( (j)->prp, f )
#define rkJointGetKFric(j,f) (j)->com->_getkfric( (j)->prp, f )

#define rkJointSetRef(j,r) (j)->com->_setref( (j)->prp, r )
#define rkJointGetRef(j,r) (j)->com->_getref( (j)->prp, r )

/* motor */
#define rkJointGetMotor(j,m)        (j)->com->_getmotor( (j)->prp, m )
#define rkJointMotorType(j)         (j)->mcom->_type( (j)->prp )
#define rkJointMotorSetInput(j,i)   (j)->mcom->_setinput( (j)->prp, i )
#define rkJointMotorInertia(j,i)    (j)->mcom->_inertia( (j)->prp, i )
#define rkJointMotorInputTrq(j,t)   (j)->mcom->_inputtrq( (j)->prp, t )
#define rkJointMotorRegistance(j,r) (j)->mcom->_regist( (j)->prp, r )
#define rkJointMotorDrivingTrq(j,t) (j)->mcom->_dtrq( (j)->prp, t )

/* ABI */
#define rkJointABIAxisInertia(j,m,h,ih) (j)->acom->_axinertia( (j)->prp, m, h, ih )
#define rkJointABIAddAbi(j,i,f,h,pi)    (j)->acom->_addabi( (j)->prp, i, f, h, pi )
#define rkJointABIAddBias(j,i,b,f,h,pb) (j)->acom->_addbias( (j)->prp, i, b, f, h, pb )
#define rkJointABIDrivingTorque(j)      (j)->acom->_dtrq( (j)->prp )
#define rkJointABIQAcc(j,r,i,b,c,h,a)   (j)->acom->_qacc( (j)->prp, r, i, b, c, h, a )
/* #define rkJointUpdateWrench(j,i,b,a)    (j)->acom->_wrench( (j)->prp, i, b, a, rkJointWrench(j) ) */
#define rkJointUpdateWrench(j,i,b,a)    (j)->acom->_wrench( j, i, b, a )

/*! \brief neutral configuration of joint.
 *
 * rkJointNeutral() sets displacement, velocity and
 * acceleration of a joint \a j for zero.
 *
 * rkJointIsNeutral() checks if the joint \a j is in
 * neutral configuration where all the components are
 * at zero values.
 * \return
 * rkJointNeutral() returns no value.
 *
 * rkJointIsNeutral() returns the true value if all
 * (i.e. the number of degrees-of-freedom) the components
 * of joint displacement of \a j are zero. Otherwise, the false
 * value is returned.
 */
void rkJointNeutral(rkJoint *j);
bool rkJointIsNeutral(rkJoint *j);

/*! \brief transfer joint frame, velocity, acceleration and torque.
 *
 * rkJointXfer() transfers a 3D frame \a fo in accordance with
 * the displacement of a joint \a j. The result is put into \a f.
 *
 * Transfer rule depends on the joint type.
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
 * rkJointXfer() returns a pointer \a f.
 * \sa
 * rkJointCreate
 */
#define rkJointXfer(j,fo,f)       (j)->com->_xfer( (j)->prp, fo, f )
#define rkJointIncVel(j,v)        (j)->com->_incvel( (j)->prp, v)
#define rkJointIncAccOnVel(j,w,a) (j)->com->_incaccvel( (j)->prp, w, a )
#define rkJointIncAcc(j,a)        (j)->com->_incacc( (j)->prp, a )
__EXPORT void rkJointIncRate(rkJoint *j, zVec3D *w, zVec6D *vel, zVec6D *acc);
#define rkJointCalcTrq(j,f)       (j)->com->_trq( (j)->prp, f )

#define rkJointTorsion(j,dev,t,d) (j)->com->_torsion( dev, t, d )

/*! \brief joint axis vector.
 *
 * rkJointAngAxis() and rkJointLinAxis() calculates axis vector of
 * the \a i'th component of a joint \a j about angular and linear
 * motion, respectively.
 * \a f is a frame attached to the link with \a j.
 * The result is put into \a a.
 * \return
 * rkJointAngJacobiCol() and rkJointLinJacobiCol() return a pointer
 * \a a if the axis vector is non-zero. Otherwise, they return the
 * null pointer.
 * \notes
 * Neither rkJointAngJacobiCol() nor rkJointLinJacobiCol() check if
 * \a i is valid. \a i which is larger than the the degree of
 * freedom of \a j might cause anything.
 */
#define rkJointAngAxis(j,i,f,a) (j)->com->_angaxis[i]( (j)->prp, f, a )
#define rkJointLinAxis(j,i,f,a) (j)->com->_linaxis[i]( (j)->prp, f, a )

/*! \brief read/write joint displacement and properties.
 *
 * rkJointQueryFRead() reads joint properties of \a j identified
 * by a string \a key from the current position of a file \a fp.
 *
 * The candidates of \a key are listed as follows:
 *  "jointtype" for joint type. Refer rkJointTypeByStr().
 *  "dis" for the initial joint displacement.
 *  "min" for the minimum joint displacement.
 *  "max" for the maximum joint displacement.
 *  "stiffness" for the joint stiffness.
 *  "viscos" for the joint viscosity.
 *  "coulomb" for Coulomb s joint friction.
 * The number of components of each property is according to the
 * joint type, and hence, the joint type has to be identified
 * first of all.
 *
 * Values are internally held in meter for linear displacements
 * or in radian for angular displacements. Angular displacements
 * are converted to values in degree when denoted in files for
 * easier estimation. These functions automatically choose the
 * input and output unit for each value according to the joint
 * type of \a j.
 * Examples: For \a j as a cylindric joint, two values are
 * denoted in [m] and [deg] when values are displacements, or
 * in [N/m] and [N.m/deg] when values are stiffnesses.
 *
 * rkJointFWrite() writes the joint properties in accordance with
 * the above rule. The field "dis" is output only when any
 * components of the joint displacement are non-zero values.
 * \a name is used instead of the key "dis", unless it is the
 * null pointer.
 * \return
 * rkJointQueryFRead() returns the true value when \a key is
 * valid, or the false value otherwise.
 *
 * rkJointFWrite() and rkJointWrite() return no value.
 */
#define rkJointQueryFRead(f,b,j,ma,mn) (j)->com->_query( f, b, (j)->prp, (ma), (mn) )
#define rkJointFWrite(f,j,n)     ( (n) ? (j)->com->_write( f, (j)->prp, n ) : (j)->com->_write( f, (j)->prp, "dis" ) )
#define rkJointWrite(j,n)        rkJointFWrite( stdout, j, n )

/* NOTE: The following macros and functions are for sharing
 * some operation codes. Do not use them in users programs. */
#define _rkJointRestTrq(s,v,c,dis,vel) ( -s*dis -v*vel -c*zSgn(vel) )

__EXPORT zVec3D *_rkJointAxisNull(void *prp, zFrame3D *f, zVec3D *a);
__EXPORT zVec3D *_rkJointAxisZ(void *prp, zFrame3D *f, zVec3D *a);

__EXPORT double rkJointTorsionDisRevol(zFrame3D *dev, zVec6D *t);
__EXPORT double rkJointTorsionDisPrism(zFrame3D *dev, zVec6D *t);

/* for ABI */
__EXPORT zMat6D *rkJointXferMat6D(zFrame3D *f, zMat6D *i, zMat6D *m);
/* __EXPORT void _rkJointUpdateWrench(void *prp, zMat6D *i, zVec6D *b, zVec6D *acc, zVec6D *w); */
__EXPORT void _rkJointUpdateWrench(rkJoint *j, zMat6D *i, zVec6D *b, zVec6D *acc);

__END_DECLS

#include <roki/rk_joint_fixed.h>   /* fixed joint */
#include <roki/rk_joint_revol.h>   /* revolutional joint */
#include <roki/rk_joint_prism.h>   /* prismatic joint */
#include <roki/rk_joint_cylin.h>   /* cylindrical joint */
#include <roki/rk_joint_hooke.h>   /* universal joint */
#include <roki/rk_joint_spher.h>   /* spherical joint */
#include <roki/rk_joint_float.h>   /* free-floating joint */
#include <roki/rk_joint_brfloat.h> /* breakable free-floating joint */

#endif /* __RK_JOINT_H__ */
