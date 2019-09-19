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
/* CLASS: rkJoint
 * joint class
 * ********************************************************** */
struct _rkJoint;

/* for forward dynamics */
typedef struct{
  byte type;
  double ref_dis, ref_trq;
} rkJointFrictionPivot;

typedef struct{
  const char *typestr; /*!< \brief a string to identify the type of joint */
  byte size; /* number of joint components */
  void (*_init)(void*);
  void *(*_alloc)(void);

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
  void (*_catdis)(void*,double*,double,double*); /* concatenate displacements */
  void (*_subdis)(void*,double*,double*); /* subtract displacements */
  void (*_cntdis)(void*,double*,double); /* continuous update */

  zFrame3D *(*_xform)(void*,zFrame3D*,zFrame3D*); /* frame transformation */
  void (*_incvel)(void*,zVec6D*); /* motion rate transformation */
  void (*_incaccvel)(void*,zVec3D*,zVec6D*);
  void (*_incacc)(void*,zVec6D*);
  void (*_trq)(void*,zVec6D*);        /* joint torque transformation */
  void (*_torsion)(zFrame3D*,zVec6D*,double*); /* inverse computation of torsion and displacement */

  /* axis vector */
  zVec3D* (**_angaxis)(void*,zFrame3D*,zVec3D*); /* angular */
  zVec3D* (**_linaxis)(void*,zFrame3D*,zVec3D*); /* linear */

  /* for forward dynamics */
  void (*_setfrictionpivot)(void*,rkJointFrictionPivot*); /* set referential displacement of friction */
  void (*_getfrictionpivot)(void*,rkJointFrictionPivot*); /* get referential displacement of friction */
  void (*_setfric)(void*,double*);  /* set joint friction force/torque */
  void (*_getfric)(void*,double*);  /* get joint friction force/torque */
  void (*_getsfric)(void*,double*); /* set static friction force/torque */
  void (*_getkfric)(void*,double*); /* get kinetic friction force/torque */

  rkMotor *(*_getmotor)(void*);
  void (*_setmotorinput)(void*,double*);
  void (*_motorinertia)(void*,double*);
  void (*_motorinputtrq)(void*,double*);
  void (*_motorregist)(void*,double*);
  void (*_motordestrq)(void*,double*);

  void (*_axinertia)(void*,zMat6D*,zMat,zMat);
  void (*_addabi)(void*,zMat6D*,zFrame3D*,zMat,zMat6D*);
  void (*_addbias)(void*,zMat6D*,zVec6D*,zFrame3D*,zMat,zVec6D*);
  void (*_dtrq)(void*);
  void (*_qacc)(void*,zMat6D*,zVec6D*,zVec6D*,zMat,zVec6D*);
  void (*_wrench)(struct _rkJoint*,zMat6D*,zVec6D*,zVec6D*);

  /* I/O */
  bool (*_regZTK)(ZTK*,char*);
  void *(*_dis_fromZTK)(void*,int,void*,ZTK*);
  void *(*_fromZTK)(void*,rkMotorArray*,ZTK*);
  void (*_dis_fprintZTK)(FILE*,int,void*);
  void (*_fprintZTK)(FILE*,void*,char*);  /* print */
} rkJointCom;

typedef struct _rkJoint{
  void *prp;
  zVec6D wrench; /* joint wrench */
  rkJointCom *com;
} rkJoint;

#define rkJointSize(j)     (j)->com->size
#define rkJointTypeStr(j)  (j)->com->typestr
#define rkJointWrench(j)   &(j)->wrench

/*! \brief initialize a joint object.
 *
 * rkJointInit() initializes a joint object \a j, cleaning
 * up all properties.
 * \return
 * rkJointInit() returns a pointer \a j if it succeeds.
 * If \a type is invalid, or it fails to allocate the internal
 * working memory, the null pointer is returned.
 */
#define rkJointInit(j) do{\
  (j)->prp = NULL;\
  (j)->com = NULL;\
} while(0)

__EXPORT rkJoint *rkJointAssign(rkJoint *j, rkJointCom *com);
__EXPORT rkJoint *rkJointQueryAssign(rkJoint *j, char *str);
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
#define rkJointLimDis(j,t,v)        (j)->com->_limdis( (j)->prp, t, v )
#define rkJointSetDis(j,v)          (j)->com->_setdis( (j)->prp, v )
#define rkJointSetVel(j,v)          (j)->com->_setvel( (j)->prp, v )
#define rkJointSetAcc(j,v)          (j)->com->_setacc( (j)->prp, v )
#define rkJointSetTrq(j,v)          (j)->com->_settrq( (j)->prp, v )
#define rkJointGetDis(j,v)          (j)->com->_getdis( (j)->prp, v )
#define rkJointGetVel(j,v)          (j)->com->_getvel( (j)->prp, v )
#define rkJointGetAcc(j,v)          (j)->com->_getacc( (j)->prp, v )
#define rkJointGetTrq(j,v)          (j)->com->_gettrq( (j)->prp, v )

#define rkJointCatDis(j,d,k,v)      (j)->com->_catdis( (j)->prp, d, k, v )
#define rkJointSubDis(j,d,sd)       (j)->com->_subdis( (j)->prp, d, sd )
#define rkJointSetDisCNT(j,v,t)     (j)->com->_cntdis( (j)->prp, v, t )

#define rkJointSetFricPivot(j,r)    (j)->com->_setfrictionpivot( (j)->prp, r )
#define rkJointGetFricPivot(j,r)    (j)->com->_getfrictionpivot( (j)->prp, r )
#define rkJointSetFric(j,f)         (j)->com->_setfric( (j)->prp, f )
#define rkJointGetFric(j,f)         (j)->com->_getfric( (j)->prp, f )
#define rkJointGetSFric(j,f)        (j)->com->_getsfric( (j)->prp, f )
#define rkJointGetKFric(j,f)        (j)->com->_getkfric( (j)->prp, f )

/* motor */
#define rkJointGetMotor(j)          (j)->com->_getmotor( (j)->prp )
#define rkJointMotorSetInput(j,i)   (j)->com->_setmotorinput( (j)->prp, i )
#define rkJointMotorInertia(j,i)    (j)->com->_motorinertia( (j)->prp, i )
#define rkJointMotorInputTrq(j,t)   (j)->com->_motorinputtrq( (j)->prp, t )
#define rkJointMotorRegistance(j,r) (j)->com->_motorregist( (j)->prp, r )
#define rkJointMotorDrivingTrq(j,t) (j)->com->_motordestrq( (j)->prp, t )

/* ABI */
#define rkJointABIAxisInertia(j,m,h,ih) (j)->com->_axinertia( (j)->prp, m, h, ih )
#define rkJointABIAddABI(j,i,f,h,pi)    (j)->com->_addabi( (j)->prp, i, f, h, pi )
#define rkJointABIAddBias(j,i,b,f,h,pb) (j)->com->_addbias( (j)->prp, i, b, f, h, pb )
#define rkJointABIDrivingTorque(j)      (j)->com->_dtrq( (j)->prp )
#define rkJointABIQAcc(j,i,b,c,h,a)     (j)->com->_qacc( (j)->prp, i, b, c, h, a )
#define rkJointUpdateWrench(j,i,b,a)    (j)->com->_wrench( j, i, b, a )

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

/*! \brief transform a joint frame, velocity, acceleration and torque.
 *
 * rkJointXform() transforms a 3D frame \a fo in accordance with
 * the displacement of a joint \a j. The result is put into \a f.
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
#define rkJointXform(j,fo,f)      (j)->com->_xform( (j)->prp, fo, f )
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

__EXPORT zVec3D *_rkJointAxisNull(void *prp, zFrame3D *f, zVec3D *a);
__EXPORT zVec3D *_rkJointAxisZ(void *prp, zFrame3D *f, zVec3D *a);

__EXPORT double rkJointRevolTorsionDis(zFrame3D *dev, zVec6D *t);
__EXPORT double rkJointPrismTorsionDis(zFrame3D *dev, zVec6D *t);

/* NOTE: The following macros and functions are for sharing
 * some operation codes. Do not use them in users programs. */
#define _rkJointRestTrq(s,v,c,dis,vel) ( -s*dis -v*vel -c*zSgn(vel) )

/* for ABI */
__EXPORT zMat6D *rkJointXformMat6D(zFrame3D *f, zMat6D *i, zMat6D *m);
__EXPORT void _rkJointUpdateWrench(rkJoint *j, zMat6D *i, zVec6D *b, zVec6D *acc);

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

#define rkJointPrpFromZTK(prp,motorarray,ztk,ztkprp) \
  ( ZTKEvalKey( prp, motorarray, ztk, ztkprp ) ? prp : NULL )

__EXPORT bool rkJointRegZTK(ZTK *ztk, char *tag);

__EXPORT rkJoint *rkJointFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk);

#define rkJointFPrintZTK(f,j,n) ( (n) ? (j)->com->_fprintZTK( f, (j)->prp, n ) : (j)->com->_fprintZTK( f, (j)->prp, "dis" ) )

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

/* add the handle to the following list when you create a new joint class. */
#define RK_JOINT_COM_ARRAY \
rkJointCom *_rk_joint_com[] = {\
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
