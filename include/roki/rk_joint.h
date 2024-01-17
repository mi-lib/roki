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
struct _rkJointCom;
typedef struct _rkJointCom rkJointCom;

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJoint ){
  void *state; /* joint state */
  void *prp;   /* joint properties */
 /* void *state;*/ /* joint state */
  zVec6D wrench; /* joint wrench */
  rkJointCom *com;
};

/* for forward dynamics */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointFrictionPivot ){
  byte type;
  double ref_dis, prev_trq;
};

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkJointCom ){
  const char *typestr; /*!< \brief a string to identify the type of joint */
  byte size; /* number of joint components */
  void (*_init)(rkJoint*);
  void *(*_alloc_state)(void);
  void *(*_alloc_prp)(void);
  void (*_copy_prp)(rkJoint*,rkJoint*);

  /* joint value manipulation function */
  void (*_limdis)(rkJoint*,double*,double*); /* limit displacements */
  void (*_setdis)(rkJoint*,double*);     /* set displacements */
  void (*_setmin)(rkJoint*,double*);     /* set minimum displacements */
  void (*_setmax)(rkJoint*,double*);     /* set maximum displacements */
  void (*_setvel)(rkJoint*,double*);     /* set velocity */
  void (*_setacc)(rkJoint*,double*);     /* set acceleration */
  void (*_settrq)(rkJoint*,double*);     /* set torque */
  void (*_getdis)(rkJoint*,double*);     /* get displacements */
  void (*_getmin)(rkJoint*,double*);     /* get minimum displacements */
  void (*_getmax)(rkJoint*,double*);     /* get maximum displacements */
  void (*_getvel)(rkJoint*,double*);     /* get velocity */
  void (*_getacc)(rkJoint*,double*);     /* get acceleration */
  void (*_gettrq)(rkJoint*,double*);     /* get torques */
  void (*_catdis)(rkJoint*,double*,double,double*); /* concatenate displacements */
  void (*_subdis)(rkJoint*,double*,double*); /* subtract displacements */
  void (*_cntdis)(rkJoint*,double*,double); /* continuous update */

  zFrame3D *(*_xform)(rkJoint*,zFrame3D*,zFrame3D*); /* frame transformation */
  void (*_incvel)(rkJoint*,zVec6D*); /* motion rate transformation */
  void (*_incaccvel)(rkJoint*,zVec3D*,zVec6D*);
  void (*_incacc)(rkJoint*,zVec6D*);
  void (*_trq)(rkJoint*,zVec6D*);        /* joint torque transformation */
  void (*_torsion)(zFrame3D*,zVec6D*,double*); /* inverse computation of torsion and displacement */

  /* axis vector */
  zVec3D* (**_angaxis)(rkJoint*,zFrame3D*,zVec3D*); /* angular */
  zVec3D* (**_linaxis)(rkJoint*,zFrame3D*,zVec3D*); /* linear */

  /* composite rigid body method */
  void (*_crb_wrench)(rkJoint*,rkMP*,zVec6D[]);
  void (*_crb_xform)(rkJoint*,zFrame3D*,zVec6D[]);

  /* for forward dynamics */
  void (*_setfrictionpivot)(rkJoint*,rkJointFrictionPivot*); /* set referential displacement of friction */
  void (*_getfrictionpivot)(rkJoint*,rkJointFrictionPivot*); /* get referential displacement of friction */
  void (*_setfriction)(rkJoint*,double*);  /* set joint friction force/torque */
  void (*_getfriction)(rkJoint*,double*);  /* get joint friction force/torque */
  void (*_getsfriction)(rkJoint*,double*); /* set static friction force/torque */
  void (*_getkfriction)(rkJoint*,double*); /* get kinetic friction force/torque */

  rkMotor *(*_getmotor)(rkJoint*);
  void (*_setmotorinput)(rkJoint*,double*);
  void (*_motorinertia)(rkJoint*,double*);
  void (*_motorinputtrq)(rkJoint*,double*);
  void (*_motorregist)(rkJoint*,double*);
  void (*_motordestrq)(rkJoint*,double*);

  void (*_axinertia)(rkJoint*,zMat6D*,zMat,zMat);
  void (*_addabi)(rkJoint*,zMat6D*,zFrame3D*,zMat,zMat6D*);
  void (*_addbias)(rkJoint*,zMat6D*,zVec6D*,zFrame3D*,zMat,zVec6D*);
  void (*_dtrq)(rkJoint*);
  void (*_qacc)(rkJoint*,zMat6D*,zVec6D*,zVec6D*,zMat,zVec6D*);
  void (*_wrench)(struct _rkJoint*,zMat6D*,zVec6D*,zVec6D*);

  /* I/O */
  void *(*_dis_fromZTK)(void*,int,void*,ZTK*);
  rkJoint *(*_fromZTK)(rkJoint*,rkMotorArray*,ZTK*);
  void (*_dis_fprintZTK)(FILE*,int,void*);
  void (*_fprintZTK)(FILE*,rkJoint*,char*);  /* print */
};

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
  (j)->prp   = NULL;\
  (j)->com   = NULL;\
} while(0)

__ROKI_EXPORT rkJoint *rkJointAssign(rkJoint *j, rkJointCom *com);
__ROKI_EXPORT rkJoint *rkJointQueryAssign(rkJoint *j, char *str);
__ROKI_EXPORT void rkJointDestroy(rkJoint *j);

__ROKI_EXPORT rkJoint *rkJointClone(rkJoint *org, rkJoint *cln);
__ROKI_EXPORT rkJoint *rkJointCopyPrp(rkJoint *src, rkJoint *dst);
__ROKI_EXPORT rkJoint *rkJointCopyState(rkJoint *src, rkJoint *dst);

/*! \brief set and get joint status.
 *
 * rkJointSetDis(), rkJointSetMin(), rJointSetMax(), rkJointSetVel(),
 * rkJointSetAcc() and rkJointSetTrq() set joint displacements, minimum
 * joint displacements, maximum joint displacements, velocities,
 * accelerations and torques \a val to a joint \a j.
 *
 * rkJointGetDis(), rkJointGetMin(), rkJointGetMax(), rkJointGetVel(),
 * rkJointGetAcc() and rkJointGetTrq() get joint displacements, minimum
 * joint displacements, maximum joint displacements, velocities,
 * accelerations and torques of \a j to \a val.
 *
 * rkJointSetDisCNT() continuously updates the joint
 * displacement of a joint \a j to \a val over a time step
 * \a dt, calculating joint velocity and acceleration in
 * accordance with a simple differentiation.
 *
 * The size of the array \a val depends on the type of \a j, basically
 * coinciding with its degree of freedom.
 * \notes
 * They do not check the size consistency. Mismatched array size might
 * cause anything.
 * \return
 * None of those functions return any value.
 */
#define rkJointLimDis(j,t,v)        (j)->com->_limdis( (j), t, v )
#define rkJointSetDis(j,v)          (j)->com->_setdis( (j), v )
#define rkJointSetMin(j,v)          (j)->com->_setmin( (j), v )
#define rkJointSetMax(j,v)          (j)->com->_setmax( (j), v )
#define rkJointSetVel(j,v)          (j)->com->_setvel( (j), v )
#define rkJointSetAcc(j,v)          (j)->com->_setacc( (j), v )
#define rkJointSetTrq(j,v)          (j)->com->_settrq( (j), v )
#define rkJointGetDis(j,v)          (j)->com->_getdis( (j), v )
#define rkJointGetMin(j,v)          (j)->com->_getmin( (j), v )
#define rkJointGetMax(j,v)          (j)->com->_getmax( (j), v )
#define rkJointGetVel(j,v)          (j)->com->_getvel( (j), v )
#define rkJointGetAcc(j,v)          (j)->com->_getacc( (j), v )
#define rkJointGetTrq(j,v)          (j)->com->_gettrq( (j), v )

#define rkJointCatDis(j,d,k,v)      (j)->com->_catdis( (j), d, k, v )
#define rkJointSubDis(j,d,sd)       (j)->com->_subdis( (j), d, sd )
#define rkJointSetDisCNT(j,v,t)     (j)->com->_cntdis( (j), v, t )

#define rkJointSetFrictionPivot(j,r) (j)->com->_setfrictionpivot( (j), r )
#define rkJointGetFrictionPivot(j,r) (j)->com->_getfrictionpivot( (j), r )
#define rkJointSetFriction(j,f)      (j)->com->_setfriction( (j), f )
#define rkJointGetFriction(j,f)      (j)->com->_getfriction( (j), f )
#define rkJointGetSFriction(j,f)     (j)->com->_getsfriction( (j), f )
#define rkJointGetKFriction(j,f)     (j)->com->_getkfriction( (j), f )

/* motor */
#define rkJointGetMotor(j)          (j)->com->_getmotor( (j) )
#define rkJointMotorSetInput(j,i)   (j)->com->_setmotorinput( (j), i )
#define rkJointMotorInertia(j,i)    (j)->com->_motorinertia( (j), i )
#define rkJointMotorInputTrq(j,t)   (j)->com->_motorinputtrq( (j), t )
#define rkJointMotorRegistance(j,r) (j)->com->_motorregist( (j), r )
#define rkJointMotorDrivingTrq(j,t) (j)->com->_motordestrq( (j), t )

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
#define rkJointABIAxisInertia(j,m,h,ih) (j)->com->_axinertia( (j), m, h, ih )
#define rkJointABIAddABI(j,i,f,h,pi)    (j)->com->_addabi( (j), i, f, h, pi )
#define rkJointABIAddBias(j,i,b,f,h,pb) (j)->com->_addbias( (j), i, b, f, h, pb )
#define rkJointABIDrivingTorque(j)      (j)->com->_dtrq( (j) )
#define rkJointABIQAcc(j,i,b,c,h,a)     (j)->com->_qacc( (j), i, b, c, h, a )
#define rkJointUpdateWrench(j,i,b,a)    (j)->com->_wrench( (j), i, b, a )

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
#define rkJointXform(j,fo,f)      (j)->com->_xform( (j), fo, f )
#define rkJointIncVel(j,v)        (j)->com->_incvel( (j), v)
#define rkJointIncAccOnVel(j,w,a) (j)->com->_incaccvel( (j), w, a )
#define rkJointIncAcc(j,a)        (j)->com->_incacc( (j), a )
__ROKI_EXPORT void rkJointIncRate(rkJoint *j, zVec3D *w, zVec6D *vel, zVec6D *acc);
#define rkJointCalcTrq(j,f)       (j)->com->_trq( (j), f )

#define rkJointTorsion(j,dev,t,d) (j)->com->_torsion( dev, t, d )

__ROKI_EXPORT double rkJointRevolTorsionDis(zFrame3D *dev, zVec6D *t);
__ROKI_EXPORT double rkJointPrismTorsionDis(zFrame3D *dev, zVec6D *t);

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
#define rkJointAngAxis(j,i,f,a) (j)->com->_angaxis[i]( (j), f, a )
#define rkJointLinAxis(j,i,f,a) (j)->com->_linaxis[i]( (j), f, a )

__ROKI_EXPORT zVec3D *_rkJointAxisNull(rkJoint *j, zFrame3D *f, zVec3D *a);
__ROKI_EXPORT zVec3D *_rkJointAxisZ(rkJoint *j, zFrame3D *f, zVec3D *a);

/* composite rigid body method */
#define rkJointCRBWrench(j,m,w) (j)->com->_crb_wrench( (j), m, w )
#define rkJointCRBXform(j,f,s)  (j)->com->_crb_xform( (j), f, s )

/* NOTE: The following macros and functions are for sharing
 * some operation codes. Do not use them in users programs. */
#define _rkJointRestTrq(s,v,c,dis,vel) ( -s*dis -v*vel -c*zSgn(vel) )

/* for ABI */
__ROKI_EXPORT zMat6D *rkJointXformMat6D(zFrame3D *f, zMat6D *i, zMat6D *m);
__ROKI_EXPORT void _rkJointUpdateWrench(rkJoint *j, zMat6D *i, zVec6D *b, zVec6D *acc);

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

#define rkJointPrpFromZTK(joint,motorarray,ztk,ztkprp) \
  ( ZTKEvalKey( joint, motorarray, ztk, ztkprp ) ? joint : NULL )

__ROKI_EXPORT rkJoint *rkJointFromZTK(rkJoint *joint, rkMotorArray *motorarray, ZTK *ztk);

#define rkJointFPrintZTK(f,j,n) ( (n) ? (j)->com->_fprintZTK( f, j, n ) : (j)->com->_fprintZTK( f, j, (char *)"dis" ) )

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
