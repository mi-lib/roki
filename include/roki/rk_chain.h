/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_chain - kinematic chain, kinematics and dynamics
 */

#ifndef __RK_CHAIN_H__
#define __RK_CHAIN_H__

#include <roki/rk_link.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkChain
 * kinematic chain class
 * ********************************************************** */

ZDECL_STRUCT( rkIK );

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkChain ){
  Z_NAMED_CLASS;
  rkLinkArray linkarray;           /*!< array of links */
  zMShape3D *shape;                /*!< multishape */
  rkMotorSpecArray motorspecarray; /*!< array of motor specifications */

  zVec3D wldcom;    /*!< position of COM in the world frame */
  zVec3D wldcomvel; /*!< velocity of COM in the world frame */
  zVec3D wldcomacc; /*!< acceleration of COM in the world frame */

  /*! \cond */
  bool _iscol; /* flag for collision check */
  rkIK *_ik; /* IK solver */
  /*! \endcond */
};

#define rkChainLinkArray(c)             ( &(c)->linkarray )
#define rkChainRoot(c)                  zArrayBuf( rkChainLinkArray(c) )
#define rkChainLink(c,i)                zArrayElemNC( rkChainLinkArray(c), i )
#define rkChainLinkNum(c)               zArraySize( rkChainLinkArray(c) )
#define rkChainShape(c)                 (c)->shape
#define rkChainMotorSpecArray(c)        ( &(c)->motorspecarray )
#define rkChainWldCOM(c)                ( &(c)->wldcom )
#define rkChainCOMVel(c)                ( &(c)->wldcomvel )
#define rkChainCOMAcc(c)                ( &(c)->wldcomacc )
#define rkChainMass(c)                  rkLinkCRBMass( rkChainRoot(c) )

#define rkChainSetShape(c,s)            ( rkChainShape(c) = (s) )
#define rkChainSetMass(c,m)             ( rkChainMass(c) = (m) )
#define rkChainSetWldCOM(c,p)           zVec3DCopy( p, rkChainWldCOM(c) )
#define rkChainSetCOMVel(c,v)           zVec3DCopy( v, rkChainCOMVel(c) )
#define rkChainSetCOMAcc(c,a)           zVec3DCopy( a, rkChainCOMAcc(c) )

#define rkChainLinkName(c,i)            zName(rkChainLink(c,i))
#define rkChainLinkJointIDOffset(c,i)   rkLinkJointIDOffset(rkChainLink(c,i))
#define rkChainLinkJoint(c,i)           rkLinkJoint(rkChainLink(c,i))
#define rkChainLinkJointDOF(c,i)        rkLinkJointDOF(rkChainLink(c,i))
#define rkChainLinkJointTypeStr(c,i)    rkLinkJointTypeStr(rkChainLink(c,i))
#define rkChainLinkMass(c,i)            rkLinkMass(rkChainLink(c,i))
#define rkChainLinkCOM(c,i)             rkLinkCOM(rkChainLink(c,i))
#define rkChainLinkInertia(c,i)         rkLinkInertia(rkChainLink(c,i))
#define rkChainLinkCRB(c,i)             rkLinkCRB(rkChainLink(c,i))
#define rkChainLinkCRBMass(c,i)         rkLinkCRBMass(rkChainLink(c,i))
#define rkChainLinkCRBCOM(c,i)          rkLinkCRBCOM(rkChainLink(c,i))
#define rkChainLinkCRBInertia(c,i)      rkLinkCRBInertia(rkChainLink(c,i))
#define rkChainLinkExtWrench(c,i)       rkLinkExtWrench(rkChainLink(c,i))
#define rkChainLinkShapeList(c,i)       rkLinkShapeList(rkChainLink(c,i))
#define rkChainLinkShapeNum(c,i)        rkLinkShapeNum(rkChainLink(c,i))
#define rkChainLinkOrgFrame(c,i)        rkLinkOrgFrame(rkChainLink(c,i))
#define rkChainLinkOrgPos(c,i)          rkLinkOrgPos(rkChainLink(c,i))
#define rkChainLinkOrgAtt(c,i)          rkLinkOrgAtt(rkChainLink(c,i))
#define rkChainLinkAdjFrame(c,i)        rkLinkAdjFrame(rkChainLink(c,i))
#define rkChainLinkAdjPos(c,i)          rkLinkAdjPos(rkChainLink(c,i))
#define rkChainLinkAdjAtt(c,i)          rkLinkAdjAtt(rkChainLink(c,i))
#define rkChainLinkWldFrame(c,i)        rkLinkWldFrame(rkChainLink(c,i))
#define rkChainLinkWldPos(c,i)          rkLinkWldPos(rkChainLink(c,i))
#define rkChainLinkWldAtt(c,i)          rkLinkWldAtt(rkChainLink(c,i))
#define rkChainLinkWldCOM(c,i)          rkLinkWldCOM(rkChainLink(c,i))
#define rkChainLinkVel(c,i)             rkLinkVel(rkChainLink(c,i))
#define rkChainLinkLinVel(c,i)          rkLinkLinVel(rkChainLink(c,i))
#define rkChainLinkLinAcc(c,i)          rkLinkLinAcc(rkChainLink(c,i))
#define rkChainLinkAngVel(c,i)          rkLinkAngVel(rkChainLink(c,i))
#define rkChainLinkAngAcc(c,i)          rkLinkAngAcc(rkChainLink(c,i))
#define rkChainLinkAcc(c,i)             rkLinkAcc(rkChainLink(c,i))
#define rkChainLinkCOMVel(c,i)          rkLinkCOMVel(rkChainLink(c,i))
#define rkChainLinkCOMAcc(c,i)          rkLinkCOMAcc(rkChainLink(c,i))
#define rkChainLinkWrench(c,i)          rkLinkWrench(rkChainLink(c,i))
#define rkChainLinkForce(c,i)           rkLinkForce(rkChainLink(c,i))
#define rkChainLinkTorque(c,i)          rkLinkTorque(rkChainLink(c,i))
#define rkChainLinkParent(c,i)          rkLinkParent(rkChainLink(c,i))
#define rkChainLinkChild(c,i)           rkLinkChild(rkChainLink(c,i))
#define rkChainLinkSibl(c,i)            rkLinkSibl(rkChainLink(c,i))

#define rkChainLinkJointNeutralize(c,i) rkLinkJointNeutralize( rkChainLink(c,i) )

#define rkChainOrgFrame(c)              rkLinkOrgFrame(rkChainRoot(c))
#define rkChainOrgPos(c)                rkLinkOrgPos(rkChainRoot(c))
#define rkChainOrgAtt(c)                rkLinkOrgAtt(rkChainRoot(c))
#define rkChainRootFrame(c)             rkLinkWldFrame(rkChainRoot(c))
#define rkChainRootPos(c)               rkLinkWldPos(rkChainRoot(c))
#define rkChainRootAtt(c)               rkLinkWldAtt(rkChainRoot(c))
#define rkChainRootVel(c)               rkLinkVel(rkChainRoot(c))
#define rkChainRootAcc(c)               rkLinkAcc(rkChainRoot(c))
#define rkChainRootLinVel(c)            rkLinkLinVel(rkChainRoot(c))
#define rkChainRootLinAcc(c)            rkLinkLinAcc(rkChainRoot(c))
#define rkChainRootAngVel(c)            rkLinkAngVel(rkChainRoot(c))
#define rkChainRootAngAcc(c)            rkLinkAngAcc(rkChainRoot(c))
#define rkChainRootWrench(c)            rkLinkWrench(rkChainRoot(c))
#define rkChainRootForce(c)             rkLinkForce(rkChainRoot(c))
#define rkChainRootTorque(c)            rkLinkTorque(rkChainRoot(c))

#define rkChainSetRootFrame(c,f)        rkLinkSetWldFrame(rkChainRoot(c),f)
#define rkChainSetRootPos(c,p)          rkLinkSetWldPos(rkChainRoot(c),p)
#define rkChainSetRootAtt(c,m)          rkLinkSetWldAtt(rkChainRoot(c),m)
#define rkChainSetRootVel(c,v)          rkLinkSetVel(rkChainRoot(c),v)
#define rkChainSetRootAcc(c,a)          rkLinkSetAcc(rkChainRoot(c),a)
#define rkChainSetRootLinVel(c,v)       rkLinkSetLinVel(rkChainRoot(c),v)
#define rkChainSetRootLinAcc(c,a)       rkLinkSetLinAcc(rkChainRoot(c),a)
#define rkChainSetRootAngVel(c,o)       rkLinkSetAngVel(rkChainRoot(c),o)
#define rkChainSetRootAngAcc(c,a)       rkLinkSetAngAcc(rkChainRoot(c),a)
#define rkChainSetRootWrench(c,f)       rkLinkSetWrench(rkChainRoot(c),f)
#define rkChainSetRootForce(c,f)        rkLinkSetForce(rkChainRoot(c),f)
#define rkChainSetRootTorque(c,n)       rkLinkSetTorque(rkChainRoot(c),n)

/*! \brief initialize and destroy a kinematic chain.
 *
 * rkChainInit() initializes a kinematic chain instance pointed by \a c.
 *
 * rkChainDestroy() destroys all internal objects of kinematic chain \a c,
 * including the array of shapes and links.
 */
__ROKI_EXPORT void rkChainInit(rkChain *c);
__ROKI_EXPORT void rkChainDestroy(rkChain *c);

/*! \brief clone a kinematic chain.
 *
 * rkChainClone() clones a kinematic chain \a org to another \a cln.
 * \return cln
 */
__ROKI_EXPORT rkChain *rkChainClone(rkChain *org, rkChain *cln);

/*! \brief copy state of a kinematic chain.
 *
 * rkChainCopyState() copies state of all links and position, velocity
 * and acceleration of the center of mass of a kinematic chain \a src
 * to that of another \a dst.
 * \return dst
 */
__ROKI_EXPORT rkChain *rkChainCopyState(rkChain *src, rkChain *dst);

/*! \brief count total number of joints of a kinematic chain.
 *
 * rkChainJointSize() counts the total number of joints of a kinematic
 * chain \a c. How to count is conforming to rkJointDOF().
 *
 * rkChainCreateDefaultJointIndex() creates a joint index of \a c,
 * which only arranges the movable joints.
 * The fixed joints are ignored.
 *
 * rkChainJointIndexSize() counts the total joint size indicated by \a idx.
 * \return
 * rkChainJointSize() and rkChainJointIndexSize() returns the
 * total joint size counted.
 *
 * rkChainCreateDefaultJointIndex() returns a pointer to the
 * newly created index. When it fails to allocate memory, the
 * null pointer is returned.
 * \sa
 * rkJointSize
 */
__ROKI_EXPORT int rkChainJointSize(rkChain *c);
__ROKI_EXPORT zIndex rkChainCreateDefaultJointIndex(rkChain *c);
__ROKI_EXPORT int rkChainJointIndexSize(rkChain *c, zIndex idx);

/*! \brief find a link of a kinematic chain from name.
 *
 * rkChainFindLink() returns a pointer to the link named \a name of a kinematic
 * chain \a chain.
 * rkChainFindLinkID() returns a link identifier of the link named \a name of
 * \a chain.
 * rkChainFindLinkJointIDOffset() returns a joint identifier offset of a link
 * \a name of \a chain.
 * \return
 * If a link with \a name does not exist in \a chain, rkChainFindLink() returns
 * the null pointer, and rkChainFindLinkID() and rkChainFindLinkJointIDOffset()
 * return -1 as an invaid value.
 */
__ROKI_EXPORT rkLink *rkChainFindLink(rkChain *chain, const char *name);
__ROKI_EXPORT int rkChainFindLinkID(rkChain *chain, const char *name);
__ROKI_EXPORT int rkChainFindLinkJointIDOffset(rkChain *chain, const char *name);

/*! \brief update joint state.
 *
 * rkChainLinkJointSetDis() sets the joint displacement of the
 * \a i'th link of a kinematic chain \a c for \a dis.
 * Components of \a dis for a revolutional joint are in radian.
 * It automatically limits components of \a dis which is out of
 * motion range.
 *
 * rkChainLinkJointSetDisCNT() continuously updates the joint
 * displacement of the \a i'th link of \a c to \a dis over a
 * time step \a dt. Then, the joint velocity and acceleration
 * is calculated in accordance with a simple differentiation.
 *
 * rkChainLinkJointGetDis() gets the joint displacement of the
 * \a i'th link of \a c and puts it into \a dis.
 *
 * rkChainSetJointDis() sets all the joint displacements of
 * \a c specified by a joint index \a index for \a dis.
 * Components corresponding to revolutional joints have to be
 * in radian. Values which are out of the motion range range of
 * the corresponding joints will be automatically limited.
 *
 * rkChainSetJointDisCNT() continuously updates the joint
 * displacements of \a c specified by \a index to \a dis over
 * a time step \a dt. The joint velocity and acceleration is
 * calculated in accordance with a simple differentiation.
 *
 * rkChainSetJointRate() sets joint velocities and accelerations
 * specified by \a index of \a c for \a vel and \a acc,
 * respectively.
 *
 * rkChainGetJointDis() gets joint displacements of \a c and
 * stores them into \a dis. Only the joints specified by \a index
 * are dealt with.
 *
 * rkChainSetJointDisAll(), rkChainSetJointDisCNTAll(),
 * rkChainSetJointRateAll() and rkChainGetJointDisAll() set or
 * get all the joint status to or from \a dis, \a vel and \a acc.
 * The correspondence between kinematic chain links and vector
 * components follows the index to be created by
 * rkChainCreateDefaultJointIndex().
 * \return
 * rkChainSetJoint family does not return any values.
 *
 * rkChainGetJointDis() returns a pointer \a dis.
 * \sa
 * rkLinkJointSetDisCNT, rkChainCreateDefaultJointIndex
 */
#define rkChainLinkJointLimDis(r,i,td,ld)  rkLinkJointLimDis( rkChainLink(r,i), td, ld )
#define rkChainLinkJointSetDis(r,i,d)      rkLinkJointSetDis( rkChainLink(r,i), d )
#define rkChainLinkJointSetDisCNT(r,i,d,t) rkLinkJointSetDisCNT( rkChainLink(r,i), d, t )
#define rkChainLinkJointSetVel(r,i,v)      rkLinkJointSetVel( rkChainLink(r,i), v )
#define rkChainLinkJointSetAcc(r,i,a)      rkLinkJointSetAcc( rkChainLink(r,i), a )
#define rkChainLinkJointSetTrq(r,i,t)      rkLinkJointSetTrq( rkChainLink(r,i), t )

#define rkChainLinkJointGetDis(r,i,d)      rkLinkJointGetDis( rkChainLink(r,i), d )
#define rkChainLinkJointGetVel(r,i,v)      rkLinkJointGetVel( rkChainLink(r,i), v )
#define rkChainLinkJointGetAcc(r,i,a)      rkLinkJointGetAcc( rkChainLink(r,i), a )
#define rkChainLinkJointGetTrq(r,i,t)      rkLinkJointGetTrq( rkChainLink(r,i), t )

#define rkChainLinkJointGetMotor(r,i,m)    rkLinkJointGetMotor( rkChainLink(r,i), m )

#define rkChainLinkJointMotorSetInput(r,i,t)  rkLinkJointMotorSetInput( rkChainLink(r,i), t )

__ROKI_EXPORT void rkChainSetJointDis(rkChain *c, zIndex idx, zVec dis);
__ROKI_EXPORT void rkChainSetJointDisCNT(rkChain *c, zIndex idx, zVec dis, double dt);
__ROKI_EXPORT void rkChainSetJointVel(rkChain *c, zIndex idx, zVec vel);
__ROKI_EXPORT void rkChainSetJointAcc(rkChain *c, zIndex idx, zVec acc);
__ROKI_EXPORT void rkChainSetJointRate(rkChain *c, zIndex idx, zVec vel, zVec acc);
__ROKI_EXPORT zVec rkChainGetJointDis(rkChain *c, zIndex idx, zVec dis);

__ROKI_EXPORT void rkChainSetJointDisAll(rkChain *c, zVec dis);
__ROKI_EXPORT void rkChainCatJointDisAll(rkChain *c, zVec dis, double k, zVec v);
__ROKI_EXPORT void rkChainSubJointDisAll(rkChain *c, zVec dis, zVec sdis);
__ROKI_EXPORT void rkChainSetJointDisCNTAll(rkChain *c, zVec dis, double dt);
__ROKI_EXPORT void rkChainSetJointVelAll(rkChain *c, zVec vel);
__ROKI_EXPORT void rkChainSetJointAccAll(rkChain *c, zVec acc);
__ROKI_EXPORT void rkChainSetJointRateAll(rkChain *c, zVec vel, zVec acc);
__ROKI_EXPORT void rkChainSetJointTrqAll(rkChain *c, zVec trq);

__ROKI_EXPORT zVec rkChainGetJointDisAll(rkChain *c, zVec dis);
__ROKI_EXPORT zVec rkChainGetJointVelAll(rkChain *c, zVec vel);
__ROKI_EXPORT zVec rkChainGetJointAccAll(rkChain *c, zVec acc);
__ROKI_EXPORT zVec rkChainGetJointTrqAll(rkChain *c, zVec trq);

__ROKI_EXPORT zVec rkChainGetConf(rkChain *chain, zVec conf);
__ROKI_EXPORT void rkChainSetConf(rkChain *chain, zVec conf);

__ROKI_EXPORT void rkChainSetMotorInputAll(rkChain *c, zVec input);

/*! \brief update kinematic chain motion state.
 *
 * rkChainUpdateFrame() updates the whole link frames of a kinematic chain
 * \a c with respect to the world frame.
 *
 * rkChainUpdateVel() and rkChainUpdateAcc() update velocities and accelerations
 * of the whole link frames of \a c with respect to the inertia frame, respectively.
 *
 * rkChainUpdateRate() updates the rate, namely, the velocities and the accelerations
 * of the whole links of \a c with respect to the frame that has an acceleration
 * of the field \a g.
 * rkChainUpdateRateGravity() updates the rate of the whole links of \a c in
 * the gravitational field.
 * rkChainUpdateRateZeroGravity() updates the rate of the whole links of \a c
 * in the gravity-free field.
 *
 * rkChainUpdateWrench() computes wrenches, namely, combinations of force and
 * torque acting at the original points of the whole links of \a c.
 * \return
 * rkChainUpdateFrame(), rkChainUpdateVel(), rkChainUpdateAcc(), rkChainUpdateRate(),
 * rkChainUpdateRateGravity(), rkChainUpdateRateZeroGravity(), and rkChainUpdateWrench()
 * do not return any values.
 * Actually, they are not functions but macros. Refer rk_chain.h for their
 * implementations.
 * \sa
 * rkLinkUpdateFrame, rkLinkUpdateVel, rkLinkUpdateAcc, rkLinkUpdateRate,
 * rkLinkUpdateWrench
 */
#define rkChainUpdateFrame(c)   rkLinkUpdateFrame( rkChainRoot(c), ZFRAME3DIDENT )
#define rkChainUpdateVel(c)     rkLinkUpdateVel( rkChainRoot(c), ZVEC6DZERO )
#define rkChainUpdateAcc(c)     rkLinkUpdateAcc( rkChainRoot(c), ZVEC6DZERO, RK_GRAVITY6D )
#define rkChainUpdateRateG(c,g) rkLinkUpdateRate( rkChainRoot(c), ZVEC6DZERO, (g) )
#define rkChainUpdateRate(c)    rkChainUpdateRateG( c, RK_GRAVITY6D )
#define rkChainUpdateRate0G(c)  rkChainUpdateRateG( c, ZVEC6DZERO )
#define rkChainUpdateWrench(c)  rkLinkUpdateWrench( rkChainRoot(c) )

/*! \brief direction vector of gravity with respect to the body frame of a kinematic chain.
 *
 * rkChainGravityDir() computes the direction vector of gravity with respect to
 * the total frame of kinematic chain \a c, and store it into \a v.
 * \return
 * rkChainGravityDir() returns a pointer to the resultant vector \a v.
 */
#define rkChainGravityDir(c,v) zMat3DRow( rkChainRootAtt(c), zZ, v )

/*! \brief position of a point on a link of a kinematic chain in the world frame. */
#define rkChainLinkPointWldPos(c,i,p,pw) rkLinkPointWldPos( rkChainLink(c,i), p, pw )

/*! \brief calculate velocity and acceleration of a point on a link
 * with respect to the inertia frame.
 */
#define rkChainLinkPointVel(c,i,p,v) rkLinkPointVel( rkChainLink(c,i), p, v )
#define rkChainLinkPointAcc(c,i,p,a) rkLinkPointAcc( rkChainLink(c,i), p, a )

/*! \brief kinematic chain forward kinematics.
 *
 * rkChainUpdateFK() updates the frame of each link of the
 * kinematic chain \a c with resect to both the total body
 * frame and the world frame.
 *
 * rkChainFK() sets the joint displacement \a dis, and then
 * compute the forward kinematics of \a c with respect to the
 * world frame.
 * \return
 * All these functions return no velues.
 * \sa
 * rkChainUpdateFrame
 */
__ROKI_EXPORT void rkChainUpdateFK(rkChain *c);
__ROKI_EXPORT void rkChainFK(rkChain *c, zVec dis);

/*! \brief neutralize all joints of a kinematic chain. */
__ROKI_EXPORT void rkChainNeutralize(rkChain *chain);

/*! \brief inverse dynamics of kinematic chain.
 *
 * rkChainUpdateID() computes the inverse dynamics of a kinematic chain \a c
 * under an acceleration of field \a g by the Newton-Euler's method. It supposes
 * that the joint displacements, velocities, and accelerations of \a c are
 * updated in advance.
 * rkChainUpdateIDGravity() computes the inverse dynamics of \a c in the
 * gravitational field.
 * rkChainUpdateIDZeroGravity() computes the inverse dynamics of \a c in the
 * gravity-free field.
 *
 * rkChainID() computes the inverse dynamics of \a c, provided the joint
 * velocity \a vel, the acceleration \a acc, and an acceleration of field
 * \a g.
 * rkChainIDGravity() and rkChainIDZeroGravity() compute the inverse dynamics
 * of \a c in the gravitational field and the gravity-free field, respectively,
 * provided \a vel and \a acc.
 *
 * rkChainFKCNT() continuously updates the joint displacement for \a dis over
 * the time step \a dt, and then, computes the inverse dynamics in the graviational
 * field. All the joint velocities and accelerations of \a c will be updated
 * in accordance with a simple numerical differentiation.
 * \return
 * rkChainUpdateID(), rkChainUpdateIDGravity(), rkChainUpdateIDZeroGravity(),
 * rkChainID(), rkChainIDGravity(), rkChainIDZeroGravity(), and rkChainFKCNT()
 * do not return any values.
 */
__ROKI_EXPORT void rkChainUpdateID_G(rkChain *c, zVec6D *g);
#define rkChainUpdateID(c)     rkChainUpdateID_G( c, RK_GRAVITY6D )
#define rkChainUpdateID0G(c)   rkChainUpdateID_G( c, ZVEC6DZERO )
__ROKI_EXPORT void rkChainID_G(rkChain *c, zVec vel, zVec acc, zVec6D *g);
#define rkChainID(c,vel,acc)   rkChainID_G( c,vel,acc, RK_GRAVITY6D )
#define rkChainID0G(c,vel,acc) rkChainID_G( c,vel,acc, ZVEC6DZERO )
__ROKI_EXPORT void rkChainFKCNT(rkChain *c, zVec dis, double dt);

/*! \brief link acceleration at zero joint acceleration.
 *
 * rkChainLinkZeroAcc() computes 6D acceleration of a point \a p on the
 * \a id th link of a kinematic chain \a c at zero-joint acceleration.
 * This corresponds to the multiplication of the rate of Jacobian matrix and
 * the joint velocity vector.
 * \a g is an acceleration of the field.
 * The result is put into \a a0.
 *
 * rkChainLinkZeroAccGravity() and rkChainLinkZeroAccZeroGravity() compute
 * 6D acceleration of a point \a p on the \a id th link of \a c at zero-joint
 * acceleration.
 * The difference between rkChainLinkZeroAccGravity() and rkChainLinkZeroAccZeroGravity()
 * are that \a a0 includes the acceleration due to the gravity in the
 * former, while it does not in the latter.
 * For both functions, the result is put into \a a0.
 * \notes
 * rkChainLinkZeroAcc(),rkChainLinkZeroAccGravity(), and rkChainLinkZeroAccZeroGravity()
 * internally zero the joint acceleration of \a c.
 * \return
 * rkChainLinkZeroAcc(),rkChainLinkZeroAccGravity(), and rkChainLinkZeroAccZeroGravity()
 * return a pointer \a a0.
 */
__ROKI_EXPORT zVec6D *rkChainLinkZeroAccG(rkChain *c, int id, zVec3D *p, zVec6D *g, zVec6D *a0);
#define rkChainLinkZeroAcc(c,i,p,a0)   rkChainLinkZeroAccG( (c), (i), (p), RK_GRAVITY6D, (a0) )
#define rkChainLinkZeroAcc0G(c,i,p,a0) rkChainLinkZeroAccG( (c), (i), (p), ZVEC6DZERO, (a0) )

/*! \brief calculate the center of mass of kinematic chain.
 *
 * rkChainUpdateCOM() computes the center of mass of kinematic
 * chain \a c with respect to the world frame. The kinematics
 * should be calculated in advance.
 *
 * rkChainUpdateCOMVel() and rkChainUpdateCOMAcc() compute the
 * velocity and acceleration of the center of mass of \a c with
 * respect to the inertia frame, respectively. The motion rate
 * of the whole links should be updated in advance.
 * \return
 * These functions return a pointer to the internal 3D vector
 * which stores the position, velocity or acceleration of COM.
 * \notes
 * rkChainUpdateCOMAcc() includes the acceleration of gravity,
 * except one explicitly sets the zero gravity to the root link.
 */
__ROKI_EXPORT zVec3D *rkChainUpdateCOM(rkChain *c);
__ROKI_EXPORT zVec3D *rkChainUpdateCOMVel(rkChain *c);
__ROKI_EXPORT zVec3D *rkChainUpdateCOMAcc(rkChain *c);

/*! \brief update the composite rigid body of a kinematic chain. */
#define rkChainUpdateCRBMass(c) rkLinkUpdateCRBMass( rkChainRoot(c) )
#define rkChainUpdateCRB(c)     rkLinkUpdateCRB( rkChainRoot(c) )

/*! \brief zero moment point of kinematic chain.
 *
 * rkChainZMP() computes the Zero Moment Point(ZMP) proposed by
 * Vukobratovic et al.(1972) of the kinematic chain \a c with
 * respect to the world frame.
 *
 * \a z is the height of the VHP proposed by Sugihara et al.
 * (2002) in the direction of gravity.
 *
 * The result is put into \a zmp.
 *
 * rkChainYawTorque() computes the torque about the vertical axis
 * (parallel to the acceleration of gravity) on which ZMP exists.
 * \notes
 * In any cases of the three, inverse dynamics has to be computed
 * in advance.
 */
__ROKI_EXPORT zVec3D *rkChainZMP(rkChain *c, double z, zVec3D *zmp);
__ROKI_EXPORT double rkChainYawTorque(rkChain *c);

/*! \brief angular momentum and kinematic energy of kinematic chain.
 *
 * rkChainAM() calculates angular momentum of a kinematic chain
 * \a c around the point \a p with respect to the world frame.
 * The result is put into \a am.
 *
 * rkChainKE() calculates kinematic energy of \a c, originating
 * from linear and angular velocity of each link.
 * \return
 * rkChainAM() returns a pointer \a am.
 * rkChainKE() returns a value calculated.
 */
__ROKI_EXPORT zVec3D *rkChainAM(rkChain *c, zVec3D *p, zVec3D *am);
__ROKI_EXPORT double rkChainKE(rkChain *c);

/*! \brief inertia matrix and bias force vector of a kinematic chain.
 *
 * rkChainInertiaMat() computes the inertia matrix of a kinematic chain
 * \a chain at the current posture, and puts it into \a inertia.
 * It is in fact a virtual function that points one of the three actual
 * implementations, namely, rkChainInertiaMatBJ(), rkChainInertiaMatUV(),
 * or rkChainInertiaMatCRB()
 *
 * rkChainInertiaMatBJ() computes the inertia matrix by summing up
 * dyadic products of momentum Jacobian matrices, it is a naive but faster
 * implementation than rkChainInertiaMatUV().
 *
 * rkChainInertiaMatUV() and rkChainInertiaMatCRB() compute the inertia
 * matrix based on the unit vector method and the composite rigid body
 * method, respectively, which were proposed by Walker and Orin, 1980:
 *  M. W. Walker and D. E. Orin, Efficient Dynamic Computer Simulation of
 *  Robotic Mechanisms, Transactions of the ASME, Journal of Dynamic Systems,
 *  Measurement, and Control, Vol. 104, PP. 205-211, 1982.
 *
 * rkChainInertiaMatCRB() is the fastest method of the three, so that it
 * is preset to rkChainInertiaMat().
 *
 * rkChainBiasVec() computes the bias force vector, namely, summation of
 * the centrifugal force, Coriolis force, gravitational force and external
 * force applied to \a chain at the current posture and velocity, and puts
 * it into \a bias. It is also based on the unit vector method.
 *
 * rkChainInertiaMatBiasVec() computes the inertia matrix and the bias force
 * vector of \a chain at once, and puts them into \a inertia and \a bias,
 * respectively.
 * It is also a virtual function that points either rkChainInertiaMatBiasVecUV()
 * or rkChainInertiaMatBiasVecCRB(). The latter is preset.
 * \notes
 * \a chain has to take the posture and the velocity at which the dynamics
 * is computed in advance for rkChainInertiaMat() and rkChainBiasVec(),
 * respectively. The acceleration of \a chain is directly modified.
 *
 * All external wrenches exerted to \a c have to be removed before calling
 * rkChainInertiaMat().
 *
 * rkChainInertiaMatUV() internally zeros velocities and accelerations of
 * the whole links of \a c.
 * \return
 * rkChainInertiaMatBiasVec(), rkChainInertiaMat() and rkChainBiasVec() return
 * the true value if they succeed to compute the matrix and/or the vector.
 * If the sizes of the given matrix and vector do not match the total degree
 * of freedom of the chain, the false value is returned.
 */
__ROKI_EXPORT zMat rkChainInertiaMatMJ(rkChain *chain, zMat inertia);
__ROKI_EXPORT bool rkChainInertiaMatUV(rkChain *chain, zMat inertia);
__ROKI_EXPORT bool rkChainInertiaMatCRB(rkChain *chain, zMat inertia);
__ROKI_EXPORT bool (* rkChainInertiaMat)(rkChain*,zMat);

__ROKI_EXPORT bool rkChainBiasVec(rkChain *chain, zVec bias);
__ROKI_EXPORT bool rkChainInertiaMatBiasVecUV(rkChain *chain, zMat inertia, zVec bias);
__ROKI_EXPORT bool rkChainInertiaMatBiasVecCRB(rkChain *chain, zMat inertia, zVec bias);
__ROKI_EXPORT bool (* rkChainInertiaMatBiasVec)(rkChain*,zMat,zVec);

/*! \brief external force applied to kinematic chain.
 *
 * rkChainNetExtWrench() calculates the net external wrench acting to
 * a kinematic chain \a c by summing up individual external forces
 * applied to each link. Orientation of the total force is with respect
 * to the body frame of \a c.
 * The result is put into \a w.
 *
 * rkChainExtWrenchDestroy() destroys the external force list of the
 * kinematic chain \a c.
 * \return
 * rkChainNetExtWrench() returns a pointer \a w.
 *
 * rkChainExtWrenchDestroy() returns no value.
 */
__ROKI_EXPORT zVec6D *rkChainNetExtWrench(rkChain *c, zVec6D *w);
__ROKI_EXPORT void rkChainExtWrenchDestroy(rkChain *c);

/*! \brief set joint identifier offset value of each link.
 *
 * rkChainSetJointIDOffset() sets the joint identifier offset values of
 * all links of a kinematic chain model \a c. Each offset value corresponds
 * to the column offset of Jacobian matrices and the component offset of
 * joint displacement vectors.
 *
 * For links with fixed joints, the offset values of them are set for -1.
 *
 * For example, suppose the kinematic chain consists of a fixed base link,
 * a link with spherical joint and a link with revolutional joint, which
 * are connected in this order. rkChainSetJointIDOffset() sets the offsets
 * of each for -1, 0 and 4, respectively.
 * \return
 * rkChainSetJointIDOffset() returns no value.
 */
__ROKI_EXPORT void rkChainSetJointIDOffset(rkChain *c);

/*! \brief make a list of vertices of a chain.
 *
 * rkChainVertList() makes a list of vertices of a chain \a chain
 * with respect to the world coordinate frame. The result is put
 * into \a vl.
 * Non-polyhedral shapes of \a chain are internally converted to
 * polyhedra, whose vertices are added to \a vl.
 * \return
 * rkChainVertList() returns a pointer \a vl if succeeds. If it
 * fails to allocate memory to store vertices, the null pointer is
 * returned.
 */
__ROKI_EXPORT zVec3DList *rkChainVertList(rkChain *chain, zVec3DList *vl);

/*! \brief generate the bounding ball of a kinematic chain.
 *
 * rkChainBBall() generates the bounding ball of a kinematic chain
 * \a chain. The result is stored in \a bb.
 * \return
 * rkChainBBall() returns a pointer \a bb if succeeds. Otherwise,
 * the null pointer is returned.
 */
__ROKI_EXPORT zSphere3D *rkChainBBall(rkChain *chain, zSphere3D *bb);

/* ZTK */

#define ZTK_TAG_RKCHAIN "chain"
#define ZTK_TAG_INIT "init"

__ROKI_EXPORT rkChain *rkChainFromZTK(rkChain *chain, ZTK *ztk);
__ROKI_EXPORT void rkChainFPrintZTK(FILE *fp, rkChain *chain);

__ROKI_EXPORT rkChain *rkChainReadZTK(rkChain *chain, const char *filename);
__ROKI_EXPORT bool rkChainWriteZTK(rkChain *c, const char *filename);

__ROKI_EXPORT rkChain *rkChainInitFromZTK(rkChain *chain, ZTK *ztk);
__ROKI_EXPORT void rkChainInitFPrintZTK(FILE *fp, rkChain *chain);

__ROKI_EXPORT rkChain *rkChainInitReadZTK(rkChain *chain, const char *filename);
__ROKI_EXPORT bool rkChainInitWriteZTK(rkChain *chain, const char *filename);

__ROKI_EXPORT void rkChainPostureFPrint(FILE *fp, rkChain *c);
__ROKI_EXPORT void rkChainConnectionFPrint(FILE *fp, rkChain *c);
__ROKI_EXPORT void rkChainExtWrenchFPrint(FILE *fp, rkChain *c);

#define rkChainPosturePrint(c)    rkChainPostureFPrint( stdout, (c) )
#define rkChainConnectionPrint(c) rkChainConnectionFPrint( stdout, (c) )
#define rkChainExtWrenchPrint(c)  rkChainExtWrenchFPrint( stdout, (c) )

__END_DECLS

#include <roki/rk_ik.h>

#endif /* __RK_CHAIN_H__ */
