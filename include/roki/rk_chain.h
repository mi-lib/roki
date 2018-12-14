/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_chain - kinematic chain, kinematics and dynamics
 */

#ifndef __RK_CHAIN_H__
#define __RK_CHAIN_H__

#include <roki/rk_link.h>

__BEGIN_DECLS

zArrayClass( rkLinkArray, rkLink );

/* ********************************************************** */
/* CLASS: rkChain
 * kinematic chain class
 * ********************************************************** */

typedef struct{
  Z_NAMED_CLASS
  rkLinkArray link;
  zMShape3D *shape;
  rkMotorArray *motor; /* may not need to be a pointer */

  double mass;
  zVec3D wldcom;
  zVec3D wldcomvel;
  zVec3D wldcomacc;

  bool _iscol;
} rkChain;

#define rkChainNum(c)              zArrayNum( &(c)->link )
#define rkChainRoot(c)             zArrayBuf( &(c)->link )
#define rkChainLink(c,i)           zArrayElem( &(c)->link, i )
#define rkChainShape(c)            (c)->shape
#define rkChainMotor(c)            (c)->motor
#define rkChainMass(c)             (c)->mass
#define rkChainWldCOM(c)           ( &(c)->wldcom )
#define rkChainCOMVel(c)           ( &(c)->wldcomvel )
#define rkChainCOMAcc(c)           ( &(c)->wldcomacc )

#define rkChainSetShape(c,s)       ( rkChainShape(c) = (s) )
#define rkChainSetMotor(c,m)       ( rkChainMotor(c) = (m) )
#define rkChainSetMass(c,m)        ( rkChainMass(c) = (m) )
#define rkChainSetWldCOM(c,p)      zVec3DCopy( p, rkChainWldCOM(c) )
#define rkChainSetCOMVel(c,v)      zVec3DCopy( v, rkChainCOMVel(c) )
#define rkChainSetCOMAcc(c,a)      zVec3DCopy( a, rkChainCOMAcc(c) )

#define rkChainLinkName(c,i)       zName(rkChainLink(c,i))
#define rkChainLinkOffset(c,i)     rkLinkOffset(rkChainLink(c,i))
#define rkChainLinkJoint(c,i)      rkLinkJoint(rkChainLink(c,i))
#define rkChainLinkJointType(c,i)  rkLinkJointType(rkChainLink(c,i))
#define rkChainLinkJointSize(c,i)  rkLinkJointSize(rkChainLink(c,i))
#define rkChainLinkMass(c,i)       rkLinkMass(rkChainLink(c,i))
#define rkChainLinkCOM(c,i)        rkLinkCOM(rkChainLink(c,i))
#define rkChainLinkInertia(c,i)    rkLinkInertia(rkChainLink(c,i))
#define rkChainLinkExtWrench(c,i)  rkLinkExtWrench(rkChainLink(c,i))
#define rkChainLinkShapeList(c,i)  rkLinkShapeList(rkChainLink(c,i))
#define rkChainLinkShapeNum(c,i)   rkLinkShapeNum(rkChainLink(c,i))
#define rkChainLinkOrgFrame(c,i)   rkLinkOrgFrame(rkChainLink(c,i))
#define rkChainLinkOrgPos(c,i)     rkLinkOrgPos(rkChainLink(c,i))
#define rkChainLinkOrgAtt(c,i)     rkLinkOrgAtt(rkChainLink(c,i))
#define rkChainLinkAdjFrame(c,i)   rkLinkAdjFrame(rkChainLink(c,i))
#define rkChainLinkAdjPos(c,i)     rkLinkAdjPos(rkChainLink(c,i))
#define rkChainLinkAdjAtt(c,i)     rkLinkAdjAtt(rkChainLink(c,i))
#define rkChainLinkWldFrame(c,i)   rkLinkWldFrame(rkChainLink(c,i))
#define rkChainLinkWldPos(c,i)     rkLinkWldPos(rkChainLink(c,i))
#define rkChainLinkWldAtt(c,i)     rkLinkWldAtt(rkChainLink(c,i))
#define rkChainLinkWldCOM(c,i)     rkLinkWldCOM(rkChainLink(c,i))
#define rkChainLinkVel(c,i)        rkLinkVel(rkChainLink(c,i))
#define rkChainLinkLinVel(c,i)     rkLinkLinVel(rkChainLink(c,i))
#define rkChainLinkLinAcc(c,i)     rkLinkLinAcc(rkChainLink(c,i))
#define rkChainLinkAngVel(c,i)     rkLinkAngVel(rkChainLink(c,i))
#define rkChainLinkAngAcc(c,i)     rkLinkAngAcc(rkChainLink(c,i))
#define rkChainLinkAcc(c,i)        rkLinkAcc(rkChainLink(c,i))
#define rkChainLinkCOMVel(c,i)     rkLinkCOMVel(rkChainLink(c,i))
#define rkChainLinkCOMAcc(c,i)     rkLinkCOMAcc(rkChainLink(c,i))
#define rkChainLinkWrench(c,i)     rkLinkWrench(rkChainLink(c,i))
#define rkChainLinkForce(c,i)      rkLinkForce(rkChainLink(c,i))
#define rkChainLinkTorque(c,i)     rkLinkTorque(rkChainLink(c,i))
#define rkChainLinkParent(c,i)     rkLinkParent(rkChainLink(c,i))
#define rkChainLinkChild(c,i)      rkLinkChild(rkChainLink(c,i))
#define rkChainLinkSibl(c,i)       rkLinkSibl(rkChainLink(c,i))

#define rkChainOrgFrame(c)         rkLinkOrgFrame(rkChainRoot(c))
#define rkChainOrgPos(c)           rkLinkOrgPos(rkChainRoot(c))
#define rkChainOrgAtt(c)           rkLinkOrgAtt(rkChainRoot(c))
#define rkChainRootFrame(c)        rkLinkWldFrame(rkChainRoot(c))
#define rkChainRootPos(c)          rkLinkWldPos(rkChainRoot(c))
#define rkChainRootAtt(c)          rkLinkWldAtt(rkChainRoot(c))
#define rkChainRootVel(c)          rkLinkVel(rkChainRoot(c))
#define rkChainRootAcc(c)          rkLinkAcc(rkChainRoot(c))
#define rkChainRootLinVel(c)       rkLinkLinVel(rkChainRoot(c))
#define rkChainRootLinAcc(c)       rkLinkLinAcc(rkChainRoot(c))
#define rkChainRootAngVel(c)       rkLinkAngVel(rkChainRoot(c))
#define rkChainRootAngAcc(c)       rkLinkAngAcc(rkChainRoot(c))
#define rkChainRootWrench(c)       rkLinkWrench(rkChainRoot(c))
#define rkChainRootForce(c)        rkLinkForce(rkChainRoot(c))
#define rkChainRootTorque(c)       rkLinkTorque(rkChainRoot(c))

#define rkChainSetRootFrame(c,f)   rkLinkSetWldFrame(rkChainRoot(c),f)
#define rkChainSetRootPos(c,p)     rkLinkSetWldPos(rkChainRoot(c),p)
#define rkChainSetRootAtt(c,m)     rkLinkSetWldAtt(rkChainRoot(c),m)
#define rkChainSetRootVel(c,v)     rkLinkSetVel(rkChainRoot(c),v)
#define rkChainSetRootAcc(c,a)     rkLinkSetAcc(rkChainRoot(c),a)
#define rkChainSetRootLinVel(c,v)  rkLinkSetLinVel(rkChainRoot(c),v)
#define rkChainSetRootLinAcc(c,a)  rkLinkSetLinAcc(rkChainRoot(c),a)
#define rkChainSetRootAngVel(c,o)  rkLinkSetAngVel(rkChainRoot(c),o)
#define rkChainSetRootAngAcc(c,a)  rkLinkSetAngAcc(rkChainRoot(c),a)
#define rkChainSetRootWrench(c,f)  rkLinkSetWrench(rkChainRoot(c),f)
#define rkChainSetRootForce(c,f)   rkLinkSetForce(rkChainRoot(c),f)
#define rkChainSetRootTorque(c,n)  rkLinkSetTorque(rkChainRoot(c),n)

/*! \brief initialize and destroy a kinematic chain.
 *
 * rkChainInit() initializes a kinematic chain instance pointed by \a c.
 *
 * rkChainDestroy() destroys all internal objects of kinematic chain \a c,
 * including the array of shapes and links.
 */
__EXPORT void rkChainInit(rkChain *c);
__EXPORT void rkChainDestroy(rkChain *c);

__EXPORT rkChain *rkChainClone(rkChain *org, rkChain *cln);

__EXPORT rkChain *rkChainCopyState(rkChain *src, rkChain *dst);

/*! \brief count total number of joints of a kinematic chain.
 *
 * rkChainJointSize() counts the total number of joints of a
 * kinematic chain \a c. How to count is conforming to rkJointSize.
 *
 * rkChainCreateDefaultJointIndex() creates a joint index of \a c,
 * which only arranges the movable joints.
 * The fixed joints are ignored.
 *
 * rkChainJointIndexSize() counts the total joint size indicated
 * by \a idx.
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
__EXPORT int rkChainJointSize(rkChain *c);
__EXPORT zIndex rkChainCreateDefaultJointIndex(rkChain *c);
__EXPORT int rkChainJointIndexSize(rkChain *c, zIndex idx);

/*! \brief update joint state.
 *
 * rkChainLinkSetJointDis() sets the joint displacement of the
 * \a i'th link of a kinematic chain \a c for \a dis.
 * Components of \a dis for a revolutional joint are in radian.
 * It automatically limits components of \a dis which is out of
 * motion range.
 *
 * rkChainLinkSetJointDisCNT() continuously updates the joint
 * displacement of the \a i'th link of \a c to \a dis over a
 * time step \a dt. Then, the joint velocity and acceleration
 * is calculated in accordance with a simple differentiation.
 *
 * rkChainLinkGetJointDis() gets the joint displacement of the
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
 * rkLinkSetJointDisCNT, rkChainCreateDefaultJointIndex
 */
#define rkChainLinkLimJointDis(r,i,td,ld)  rkLinkLimJointDis( rkChainLink(r,i), td, ld )
#define rkChainLinkSetJointDis(r,i,d)      rkLinkSetJointDis( rkChainLink(r,i), d )
#define rkChainLinkSetJointDisCNT(r,i,d,t) rkLinkSetJointDisCNT( rkChainLink(r,i), d, t )
#define rkChainLinkGetJointDis(r,i,d)      rkLinkGetJointDis( rkChainLink(r,i), d )
#define rkChainLinkGetJointVel(r,i,v)      rkLinkGetJointVel( rkChainLink(r,i), v )
#define rkChainLinkGetJointAcc(r,i,a)      rkLinkGetJointAcc( rkChainLink(r,i), a )
#define rkChainLinkGetJointTrq(r,i,d)      rkLinkGetJointTrq( rkChainLink(r,i), d )
#define rkChainLinkGetJointMotor(r,i,m)    rkLinkGetJointMotor( rkChainLink(r,i), m )

__EXPORT void rkChainSetJointDis(rkChain *c, zIndex idx, zVec dis);
__EXPORT void rkChainSetJointDisCNT(rkChain *c, zIndex idx, zVec dis, double dt);
__EXPORT void rkChainSetJointVel(rkChain *c, zIndex idx, zVec vel);
__EXPORT void rkChainSetJointRate(rkChain *c, zIndex idx, zVec vel, zVec acc);
__EXPORT zVec rkChainGetJointDis(rkChain *c, zIndex idx, zVec dis);

__EXPORT void rkChainSetJointDisAll(rkChain *c, zVec dis);
__EXPORT void rkChainCatJointDisAll(rkChain *c, zVec dis, double k, zVec v);
__EXPORT void rkChainSubJointDisAll(rkChain *c, zVec dis, zVec sdis);
__EXPORT void rkChainSetJointDisCNTAll(rkChain *c, zVec dis, double dt);
__EXPORT void rkChainSetJointVelAll(rkChain *c, zVec vel);
__EXPORT void rkChainSetJointRateAll(rkChain *c, zVec vel, zVec acc);
__EXPORT zVec rkChainGetJointDisAll(rkChain *c, zVec dis);
__EXPORT zVec rkChainGetJointVelAll(rkChain *c, zVec vel);
__EXPORT zVec rkChainGetJointAccAll(rkChain *c, zVec acc);
__EXPORT zVec rkChainGetJointTrqAll(rkChain *c, zVec trq);

__EXPORT zVec rkChainGetConf(rkChain *chain, zVec conf);
__EXPORT void rkChainSetConf(rkChain *chain, zVec conf);

/*! \brief update kinematic chain motion state.
 *
 * rkChainUpdateFrame() updates the whole link frame of the
 * kinematic chain \a r with respect to the world frame.
 *
 * rkChainUpdateRate() updates the motion rates, namely,
 * velocities and accelerations of the whole links of \a c
 * with respect to the inertia frame.
 *
 * rkChainUpdateForce() computes forces and moments acting
 * to the whole links of \a c.
 * \return
 * All of those functions return no values.
 * \sa
 * rkLinkUpdateFrame, rkLinkUpdateRate, rkLinkUpdateForce
 */
#define rkChainUpdateFrame(c)  \
  rkLinkUpdateFrame( rkChainRoot(c), ZFRAME3DIDENT )
#define rkChainUpdateVel(c)   \
  rkLinkUpdateVel( rkChainRoot(c), ZVEC6DZERO )
#define rkChainUpdateAcc(c)   \
  rkLinkUpdateAcc( rkChainRoot(c), ZVEC6DZERO, RK_GRAVITY6D )
#define rkChainUpdateRate(c)   \
  rkLinkUpdateRate( rkChainRoot(c), ZVEC6DZERO, RK_GRAVITY6D )
#define rkChainUpdateWrench(c) \
  rkLinkUpdateWrench( rkChainRoot(c) )

/*! \brief gravity orientation with respect to the root link.
 *
 * rkChainGravityDir() computes the direction vector of
 * gravity with respect to the total frame of kinematic
 * chain \a c, and store it into \a v.
 * \return
 * rkChainGravityDir() returns a pointer to the resultant
 * vector \a v.
 */
__EXPORT zVec3D *rkChainGravityDir(rkChain *c, zVec3D *v);

/*! \brief calculate velocity of a point on a link with respect
 *  to the inertia frame.
 */
#define rkChainLinkPointVel(c,i,p,v) rkLinkPointVel( rkChainLink(c,i), p, v )

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
__EXPORT void rkChainUpdateFK(rkChain *c);
__EXPORT void rkChainFK(rkChain *c, zVec dis);

/*! \brief inverse dynamics of kinematic chain.
 *
 * rkChainID() computes inverse dynamics of a kinematic chain
 * \a c in accordance with Newton-Euler's method.
 * Joint displacement, velocity, and acceleration of each link
 * should be set and posture of \a c should be updated before
 * calling this function.
 *
 * rkChainFKCNT() continuously updates the joint displacement
 * for \a dis over the time step \a dt, and then computs the
 * inverse dynamics.
 * All the joint velocity and accelerations will be updated in
 * accordance with a simple numerical differentiation.
 * \return
 * Neither rkChainID() nor rkChainFKCNT() return any values.
 */
__EXPORT void rkChainUpdateID(rkChain *c);
__EXPORT void rkChainID(rkChain *c, zVec vel, zVec acc);
__EXPORT void rkChainFKCNT(rkChain *c, zVec dis, double dt);

/*! \brief calculate the center of mass of kinematic chain.
 *
 * rkChainCalcCOM() computes the center of mass of kinematic
 * chain \a c with respect to the world frame. The kinematics
 * should be calculated in advance.
 *
 * rkChainCalcCOMVel() and rkChainCalcCOMAcc() compute the
 * velocity and acceleration of the center of mass of \a c with
 * respect to the inertia frame, respectively. The motion rate
 * of the whole links should be updated in advance.
 * \return
 * These functions return a pointer to the internal 3D vector
 * which stores the position, velocity or acceleration of COM.
 * \notes
 * rkChainCalcCOMAcc() includes the acceleration of gravity,
 * except one explicitly sets the zero gravity to the root link.
 */
__EXPORT zVec3D *rkChainCalcCOM(rkChain *c);
__EXPORT zVec3D *rkChainCalcCOMVel(rkChain *c);
__EXPORT zVec3D *rkChainCalcCOMAcc(rkChain *c);

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
__EXPORT zVec3D *rkChainZMP(rkChain *c, double z, zVec3D *zmp);
__EXPORT double rkChainYawTorque(rkChain *c);

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
__EXPORT zVec3D *rkChainAM(rkChain *c, zVec3D *p, zVec3D *am);
__EXPORT double rkChainKE(rkChain *c);

/*! \brief external force applied to kinematic chain.
 *
 * rkChainCalcExtForce6D() calculates the total external 6D force
 * applied to the kinematic chain \a c, the summation of the
 * external forces applied to each link of \a c.
 * Orientation of the total force is with respect to the total
 * frame of \a c.
 * The result is put into \a f.
 *
 * rkChainExtForceDestroy() destroys the external force list on
 * the whole links of the kinematic chain \a c, freeing all cells.
 * \return
 * rkChainCalcExtForce6D() returns a pointer \a f.
 *
 * rkChainExtForceDestroy() returns no value.
 */
__EXPORT zVec6D *rkChainCalcExtWrench(rkChain *c, zVec6D *w);
__EXPORT void rkChainExtWrenchDestroy(rkChain *c);

/*! \brief set offset value of each link.
 *
 * rkChainSetOffset() sets the offset values of all links of a
 * kinematic chain model \a c. Each offset value corresponds to
 * the column offset of Jacobian matrices and the component
 * offset of joint displacement vectors.
 *
 * For links with fixed joints, the offset values of them are
 * set for -1.
 *
 * For example, suppose the kinematic chain consists of a fixed
 * base link, a link with spherical joint and a link with
 * revolutional joint, which are connected in this order.
 * rkChainSetOffset() sets the offsets of each for -1, 0 and 4,
 * respectively.
 * \return
 * rkChainSetOffset() returns no value.
 */
__EXPORT void rkChainSetOffset(rkChain *c);

/*! \brief make a list of vertices of a chain.
 *
 * rkChain2VertList() makes a list of vertices of a chain \a chain
 * with respect to the world coordinate frame. The result is put
 * into \a vl.
 * \return
 * rkChain2VertList() returns a pointer \a vl if succeeds. If it
 * fails to allocate memory to store vertices, the null pointer is
 * returned.
 */
__EXPORT zVec3DList *rkChain2VertList(rkChain *chain, zVec3DList *vl);

#define RK_CHAIN_SUFFIX "zkc"
#define RK_CHAIN_INIT_SUFFIX "zkci"

__EXPORT bool rkChainReadFile(rkChain *c, char filename[]);
__EXPORT rkChain *rkChainFRead(FILE *fp, rkChain *c);
__EXPORT bool rkChainInitReadFile(rkChain *c, char filename[]);
__EXPORT rkChain *rkChainInitFRead(FILE *fp, rkChain *c);

/*! \brief read 3D multiple-shapes from a file and create a mono-link chain.
 *
 * rkChainMShape3DReadFile() reads a file \a filename which defines
 * 3D multiple-shapes and create a chain \a chain comprising a sole link.
 */
__EXPORT bool rkChainMShape3DReadFile(rkChain *chain, char filename[]);

__EXPORT bool rkChainWriteFile(rkChain *c, char filename[]);
__EXPORT void rkChainFWrite(FILE *fp, rkChain *c);
__EXPORT bool rkChainInitWriteFile(rkChain *c, char filename[]);
__EXPORT void rkChainInitFWrite(FILE *fp, rkChain *c);
__EXPORT void rkChainPostureFWrite(FILE *fp, rkChain *c);
__EXPORT void rkChainConnectionFWrite(FILE *fp, rkChain *c);
__EXPORT void rkChainExtWrenchFWrite(FILE *fp, rkChain *c);
#define rkChainWrite(c)           rkChainFWrite( stdout, (c) )
#define rkChainInitWrite(c)       rkChainInitFWrite( stdout, (c) )
#define rkChainPostureWrite(c)    rkChainPostureFWrite( stdout, (c) )
#define rkChainConnectionWrite(c) rkChainConnectionFWrite( stdout, (c) )
#define rkChainExtWrenchWrite(c)  rkChainExtWrenchFWrite( stdout, (c) )

__END_DECLS

#endif /* __RK_CHAIN_H__ */
