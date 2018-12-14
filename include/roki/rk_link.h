/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_link - link structure, kinematics and dynamics
 */

#ifndef __RK_LINK_H__
#define __RK_LINK_H__

#include <roki/rk_body.h>
#include <roki/rk_joint.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkLink
 * link class
 * ********************************************************** */
/* for ABI method */
typedef struct{
  zMat6D m; /* mass matrix */
  zMat6D i; /* ABI matrix */
  zVec6D f; /* bias force */
  zVec6D b; /* ABbias force */
  zVec6D c;
  rkWrenchList wlist; /* contact force list */
  /* workspace */
  zVec6D w;  /* net wrench except rigid contact forces */
  zVec6D w0; /* net wrench except contact forces */
  zVec6D b0; /* ABbios at no rigid contact forces */
  zVec6D a0; /* link acc at no rigid contact forces */
  bool abi_backward_path;
  /* joint inertia */
  zMat axi, iaxi;
} rkABIPrp;

typedef struct _rkLink{
  Z_NAMED_CLASS
  rkJoint joint;     /*!< \brief joint */
  rkBody body;       /*!< \brief rigid body */
  int offset;        /*!< \brief link identifier offset due to joint size */
  zFrame3D orgframe; /*!< \brief original adjacent transformation */
  zFrame3D adjframe; /*!< \brief adjacent transformation */
  struct _rkLink *parent; /*!< \brief a pointer to the parent link */
  struct _rkLink *child;  /*!< \brief a pointer to a child link */
  struct _rkLink *sibl;   /*!< \brief a pointer to a sibling link */
  /*! \cond */
  rkABIPrp _abiprp;  /* for ABI method */
  void *_util;  /* for utility */
  /* additional property */
  /* 1: constraint list for inverse kinematics
     2: visualization information
   */
  /*! \endcond */
} rkLink;

#define RK_LINK_INVALID (-1)

#define rkLinkOffset(l)        (l)->offset
#define rkLinkJoint(l)         ( &(l)->joint )
#define rkLinkJointType(l)     rkJointType( rkLinkJoint(l) )
#define rkLinkJointSize(l)     rkJointSize( rkLinkJoint(l) )
#define rkLinkBody(l)          ( &(l)->body )
#define rkLinkMP(l)            ( &rkLinkBody(l)->mp )
#define rkLinkMass(l)          rkBodyMass( rkLinkBody(l) )
#define rkLinkCOM(l)           rkBodyCOM( rkLinkBody(l) )
#define rkLinkInertia(l)       rkBodyInertia( rkLinkBody(l) )
#define rkLinkVel(l)           rkBodyVel( rkLinkBody(l) )
#define rkLinkAcc(l)           rkBodyAcc( rkLinkBody(l) )
#define rkLinkLinVel(l)        rkBodyLinVel( rkLinkBody(l) )
#define rkLinkLinAcc(l)        rkBodyLinAcc( rkLinkBody(l) )
#define rkLinkAngVel(l)        rkBodyAngVel( rkLinkBody(l) )
#define rkLinkAngAcc(l)        rkBodyAngAcc( rkLinkBody(l) )
#define rkLinkCOMVel(l)        rkBodyCOMVel( rkLinkBody(l) )
#define rkLinkCOMAcc(l)        rkBodyCOMAcc( rkLinkBody(l) )
#define rkLinkExtWrench(l)     rkBodyExtWrench( rkLinkBody(l) )
#define rkLinkShapeList(l)     rkBodyShapeList( rkLinkBody(l) )
#define rkLinkShapeNum(l)      rkBodyShapeNum( rkLinkBody(l) )
#define rkLinkShapeIsEmpty(l)  rkBodyShapeIsEmpty( rkLinkBody(l) )
#define rkLinkStuff(l)         rkBodyStuff( rkLinkBody(l) )
#define rkLinkOrgFrame(l)      ( &(l)->orgframe )
#define rkLinkOrgPos(l)        zFrame3DPos(rkLinkOrgFrame(l))
#define rkLinkOrgAtt(l)        zFrame3DAtt(rkLinkOrgFrame(l))
#define rkLinkAdjFrame(l)      ( &(l)->adjframe )
#define rkLinkAdjPos(l)        zFrame3DPos(rkLinkAdjFrame(l))
#define rkLinkAdjAtt(l)        zFrame3DAtt(rkLinkAdjFrame(l))
#define rkLinkWldFrame(l)      rkBodyFrame( rkLinkBody(l) )
#define rkLinkWldPos(l)        rkBodyPos( rkLinkBody(l) )
#define rkLinkWldAtt(l)        rkBodyAtt( rkLinkBody(l) )
#define rkLinkWldCOM(l)        rkBodyWldCOM( rkLinkBody(l) )
#define rkLinkWrench(l)        rkJointWrench( rkLinkJoint(l) )
#define rkLinkForce(l)         zVec6DLin(rkLinkWrench(l))
#define rkLinkTorque(l)        zVec6DAng(rkLinkWrench(l))
#define rkLinkParent(l)        (l)->parent
#define rkLinkChild(l)         (l)->child
#define rkLinkSibl(l)          (l)->sibl
#define rkLinkABIPrp(l)        ( &(l)->_abiprp )
#define rkLinkExtWrenchBuf(l)  ( &(l)->_abiprp.wlist )

#define rkLinkSetOffset(l,o)   ( rkLinkOffset(l) = (o) )
#define rkLinkSetMass(l,m)     rkBodySetMass( rkLinkBody(l), m )
#define rkLinkSetCOM(l,c)      rkBodySetCOM( rkLinkBody(l), c )
#define rkLinkSetInertia(l,i)  rkBodySetInertia( rkLinkBody(l), i )
#define rkLinkSetVel(l,v)      rkBodySetVel( rkLinkBody(l), v )
#define rkLinkSetAcc(l,a)      rkBodySetAcc( rkLinkBody(l), a )
#define rkLinkSetLinVel(l,v)   rkBodySetLinVel( rkLinkBody(l), v )
#define rkLinkSetLinAcc(l,a)   rkBodySetLinAcc( rkLinkBody(l), a )
#define rkLinkSetAngVel(l,v)   rkBodySetAngVel( rkLinkBody(l), v )
#define rkLinkSetAngAcc(l,a)   rkBodySetAngAcc( rkLinkBody(l), a )
#define rkLinkSetCOMVel(l,v)   rkBodySetCOMVel( rkLinkBody(l), v )
#define rkLinkSetCOMAcc(l,a)   rkBodySetCOMAcc( rkLinkBody(l), a )
#define rkLinkSetStuff(l,s)    rkBodySetStuff( rkLinkBody(l), s )
#define rkLinkSetOrgFrame(l,f) zFrame3DCopy( f, rkLinkOrgFrame(l) )
#define rkLinkSetOrgPos(l,p)   zFrame3DSetPos( rkLinkOrgFrame(l), p )
#define rkLinkSetOrgAtt(l,a)   zFrame3DSetAtt( rkLinkOrgFrame(l), a )
#define rkLinkSetAdjFrame(l,f) zFrame3DCopy( f, rkLinkAdjFrame(l) )
#define rkLinkSetAdjPos(l,p)   zFrame3DSetPos( rkLinkAdjFrame(l), p )
#define rkLinkSetAdjAtt(l,a)   zFrame3DSetAtt( rkLinkAdjFrame(l), a )
#define rkLinkSetWldFrame(l,f) rkBodySetFrame( rkLinkBody(l), f )
#define rkLinkSetWldPos(l,p)   rkBodySetPos( rkLinkBody(l), p )
#define rkLinkSetWldAtt(l,a)   rkBodySetAtt( rkLinkBody(l), a )
#define rkLinkSetWldCOM(l,c)   rkBodySetWldCOM( rkLinkBody(l), c )
#define rkLinkSetWrench(l,f)   zVec6DCopy( f, rkLinkWrench(l) )
#define rkLinkSetForce(l,f)    zVec3DCopy( f, rkLinkForce(l) )
#define rkLinkSetTorque(l,n)   zVec3DCopy( n, rkLinkTorque(l) )
#define rkLinkSetParent(l,p)   ( rkLinkParent(l) = (p) )
#define rkLinkSetChild(l,c)    ( rkLinkChild(l) = (c) )
#define rkLinkSetSibl(l,b)     ( rkLinkSibl(l) = (b) )

/*! \brief initialization and destruction of link object.
 *
 * rkLinkInit() initializes a link object \a l, cleaning up
 * all inner properties.
 *
 * rkLinkDestroy() destroys \a l, freeing the memory space
 * allocated for its name and extern force, and cleaning it up.
 * \return
 * Neither rkLinkInit() nor rkLinkDestroy() returns any values.
 */
__EXPORT void rkLinkInit(rkLink *l);
__EXPORT void rkLinkDestroy(rkLink *l);

__EXPORT rkLink *rkLinkClone(rkLink *org, rkLink *cln, zMShape3D *so, zMShape3D *sc);

__EXPORT rkLink *rkLinkCopyState(rkLink *src, rkLink *dst);

/*! \brief add link branch.
 *
 * rkLinkAddSibl() connects a link \a bl to the other \a l
 * as a sibling link, namely, a link which have the same
 * parent link with \a l.
 *
 * rkLinkAddChild() connects a link \a cl to the other \a l
 * as a child link. One link can have plural children.
 * When \a l already has more than one child, \a cl is
 * connected to the youngest link of them as its sibling link.
 * \return
 * Each of rkLinkAddSibl() and rkLinkAddChild() returns a
 * pointer to the added link.
 */
__EXPORT rkLink *rkLinkAddSibl(rkLink *l, rkLink *bl);
__EXPORT rkLink *rkLinkAddChild(rkLink *l, rkLink *cl);

/* \brief compute the inertial ellipsoid of a link.
 */
#define rkLinkInertiaEllips(l,e) rkBodyInertiaEllips( rkLinkBody(l), e )

/*! \brief push and pop of external force applied to link.
 *
 * rkLinkExtForcePush() pushes a new external force list cell
 * \a cell to the list on a link \a l.
 *
 * rkLinkExtForcePop() pops the latest external force list
 * cell from the list on \a l.
 *
 * rkLinkExtForceDelete() destroys the external force list
 * on \a l, freeing all cells.
 * \notes
 * When the external force list on \a l includes statically-allocated
 * cells, zListExtForceDelete() causes segmentation fault.
 * \return
 * rkLinkExtForcePush() returns a pointer to the cell pushed.
 * rkLinkExtForcePop() returns a pointer to the cell poped.
 * rkLinkExtForceDelete() returns no value.
 * \sa
 * rkBodyExtForcePush, rkBodyExtForcePop, rkBodyExtForceDelete
 */
#define rkLinkExtWrenchPush(l,f)  rkBodyExtWrenchPush( rkLinkBody(l), f )
#define rkLinkExtWrenchPop(l)     rkBodyExtWrenchPop( rkLinkBody(l) )
#define rkLinkExtWrenchDestroy(l) rkBodyExtWrenchDestroy( rkLinkBody(l) )

/*! \brief calculation of total 6D external force acting to link.
 *
 * rkLinkCalcExtForce6D() calculates the total 6D external
 * force equivalent to the sumation of forces contained in
 * the force list on a link \a l.
 * The result is put into \a f.
 * \return
 * rkLinkCalcExtForce6D() returns a pointer \a f.
 * \sa
 * rkBodyCalcExtForce6D
 */
#define rkLinkCalcExtWrench(l,w) rkBodyCalcExtWrench( rkLinkBody(l), w )

/*! \brief push and pop of shape attached to link.
 *
 * rkLinkShapePush() pushes a new shape \a shape to the
 * shape list of a link \a l.
 *
 * rkLinkShapePop() pops the last shape attached to \a l
 * from the list.
 *
 * rkLinkShapeDelete() destroys the shape list of \a l,
 * freeing all cells.
 * \notes
 * When the shape list of \a l includes statically-allocated
 * cells, rkLinkShapeDelete() causes segmentation fault.
 * \return
 * rkLinkShapePush() returns a pointer to the cell pushed.
 * rkLinkShapePop() returns a pointer to the shape poped.
 * rkLinkShapeDelete() returns no value.
 * \sa
 * rkBodyShapePush, rkBodyShapePop, rkBodyShapeDelete
 */
#define rkLinkShapePush(l,s)  rkBodyShapePush( rkLinkBody(l), s )
#define rkLinkShapePop(l)     rkBodyShapePop( rkLinkBody(l) )
#define rkLinkShapeDestroy(l) rkBodyShapeDestroy( rkLinkBody(l) )

#define rkLinkContigVert(l,p,d) rkBodyContigVert( rkLinkBody(l), p, d )

/*! \brief velocity of a point on link.
 *
 * rkLinkPointVel() calculates the velocity of a point \a p
 * attached to the local frame of a link \a l with respect
 * to the inertia frame. The result is put into \a v.
 * \return
 * rkLinkPointVel() returns a pointer \a v.
 * \notes
 * \a p is with respect to the local frame of \a l.
 */
__EXPORT zVec3D *rkLinkPointVel(rkLink *l, zVec3D *p, zVec3D *v);
__EXPORT zVec3D *rkLinkPointAcc(rkLink *l, zVec3D *p, zVec3D *a);

/*! \brief compute inertia tensor of a link with respect to the inertial frame.
 *
 * rkLinkWldInertia() computes the inertia tensor of a link \a l
 * with respect to the inertial frame.
 * The result is put where \a i points.
 * \return \a i
 */
__EXPORT zMat3D *rkLinkWldInertia(rkLink *l, zMat3D *i);

/*! \brief set and get joint displacement.
 *
 * rkLinkSetJointDis() sets the joint displacement of \a l
 * for \a dis. It automatically limits components of \a dis
 * which is out of motion range.
 *
 * rkLinkSetJointDisCNT() continuously updates the joint
 * displacement of \a l to \a dis over the time step \a dt.
 * Then, the velocity and acceleration is calculated in
 * accordance with a simple differentiation.
 *
 * rkLinkGetJointDis() gets the joint displacement of \a l
 * and puts it into \a dis.
 * \return
 * These functions return no value.
 */
#define rkLinkLimJointDis(l,td,ld)  rkJointLimDis( rkLinkJoint(l), td, ld )
#define rkLinkSetJointDis(l,d)      rkJointSetDis( rkLinkJoint(l), d )
#define rkLinkSetJointDisCNT(l,d,t) rkJointSetDisCNT( rkLinkJoint(l), d, t )
#define rkLinkGetJointDis(l,d)      rkJointGetDis( rkLinkJoint(l), d )
#define rkLinkGetJointVel(l,v)      rkJointGetVel( rkLinkJoint(l), v )
#define rkLinkGetJointAcc(l,a)      rkJointGetAcc( rkLinkJoint(l), a )
#define rkLinkGetJointMin(l,i)      rkJointGetMin( rkLinkJoint(l), i )
#define rkLinkGetJointMax(l,i)      rkJointGetMax( rkLinkJoint(l), i )
#define rkLinkGetJointTrq1(l,i)     rkJointGetTrq1( rkLinkJoint(l), i )
#define rkLinkGetJointTrq(l,d)      rkJointGetTrq( rkLinkJoint(l), d )

#define rkLinkGetJointMotor(l,m)    rkJointGetMotor( rkLinkJoint(l), m )

/*! \brief update link motion state.
 *
 * rkLinkUpdateFrame() updates the frame and COM of link \a l
 * with respect to the world frame.
 * The children and siblings of \a l are recursively updated.
 *
 * rkLinkUpdateRate() updates the velocity and acceleration
 * of \a l with respect to the inertia frame. Note that the
 * orientation of those velocity and acceleration are with
 * repect to the frame of \a l itself.
 *
 * rkLinkUpdateForce() updates the joint force of \a l. It
 * recursively computes the joint forces of the descendants
 * of \a l, accumulating them and subtracting them from the
 * net inertia force. Basically, it is an implementation of
 * Newton=Euler s method proposed by Luh, Walker and Paul(1980).
 * Note that the orientation of those force and torque are
 * with repect to the frame of \a l itself.
 * \return
 * All these functions return no values.
 * \notes
 * Before calling rkLinkUpdateForce(), the state of \a l
 * with respect to the world frame has to be updated by
 * rkLinkUpdateRate(). It is not simple in the case of
 * underactuated systems contacting with the environment.
 * Be careful when coding inverse dynamics. Use them in
 * correct ways, considering the purpose of the computation.
 */
__EXPORT void rkLinkUpdateFrame(rkLink *l, zFrame3D *pwf);
__EXPORT void rkLinkUpdateVel(rkLink *l, zVec6D *pvel);
__EXPORT void rkLinkUpdateAcc(rkLink *l, zVec6D *pvel, zVec6D *pacc);
__EXPORT void rkLinkUpdateRate(rkLink *l, zVec6D *pvel, zVec6D *pacc);
__EXPORT void rkLinkUpdateWrench(rkLink *l);

__EXPORT void rkLinkConfToJointDis(rkLink *l);

/*! \brief angular momentum and kinematic energy of link.
 *
 * rkLinkAM() calculates angular momentum of a link \a l
 * around the point \a p. The result will be stored into \a am.
 * Both \a p and \a am are with respect to the local frame of
 * \a l itself.
 *
 * rkLinkKE Energy() calculates kinematic energy originating
 * from linear and angular velocity of \a l.
 * \return
 * rkLinkAM() returns a pointer \a am.
 * rkLinkKE() returns a value calculated.
 * \sa
 * rkBodyAM, rkBodyKE
 */
#define rkLinkAM(l,p,m) rkBodyAM( rkLinkBody(l), (p), (m) )
#define rkLinkKE(l)     rkBodyKE( rkLinkBody(l) )

#define RK_LINK_TAG "link"

/*! \brief input and output link properties.
 *
 * rkLinkFRead() reads the properties of link \a l from
 * the current position of the file \a fp.
 * An acceptable keywords and their meanings are as follows.
 *
 *  name: <name>
 *   link name.
 *  jointtype: <type>
 *   joint type descriptor chosen from 'fix', 'revolute',
 *   'prism' and 'free'.
 *  max: <value>
 *  min: <value>
 *   maximum and minimum limit of joint angle, respectively.
 *  stiffness: <value>
 *  viscos: <value>
 *   joint stiffness and viscos friction coefficient, respectively.
 *  mass: <value>
 *   mass of link.
 *  COM: { <x>, <y>, <z> }
 *   centor of mass of link in the local frame.
 *  inertia: {
 *    <ixx>, <ixy>, <ixz>,
 *    <iyx>, <iyy>, <iyz>,
 *    <izx>, <izy>, <izz> }
 *   link inertia around COM in the local frame, which must be symmetric.
 *  frame: {
 *    <rxx>, <rxy>, <rxz>, <x>,
 *    <ryx>, <ryy>, <ryz>, <y>,
 *    <rzx>, <rzy>, <rzz>, <z> }
 *   transformation frame with respect to the parent link
 *   consisting of attitude matrix and original point vector.
 *  DH: { <a>, <alpha>, <d>, <theta> }
 *   Denavit=Hertenberg parameters(compatible with frame).
 *  parent: <name>
 *   the name of parent link.
 *  shape: <name>
 *   the name of shape(plural specification permitted).
 *
 * No need to complete all the above keys.
 * It will abort the operation in the following cases.
 * 1. it fails to allocate memory.
 * 2. link identifier is duplicated.
 * 3. non-existing shape name is specified.
 * 4. link name is not defined.
 *
 * rkLinkFWrite() writes out the properties of link \a l to
 * the current position of file \a fp in the above format.
 * rkLinkWrite() outputs the result to the standard output.
 *
 * rkLinkPostureFWrite() writes out the posture of link \a l
 * in the adjacent and world frames to the
 * current position of file \a fp.
 * rkLinkPostureWrite() writes out the result to the standard
 * output.
 *
 * rkLinkConnectionFWrite() writes out the connectivity of
 * link \a l to the current position of file \a fp.
 * \a n is a width of indent.
 * rkLinkConnectionWrite() writes out the connectivity to
 * the standard output.
 *
 * rkLinkExtForceFWrite() writes out the whole external force
 * applied to link \a l. See zForceListCellFWrite().
 * rkLinkExtForceWrite() writes them to the standard output.
 * \return
 * rkLinkFRead() returns a pointer \a l if succeed, or the
 * null pointer otherwise.
 *
 * rkLinkFWrite(), rkLinkWrite(), rkLinkPostureFWrite(),
 * rkLinkPostureWrite(), rkLinkConnectionFWrite(),
 * rkLinkConnectionWrite(), rkLinkExtForceFWrite() and
 * rkLinkExtForceWrite() return no values.
 */
__EXPORT rkLink *rkLinkFRead(FILE *fp, rkLink *l, rkLink *larray, int nl, zShape3D *sarray, int ns, rkMotor *marray, int nm);
__EXPORT void rkLinkFWrite(FILE *fp, rkLink *l);
__EXPORT void rkLinkPostureFWrite(FILE *fp, rkLink *l);
__EXPORT void rkLinkConnectionFWrite(FILE *fp, rkLink *l, int n);
__EXPORT void rkLinkExtWrenchFWrite(FILE *fp, rkLink *l);
#define rkLinkWrite(l)             rkLinkFWrite( stdout, (l) )
#define rkLinkPostureWrite(l)      rkLinkPostureFWrite( stdout, (l) )
#define rkLinkConnectionWrite(l,n) rkLinkConnectionFWrite( stdout, (l), (n) )
#define rkLinkExtWrenchWrite(l)     rkLinkExtWrenchFWrite( stdout, (l) )

__END_DECLS

#endif /* __ZDF_LINK_H__ */
