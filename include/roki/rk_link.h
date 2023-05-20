/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_link - link structure, kinematics and dynamics
 */

#ifndef __RK_LINK_H__
#define __RK_LINK_H__

#include <roki/rk_joint.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkLink
 * link class
 * ********************************************************** */
/* for ABI method */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkABIPrp ){
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
  /* joint inertial tensor */
  zMat axi, iaxi;
};

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkLink ){
  Z_NAMED_CLASS;
  rkJoint joint;       /*!< \brief joint */
  rkBody body;         /*!< \brief rigid body */
  rkMP crb;            /*!< \brief composite rigid body */
  int joint_id_offset; /*!< \brief joint identifier offset due to joint size */
  zFrame3D orgframe;   /*!< \brief original adjacent transformation */
  zFrame3D adjframe;   /*!< \brief adjacent transformation */
  rkLink *parent;      /*!< \brief a pointer to the parent link */
  rkLink *child;       /*!< \brief a pointer to a child link */
  rkLink *sibl;        /*!< \brief a pointer to a sibling link */
  /*! \cond */
  rkABIPrp _abiprp;  /* for ABI method */
  /* additional property */
  /* 1: constraint list for inverse kinematics
     2: visualization information
   */
  /*! \endcond */
#ifdef __cplusplus
  rkJoint &Joint();
  rkBody &Body();
  int jointIDoffset() const;
  zFrame3D &orgFrame();
  zFrame3D &adjFrame();
  rkLink *Parent() const;
  rkLink *Child() const;
  rkLink *Sibl() const;
#endif /* __cplusplus */
};

#define RK_LINK_INVALID (-1)

#define rkLinkJointIDOffset(l) (l)->joint_id_offset
#define rkLinkJoint(l)         ( &(l)->joint )
#define rkLinkJointSize(l)     rkJointSize( rkLinkJoint(l) )
#define rkLinkJointTypeStr(l)  rkJointTypeStr( rkLinkJoint(l) )
#define rkLinkBody(l)          ( &(l)->body )
#define rkLinkCRB(l)           ( &(l)->crb )
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

#define rkLinkSetJointIDOffset(l,o) ( rkLinkJointIDOffset(l) = (o) )
#define rkLinkSetMass(l,m)     do{\
  rkBodySetMass( rkLinkBody(l), m );\
  rkMPSetMass( rkLinkCRB(l), rkLinkMass(l) );\
} while(0)
#define rkLinkSetCOM(l,c)      do{\
  rkBodySetCOM( rkLinkBody(l), c );\
  rkMPSetCOM( rkLinkCRB(l), rkLinkCOM(l) );\
} while(0)
#define rkLinkSetInertia(l,i)  do{\
  rkBodySetInertia( rkLinkBody(l), i );\
  rkMPSetInertia( rkLinkCRB(l), rkLinkInertia(l) );\
} while(0)
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
 * allocated for its name and external force, and cleaning it up.
 * \return
 * Neither rkLinkInit() nor rkLinkDestroy() returns any values.
 */
__ROKI_EXPORT void rkLinkInit(rkLink *l);
__ROKI_EXPORT void rkLinkDestroy(rkLink *l);

/*! \brief clone a link.
 *
 * rkLinkClone() clones a link \a org, namely, copies its body and
 * connectivities with other links, to another \a cln.
 *
 * The multishapes associated with \a org and \a cln are pointed by
 * \a so and \sc, respectively. It is supposed that the orders of the
 * shapes of \a org and \a cln are the same in \a so and \sc. Namely,
 * if the k-th shape of \a so is attached with \a org, the k-th shape
 * of \a sc is supposed to be attached with \a cln.
 * \return cln
 */
__ROKI_EXPORT rkLink *rkLinkClone(rkLink *org, rkLink *cln, zMShape3D *so, zMShape3D *sc);

/*! \brief zero velocity and acceleration of a link.
 *
 * rkLinkZeroRate() zeroes velocity and acceleration of a link.
 */
#define rkLinkZeroRate(l) rkBodyZeroRate( rkLinkBody(l) )

/*! \brief copy state of a link.
 *
 * rkLinkCopyState() copies state of a link \a src to that of another
 * \a dst. The state includes frame, velocity, acceleration, the
 * position, velocity and acceleration of the center of mass, and the
 * net external wrench.
 * \return dst
 */
__ROKI_EXPORT rkLink *rkLinkCopyState(rkLink *src, rkLink *dst);

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
__ROKI_EXPORT rkLink *rkLinkAddSibl(rkLink *l, rkLink *bl);
__ROKI_EXPORT rkLink *rkLinkAddChild(rkLink *l, rkLink *cl);

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

/*! \brief calculation of total external wrench acting to link.
 *
 * rkLinkNetExtWrench() calculates the net external wrench acting to
 * a link \a l by summing up individual external forces in the force
 * list. The result is put into \a w.
 * \return
 * rkLinkNetExtWrench() returns a pointer \a w.
 * \sa
 * rkBodyNetExtWrench
 */
#define rkLinkNetExtWrench(l,w) rkBodyNetExtWrench( rkLinkBody(l), w )

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

/*! \brief position of a point on a link in the world frame. */
#define rkLinkPointWldPos(l,p,pw) zXform3D( rkLinkWldFrame(l), p, pw )

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
__ROKI_EXPORT zVec3D *rkLinkPointVel(rkLink *l, zVec3D *p, zVec3D *v);
__ROKI_EXPORT zVec3D *rkLinkPointAcc(rkLink *l, zVec3D *p, zVec3D *a);

/*! \brief compute inertia tensor of a link with respect to the inertial frame.
 *
 * rkLinkWldInertia() computes the inertia tensor of a link \a l
 * with respect to the inertial frame.
 * The result is put where \a i points.
 * \return \a i
 */
__ROKI_EXPORT zMat3D *rkLinkWldInertia(rkLink *l, zMat3D *i);

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
#define rkLinkJointLimDis(l,td,ld)  rkJointLimDis( rkLinkJoint(l), td, ld )
#define rkLinkJointSetDis(l,d)      rkJointSetDis( rkLinkJoint(l), d )
#define rkLinkJointSetVel(l,v)      rkJointSetVel( rkLinkJoint(l), v )
#define rkLinkJointSetAcc(l,a)      rkJointSetAcc( rkLinkJoint(l), a )
#define rkLinkJointSetMin(l,m)      rkJointSetMin( rkLinkJoint(l), m )
#define rkLinkJointSetMax(l,m)      rkJointSetMax( rkLinkJoint(l), m )
#define rkLinkJointSetDisCNT(l,d,t) rkJointSetDisCNT( rkLinkJoint(l), d, t )
#define rkLinkJointSetTrq(l,t)      rkJointSetTrq( rkLinkJoint(l), t )

#define rkLinkJointGetDis(l,d)      rkJointGetDis( rkLinkJoint(l), d )
#define rkLinkJointGetVel(l,v)      rkJointGetVel( rkLinkJoint(l), v )
#define rkLinkJointGetAcc(l,a)      rkJointGetAcc( rkLinkJoint(l), a )
#define rkLinkJointGetMin(l,m)      rkJointGetMin( rkLinkJoint(l), m )
#define rkLinkJointGetMax(l,m)      rkJointGetMax( rkLinkJoint(l), m )
#define rkLinkJointGetTrq1(l,i)     rkJointGetTrq1( rkLinkJoint(l), i )
#define rkLinkJointGetTrq(l,t)      rkJointGetTrq( rkLinkJoint(l), t )

#define rkLinkJointGetMotor(l,m)      rkJointGetMotor( rkLinkJoint(l), m )
#define rkLinkJointMotorSetInput(l,t) rkJointMotorSetInput( rkLinkJoint(l), t )

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
__ROKI_EXPORT void rkLinkUpdateFrame(rkLink *l, zFrame3D *pwf);
__ROKI_EXPORT void rkLinkUpdateVel(rkLink *l, zVec6D *pvel);
__ROKI_EXPORT void rkLinkUpdateAcc(rkLink *l, zVec6D *pvel, zVec6D *pacc);
__ROKI_EXPORT void rkLinkUpdateRate(rkLink *l, zVec6D *pvel, zVec6D *pacc);
__ROKI_EXPORT void rkLinkUpdateWrench(rkLink *l);

/*! \brief update mass of the composite rigit body of a link. */
__ROKI_EXPORT double rkLinkUpdateCRBMass(rkLink *link);
/*! \brief update the composite rigit body of a link. */
__ROKI_EXPORT rkMP *rkLinkUpdateCRB(rkLink *link);

__ROKI_EXPORT void rkLinkConfToJointDis(rkLink *l);

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

/*! \brief compute volume of a link. */
#define rkLinkShapeVolume(l) rkBodyShapeVolume( rkLinkBody(l) )
/*! \brief compute mass property of a link. */
#define rkLinkShapeMP(l,d,mp) rkBodyShapeMP( rkLinkBody(l), (d), (mp) )

/* ********************************************************** */
/*! \struct rkLinkArray
 * \brief array of links
 * ********************************************************** */

zArrayClass( rkLinkArray, rkLink );

/* ***** ZTK ***** */

#define ZTK_TAG_RKLINK "link"

/*! \brief read link properties from a ZTK file and print link properties.
 *
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
 * rkLinkFPrintZTK() prints out properties of a link \a l to
 * the current position of a file \a fp in the above format.
 *
 * rkLinkPostureFPrint() prints out the posture of a link \a l
 * in the adjacent and world frames to the current position
 * of a file \a fp.
 * rkLinkPosturePrint() prints out the result to the standard
 * output.
 *
 * rkLinkConnectionFPrint() prints out the connectivity of
 * a link \a l to the current position of a file \a fp.
 * \a n is a width of indent.
 * rkLinkConnectionPrint() prints out the connectivity to
 * the standard output.
 *
 * rkLinkExtForceFPrint() prints out the whole external forces
 * applied to a link \a l. See zForceListCellFPrint().
 * rkLinkExtForcePrint() prints them out to the standard output.
 * \return
 * rkLinkFromZTK() returns a pointer \a l if succeeding, or the
 * null pointer otherwise.
 *
 * rkLinkFPrintZTK(), rkLinkPostureFPrint(),
 * rkLinkPosturePrint(), rkLinkConnectionFPrint(),
 * rkLinkConnectionPrint(), rkLinkExtForceFPrint() and
 * rkLinkExtForcePrint() return no values.
 */
__ROKI_EXPORT rkLink *rkLinkFromZTK(rkLink *link, rkLinkArray *larray, zShape3DArray *sarray, rkMotorArray *motorarray, ZTK *ztk);
__ROKI_EXPORT rkLink *rkLinkConnectFromZTK(rkLink *link, rkLinkArray *larray, ZTK *ztk);

__ROKI_EXPORT void rkLinkFPrintZTK(FILE *fp, rkLink *l);

__ROKI_EXPORT void rkLinkPostureFPrint(FILE *fp, rkLink *l);
__ROKI_EXPORT void rkLinkConnectionFPrint(FILE *fp, rkLink *l, int n);
__ROKI_EXPORT void rkLinkExtWrenchFPrint(FILE *fp, rkLink *l);
#define rkLinkPosturePrint(l)      rkLinkPostureFPrint( stdout, (l) )
#define rkLinkConnectionPrint(l,n) rkLinkConnectionFPrint( stdout, (l), (n) )
#define rkLinkExtWrenchPrint(l)     rkLinkExtWrenchFPrint( stdout, (l) )

#ifdef __cplusplus
inline rkJoint &rkLink::Joint(){ return joint; }
inline rkBody &rkLink::Body(){ return body; }
inline int rkLink::jointIDoffset() const { return joint_id_offset; }
inline zFrame3D &rkLink::orgFrame(){ return orgframe; }
inline zFrame3D &rkLink::adjFrame(){ return adjframe; }
inline rkLink *rkLink::Parent() const { return parent; }
inline rkLink *rkLink::Child() const { return child; }
inline rkLink *rkLink::Sibl() const { return sibl; }
#endif /* __cplusplus */

__END_DECLS

#endif /* __ZDF_LINK_H__ */
