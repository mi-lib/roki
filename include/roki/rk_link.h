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

#define rkLinkJointIDOffset(link) (link)->joint_id_offset
#define rkLinkJoint(link)         ( &(link)->joint )
#define rkLinkJointDOF(link)      rkJointDOF( rkLinkJoint(link) )
#define rkLinkJointTypeStr(link)  rkJointTypeStr( rkLinkJoint(link) )
#define rkLinkBody(link)          ( &(link)->body )
#define rkLinkMP(link)            ( &rkLinkBody(link)->mp )
#define rkLinkMass(link)          rkBodyMass( rkLinkBody(link) )
#define rkLinkCOM(link)           rkBodyCOM( rkLinkBody(link) )
#define rkLinkInertia(link)       rkBodyInertia( rkLinkBody(link) )
#define rkLinkCRB(link)           ( &(link)->crb )
#define rkLinkCRBMass(link)       rkMPMass( rkLinkCRB(link) )
#define rkLinkCRBCOM(link)        rkMPCOM( rkLinkCRB(link) )
#define rkLinkCRBInertia(link)    rkMPInertia( rkLinkCRB(link) )
#define rkLinkVel(link)           rkBodyVel( rkLinkBody(link) )
#define rkLinkAcc(link)           rkBodyAcc( rkLinkBody(link) )
#define rkLinkLinVel(link)        rkBodyLinVel( rkLinkBody(link) )
#define rkLinkLinAcc(link)        rkBodyLinAcc( rkLinkBody(link) )
#define rkLinkAngVel(link)        rkBodyAngVel( rkLinkBody(link) )
#define rkLinkAngAcc(link)        rkBodyAngAcc( rkLinkBody(link) )
#define rkLinkCOMVel(link)        rkBodyCOMVel( rkLinkBody(link) )
#define rkLinkCOMAcc(link)        rkBodyCOMAcc( rkLinkBody(link) )
#define rkLinkExtWrench(link)     rkBodyExtWrench( rkLinkBody(link) )
#define rkLinkShapeList(link)     rkBodyShapeList( rkLinkBody(link) )
#define rkLinkShapeNum(link)      rkBodyShapeNum( rkLinkBody(link) )
#define rkLinkShapeIsEmpty(link)  rkBodyShapeIsEmpty( rkLinkBody(link) )
#define rkLinkStuff(link)         rkBodyStuff( rkLinkBody(link) )
#define rkLinkOrgFrame(link)      ( &(link)->orgframe )
#define rkLinkOrgPos(link)        zFrame3DPos(rkLinkOrgFrame(link))
#define rkLinkOrgAtt(link)        zFrame3DAtt(rkLinkOrgFrame(link))
#define rkLinkAdjFrame(link)      ( &(link)->adjframe )
#define rkLinkAdjPos(link)        zFrame3DPos(rkLinkAdjFrame(link))
#define rkLinkAdjAtt(link)        zFrame3DAtt(rkLinkAdjFrame(link))
#define rkLinkWldFrame(link)      rkBodyFrame( rkLinkBody(link) )
#define rkLinkWldPos(link)        rkBodyPos( rkLinkBody(link) )
#define rkLinkWldAtt(link)        rkBodyAtt( rkLinkBody(link) )
#define rkLinkWldCOM(link)        rkBodyWldCOM( rkLinkBody(link) )
#define rkLinkWrench(link)        rkJointWrench( rkLinkJoint(link) )
#define rkLinkForce(link)         zVec6DLin(rkLinkWrench(link))
#define rkLinkTorque(link)        zVec6DAng(rkLinkWrench(link))
#define rkLinkParent(link)        (link)->parent
#define rkLinkChild(link)         (link)->child
#define rkLinkSibl(link)          (link)->sibl
#define rkLinkABIPrp(link)        ( &(link)->_abiprp )
#define rkLinkExtWrenchBuf(link)  ( &(link)->_abiprp.wlist )

#define rkLinkSetJointIDOffset(link,o) ( rkLinkJointIDOffset(link) = (o) )
#define rkLinkSetMass(link,m)     do{\
  rkBodySetMass( rkLinkBody(link), m );\
  rkMPSetMass( rkLinkCRB(link), rkLinkMass(link) );\
} while(0)
#define rkLinkSetCOM(link,c)      do{\
  rkBodySetCOM( rkLinkBody(link), c );\
  rkMPSetCOM( rkLinkCRB(link), rkLinkCOM(link) );\
} while(0)
#define rkLinkSetInertia(link,i)  do{\
  rkBodySetInertia( rkLinkBody(link), i );\
  rkMPSetInertia( rkLinkCRB(link), rkLinkInertia(link) );\
} while(0)
#define rkLinkSetVel(link,v)      rkBodySetVel( rkLinkBody(link), v )
#define rkLinkSetAcc(link,a)      rkBodySetAcc( rkLinkBody(link), a )
#define rkLinkSetLinVel(link,v)   rkBodySetLinVel( rkLinkBody(link), v )
#define rkLinkSetLinAcc(link,a)   rkBodySetLinAcc( rkLinkBody(link), a )
#define rkLinkSetAngVel(link,v)   rkBodySetAngVel( rkLinkBody(link), v )
#define rkLinkSetAngAcc(link,a)   rkBodySetAngAcc( rkLinkBody(link), a )
#define rkLinkSetCOMVel(link,v)   rkBodySetCOMVel( rkLinkBody(link), v )
#define rkLinkSetCOMAcc(link,a)   rkBodySetCOMAcc( rkLinkBody(link), a )
#define rkLinkSetStuff(link,s)    rkBodySetStuff( rkLinkBody(link), s )
#define rkLinkSetOrgFrame(link,f) zFrame3DCopy( f, rkLinkOrgFrame(link) )
#define rkLinkSetOrgPos(link,p)   zFrame3DSetPos( rkLinkOrgFrame(link), p )
#define rkLinkSetOrgAtt(link,a)   zFrame3DSetAtt( rkLinkOrgFrame(link), a )
#define rkLinkSetAdjFrame(link,f) zFrame3DCopy( f, rkLinkAdjFrame(link) )
#define rkLinkSetAdjPos(link,p)   zFrame3DSetPos( rkLinkAdjFrame(link), p )
#define rkLinkSetAdjAtt(link,a)   zFrame3DSetAtt( rkLinkAdjFrame(link), a )
#define rkLinkSetWldFrame(link,f) rkBodySetFrame( rkLinkBody(link), f )
#define rkLinkSetWldPos(link,p)   rkBodySetPos( rkLinkBody(link), p )
#define rkLinkSetWldAtt(link,a)   rkBodySetAtt( rkLinkBody(link), a )
#define rkLinkSetWldCOM(link,c)   rkBodySetWldCOM( rkLinkBody(link), c )
#define rkLinkSetWrench(link,f)   zVec6DCopy( f, rkLinkWrench(link) )
#define rkLinkSetForce(link,f)    zVec3DCopy( f, rkLinkForce(link) )
#define rkLinkSetTorque(link,n)   zVec3DCopy( n, rkLinkTorque(link) )
#define rkLinkSetParent(link,p)   ( rkLinkParent(link) = (p) )
#define rkLinkSetChild(link,c)    ( rkLinkChild(link) = (c) )
#define rkLinkSetSibl(link,b)     ( rkLinkSibl(link) = (b) )

/*! \brief initialization and destruction of link object.
 *
 * rkLinkInit() initializes a link object \a link, cleaning up all inner properties.
 *
 * rkLinkDestroy() destroys \a link, freeing the memory space allocated for its name
 * and external force, and cleaning it up.
 * \return
 * Neither rkLinkInit() nor rkLinkDestroy() returns any values.
 */
__ROKI_EXPORT void rkLinkInit(rkLink *link);
__ROKI_EXPORT void rkLinkDestroy(rkLink *link);

/*! \brief clone a link.
 *
 * rkLinkClone() clones a link \a org, namely, copies its body and connectivities
 * with other links, to another \a cln.
 *
 * The multishapes associated with \a org and \a cln are pointed by \a shape_org
 * and \a shape_cln, respectively. It is supposed that the orders of the shapes
 * of \a org and \a cln are the same in \a shape_org and \a shape_cln. Namely,
 * if the k-th shape of \a shape_org is referred from \a org, the k-th shape
 * of \a shape_cln is referred from \a cln.
 *
 * \a msarray_org and \a msarray_cln are arrays of motor specifications associated
 * with \a org and \a cln, respectively. The same condition about the order with
 * the multishapes is assumed.
 * \return
 * rkLinkClone() returns \a cln if it succeeds. Otherwise, the null pointer is
 * returned.
 */
__ROKI_EXPORT rkLink *rkLinkClone(rkLink *org, rkLink *cln, zMShape3D *shape_org, zMShape3D *shape_cln, rkMotorSpecArray *msarray_org, rkMotorSpecArray *msarray_cln);

/*! \brief zero velocity and acceleration of a link.
 *
 * rkLinkZeroRate() zeroes velocity and acceleration of a link.
 */
#define rkLinkZeroRate(link) rkBodyZeroRate( rkLinkBody(link) )

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
 * rkLinkAddSibl() connects a link \a bl to the other \a link
 * as a sibling link, namely, a link which have the same
 * parent link with \a link.
 *
 * rkLinkAddChild() connects a link \a cl to the other \a link
 * as a child link. One link can have plural children.
 * When \a link already has more than one child, \a cl is
 * connected to the youngest link of them as its sibling link.
 * \return
 * Each of rkLinkAddSibl() and rkLinkAddChild() returns a
 * pointer to the added link.
 */
__ROKI_EXPORT rkLink *rkLinkAddSibl(rkLink *link, rkLink *sibl);
__ROKI_EXPORT rkLink *rkLinkAddChild(rkLink *link, rkLink *child);

/* \brief compute the inertial ellipsoid of a link.
 */
#define rkLinkInertiaEllips(link,e) rkBodyInertiaEllips( rkLinkBody(link), e )

/*! \brief push and pop of external force applied to link.
 *
 * rkLinkExtForcePush() pushes a new external force list cell
 * \a cell to the list on a link \a link.
 *
 * rkLinkExtForcePop() pops the latest external force list
 * cell from the list on \a link.
 *
 * rkLinkExtForceDelete() destroys the external force list
 * on \a link, freeing all cells.
 * \notes
 * When the external force list on \a link includes statically-allocated
 * cells, zListExtForceDelete() causes segmentation fault.
 * \return
 * rkLinkExtForcePush() returns a pointer to the cell pushed.
 * rkLinkExtForcePop() returns a pointer to the cell poped.
 * rkLinkExtForceDelete() returns no value.
 * \sa
 * rkBodyExtForcePush, rkBodyExtForcePop, rkBodyExtForceDelete
 */
#define rkLinkExtWrenchPush(link,f)  rkBodyExtWrenchPush( rkLinkBody(link), f )
#define rkLinkExtWrenchPop(link)     rkBodyExtWrenchPop( rkLinkBody(link) )
#define rkLinkExtWrenchDestroy(link) rkBodyExtWrenchDestroy( rkLinkBody(link) )

/*! \brief calculation of total external wrench acting to link.
 *
 * rkLinkNetExtWrench() calculates the net external wrench acting to
 * a link \a link by summing up individual external forces in the force
 * list. The result is put into \a w.
 * \return
 * rkLinkNetExtWrench() returns a pointer \a w.
 * \sa
 * rkBodyNetExtWrench
 */
#define rkLinkNetExtWrench(link,w) rkBodyNetExtWrench( rkLinkBody(link), w )

/*! \brief push and pop of shape attached to link.
 *
 * rkLinkShapePush() pushes a new shape \a shape to the
 * shape list of a link \a link.
 *
 * rkLinkShapePop() pops the last shape attached to \a link
 * from the list.
 *
 * rkLinkShapeDelete() destroys the shape list of \a link,
 * freeing all cells.
 * \notes
 * When the shape list of \a link includes statically-allocated
 * cells, rkLinkShapeDelete() causes segmentation fault.
 * \return
 * rkLinkShapePush() returns a pointer to the cell pushed.
 * rkLinkShapePop() returns a pointer to the shape poped.
 * rkLinkShapeDelete() returns no value.
 * \sa
 * rkBodyShapePush, rkBodyShapePop, rkBodyShapeDelete
 */
#define rkLinkShapePush(link,s)  rkBodyShapePush( rkLinkBody(link), s )
#define rkLinkShapePop(link)     rkBodyShapePop( rkLinkBody(link) )
#define rkLinkShapeDestroy(link) rkBodyShapeDestroy( rkLinkBody(link) )

#define rkLinkContigVert(link,p,d) rkBodyContigVert( rkLinkBody(link), p, d )

/*! \brief position of a point on a link in the world frame. */
#define rkLinkPointWldPos(link,p,pw) zXform3D( rkLinkWldFrame(link), p, pw )

/*! \brief velocity of a point on link.
 *
 * rkLinkPointVel() calculates the velocity of a point \a p
 * attached to the local frame of a link \a link with respect
 * to the inertia frame. The result is put into \a v.
 * \return
 * rkLinkPointVel() returns a pointer \a v.
 * \notes
 * \a p is with respect to the local frame of \a link.
 */
__ROKI_EXPORT zVec3D *rkLinkPointVel(rkLink *link, zVec3D *p, zVec3D *v);
__ROKI_EXPORT zVec3D *rkLinkPointAcc(rkLink *link, zVec3D *p, zVec3D *a);

/*! \brief compute inertia tensor of a link with respect to the inertial frame.
 *
 * rkLinkWldInertia() computes the inertia tensor of a link \a link
 * with respect to the inertial frame.
 * The result is put where \a i points.
 * \return \a i
 */
__ROKI_EXPORT zMat3D *rkLinkWldInertia(rkLink *link, zMat3D *i);

/*! \brief set and get joint displacement.
 *
 * rkLinkSetJointDis() sets the joint displacement of \a link
 * for \a dis. It automatically limits components of \a dis
 * which is out of motion range.
 *
 * rkLinkSetJointDisCNT() continuously updates the joint
 * displacement of \a link to \a dis over the time step \a dt.
 * Then, the velocity and acceleration is calculated in
 * accordance with a simple differentiation.
 *
 * rkLinkGetJointDis() gets the joint displacement of \a link
 * and puts it into \a dis.
 * \return
 * These functions return no value.
 */
#define rkLinkJointLimDis(link,td,ld)    rkJointLimDis( rkLinkJoint(link), td, ld )
#define rkLinkJointSetDis(link,d)        rkJointSetDis( rkLinkJoint(link), d )
#define rkLinkJointSetVel(link,v)        rkJointSetVel( rkLinkJoint(link), v )
#define rkLinkJointSetAcc(link,a)        rkJointSetAcc( rkLinkJoint(link), a )
#define rkLinkJointSetMin(link,m)        rkJointSetMin( rkLinkJoint(link), m )
#define rkLinkJointSetMax(link,m)        rkJointSetMax( rkLinkJoint(link), m )
#define rkLinkJointSetDisCNT(link,d,t)   rkJointSetDisCNT( rkLinkJoint(link), d, t )
#define rkLinkJointSetTrq(link,t)        rkJointSetTrq( rkLinkJoint(link), t )

#define rkLinkJointGetDis(link,d)        rkJointGetDis( rkLinkJoint(link), d )
#define rkLinkJointGetVel(link,v)        rkJointGetVel( rkLinkJoint(link), v )
#define rkLinkJointGetAcc(link,a)        rkJointGetAcc( rkLinkJoint(link), a )
#define rkLinkJointGetMin(link,m)        rkJointGetMin( rkLinkJoint(link), m )
#define rkLinkJointGetMax(link,m)        rkJointGetMax( rkLinkJoint(link), m )
#define rkLinkJointGetTrq1(link,i)       rkJointGetTrq1( rkLinkJoint(link), i )
#define rkLinkJointGetTrq(link,t)        rkJointGetTrq( rkLinkJoint(link), t )

#define rkLinkJointMotor(link)           rkJointMotor( rkLinkJoint(link) )
#define rkLinkJointMotorSetInput(link,t) rkJointMotorSetInput( rkLinkJoint(link), t )

#define rkLinkJointNeutralize(link)      rkJointNeutralize( rkLinkJoint(link) )

/*! \brief update link motion state.
 *
 * rkLinkUpdateFrame() updates the frame and COM of link \a link
 * with respect to the world frame.
 * The children and siblings of \a link are recursively updated.
 *
 * rkLinkUpdateRate() updates the velocity and acceleration
 * of \a link with respect to the inertia frame. Note that the
 * orientation of those velocity and acceleration are with
 * repect to the frame of \a link itself.
 *
 * rkLinkUpdateForce() updates the joint force of \a link. It
 * recursively computes the joint forces of the descendants
 * of \a link, accumulating them and subtracting them from the
 * net inertia force. Basically, it is an implementation of
 * Newton=Euler s method proposed by Luh, Walker and Paul(1980).
 * Note that the orientation of those force and torque are
 * with repect to the frame of \a link itself.
 * \return
 * All these functions return no values.
 * \notes
 * Before calling rkLinkUpdateForce(), the state of \a link
 * with respect to the world frame has to be updated by
 * rkLinkUpdateRate(). It is not simple in the case of
 * underactuated systems contacting with the environment.
 * Be careful when coding inverse dynamics. Use them in
 * correct ways, considering the purpose of the computation.
 */
__ROKI_EXPORT void rkLinkUpdateFrame(rkLink *link, zFrame3D *pwf);
__ROKI_EXPORT void rkLinkUpdateVel(rkLink *link, zVec6D *pvel);
__ROKI_EXPORT void rkLinkUpdateAcc(rkLink *link, zVec6D *pvel, zVec6D *pacc);
__ROKI_EXPORT void rkLinkUpdateRate(rkLink *link, zVec6D *pvel, zVec6D *pacc);
__ROKI_EXPORT void rkLinkUpdateWrench(rkLink *link);

/*! \brief update mass of the composite rigit body of a link. */
__ROKI_EXPORT double rkLinkUpdateCRBMass(rkLink *link);
/*! \brief update the composite rigit body of a link. */
__ROKI_EXPORT rkMP *rkLinkUpdateCRB(rkLink *link);

/*! \brief convert 6D configuration of a link to joint displacement. */
__ROKI_EXPORT void rkLinkConfToJointDis(rkLink *link);

/*! \brief angular momentum and kinematic energy of link.
 *
 * rkLinkAM() calculates angular momentum of a link \a link
 * around the point \a p. The result will be stored into \a am.
 * Both \a p and \a am are with respect to the local frame of
 * \a link itself.
 *
 * rkLinkKE Energy() calculates kinematic energy originating
 * from linear and angular velocity of \a link.
 * \return
 * rkLinkAM() returns a pointer \a am.
 * rkLinkKE() returns a value calculated.
 * \sa
 * rkBodyAM, rkBodyKE
 */
#define rkLinkAM(link,p,m) rkBodyAM( rkLinkBody(link), (p), (m) )
#define rkLinkKE(link)     rkBodyKE( rkLinkBody(link) )

/*! \brief compute volume of a link. */
#define rkLinkShapeVolume(link) rkBodyShapeVolume( rkLinkBody(link) )
/*! \brief compute mass property of a link. */
#define rkLinkShapeMP(link,d,mp) rkBodyShapeMP( rkLinkBody(link), (d), (mp) )

/* ********************************************************** */
/*! \struct rkLinkArray
 * \brief array of links
 * ********************************************************** */

zArrayClass( rkLinkArray, rkLink );

__ROKI_EXPORT rkLinkArray *rkLinkArrayAlloc(rkLinkArray *linkarray, int size);

__ROKI_EXPORT void rkLinkArrayDestroy(rkLinkArray *linkarray);

__ROKI_EXPORT rkLinkArray *rkLinkArrayClone(rkLinkArray *org, rkLinkArray *cln, zMShape3D *shape_org, zMShape3D *shape_cln, rkMotorSpecArray *msarray_org, rkMotorSpecArray *msarray_cln);

__ROKI_EXPORT void rkLinkArrayFPrintZTK(FILE *fp, rkLinkArray *linkarray);

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
 * rkLinkFPrintZTK() prints out properties of a link \a link to
 * the current position of a file \a fp in the above format.
 *
 * rkLinkPostureFPrint() prints out the posture of a link \a link
 * in the adjacent and world frames to the current position
 * of a file \a fp.
 * rkLinkPosturePrint() prints out the result to the standard
 * output.
 *
 * rkLinkConnectionFPrint() prints out the connectivity of
 * a link \a link to the current position of a file \a fp.
 * \a n is a width of indent.
 * rkLinkConnectionPrint() prints out the connectivity to
 * the standard output.
 *
 * rkLinkExtForceFPrint() prints out the whole external forces
 * applied to a link \a link. See zForceListCellFPrint().
 * rkLinkExtForcePrint() prints them out to the standard output.
 * \return
 * rkLinkFromZTK() returns a pointer \a link if succeeding, or the
 * null pointer otherwise.
 *
 * rkLinkFPrintZTK(), rkLinkPostureFPrint(),
 * rkLinkPosturePrint(), rkLinkConnectionFPrint(),
 * rkLinkConnectionPrint(), rkLinkExtForceFPrint() and
 * rkLinkExtForcePrint() return no values.
 */
__ROKI_EXPORT rkLink *rkLinkFromZTK(rkLink *link, rkLinkArray *larray, zShape3DArray *sarray, rkMotorSpecArray *motorspecarray, ZTK *ztk);
__ROKI_EXPORT rkLink *rkLinkConnectFromZTK(rkLink *link, rkLinkArray *larray, ZTK *ztk);

__ROKI_EXPORT void rkLinkFPrintZTK(FILE *fp, rkLink *link);

__ROKI_EXPORT void rkLinkPostureFPrint(FILE *fp, rkLink *link);
__ROKI_EXPORT void rkLinkConnectionFPrint(FILE *fp, rkLink *link, int n);
__ROKI_EXPORT void rkLinkExtWrenchFPrint(FILE *fp, rkLink *link);
#define rkLinkPosturePrint(link)      rkLinkPostureFPrint( stdout, (link) )
#define rkLinkConnectionPrint(link,n) rkLinkConnectionFPrint( stdout, (link), (n) )
#define rkLinkExtWrenchPrint(link)     rkLinkExtWrenchFPrint( stdout, (link) )

__END_DECLS

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

#endif /* __ZDF_LINK_H__ */
