/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_link - link structure, kinematics and dynamics
 */

#ifndef __RK_LINK_H__
#define __RK_LINK_H__

#include <roki/rk_joint.h>

/* this is only for rkWrenchList in rkABIPrp, and to be removed. */
#include <roki/rk_force.h>

__BEGIN_DECLS

/* ********************************************************** */
/*! \struct rkLink
 * \brief link class
 * ********************************************************** */
/* for ABI method */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkABIPrp ){
  zMat6D m; /*!< \brief mass matrix */
  zMat6D i; /*!< \brief ABI matrix */
  zVec6D f; /*!< \brief bias force */
  zVec6D b; /*!< \brief ABbias force */
  zVec6D c;
  rkWrenchList wlist; /*!< \brief contact force list */
  /* workspace */
  zVec6D w;  /*!< \brief net wrench except rigid contact forces */
  zVec6D w0; /*!< \brief net wrench except contact forces */
  zVec6D b0; /*!< \brief ABbios at no rigid contact forces */
  zVec6D a0; /*!< \brief link acc at no rigid contact forces */
  bool abi_backward_path;
  zMat axi, iaxi; /*!< \brief joint inertial tensor */
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
  rkLink();
  ~rkLink();
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
#define rkLinkJointWrench(link)   rkJointWrench( rkLinkJoint(link) )
#define rkLinkJointForce(link)    zVec6DLin(rkLinkJointWrench(link))
#define rkLinkJointTorque(link)   zVec6DAng(rkLinkJointWrench(link))
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
#define rkLinkSetMP(link,mp)      do{\
  rkLinkSetMass( link, rkMPMass(mp) );\
  rkLinkSetCOM( link, rkMPCOM(mp) );\
  rkLinkSetInertia( link, rkMPInertia(mp) );\
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
#define rkLinkSetJointWrench(link,f) zVec6DCopy( f, rkLinkJointWrench(link) )
#define rkLinkSetJointForce(link,f)  zVec3DCopy( f, rkLinkJointForce(link) )
#define rkLinkSetJointTorque(link,n) zVec3DCopy( n, rkLinkJointTorque(link) )
#define rkLinkSetParent(link,p)   ( rkLinkParent(link) = (p) )
#define rkLinkSetChild(link,c)    ( rkLinkChild(link) = (c) )
#define rkLinkSetSibl(link,b)     ( rkLinkSibl(link) = (b) )

#define rkLinkJointDestroy(link)  rkJointDestroy( rkLinkJoint(link) )
#define rkLinkStuffDestroy(link)  rkBodyStuffDestroy( rkLinkBody(link) )

/*! \brief initialization and destruction of link object.
 *
 * rkLinkInit() initializes a link object \a link, cleaning up all inner properties.
 *
 * rkLinkDestroy() destroys \a link, freeing the memory space allocated for its name and external force,
 * and cleaning it up.
 * \return
 * Neither rkLinkInit() nor rkLinkDestroy() returns any values.
 */
__ROKI_EXPORT void rkLinkInit(rkLink *link);
__ROKI_EXPORT void rkLinkDestroy(rkLink *link);

/*! \brief clone a link.
 *
 * rkLinkClone() clones a link \a org, namely, copies its body and connectivities with other links, to
 * another \a cln.
 *
 * The multishapes associated with \a org and \a cln are pointed by \a shape_org and \a shape_cln,
 * respectively. It is supposed that the orders of the shapes of \a org and \a cln are the same in
 * \a shape_org and \a shape_cln. Namely, if the k-th shape of \a shape_org is referred from \a org,
 * the k-th shape of \a shape_cln is referred from \a cln.
 *
 * \a msarray_org and \a msarray_cln are arrays of motor specifications associated with \a org and \a cln,
 * respectively. The same condition about the order with the multishapes is assumed.
 * \return
 * rkLinkClone() returns \a cln if it succeeds. Otherwise, the null pointer is returned.
 */
__ROKI_EXPORT rkLink *rkLinkClone(rkLink *org, rkLink *cln, zMShape3D *shape_org, zMShape3D *shape_cln, rkMotorSpecArray *msarray_org, rkMotorSpecArray *msarray_cln);

/*! \brief zero velocity and acceleration of a link.
 *
 * rkLinkZeroRate() zeroes velocity and acceleration of a link.
 */
#define rkLinkZeroRate(link) rkBodyZeroRate( rkLinkBody(link) )

/*! \brief copy state of a link.
 *
 * rkLinkCopyState() copies state of a link \a src to that of another \a dest. The state includes frame,
 * velocity, acceleration, the position, velocity and acceleration of the center of mass, and the external
 * wrench.
 * \retval dest
 */
__ROKI_EXPORT rkLink *rkLinkCopyState(rkLink *src, rkLink *dest);

/*! \brief add link branch.
 *
 * rkLinkAddSibl() connects a link \a bl to the other \a link as a sibling link, namely, a link which have
 * the same parent link with \a link.
 *
 * rkLinkAddChild() connects a link \a cl to the other \a link as a child link. One link can have plural
 * children.
 * When \a link already has more than one child, \a cl is connected to the youngest link of them as its
 * sibling link.
 * \return
 * Each of rkLinkAddSibl() and rkLinkAddChild() returns a pointer to the added link.
 */
__ROKI_EXPORT rkLink *rkLinkAddSibl(rkLink *link, rkLink *sibl);
__ROKI_EXPORT rkLink *rkLinkAddChild(rkLink *link, rkLink *child);

/* \brief compute the inertial ellipsoid of a link.
 */
#define rkLinkInertiaEllips(link,e) rkBodyInertiaEllips( rkLinkBody(link), e )

/*! \brief set and add external wrench or force applied to a link.
 *
 * rkLinkSetExtWrench() sets the external wrench of a link \a link for \a wrench.
 * rkLinkAddExtWrench() adds the external wrench \a wrench to a link \a link.
 * rkLinkZeroExtWrench() zeroes the external wrench of a link \a link.
 *
 * rkLinkSetExtForce() sets the external force acting at \a pos of a link \a link for \a force.
 * rkLinkAddExtForce() adds the external force \a force acting at \a pos to a link \a link.
 * \return
 * rkLinkSetExtWrench(), rkLinkAddExtWrench(), rkLinkZeroExtWrench(), rkLinkSetExtForce(), and
 * rkLinkAddExtForce() are macros. See rk_link.h.
 * \sa
 * rkBodySetExtWrench, rkBodyAddExtWrench, rkBodyZeroExtWrench, rkBodySetExtForce, rkBodyAddExtForce
 */
#define rkLinkSetExtWrench(link,wrench)   rkBodySetExtWrench( rkLinkBody(link), wrench )
#define rkLinkAddExtWrench(link,wrench)   rkBodyAddExtWrench( rkLinkBody(link), wrench )
#define rkLinkZeroExtWrench(link)         rkBodyZeroExtWrench( rkLinkBody(link) )
#define rkLinkSetExtForce(link,force,pos) rkBodySetExtForce( rkLinkBody(link), force, pos )
#define rkLinkAddExtForce(link,force,pos) rkBodyAddExtForce( rkLinkBody(link), force, pos )

/*! \brief push and pop of shape attached to link.
 *
 * rkLinkShapePush() pushes a new shape \a shape to the shape list of a link \a link.
 *
 * rkLinkShapePop() pops the last shape attached to \a link from the list.
 *
 * rkLinkShapeDelete() destroys the shape list of \a link, freeing all cells.
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
 * rkLinkPointVel() calculates the velocity of a point \a p attached to the local frame of a link
 * \a link with respect to the inertia frame. The result is put into \a v.
 * \return
 * rkLinkPointVel() returns a pointer \a v.
 * \notes
 * \a p is with respect to the local frame of \a link.
 */
__ROKI_EXPORT zVec3D *rkLinkPointVel(const rkLink *link, const zVec3D *p, zVec3D *v);
__ROKI_EXPORT zVec3D *rkLinkPointAcc(const rkLink *link, const zVec3D *p, zVec3D *a);

/*! \brief compute inertia tensor of a link with respect to the inertial frame.
 *
 * rkLinkWldInertia() computes the inertia tensor of a link \a link with respect to the inertial frame.
 * The result is put where \a i points.
 * \retval \a inertia
 */
__ROKI_EXPORT zMat3D *rkLinkWldInertia(const rkLink *link, zMat3D *inertia);

/*! \brief set and get joint state of a link.
 *
 * rkLinkJointTestDis() tests if the given displacement \a testval is in the movable range of a joint
 * \a joint, and correct it to \a val, if necessary.
 *
 * rkLinkSetJointDis() sets the joint displacement of \a link for \a val. It automatically adjusts
 * components of \a val to be in the movable range.
 *
 * rkLinkSetJointDisCNT() continuously updates the joint displacement of \a link to \a val over the
 * time step \a dt, and then, calculates the velocity and acceleration approximately in accordance
 * with a simple differentiation.
 *
 * rkLinkGetJointDis() gets the joint displacement of \a link and puts it into \a val.
 * \return
 * These functions return no value.
 */
#define rkLinkJointTestDis(link,testval,val) rkJointTestDis( rkLinkJoint(link), testval, val )
#define rkLinkJointSetDis(link,val)          rkJointSetDis( rkLinkJoint(link), val )
#define rkLinkJointSetVel(link,val)          rkJointSetVel( rkLinkJoint(link), val )
#define rkLinkJointSetAcc(link,val)          rkJointSetAcc( rkLinkJoint(link), val )
#define rkLinkJointSetMin(link,val)          rkJointSetMin( rkLinkJoint(link), val )
#define rkLinkJointSetMax(link,val)          rkJointSetMax( rkLinkJoint(link), val )
#define rkLinkJointSetDisCNT(link,val,dt)    rkJointSetDisCNT( rkLinkJoint(link), val, dt )
#define rkLinkJointSetTrq(link,val)          rkJointSetTrq( rkLinkJoint(link), val )

#define rkLinkJointGetDis(link,val)          rkJointGetDis( rkLinkJoint(link), val )
#define rkLinkJointGetVel(link,val)          rkJointGetVel( rkLinkJoint(link), val )
#define rkLinkJointGetAcc(link,val)          rkJointGetAcc( rkLinkJoint(link), val )
#define rkLinkJointGetMin(link,val)          rkJointGetMin( rkLinkJoint(link), val )
#define rkLinkJointGetMax(link,val)          rkJointGetMax( rkLinkJoint(link), val )
#define rkLinkJointGetTrq1(link,i)           rkJointGetTrq1( rkLinkJoint(link), i )
#define rkLinkJointGetTrq(link,val)          rkJointGetTrq( rkLinkJoint(link), val )

#define rkLinkJointMotor(link)               rkJointMotor( rkLinkJoint(link) )
#define rkLinkJointMotorSetInput(link,input) rkJointMotorSetInput( rkLinkJoint(link), input )

#define rkLinkJointNeutralize(link)          rkJointNeutralize( rkLinkJoint(link) )

/*! \brief update link motion state.
 *
 * rkLinkUpdateFrame() updates the frame and COM of link \a link with respect to the world frame.
 * The children and siblings of \a link are recursively updated.
 *
 * rkLinkUpdateVel(), rkLinkUpdateAcc(), and rkLinkUpdateRate() updates the velocity and acceleration
 * of \a link with respect to the inertia frame. Note that the orientation of those velocity and
 * acceleration are with repect to the frame of \a link itself.
 *
 * rkLinkUpdateJointWrench() updates the joint wrench of \a link. It recursively computes the
 * joint wrenches of the descendants of \a link, accumulating them and subtracting them from the
 * net inertial wrench. Basically, it is an implementation of Newton=Euler s method proposed by Luh,
 * Walker and Paul(1980).
 * Note that the orientation of the wrench is with repect to the frame of \a link itself.
 * \return
 * All these functions return no values.
 */
__ROKI_EXPORT void rkLinkUpdateFrame(rkLink *link, const zFrame3D *pwf);
__ROKI_EXPORT void rkLinkUpdateVel(rkLink *link, const zVec6D *pvel);
__ROKI_EXPORT void rkLinkUpdateAcc(rkLink *link, const zVec6D *pvel, const zVec6D *pacc);
__ROKI_EXPORT void rkLinkUpdateRate(rkLink *link, const zVec6D *pvel, const zVec6D *pacc);
__ROKI_EXPORT void rkLinkUpdateJointWrench(rkLink *link);

/*! \brief update mass of the composite rigit body of a link. */
__ROKI_EXPORT double rkLinkUpdateCRBMass(rkLink *link);
/*! \brief update the composite rigit body of a link. */
__ROKI_EXPORT rkMP *rkLinkUpdateCRB(rkLink *link);

/*! \brief convert 6D configuration of a link to joint displacement. */
__ROKI_EXPORT void rkLinkConfToJointDis(rkLink *link);

/*! \brief linear / angular momentum and kinematic energy of a link.
 *
 * rkLinkLinearMomentum() calculates the linear momentum of a link \a link. The result is stored into
 * \a momentum. \a momentum is with respect to the local frame of \a link itself.
 *
 * rkLinkAngularMomentum() calculates the angular momentum of a link \a link about the point \a pos.
 * The result is stored into \a am.
 * Both \a pos and \a am are with respect to the local frame of \a link itself.
 *
 * rkLinkKineticEnergy Energy() calculates the kinetic energy originating from linear and angular
 * velocities of \a link.
 * \return
 * rkLinkLinearMomentum() returns a pointer \a momentum.
 * rkLinkAngularMomentum() returns a pointer \a am.
 * rkLinkKineticEnergy() returns a value calculated.
 * \sa
 * rkBodyAngularMomentum, rkBodyKineticEnergy
 */
#define rkLinkLinearMomentum(link,momentum) rkBodyLinearMomentum( rkLinkBody(link), momentum )
#define rkLinkAngularMomentum(link,pos,am)  rkBodyAngularMomentum( rkLinkBody(link), (pos), (am) )
#define rkLinkKineticEnergy(link)           rkBodyKineticEnergy( rkLinkBody(link) )

/*! \brief recursively computes linear / angular momentum of a link.
 *
 * rkLinkLinearMomentumRecursive() calculates the linear momentum of a link \a link in a recursive way.
 * The result, which is the same with rkLinkLinearMomentum(), is stored into \a momentum.
 *
 * rkLinkAngularMomentumRecursive() calculates the angular momentum about a point \a pos of a link
 * \a link in a recursive way. \a pos and \a am are with respect to the link frame. The result, which
 * is the same with rkLinkAngularMomentum(), is stored into \a am.
 */
__ROKI_EXPORT zVec3D *rkLinkLinearMomentumRecursive(const rkLink *link, zVec3D *momentum);
__ROKI_EXPORT zVec3D *rkLinkAngularMomentumRecursive(const rkLink *link, const zVec3D *pos, zVec3D *am);

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

#define ZTK_TAG_ROKI_LINK           "roki::link"

#define ZTK_KEY_ROKI_LINK_NAME      "name"
#define ZTK_KEY_ROKI_LINK_JOINTTYPE "jointtype"
#define ZTK_KEY_ROKI_LINK_MASS      "mass"
#define ZTK_KEY_ROKI_LINK_DENSITY   "density"
#define ZTK_KEY_ROKI_LINK_STUFF     "stuff"
#define ZTK_KEY_ROKI_LINK_COM       "COM"
#define ZTK_KEY_ROKI_LINK_INERTIA   "inertia"
#define ZTK_KEY_ROKI_LINK_POS       "pos"
#define ZTK_KEY_ROKI_LINK_ATT       "att"
#define ZTK_KEY_ROKI_LINK_ROT       "rot"
#define ZTK_KEY_ROKI_LINK_FRAME     "frame"
#define ZTK_KEY_ROKI_LINK_DH        "DH"
#define ZTK_KEY_ROKI_LINK_SHAPE     "shape"
#define ZTK_KEY_ROKI_LINK_PARENT    "parent"

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
 * \a n is a width of indentation.
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

/* print current 6D posture of a link out to a file. */
__ROKI_EXPORT void rkLinkPostureFPrint(FILE *fp, rkLink *link);
#define rkLinkPosturePrint(link)      rkLinkPostureFPrint( stdout, (link) )

/* print connectivity of a link of a kinematic chain out to a file. */
__ROKI_EXPORT void rkLinkConnectivityFPrint(FILE *fp, rkLink *link, rkLink *root, ulong branch_bit, int depth);
#define rkLinkConnectivityPrint(link,root,branch_bit,depth) rkLinkConnectivityFPrint( stdout, (link), (root), (branch_bit), (depth) )

/* print external wrench applied to a link out to a file. */
__ROKI_EXPORT void rkLinkExtWrenchFPrint(FILE *fp, rkLink *link);
#define rkLinkExtWrenchPrint(link)    rkLinkExtWrenchFPrint( stdout, (link) )

__END_DECLS

#ifdef __cplusplus
inline rkLink::rkLink(){ rkLinkInit( this ); }
inline rkLink::~rkLink(){ rkLinkDestroy( this ); }
#endif /* __cplusplus */

#endif /* __ZDF_LINK_H__ */
