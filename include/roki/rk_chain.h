/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_chain - kinematic chain, kinematics and dynamics
 */

#ifndef __RK_CHAIN_H__
#define __RK_CHAIN_H__

#include <roki/rk_link.h>
#include <roki/rk_ik_cell.h>

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
#ifdef __cplusplus
  int getLinkNum() const;
  rkLink *link(int i) const;
  rkLink *root() const;
  zVec3D *COM();
  zVec3D *COMVel();
  zVec3D *COMAcc();
  double mass() const;
  zFrame3D *orgFrame() const;
  zVec3D *orgPos() const;
  zMat3D *orgAtt() const;
  zFrame3D *rootFrame() const;
  zVec3D *rootPos() const;
  zMat3D *rootAtt() const;
  zVec6D *rootVel() const;
  zVec3D *rootLinVel() const;
  zVec3D *rootAngVel() const;
  zVec6D *rootAcc() const;
  zVec3D *rootLinAcc() const;
  zVec3D *rootAngAcc() const;
  zVec6D *rootWrench() const;
  zVec3D *rootForce() const;
  zVec3D *rootTorque() const;

  rkChain();
  ~rkChain();
  void init();
  void destroy();
  rkChain *clone(rkChain *dest);
  rkChain *copyState(rkChain *dest);
  int jointSize();
  zIndex createDefaultJointIndex();
  int jointIndexSize(zIndex idx);
  rkLink *findLink(const char *name);
  int findLinkID(const char *name);
  int findLinkJointIDOffset(const char *name);
  void setJointDis(zIndex idx, zVec dis);
  void setJointDis(zVec dis);
  void setJointDisCNT(zIndex idx, zVec dis, double dt);
  void setJointDisCNT(zVec dis, double dt);
  void setJointVel(zIndex idx, zVec vel);
  void setJointVel(zVec vel);
  void setJointAcc(zIndex idx, zVec acc);
  void setJointAcc(zVec acc);
  void setJointRate(zIndex idx, zVec vel, zVec acc);
  void setJointRate(zVec vel, zVec acc);
  void setJointTrq(zVec trq);
  zVec getJointDis(zIndex idx, zVec dis);
  zVec getJointDis(zVec dis);
  zVec getJointVel(zVec vel);
  zVec getJointAcc(zVec acc);
  zVec getJointTrq(zVec trq);

  void setConf(zVec conf);
  zVec getConf(zVec conf);
  void setMotorInput(zVec input);

  void updateFrame();
  void updateVel();
  void updateAcc();
  void updateRateG(zVec6D *g);
  void updateRate();
  void updateRate0G();
  void updateWrench();

  zVec3D *gravityDir(zVec3D *v);
  zVec3D *linkPointPos(int i, zVec3D *p, zVec3D *world_p);
  zVec3D *linkPointVel(int i, zVec3D *p, zVec3D *vel);
  zVec3D *linkPointAcc(int i, zVec3D *p, zVec3D *acc);
  void updateForwardKinematics();
  void ForwardKinematics(zVec dis);
  void neutralize();
  void updateInverseDynamicsG(zVec6D *g);
  void updateInverseDynamics();
  void updateInverseDynamics0G();
  void InverseDynamicsG(zVec vel, zVec acc, zVec6D *g);
  void InverseDynamics(zVec vel, zVec acc);
  void InverseDynamics0G(zVec vel, zVec acc);
  void ForwardKinematicsCNT(zVec dis, double dt);

  zVec6D *linkZeroAccG(int id, zVec3D *p, zVec6D *g, zVec6D *a0);
  zVec6D *linkZeroAcc(int id, zVec3D *p, zVec6D *a0);
  zVec6D *linkZeroAcc0G(int id, zVec3D *p, zVec6D *a0);

  zVec3D *updateCOM();
  zVec3D *updateCOMVel();
  zVec3D *updateCOMAcc();

  void updateCRBMass();
  void updateCRB();
  zVec3D *ZMP(double z, zVec3D *zmp);
  double yawTorque();
  zVec3D *angularMomentum(zVec3D *p, zVec3D *am);
  double kineticEnergy();

  zMat getInertiaMat(zMat inertia);
  zVec getBiasVec(zVec bias);
  bool getInertiaMatBiasVec(zMat inertia, zVec bias);
  zVec6D *netExternalWrench(zVec6D *wrench);
  void destroyExternalWrench();

  zSphere3D *getBoundingBall(zSphere3D *bb);

  rkChain *fromZTK(ZTK *ztk);
  void fprintZTK(FILE *fp);
  rkChain *readZTK(const char *filename);
  bool writeZTK(const char *filename);

  zMat InverseKinematicsConstraintMat();
  zVec InverseKinematicsConstraintVec();
  zIndex InverseKinematicsJointIndex();
  rkChain *createInverseKinematics();
  void destroyInverseKinematics();
  bool registerInverseKinematicsJointID(int id, double weight);
  bool unregisterInverseKinematicsJointID(int id);
  bool registerInverseKinematicsJoint(const char *name, double weight);
  bool unregisterInverseKinematicsJoint(const char *name);
  bool registerInverseKinematicsJointAll(double weight);
  rkIKCell *registerInverseKinematicsCell(const char *name, int priority, rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util);
  rkIKCell *registerInverseKinematicsCell(int priority, rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util);
  rkIKCell *registerInverseKinematicsCell(rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util);
  bool unregisterInverseKinematicsCell(rkIKCell *cell);
  rkIKCell *registerInverseKinematicsCellWorldPos(const char *name, int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellWorldPos(int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellWorldPos(rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellWorldAtt(const char *name, int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellWorldAtt(int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellWorldAtt(rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellLinkToLinkPos(const char *name, int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellLinkToLinkPos(int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellLinkToLinkPos(rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellLinkToLinkAtt(const char *name, int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellLinkToLinkAtt(int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellLinkToLinkAtt(rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellCOM(const char *name, int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellCOM(int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellCOM(rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellAngularMomentum(const char *name, int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellAngularMomentum(int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellAngularMomentum(rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellAngularMomentumCOM(const char *name, int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellAngularMomentumCOM(int priority, rkIKAttr *attr, ubyte mask);
  rkIKCell *registerInverseKinematicsCellAngularMomentumCOM(rkIKAttr *attr, ubyte mask);
  rkIKCell *findInverseKinematicsByName(const char *name);
  bool setCellPriority(rkIKCell *cell, int priority);
  void disableInverseKinematics();
  void bindInverseKinematics();
  void resetInverseKinematicsAccumulator();
  int InverseKinematics(zVec dis, double tol, int iter);
  int InverseKinematicsRJO(zVec dis, double tol, int iter);
#endif /* __cplusplus */
};

#define rkChainLinkArray(chain)             ( &(chain)->linkarray )
#define rkChainRoot(chain)                  zArrayBuf( rkChainLinkArray(chain) )
#define rkChainLink(chain,i)                zArrayElemNC( rkChainLinkArray(chain), i )
#define rkChainLinkNum(chain)               zArraySize( rkChainLinkArray(chain) )
#define rkChainShape(chain)                 (chain)->shape
#define rkChainMotorSpecArray(chain)        ( &(chain)->motorspecarray )
#define rkChainWldCOM(chain)                ( &(chain)->wldcom )
#define rkChainCOMVel(chain)                ( &(chain)->wldcomvel )
#define rkChainCOMAcc(chain)                ( &(chain)->wldcomacc )
#define rkChainMass(chain)                  rkLinkCRBMass( rkChainRoot(chain) )

#define rkChainSetShape(chain,s)            ( rkChainShape(chain) = (s) )
#define rkChainSetMass(chain,m)             ( rkChainMass(chain) = (m) )
#define rkChainSetWldCOM(chain,p)           zVec3DCopy( p, rkChainWldCOM(chain) )
#define rkChainSetCOMVel(chain,v)           zVec3DCopy( v, rkChainCOMVel(chain) )
#define rkChainSetCOMAcc(chain,a)           zVec3DCopy( a, rkChainCOMAcc(chain) )

#define rkChainLinkName(chain,i)            zName(rkChainLink(chain,i))
#define rkChainLinkJointIDOffset(chain,i)   rkLinkJointIDOffset(rkChainLink(chain,i))
#define rkChainLinkJoint(chain,i)           rkLinkJoint(rkChainLink(chain,i))
#define rkChainLinkJointDOF(chain,i)        rkLinkJointDOF(rkChainLink(chain,i))
#define rkChainLinkJointTypeStr(chain,i)    rkLinkJointTypeStr(rkChainLink(chain,i))
#define rkChainLinkMass(chain,i)            rkLinkMass(rkChainLink(chain,i))
#define rkChainLinkCOM(chain,i)             rkLinkCOM(rkChainLink(chain,i))
#define rkChainLinkInertia(chain,i)         rkLinkInertia(rkChainLink(chain,i))
#define rkChainLinkCRB(chain,i)             rkLinkCRB(rkChainLink(chain,i))
#define rkChainLinkCRBMass(chain,i)         rkLinkCRBMass(rkChainLink(chain,i))
#define rkChainLinkCRBCOM(chain,i)          rkLinkCRBCOM(rkChainLink(chain,i))
#define rkChainLinkCRBInertia(chain,i)      rkLinkCRBInertia(rkChainLink(chain,i))
#define rkChainLinkExtWrench(chain,i)       rkLinkExtWrench(rkChainLink(chain,i))
#define rkChainLinkShapeList(chain,i)       rkLinkShapeList(rkChainLink(chain,i))
#define rkChainLinkShapeNum(chain,i)        rkLinkShapeNum(rkChainLink(chain,i))
#define rkChainLinkOrgFrame(chain,i)        rkLinkOrgFrame(rkChainLink(chain,i))
#define rkChainLinkOrgPos(chain,i)          rkLinkOrgPos(rkChainLink(chain,i))
#define rkChainLinkOrgAtt(chain,i)          rkLinkOrgAtt(rkChainLink(chain,i))
#define rkChainLinkAdjFrame(chain,i)        rkLinkAdjFrame(rkChainLink(chain,i))
#define rkChainLinkAdjPos(chain,i)          rkLinkAdjPos(rkChainLink(chain,i))
#define rkChainLinkAdjAtt(chain,i)          rkLinkAdjAtt(rkChainLink(chain,i))
#define rkChainLinkWldFrame(chain,i)        rkLinkWldFrame(rkChainLink(chain,i))
#define rkChainLinkWldPos(chain,i)          rkLinkWldPos(rkChainLink(chain,i))
#define rkChainLinkWldAtt(chain,i)          rkLinkWldAtt(rkChainLink(chain,i))
#define rkChainLinkWldCOM(chain,i)          rkLinkWldCOM(rkChainLink(chain,i))
#define rkChainLinkVel(chain,i)             rkLinkVel(rkChainLink(chain,i))
#define rkChainLinkLinVel(chain,i)          rkLinkLinVel(rkChainLink(chain,i))
#define rkChainLinkLinAcc(chain,i)          rkLinkLinAcc(rkChainLink(chain,i))
#define rkChainLinkAngVel(chain,i)          rkLinkAngVel(rkChainLink(chain,i))
#define rkChainLinkAngAcc(chain,i)          rkLinkAngAcc(rkChainLink(chain,i))
#define rkChainLinkAcc(chain,i)             rkLinkAcc(rkChainLink(chain,i))
#define rkChainLinkCOMVel(chain,i)          rkLinkCOMVel(rkChainLink(chain,i))
#define rkChainLinkCOMAcc(chain,i)          rkLinkCOMAcc(rkChainLink(chain,i))
#define rkChainLinkWrench(chain,i)          rkLinkWrench(rkChainLink(chain,i))
#define rkChainLinkForce(chain,i)           rkLinkForce(rkChainLink(chain,i))
#define rkChainLinkTorque(chain,i)          rkLinkTorque(rkChainLink(chain,i))
#define rkChainLinkParent(chain,i)          rkLinkParent(rkChainLink(chain,i))
#define rkChainLinkChild(chain,i)           rkLinkChild(rkChainLink(chain,i))
#define rkChainLinkSibl(chain,i)            rkLinkSibl(rkChainLink(chain,i))

#define rkChainLinkJointNeutralize(chain,i) rkLinkJointNeutralize( rkChainLink(chain,i) )

#define rkChainOrgFrame(chain)              rkLinkOrgFrame(rkChainRoot(chain))
#define rkChainOrgPos(chain)                rkLinkOrgPos(rkChainRoot(chain))
#define rkChainOrgAtt(chain)                rkLinkOrgAtt(rkChainRoot(chain))
#define rkChainRootFrame(chain)             rkLinkWldFrame(rkChainRoot(chain))
#define rkChainRootPos(chain)               rkLinkWldPos(rkChainRoot(chain))
#define rkChainRootAtt(chain)               rkLinkWldAtt(rkChainRoot(chain))
#define rkChainRootVel(chain)               rkLinkVel(rkChainRoot(chain))
#define rkChainRootAcc(chain)               rkLinkAcc(rkChainRoot(chain))
#define rkChainRootLinVel(chain)            rkLinkLinVel(rkChainRoot(chain))
#define rkChainRootLinAcc(chain)            rkLinkLinAcc(rkChainRoot(chain))
#define rkChainRootAngVel(chain)            rkLinkAngVel(rkChainRoot(chain))
#define rkChainRootAngAcc(chain)            rkLinkAngAcc(rkChainRoot(chain))
#define rkChainRootWrench(chain)            rkLinkWrench(rkChainRoot(chain))
#define rkChainRootForce(chain)             rkLinkForce(rkChainRoot(chain))
#define rkChainRootTorque(chain)            rkLinkTorque(rkChainRoot(chain))

#define rkChainSetRootFrame(chain,f)        rkLinkSetWldFrame(rkChainRoot(chain),f)
#define rkChainSetRootPos(chain,p)          rkLinkSetWldPos(rkChainRoot(chain),p)
#define rkChainSetRootAtt(chain,m)          rkLinkSetWldAtt(rkChainRoot(chain),m)
#define rkChainSetRootVel(chain,v)          rkLinkSetVel(rkChainRoot(chain),v)
#define rkChainSetRootAcc(chain,a)          rkLinkSetAcc(rkChainRoot(chain),a)
#define rkChainSetRootLinVel(chain,v)       rkLinkSetLinVel(rkChainRoot(chain),v)
#define rkChainSetRootLinAcc(chain,a)       rkLinkSetLinAcc(rkChainRoot(chain),a)
#define rkChainSetRootAngVel(chain,o)       rkLinkSetAngVel(rkChainRoot(chain),o)
#define rkChainSetRootAngAcc(chain,a)       rkLinkSetAngAcc(rkChainRoot(chain),a)
#define rkChainSetRootWrench(chain,f)       rkLinkSetWrench(rkChainRoot(chain),f)
#define rkChainSetRootForce(chain,f)        rkLinkSetForce(rkChainRoot(chain),f)
#define rkChainSetRootTorque(chain,n)       rkLinkSetTorque(rkChainRoot(chain),n)

/*! \brief initialize and destroy a kinematic chain.
 *
 * rkChainInit() initializes a kinematic chain instance pointed by \a chain.
 *
 * rkChainDestroy() destroys all internal objects of kinematic chain \a chain,
 * including the array of shapes and links.
 */
__ROKI_EXPORT void rkChainInit(rkChain *chain);
__ROKI_EXPORT void rkChainDestroy(rkChain *chain);

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
 * chain \a chain. How to count is conforming to rkJointDOF().
 *
 * rkChainCreateDefaultJointIndex() creates a joint index of \a chain,
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
__ROKI_EXPORT int rkChainJointSize(rkChain *chain);
__ROKI_EXPORT zIndex rkChainCreateDefaultJointIndex(rkChain *chain);
__ROKI_EXPORT int rkChainJointIndexSize(rkChain *chain, zIndex idx);

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
 * \a i'th link of a kinematic chain \a chain for \a dis.
 * Components of \a dis for a revolutional joint are in radian.
 * It automatically limits components of \a dis which is out of
 * motion range.
 *
 * rkChainLinkJointSetDisCNT() continuously updates the joint
 * displacement of the \a i'th link of \a chain to \a dis over a
 * time step \a dt. Then, the joint velocity and acceleration
 * is calculated in accordance with a simple differentiation.
 *
 * rkChainLinkJointGetDis() gets the joint displacement of the
 * \a i'th link of \a chain and puts it into \a dis.
 *
 * rkChainSetJointDis() sets all the joint displacements of
 * \a chain specified by a joint index \a index for \a dis.
 * Components corresponding to revolutional joints have to be
 * in radian. Values which are out of the motion range range of
 * the corresponding joints will be automatically limited.
 *
 * rkChainSetJointDisCNT() continuously updates the joint
 * displacements of \a chain specified by \a index to \a dis over
 * a time step \a dt. The joint velocity and acceleration is
 * calculated in accordance with a simple differentiation.
 *
 * rkChainSetJointRate() sets joint velocities and accelerations
 * specified by \a index of \a chain for \a vel and \a acc,
 * respectively.
 *
 * rkChainGetJointDis() gets joint displacements of \a chain and
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
#define rkChainLinkJointLimDis(chain,i,td,ld)  rkLinkJointLimDis( rkChainLink(chain,i), td, ld )
#define rkChainLinkJointSetDis(chain,i,d)      rkLinkJointSetDis( rkChainLink(chain,i), d )
#define rkChainLinkJointSetDisCNT(chain,i,d,t) rkLinkJointSetDisCNT( rkChainLink(chain,i), d, t )
#define rkChainLinkJointSetVel(chain,i,v)      rkLinkJointSetVel( rkChainLink(chain,i), v )
#define rkChainLinkJointSetAcc(chain,i,a)      rkLinkJointSetAcc( rkChainLink(chain,i), a )
#define rkChainLinkJointSetTrq(chain,i,t)      rkLinkJointSetTrq( rkChainLink(chain,i), t )

#define rkChainLinkJointGetDis(chain,i,d)      rkLinkJointGetDis( rkChainLink(chain,i), d )
#define rkChainLinkJointGetVel(chain,i,v)      rkLinkJointGetVel( rkChainLink(chain,i), v )
#define rkChainLinkJointGetAcc(chain,i,a)      rkLinkJointGetAcc( rkChainLink(chain,i), a )
#define rkChainLinkJointGetTrq(chain,i,t)      rkLinkJointGetTrq( rkChainLink(chain,i), t )

#define rkChainLinkJointGetMotor(chain,i,m)    rkLinkJointGetMotor( rkChainLink(chain,i), m )

#define rkChainLinkJointMotorSetInput(chain,i,t)  rkLinkJointMotorSetInput( rkChainLink(chain,i), t )

__ROKI_EXPORT void rkChainSetJointDis(rkChain *chain, zIndex idx, zVec dis);
__ROKI_EXPORT void rkChainSetJointDisCNT(rkChain *chain, zIndex idx, zVec dis, double dt);
__ROKI_EXPORT void rkChainSetJointVel(rkChain *chain, zIndex idx, zVec vel);
__ROKI_EXPORT void rkChainSetJointAcc(rkChain *chain, zIndex idx, zVec acc);
__ROKI_EXPORT void rkChainSetJointRate(rkChain *chain, zIndex idx, zVec vel, zVec acc);
__ROKI_EXPORT zVec rkChainGetJointDis(rkChain *chain, zIndex idx, zVec dis);

__ROKI_EXPORT void rkChainSetJointDisAll(rkChain *chain, zVec dis);
__ROKI_EXPORT void rkChainCatJointDisAll(rkChain *chain, zVec dis, double k, zVec v);
__ROKI_EXPORT void rkChainSubJointDisAll(rkChain *chain, zVec dis, zVec sdis);
__ROKI_EXPORT void rkChainSetJointDisCNTAll(rkChain *chain, zVec dis, double dt);
__ROKI_EXPORT void rkChainSetJointVelAll(rkChain *chain, zVec vel);
__ROKI_EXPORT void rkChainSetJointAccAll(rkChain *chain, zVec acc);
__ROKI_EXPORT void rkChainSetJointRateAll(rkChain *chain, zVec vel, zVec acc);
__ROKI_EXPORT void rkChainSetJointTrqAll(rkChain *chain, zVec trq);

__ROKI_EXPORT zVec rkChainGetJointDisAll(rkChain *chain, zVec dis);
__ROKI_EXPORT zVec rkChainGetJointVelAll(rkChain *chain, zVec vel);
__ROKI_EXPORT zVec rkChainGetJointAccAll(rkChain *chain, zVec acc);
__ROKI_EXPORT zVec rkChainGetJointTrqAll(rkChain *chain, zVec trq);

__ROKI_EXPORT void rkChainSetConf(rkChain *chain, zVec conf);
__ROKI_EXPORT zVec rkChainGetConf(rkChain *chain, zVec conf);

__ROKI_EXPORT void rkChainSetMotorInputAll(rkChain *chain, zVec input);

/*! \brief update kinematic chain motion state.
 *
 * rkChainUpdateFrame() updates the whole link frames of a kinematic chain
 * \a chain with respect to the world frame.
 *
 * rkChainUpdateVel() and rkChainUpdateAcc() update velocities and accelerations
 * of the whole link frames of \a chain with respect to the inertia frame, respectively.
 *
 * rkChainUpdateRate() updates the rate, namely, the velocities and the accelerations
 * of the whole links of \a chain with respect to the frame that has an acceleration
 * of the field \a g.
 * rkChainUpdateRateGravity() updates the rate of the whole links of \a chain in
 * the gravitational field.
 * rkChainUpdateRateZeroGravity() updates the rate of the whole links of \a chain
 * in the gravity-free field.
 *
 * rkChainUpdateWrench() computes wrenches, namely, combinations of force and
 * torque acting at the original points of the whole links of \a chain.
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
#define rkChainUpdateFrame(chain)   rkLinkUpdateFrame( rkChainRoot(chain), ZFRAME3DIDENT )
#define rkChainUpdateVel(chain)     rkLinkUpdateVel( rkChainRoot(chain), ZVEC6DZERO )
#define rkChainUpdateAcc(chain)     rkLinkUpdateAcc( rkChainRoot(chain), ZVEC6DZERO, RK_GRAVITY6D )
#define rkChainUpdateRateG(chain,g) rkLinkUpdateRate( rkChainRoot(chain), ZVEC6DZERO, (g) )
#define rkChainUpdateRate(chain)    rkChainUpdateRateG( chain, RK_GRAVITY6D )
#define rkChainUpdateRate0G(chain)  rkChainUpdateRateG( chain, ZVEC6DZERO )
#define rkChainUpdateWrench(chain)  rkLinkUpdateWrench( rkChainRoot(chain) )

/*! \brief direction vector of gravity with respect to the body frame of a kinematic chain.
 *
 * rkChainGravityDir() computes the direction vector of gravity with respect to
 * the total frame of kinematic chain \a chain, and store it into \a v.
 * \return
 * rkChainGravityDir() returns a pointer to the resultant vector \a v.
 */
#define rkChainGravityDir(chain,v) zMat3DRow( rkChainRootAtt(chain), zZ, v )

/*! \brief position of a point on a link of a kinematic chain in the world frame. */
#define rkChainLinkPointWldPos(chain,i,p,pw) rkLinkPointWldPos( rkChainLink(chain,i), p, pw )

/*! \brief calculate velocity and acceleration of a point on a link
 * with respect to the inertia frame.
 */
#define rkChainLinkPointVel(chain,i,p,v) rkLinkPointVel( rkChainLink(chain,i), p, v )
#define rkChainLinkPointAcc(chain,i,p,a) rkLinkPointAcc( rkChainLink(chain,i), p, a )

/*! \brief kinematic chain forward kinematics.
 *
 * rkChainUpdateFK() updates the frame of each link of the
 * kinematic chain \a chain with resect to both the total body
 * frame and the world frame.
 *
 * rkChainFK() sets the joint displacement \a dis, and then
 * compute the forward kinematics of \a chain with respect to the
 * world frame.
 * \return
 * All these functions return no velues.
 * \sa
 * rkChainUpdateFrame
 */
__ROKI_EXPORT void rkChainUpdateFK(rkChain *chain);
__ROKI_EXPORT void rkChainFK(rkChain *chain, zVec dis);

/*! \brief neutralize all joints of a kinematic chain. */
__ROKI_EXPORT void rkChainNeutralize(rkChain *chain);

/*! \brief inverse dynamics of kinematic chain.
 *
 * rkChainUpdateID_G() computes the inverse dynamics of a kinematic chain \a chain
 * under an acceleration of field \a g by the Newton-Euler's method. It supposes
 * that the joint displacements, velocities, and accelerations of \a chain are
 * updated in advance.
 * rkChainUpdateID() computes the inverse dynamics of \a chain in the gravitational field.
 * rkChainUpdateID0G() computes the inverse dynamics of \a chain in the gravity-free field.
 *
 * rkChainID_G() computes the inverse dynamics of \a chain, provided the joint velocity
 * \a vel, the acceleration \a acc, and an acceleration of field \a g.
 * rkChainID() and rkChainID0G() compute the inverse dynamics of \a chain in the
 * gravitational field and the gravity-free field, respectively, provided \a vel and \a acc.
 *
 * rkChainFKCNT() continuously updates the joint displacement for \a dis over
 * the time step \a dt, and then, computes the inverse dynamics in the graviational
 * field. All the joint velocities and accelerations of \a chain will be updated
 * in accordance with a simple numerical differentiation.
 * \return
 * rkChainUpdateID_G(), rkChainUpdateID(), rkChainUpdateID0G(), rkChainID_G(), rkChainID(),
 * rkChainID0G(), and rkChainFKCNT() do not return any values.
 */
__ROKI_EXPORT void rkChainUpdateID_G(rkChain *chain, const zVec6D *g);
#define rkChainUpdateID(chain)     rkChainUpdateID_G( chain, RK_GRAVITY6D )
#define rkChainUpdateID0G(chain)   rkChainUpdateID_G( chain, ZVEC6DZERO )
__ROKI_EXPORT void rkChainID_G(rkChain *chain, zVec vel, zVec acc, const zVec6D *g);
#define rkChainID(chain,vel,acc)   rkChainID_G( chain, vel, acc, RK_GRAVITY6D )
#define rkChainID0G(chain,vel,acc) rkChainID_G( chain, vel, acc, ZVEC6DZERO )
__ROKI_EXPORT void rkChainFKCNT(rkChain *chain, zVec dis, double dt);

/*! \brief link acceleration at zero joint acceleration.
 *
 * rkChainLinkZeroAccG() computes 6D acceleration of a point \a p on the \a id th link of
 * a kinematic chain \a chain at zero-joint acceleration. This corresponds to the multiplication
 * of the rate of Jacobian matrix and the joint velocity vector.
 * \a g is an acceleration of the field.
 * The result is put into \a a0.
 *
 * rkChainLinkZeroAcc() and rkChainLinkZeroAcc0G() compute 6D acceleration of a point \a p
 * on the \a id th link of \a chain at zero-joint acceleration.
 * The difference between rkChainLinkZeroAcc() and rkChainLinkZeroAcc0G() are that \a a0
 * includes the acceleration due to the gravity in the former, while it does not in the latter.
 * For both functions, the result is put into \a a0.
 * \notes
 * rkChainLinkZeroAccG(),rkChainLinkZeroAcc(), and rkChainLinkZeroAcc0G() internally zero the
 * joint acceleration of \a chain.
 * \return
 * rkChainLinkZeroAccG(),rkChainLinkZeroAcc(), and rkChainLinkZeroAcc0G() return a pointer \a a0.
 */
__ROKI_EXPORT zVec6D *rkChainLinkZeroAccG(rkChain *chain, int id, zVec3D *p, const zVec6D *g, zVec6D *a0);
#define rkChainLinkZeroAcc(chain,i,p,a0)   rkChainLinkZeroAccG( (chain), (i), (p), RK_GRAVITY6D, (a0) )
#define rkChainLinkZeroAcc0G(chain,i,p,a0) rkChainLinkZeroAccG( (chain), (i), (p), ZVEC6DZERO, (a0) )

/*! \brief calculate the center of mass of kinematic chain.
 *
 * rkChainUpdateCOM() computes the center of mass of kinematic chain \a chain with respect to
 * the world frame. The kinematics should be calculated in advance.
 *
 * rkChainUpdateCOMVel() and rkChainUpdateCOMAcc() compute the velocity and acceleration of the
 * center of mass of \a chain with respect to the inertia frame, respectively. The motion rate
 * of the whole links should be updated in advance.
 * \return
 * These functions return a pointer to the internal 3D vector which stores the position, velocity
 * or acceleration of COM.
 * \notes
 * rkChainUpdateCOMAcc() includes the acceleration of gravity, except one explicitly sets the
 * zero gravity to the root link.
 */
__ROKI_EXPORT zVec3D *rkChainUpdateCOM(rkChain *chain);
__ROKI_EXPORT zVec3D *rkChainUpdateCOMVel(rkChain *chain);
__ROKI_EXPORT zVec3D *rkChainUpdateCOMAcc(rkChain *chain);

/*! \brief update the composite rigid body of a kinematic chain. */
#define rkChainUpdateCRBMass(chain) rkLinkUpdateCRBMass( rkChainRoot(chain) )
#define rkChainUpdateCRB(chain)     rkLinkUpdateCRB( rkChainRoot(chain) )

/*! \brief zero moment point of kinematic chain.
 *
 * rkChainZMP() computes the Zero Moment Point(ZMP) proposed by Vukobratovic et al.(1972) of the
 * kinematic chain \a chain with respect to the world frame.
 *
 * \a z is the height of the VHP proposed by Sugihara et al. (2002) in the direction of gravity.
 *
 * The result is put into \a zmp.
 *
 * rkChainYawTorque() computes the torque about the vertical axis (parallel to the acceleration
 * of gravity) on which ZMP exists.
 * \notes
 * In any cases of the three, inverse dynamics has to be computed in advance.
 */
__ROKI_EXPORT zVec3D *rkChainZMP(rkChain *chain, double z, zVec3D *zmp);
__ROKI_EXPORT double rkChainYawTorque(rkChain *chain);

/*! \brief angular momentum and kinematic energy of kinematic chain.
 *
 * rkChainAM() calculates angular momentum of a kinematic chain \a chain around the point \a p
 * with respect to the world frame. The result is put into \a am.
 *
 * rkChainKE() calculates kinematic energy of \a chain, originating from linear and angular
 * velocity of each link.
 * \return
 * rkChainAM() returns a pointer \a am.
 * rkChainKE() returns a value calculated.
 */
__ROKI_EXPORT zVec3D *rkChainAM(rkChain *chain, zVec3D *p, zVec3D *am);
__ROKI_EXPORT double rkChainKE(rkChain *chain);

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
 * All external wrenches exerted to \a chain have to be removed before calling
 * rkChainInertiaMat().
 *
 * rkChainInertiaMatUV() internally zeros velocities and accelerations of
 * the whole links of \a chain.
 * \return
 * rkChainInertiaMatBiasVec(), rkChainInertiaMat() and rkChainBiasVec() return
 * the true value if they succeed to compute the matrix and/or the vector.
 * If the sizes of the given matrix and vector do not match the total degree
 * of freedom of the chain, the false value is returned.
 */
__ROKI_EXPORT zMat rkChainInertiaMatMJ(rkChain *chain, zMat inertia);
__ROKI_EXPORT zMat rkChainInertiaMatUV(rkChain *chain, zMat inertia);
__ROKI_EXPORT zMat rkChainInertiaMatCRB(rkChain *chain, zMat inertia);
__ROKI_EXPORT zMat (* rkChainInertiaMat)(rkChain*,zMat);

__ROKI_EXPORT zVec rkChainBiasVec(rkChain *chain, zVec bias);

__ROKI_EXPORT bool rkChainInertiaMatBiasVecUV(rkChain *chain, zMat inertia, zVec bias);
__ROKI_EXPORT bool rkChainInertiaMatBiasVecCRB(rkChain *chain, zMat inertia, zVec bias);
__ROKI_EXPORT bool (* rkChainInertiaMatBiasVec)(rkChain*,zMat,zVec);

/*! \brief external force applied to kinematic chain.
 *
 * rkChainNetExtWrench() calculates the net external wrench acting to
 * a kinematic chain \a chain by summing up individual external forces
 * applied to each link. Orientation of the total force is with respect
 * to the body frame of \a chain.
 * The result is put into \a w.
 *
 * rkChainExtWrenchDestroy() destroys the external force list of the
 * kinematic chain \a chain.
 * \return
 * rkChainNetExtWrench() returns a pointer \a w.
 *
 * rkChainExtWrenchDestroy() returns no value.
 */
__ROKI_EXPORT zVec6D *rkChainNetExtWrench(rkChain *chain, zVec6D *w);
__ROKI_EXPORT void rkChainExtWrenchDestroy(rkChain *chain);

/*! \brief set joint identifier offset value of each link.
 *
 * rkChainSetJointIDOffset() sets the joint identifier offset values of
 * all links of a kinematic chain model \a chain. Each offset value corresponds
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
__ROKI_EXPORT void rkChainSetJointIDOffset(rkChain *chain);

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
 * rkChainBoundingBall() generates the bounding ball of a kinematic chain
 * \a chain. The result is stored in \a bb.
 * \return
 * rkChainBoundingBall() returns a pointer \a bb if succeeds. Otherwise,
 * the null pointer is returned.
 */
__ROKI_EXPORT zSphere3D *rkChainBoundingBall(rkChain *chain, zSphere3D *bb);

/* ZTK */

#define ZTK_TAG_ROKI_CHAIN      "roki::chain"
#define ZTK_TAG_ROKI_CHAIN_INIT "roki::chain::init"

#define ZTK_KEY_ROKI_CHAIN_NAME       "name"
#define ZTK_KEY_ROKI_CHAIN_INIT_POS   "pos"
#define ZTK_KEY_ROKI_CHAIN_INIT_ATT   "att"
#define ZTK_KEY_ROKI_CHAIN_INIT_FRAME "frame"
#define ZTK_KEY_ROKI_CHAIN_INIT_JOINT "joint"

__ROKI_EXPORT rkChain *rkChainFromZTK(rkChain *chain, ZTK *ztk);
__ROKI_EXPORT void rkChainFPrintZTK(FILE *fp, rkChain *chain);

__ROKI_EXPORT rkChain *rkChainReadZTK(rkChain *chain, const char *filename);
__ROKI_EXPORT bool rkChainWriteZTK(rkChain *chain, const char *filename);

__ROKI_EXPORT rkChain *rkChainInitFromZTK(rkChain *chain, ZTK *ztk);
__ROKI_EXPORT void rkChainInitFPrintZTK(FILE *fp, rkChain *chain);

__ROKI_EXPORT rkChain *rkChainInitReadZTK(rkChain *chain, const char *filename);
__ROKI_EXPORT bool rkChainInitWriteZTK(rkChain *chain, const char *filename);

/* print current 6D postures of all links of a kinematic chain out to a file. */
__ROKI_EXPORT void rkChainPostureFPrint(FILE *fp, rkChain *chain);
#define rkChainPosturePrint(chain)    rkChainPostureFPrint( stdout, (chain) )

/* print connectivity of a kinematic chain out to a file. */
__ROKI_EXPORT void rkChainConnectivityFPrint(FILE *fp, rkChain *chain);
#define rkChainConnectivityPrint(chain) rkChainConnectivityFPrint( stdout, (chain) )

/* print external wrenches applied to a kinematic chain out to a file. */
__ROKI_EXPORT void rkChainExtWrenchFPrint(FILE *fp, rkChain *chain);
#define rkChainExtWrenchPrint(chain)  rkChainExtWrenchFPrint( stdout, (chain) )

__END_DECLS

#include <roki/rk_ik.h>

#ifdef __cplusplus
inline int rkChain::getLinkNum() const { return rkChainLinkNum( this ); }
inline rkLink *rkChain::link(int i) const { return rkChainLink( this, i ); }
inline rkLink *rkChain::root() const { return rkChainRoot( this ); }
inline zVec3D *rkChain::COM(){ return rkChainWldCOM( this ); }
inline zVec3D *rkChain::COMVel(){ return rkChainCOMVel( this ); }
inline zVec3D *rkChain::COMAcc(){ return rkChainCOMAcc( this ); }
inline double rkChain::mass() const { return rkChainMass( this ); }
inline zFrame3D *rkChain::orgFrame() const { return rkChainOrgFrame( this ); }
inline zVec3D *rkChain::orgPos() const { return rkChainOrgPos( this ); }
inline zMat3D *rkChain::orgAtt() const { return rkChainOrgAtt( this ); }
inline zFrame3D *rkChain::rootFrame() const { return rkChainRootFrame( this ); }
inline zVec3D *rkChain::rootPos() const { return rkChainRootPos( this ); }
inline zMat3D *rkChain::rootAtt() const { return rkChainRootAtt( this ); }
inline zVec6D *rkChain::rootVel() const { return rkChainRootVel( this ); }
inline zVec3D *rkChain::rootLinVel() const { return rkChainRootLinVel( this ); }
inline zVec3D *rkChain::rootAngVel() const { return rkChainRootAngVel( this ); }
inline zVec6D *rkChain::rootAcc() const { return rkChainRootAcc( this ); }
inline zVec3D *rkChain::rootLinAcc() const { return rkChainRootLinAcc( this ); }
inline zVec3D *rkChain::rootAngAcc() const { return rkChainRootAngAcc( this ); }
inline zVec6D *rkChain::rootWrench() const { return rkChainRootWrench( this ); }
inline zVec3D *rkChain::rootForce() const { return rkChainRootForce( this ); }
inline zVec3D *rkChain::rootTorque() const { return rkChainRootTorque( this ); }

inline rkChain::rkChain(){ rkChainInit( this ); }
inline rkChain::~rkChain(){ rkChainDestroy( this ); }
inline void rkChain::init(){ rkChainInit( this ); }
inline void rkChain::destroy(){ rkChainDestroy( this ); }
inline rkChain *rkChain::clone(rkChain *dest){ return rkChainClone( this, dest ); }
inline rkChain *rkChain::copyState(rkChain *dest){ return rkChainCopyState( this, dest ); }
inline int rkChain::jointSize(){ return rkChainJointSize( this ); }
inline zIndex rkChain::createDefaultJointIndex(){ return rkChainCreateDefaultJointIndex( this ); }
inline int rkChain::jointIndexSize(zIndex idx){ return rkChainJointIndexSize( this, idx ); }
inline rkLink *rkChain::findLink(const char *name){ return rkChainFindLink( this, name ); }
inline int rkChain::findLinkID(const char *name){ return rkChainFindLinkID( this, name ); }
inline int rkChain::findLinkJointIDOffset(const char *name){ return rkChainFindLinkJointIDOffset( this, name ); }
inline void rkChain::setJointDis(zIndex idx, zVec dis){ rkChainSetJointDis( this, idx, dis ); }
inline void rkChain::setJointDis(zVec dis){ rkChainSetJointDisAll( this, dis ); }
inline void rkChain::setJointDisCNT(zIndex idx, zVec dis, double dt){ rkChainSetJointDisCNT( this, idx, dis, dt ); }
inline void rkChain::setJointDisCNT(zVec dis, double dt){ rkChainSetJointDisCNTAll( this, dis, dt ); }
inline void rkChain::setJointVel(zIndex idx, zVec vel){ rkChainSetJointVel( this, idx, vel ); }
inline void rkChain::setJointVel(zVec vel){ rkChainSetJointVelAll( this, vel ); }
inline void rkChain::setJointAcc(zIndex idx, zVec acc){ rkChainSetJointAcc( this, idx, acc ); }
inline void rkChain::setJointAcc(zVec acc){ rkChainSetJointAccAll( this, acc ); }
inline void rkChain::setJointRate(zIndex idx, zVec vel, zVec acc){ rkChainSetJointRate( this, idx, vel, acc ); }
inline void rkChain::setJointRate(zVec vel, zVec acc){ rkChainSetJointRateAll( this, vel, acc ); }
inline void rkChain::setJointTrq(zVec trq){ rkChainSetJointTrqAll( this, trq ); }
inline zVec rkChain::getJointDis(zIndex idx, zVec dis){ return rkChainGetJointDis( this, idx, dis ); }
inline zVec rkChain::getJointDis(zVec dis){ return rkChainGetJointDisAll( this, dis ); }
inline zVec rkChain::getJointVel(zVec vel){ return rkChainGetJointVelAll( this, vel ); }
inline zVec rkChain::getJointAcc(zVec acc){ return rkChainGetJointAccAll( this, acc ); }
inline zVec rkChain::getJointTrq(zVec trq){ return rkChainGetJointTrqAll( this, trq ); }

inline void rkChain::setConf(zVec conf){ rkChainSetConf( this, conf ); }
inline zVec rkChain::getConf(zVec conf){ return rkChainGetConf( this, conf ); }
inline void rkChain::setMotorInput(zVec input){ rkChainSetMotorInputAll( this, input ); }

inline void rkChain::updateFrame(){ rkChainUpdateFrame( this ); }
inline void rkChain::updateVel(){ rkChainUpdateVel( this ); }
inline void rkChain::updateAcc(){ rkChainUpdateAcc( this ); }
inline void rkChain::updateRateG(zVec6D *g){ rkChainUpdateRateG( this, g ); }
inline void rkChain::updateRate(){ rkChainUpdateRate( this ); }
inline void rkChain::updateRate0G(){ rkChainUpdateRate0G( this ); }
inline void rkChain::updateWrench(){ rkChainUpdateWrench( this ); }

inline zVec3D *rkChain::gravityDir(zVec3D *v){ return rkChainGravityDir( this, v ); }
inline zVec3D *rkChain::linkPointPos(int i, zVec3D *p, zVec3D *world_p){ return rkChainLinkPointWldPos( this, i, p, world_p ); }
inline zVec3D *rkChain::linkPointVel(int i, zVec3D *p, zVec3D *vel){ return rkChainLinkPointVel( this, i, p, vel ); }
inline zVec3D *rkChain::linkPointAcc(int i, zVec3D *p, zVec3D *acc){ return rkChainLinkPointAcc( this, i, p, acc ); }
inline void rkChain::updateForwardKinematics(){ rkChainUpdateFK( this ); }
inline void rkChain::ForwardKinematics(zVec dis){ rkChainFK( this, dis ); }
inline void rkChain::neutralize(){ rkChainNeutralize( this ); }
inline void rkChain::updateInverseDynamicsG(zVec6D *g){ rkChainUpdateID_G( this, g ); }
inline void rkChain::updateInverseDynamics(){ rkChainUpdateID( this ); }
inline void rkChain::updateInverseDynamics0G(){ rkChainUpdateID0G( this ); }
inline void rkChain::InverseDynamicsG(zVec vel, zVec acc, zVec6D *g){ rkChainID_G( this, vel, acc, g ); }
inline void rkChain::InverseDynamics(zVec vel, zVec acc){ rkChainID( this, vel, acc ); }
inline void rkChain::InverseDynamics0G(zVec vel, zVec acc){ rkChainID0G( this, vel, acc ); }
inline void rkChain::ForwardKinematicsCNT(zVec dis, double dt){ rkChainFKCNT( this, dis, dt ); }

inline zVec6D *rkChain::linkZeroAccG(int id, zVec3D *p, zVec6D *g, zVec6D *a0){ return rkChainLinkZeroAccG( this, id, p, g, a0 ); }
inline zVec6D *rkChain::linkZeroAcc(int id, zVec3D *p, zVec6D *a0){ return rkChainLinkZeroAcc( this, id, p, a0 ); }
inline zVec6D *rkChain::linkZeroAcc0G(int id, zVec3D *p, zVec6D *a0){ return rkChainLinkZeroAcc0G( this, id, p, a0 ); }

inline zVec3D *rkChain::updateCOM(){ return rkChainUpdateCOM( this ); }
inline zVec3D *rkChain::updateCOMVel(){ return rkChainUpdateCOMVel( this ); }
inline zVec3D *rkChain::updateCOMAcc(){ return rkChainUpdateCOMAcc( this ); }

inline void rkChain::updateCRBMass(){ rkChainUpdateCRBMass( this ); }
inline void rkChain::updateCRB(){ rkChainUpdateCRB( this ); }
inline zVec3D *rkChain::ZMP(double z, zVec3D *zmp){ return rkChainZMP( this, z, zmp ); }
inline double rkChain::yawTorque(){ return rkChainYawTorque( this ); }
inline zVec3D *rkChain::angularMomentum(zVec3D *p, zVec3D *am){ return rkChainAM( this, p, am ); }
inline double rkChain::kineticEnergy(){ return rkChainKE( this ); }

inline zMat rkChain::getInertiaMat(zMat inertia){ return rkChainInertiaMat( this, inertia ); }
inline zVec rkChain::getBiasVec(zVec bias){ return rkChainBiasVec( this, bias ); }
inline bool rkChain::getInertiaMatBiasVec(zMat inertia, zVec bias){ return rkChainInertiaMatBiasVec( this, inertia, bias ); }
inline zVec6D *rkChain::netExternalWrench(zVec6D *wrench){ return rkChainNetExtWrench( this, wrench ); }
inline void rkChain::destroyExternalWrench(){ rkChainExtWrenchDestroy( this ); }

inline zSphere3D *rkChain::getBoundingBall(zSphere3D *bb){ return rkChainBoundingBall( this, bb ); }

inline rkChain *rkChain::fromZTK(ZTK *ztk){ return rkChainFromZTK( this, ztk ); }
inline void rkChain::fprintZTK(FILE *fp){ rkChainFPrintZTK( fp, this ); }
inline rkChain *rkChain::readZTK(const char *filename){ return rkChainReadZTK( this, filename ); }
inline bool rkChain::writeZTK(const char *filename){ return rkChainWriteZTK( this, filename ); }

inline zMat rkChain::InverseKinematicsConstraintMat(){ return rkChainIKConstraintMat( this ); }
inline zVec rkChain::InverseKinematicsConstraintVec(){ return rkChainIKConstraintVec( this ); }
inline zIndex rkChain::InverseKinematicsJointIndex(){ return rkChainIKJointIndex( this ); }
inline rkChain *rkChain::createInverseKinematics(){ return rkChainCreateIK( this ); }
inline void rkChain::destroyInverseKinematics(){ rkChainDestroyIK( this ); }
inline bool rkChain::registerInverseKinematicsJointID(int id, double weight){ return rkChainRegisterIKJointID( this, id, weight ); }
inline bool rkChain::unregisterInverseKinematicsJointID(int id){ return rkChainUnregisterIKJointID( this, id ); }
inline bool rkChain::registerInverseKinematicsJoint(const char *name, double weight){ return rkChainRegisterIKJoint( this, name, weight ); }
inline bool rkChain::unregisterInverseKinematicsJoint(const char *name){ return rkChainUnregisterIKJoint( this, name ); }
inline bool rkChain::registerInverseKinematicsJointAll(double weight){ return rkChainRegisterIKJointAll( this, weight ); }
inline rkIKCell *rkChain::registerInverseKinematicsCell(const char *name, int priority, rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util){ return rkChainRegisterIKCell( this, name, priority, attr, mask, constraint, util ); }
inline rkIKCell *rkChain::registerInverseKinematicsCell(int priority, rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util){ return rkChainRegisterIKCell( this, NULL, priority, attr, mask, constraint, util ); }
inline rkIKCell *rkChain::registerInverseKinematicsCell(rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util){ return rkChainRegisterIKCell( this, NULL, 0, attr, mask, constraint, util ); }
inline bool rkChain::unregisterInverseKinematicsCell(rkIKCell *cell){ return rkChainUnregisterAndDestroyIKCell( this, cell ); }

inline rkIKCell *rkChain::registerInverseKinematicsCellWorldPos(const char *name, int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellWldPos( this, name, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellWorldPos(int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellWldPos( this, NULL, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellWorldPos(rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellWldPos( this, NULL, 0, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellWorldAtt(const char *name, int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellWldAtt( this, name, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellWorldAtt(int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellWldAtt( this, NULL, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellWorldAtt(rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellWldAtt( this, NULL, 0, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellLinkToLinkPos(const char *name, int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellL2LPos( this, name, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellLinkToLinkPos(int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellL2LPos( this, NULL, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellLinkToLinkPos(rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellL2LPos( this, NULL, 0, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellLinkToLinkAtt(const char *name, int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellL2LAtt( this, name, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellLinkToLinkAtt(int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellL2LAtt( this, NULL, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellLinkToLinkAtt(rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellL2LAtt( this, NULL, 0, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellCOM(const char *name, int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellCOM( this, name, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellCOM(int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellCOM( this, NULL, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellCOM(rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellCOM( this, NULL, 0, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellAngularMomentum(const char *name, int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellAM( this, name, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellAngularMomentum(int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellAM( this, NULL, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellAngularMomentum(rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellAM( this, NULL, 0, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellAngularMomentumCOM(const char *name, int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellAMCOM( this, name, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellAngularMomentumCOM(int priority, rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellAMCOM( this, NULL, priority, attr, mask ); }
inline rkIKCell *rkChain::registerInverseKinematicsCellAngularMomentumCOM(rkIKAttr *attr, ubyte mask){ return rkChainRegisterIKCellAMCOM( this, NULL, 0, attr, mask ); }

inline rkIKCell *rkChain::findInverseKinematicsByName(const char *name){ return rkChainFindIKCellByName( this, name ); }
inline bool rkChain::setCellPriority(rkIKCell *cell, int priority){ return rkChainSetIKCellPriority( this, cell, priority ); }
inline void rkChain::disableInverseKinematics(){ rkChainDisableIK( this ); }
inline void rkChain::bindInverseKinematics(){ rkChainBindIK( this ); }
inline void rkChain::resetInverseKinematicsAccumulator(){ rkChainZeroIKAcm( this ); }
inline int rkChain::InverseKinematics(zVec dis, double tol, int iter){ return rkChainIK( this, dis, tol, iter ); }
inline int rkChain::InverseKinematicsRJO(zVec dis, double tol, int iter){ return rkChainIK_RJO( this, dis, tol, iter ); }
#endif /* __cplusplus */

#endif /* __RK_CHAIN_H__ */
