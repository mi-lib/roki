/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_ik - inverse kinematics
 */

#ifndef __RK_IK_H__
#define __RK_IK_H__

#include <roki/rk_jacobi.h>
#include <roki/rk_ik_cell.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkIK
 * inverse kinematics solver class
 * **********************************************************

 Inverse Kinematics Computation Procedure

 Suppose \a chain is a kinematic chain.

 1. rkChainCreateIK( &chain );

 2. Register cooperating joints, using
     rkChainRegIKJoint( &chain, id, weight );
     ...
     \a id : the identifier of a link which the joint is assigned to.
    rkChainRegIKJoint() is also available for changing the weight
    on the joint displacement. rkChainRegIKJoint( &ik, id, weight2 )
    changes the weight from \a weight to \a weight2.

 3. Register constraint cell of inverse kinematics, using
     entry = rkChainRegIKCell( &chain, &attr, ref_fp, jacobi_fp, vel_fp, bind_fp, acm_fp, util );
     ...
     \a attr : attributes of the attented property (see 'rk_ik_cell.h/c')
     \a ref_fp : a function that provides reference (a set of three values)
     \a jacobi_fp : a function that computes Jacobian matrix relating
        joint velocity to the constrained property
     \a vel_fp : a function that computes the constrained 3D vector
        (velocity or residual error of displacement in most cases)
     \a bind_fp : a function that computes the current amount of the
        constrained property
     \a util : programmers' utility to attach any type of data chunk.
    Note that the constraint is not activated just by being registered;
    calling rkIKSetRef() family function activates it.

 4. Initialize the posture of \a chain.

 5. rkChainDeactivateIK( &chain );

 6. Bind the current status of all constraints by calling
    rkChainBindIK( &chain ), if necessary.

 7. Set the referential values of constrained properties by rkIKCellSetRef()
    and so forth. Note that rkIKCellSetRef() internally calls rkIKCellSetMask()
    to activate the constraint.

 8. rkChainIK( &chain, dis, tol, iter );
     \a dis : a vector to store the solution of inverse kinematics
     \a tol : tolerance of error
     \a iter : the maximum number of iteration (if 0, the default number
        is applied.

 9. rkChainDestroyIK( &chain );
    when terminating the program.

 * ***********************************************************/

typedef struct _rkIK{
  bool *joint_sw;       /*!< joint cooperation switch */
  double *joint_weight; /*!< joint cooperating weight */
  zVec joint_vel;       /*!< joint velocity */
  double eval;          /*!< evaluation function */

  /*! \cond */
  rkIKCellList clist;   /* constraint cell list */
  zMat _c_mat_cell;     /* constraint coefficient matrix cell */
  zVec3D _c_srv_cell;   /* strict referential velocity vector cell */

  zIndex _j_idx;        /* cooperative joint index */
  zIndex _j_ofs;        /* reverse index */
  zVec _j_vel;          /* joint velocity vector */
  zVec _j_wn;           /* weight on joint velocity norm */
  zMat _c_mat;          /* constraint coefficient matrix */
  zVec _c_srv;          /* strict referential velocity vector */
  zVec _c_we;           /* weight on residual constraint error */
  zVec (*_jv)(struct _rkIK*); /* joint velocity computation method */
  /* workspace for joint velocity computation */
  zLE __le;
  zVec __c;
  /*! \endcond */
} rkIK;

/*! \brief create and destroy inverse kinematics solver.
 *
 * rkChainCreateIK() creates the inverse kinematics solver of a kinematic
 * chain \a chain.
 *
 * rkChainDestroyIK() destroys the inverse kinematics solver of \a chain.
 * \return
 * rkIKCreate() returns a pointer \a chain if succeeding. If it failes to
 * allocate the internal memory, the null pointer is returned.
 *
 * rkChainDestroyIK() returns no value.
 */
__EXPORT rkChain *rkChainCreateIK(rkChain *chain);
__EXPORT void rkChainDestroyIK(rkChain *chain);

/*! \brief register/unregister cooperating joints of the inverse kinematics.
 *
 * rkChainRegIKJoint() registers a joint assigned to \a id th link of a
 * kinematic chain \a chain as one of the cooperating joints.
 * \a weight is the weighting value to the joint displacement.
 * rkChainUnregIKJoint() unregisters the joint \a id from the cooperating
 * joints.
 * \return
 * rkChainRegIKJoint() returns the boolean value.
 * If \a id is a valid link identifier, it returns the true value.
 * Otherwise, the false value is returned.
 */
__EXPORT bool rkChainRegIKJoint(rkChain *chain, uint id, double weight);
__EXPORT bool rkChainRegIKJointAll(rkChain *chain, double weight);
__EXPORT bool rkChainUnregIKJoint(rkChain *chain, uint id);

/*! \brief register/unregister a constraint cell of the inverse kinematics.
 *
 * rkChainRegIKCell() registers a constraint cell denoted by \a attr, \a rf,
 * \a mf, \a vf, and \a bf, to the cell list of the inverse kinematics
 * solver of \a chain.
 * \a attr contains attributes of the attented property (link or point to
 * be constrained). See rk_ik_cell.h for detail.
 * \a rf points a function that provides reference (a set of three values).
 * \a mf points a function that computes Jacobian matrix relating joint
 * velocity to the constrained property.
 * \a vf is a function that computes the constrained 3D vector (velocity
 * or residual error of displacement in many cases).
 * \a bf is a function that computes the current amount of the constrained
 * property.
 * \a util is available for programmers' conveniences, which points any type
 * of data chunk.
 *
 * rkChainUnregIKCell() unregisters a constraint cell of the inverse kinematics
 * solver of \a chain. \a id is the identifier of the cell to be unregistered.
 * \return
 * rkChainRegIKCell() returns a pointer to the registered cell. If it fails,
 * the null pointer is returned.
 * rkChainUnregIKCell() returns the boolean value. If it fails to reallocate
 * internal memory for the inverse kinematics, the false value is returned.
 * Otherwise, the true value is returned.
 */
__EXPORT rkIKCell *rkChainRegIKCell(rkChain *chain, rkIKCellAttr *attr, int mask, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKSRV_fp vf, rkIKBind_fp bf, rkIKAcm_fp af, void *util);
__EXPORT bool rkChainUnregIKCell(rkChain *chain, rkIKCell *cell);

__EXPORT rkIKCell *rkIKFindCell(rkIK *ik, int id);
__EXPORT rkIKCell *rkChainFindIKCell(rkChain *chain, int id);

/*! \brief deactivate and bind constraint properties.
 *
 * rkChainDeactivateIK() deactivates all the constraints registered
 * to the inverse kinematics solver of a kinematic chain \a chain.
 * In order to activate each constraint cell, call rkIKSetRef family
 * functions, or activate it manually by rkIKCellOn().
 *
 * rkChainBindIK() sets the references of the constrained properties
 * of \a chain for the current values. It activates the all cells
 * that binding functions are assigned.
 * \return
 * They return no values.
 */
__EXPORT void rkChainDeactivateIK(rkChain *chain);
__EXPORT void rkChainBindIK(rkChain *chain);
__EXPORT void rkChainZeroIKAcm(rkChain *chain);

/*! \brief solve inverse kinematics.
 *
 * rkChainCreateIKEq() creates the motion rate constraint equation for the
 * inverse kinematics solver of a kinematic chain \a chain. It computes
 * coefficient matrix and strict referential vector of constrained values.
 *
 * rkChainIKRate() computes the joint velocity vector by solving the motion
 * rate constraint equation J q = v. The solution method can be defined by
 * rkIKSetJointVelMethod(). The following methods are predefined:
 *  - rkIKJointVelMP() for Moore-Penrose generalized inverse
 *  - rkIKJointVelSR() for Singularity-robust inverse
 *  - rkIKJointVelAD() for auto error-damped inverse
 * rkIKJointVelAD() is initially chosen as the default method.
 *
 * rkChainIK() solves the invserse kinematics of \a chain with numerical
 * iteration based on Levenberg-Marquardt method. It internally calls
 * rkChainIKOne() in an interative manner.
 * \return
 * rkChainCreateIKEq() does not return any value.
 * rkChainIKRate() returns a pointer to the joint rate vector.
 * rkChainIK() returns the actual number of iteration.
 */
__EXPORT void rkChainCreateIKEq(rkChain *chain);

/*! \brief resolve the motion rate. */
#define rkIKSetJointVelMethod(ik,f) ( (ik)->_jv = (f) )
__EXPORT zVec rkIKJointVelMP(rkIK *ik);
__EXPORT zVec rkIKJointVelSR(rkIK *ik);
__EXPORT zVec rkIKJointVelAD(rkIK *ik);

__EXPORT zVec rkChainIKRate(rkChain *chain);
__EXPORT zVec rkChainIKOne(rkChain *chain, zVec dis, double dt);
__EXPORT int rkChainIK(rkChain *chain, zVec dis, double tol, int iter);

/*! \brief register a constraint cell of the inverse kinematics. */
__EXPORT rkIKCell *rkChainRegIKCellWldPos(rkChain *chain, rkIKCellAttr *attr, int mask);
__EXPORT rkIKCell *rkChainRegIKCellWldAtt(rkChain *chain, rkIKCellAttr *attr, int mask);
__EXPORT rkIKCell *rkChainRegIKCellL2LPos(rkChain *chain, rkIKCellAttr *attr, int mask);
__EXPORT rkIKCell *rkChainRegIKCellL2LAtt(rkChain *chain, rkIKCellAttr *attr, int mask);
__EXPORT rkIKCell *rkChainRegIKCellCOM(rkChain *chain, rkIKCellAttr *attr, int mask);
__EXPORT rkIKCell *rkChainRegIKCellAM(rkChain *chain, rkIKCellAttr *attr, int mask);
__EXPORT rkIKCell *rkChainRegIKCellAMCOM(rkChain *chain, rkIKCellAttr *attr, int mask);

/* ********************************************************** */
/* IK configuration file I/O
 * ********************************************************** */

#define RK_IK_JOINT_WEIGHT_DEFAULT 0.001

/* ZTK */

#define ZTK_TAG_RKIK "ik"

__EXPORT rkChain *rkChainIKConfFromZTK(rkChain *ik, ZTK *ztk);
__EXPORT rkChain *rkChainIKConfReadZTK(rkChain *chain, char filename[]);

__END_DECLS

#include <roki/rk_ik_seq.h> /* inverse kinematics sequence */
#include <roki/rk_ik_imp.h> /* inverse kinematics impedance control */

#endif /* __RK_IK_H__ */
