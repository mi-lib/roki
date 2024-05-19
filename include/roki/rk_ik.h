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

 1. Create the inverse kinematics solver by
     rkChainCreateIK( &chain );

 2. Register cooperating joints by
     rkChainRegIKJoint( &chain, name, weight );
     ...
     \a name : name of a link which the joint is associated with.
    rkChainRegIKJoint() is also available for changing the weight
    on the joint displacement. rkChainRegIKJoint( &ik, name, weight2 )
    changes the weight from \a weight to \a weight2.

 3. Register constraint cell of inverse kinematics, using
     entry = rkChainRegIKCell( &chain, &attr, ref_fp, mf_fp, vf_fp, bind_fp, acm_fp, util );
     ...
     \a attr : attributes of the attention property (see 'rk_ik_cell.h/c')
     \a ref_fp : a function that provides the reference (a set of three values)
     \a mf_fp : a function that computes the constraint matrix
     \a vf_fp : a function that computes the 3D constraint vector
        (velocity or residual error of displacement in most cases)
     \a bind_fp : a function that computes the current vector to be constrained
     \a util : programmers' utility to attach any type of data chunk.
    Note that the constraint is not activated just by being registered;
    calling rkIKSetRef() family function activates it.

 4. Initialize the posture of \a chain.

 5. Deactivate all constraints if necessary by
     rkChainDeactivateIK( &chain );

 6. Bind the current status of all constraints if necessary by
     rkChainBindIK( &chain );

 7. Set the referential values of the constraints by rkIKCellSetRef() and so forth.
    Note that rkIKCellSetRef() internally calls rkIKCellSetMask() to activate
    the constraint.

 8. Solve the inverse kinematics by
     rkChainIK( &chain, dis, tol, iter );
     \a dis : a vector to store the solution of inverse kinematics
     \a tol : tolerance of error
     \a iter : the maximum number of iteration (if 0, Z_MAX_ITER_NUM is applied).

 9. Destroy the inverse kinematics solver if necessary after the computation by
     rkChainDestroyIK( &chain );
    Note that rkChainDestroyIK() is internally called by rkChainDestroy(). So,
    it is not necessary to call it explicitly in many cases.

 * ***********************************************************/

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIK ){
  bool *joint_sw;       /*!< joint cooperation switch */
  double *joint_weight; /*!< joint cooperating weight */
  zVec joint_vec;       /*!< joint vector */
  double eval;          /*!< evaluation function */

  /*! \cond */
  rkIKCellList clist;       /* list of constraint cells */
  zMat _c_mat_cell;         /* cellular workspace for constraint matrix */
  zVec3D _c_vec_cell;       /* cellular workspace for constraint vector */

  zIndex _j_idx;            /* cooperative joint index */
  zIndex _j_ofs;            /* reverse index */
  zVec _j_vec;              /* cooperative joint vector */
  zVec _j_wn;               /* weight on joint vector */
  zMat _c_mat;              /* constraint matrix */
  zVec _c_vec;              /* constraint vector */
  zVec _c_we;               /* weight on residual constraint error */
  zVec (*_solve_eq)(rkIK*); /* motion constraint equation solver */
  /* workspace for motion constraint equation solver */
  zLEWorkspace __le;
  zVec __c;
  /*! \endcond */
};

#define rkChainIKConstraintMat(chain) (chain)->_ik->_c_mat
#define rkChainIKConstraintVec(chain) (chain)->_ik->_c_vec
#define rkChainIKJointIndex(chain)    (chain)->_ik->_j_idx

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
__ROKI_EXPORT rkChain *rkChainCreateIK(rkChain *chain);
__ROKI_EXPORT void rkChainDestroyIK(rkChain *chain);

/*! \brief clone an inverse kinematics solver of a kinematic chain.
 *
 * rkChainCloneIK() clones an inverse kinematics solver of a kinematic chain \a src
 * and set it in another kinematic chain \a dest.
 * \return
 * rkChainCloneIK() returns the boolean value. If it succeeds to duplicate the
 * inverse kinematics solver, the true value is returned. Otherwise, the false
 * value is returned.
 */
__ROKI_EXPORT bool rkChainCloneIK(rkChain *src, rkChain *dest);

/*! \brief register/unregister cooperating joints of the inverse kinematics.
 *
 * rkChainRegIKJointID() registers a joint assigned to \a id th link of a
 * kinematic chain \a chain as one of the cooperating joints.
 * \a weight is the weighting value to the joint displacement.
 * rkChainUnregIKJointID() unregisters the joint \a id from the cooperating
 * joints.
 * \return
 * rkChainRegIKJointID() returns the boolean value.
 * If \a id is a valid link identifier, it returns the true value.
 * Otherwise, the false value is returned.
 */
__ROKI_EXPORT bool rkChainRegIKJointID(rkChain *chain, int id, double weight);
__ROKI_EXPORT bool rkChainUnregIKJointID(rkChain *chain, int id);

#define rkChainRegIKJoint(chain,name,weight) \
  rkChainRegIKJointID( chain, rkChainFindLinkID( chain, name ), weight )
#define rkChainUnregIKJoint(chain,name) \
  rkChainUnregIKJointID( chain, rkChainFindLinkID( chain, name ) )

__ROKI_EXPORT bool rkChainRegIKJointAll(rkChain *chain, double weight);

/*! \brief register/unregister a constraint cell of the inverse kinematics.
 *
 * rkChainRegIKCell() registers a constraint cell denoted by \a attr, \a rf,
 * \a mf, \a vf, and \a bf, to the cell list of the inverse kinematics
 * solver of \a chain.
 * \a attr contains attributes of the attention property (link or point to
 * be constrained). See rk_ik_cell.h for detail.
 * \a rf points a function that provides reference (a set of three values).
 * \a mf points a function that computes the constraint matrix.
 * \a vf is a function that computes the 3D constraint vector (velocity or
 * residual error of displacement in most cases).
 * \a bf is a function that computes the current vector to be constrained.
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
__ROKI_EXPORT rkIKCell *rkChainRegIKCell(rkChain *chain, rkIKAttr *attr, int mask, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKCVec_fp vf, rkIKBind_fp bf, rkIKAcm_fp af, void *util);
__ROKI_EXPORT bool rkChainUnregIKCell(rkChain *chain, rkIKCell *cell);

/*! \brief register a constraint cell of the inverse kinematics. */
__ROKI_EXPORT rkIKCell *rkChainRegIKCellWldPos(rkChain *chain, rkIKAttr *attr, int mask);
__ROKI_EXPORT rkIKCell *rkChainRegIKCellWldAtt(rkChain *chain, rkIKAttr *attr, int mask);
__ROKI_EXPORT rkIKCell *rkChainRegIKCellL2LPos(rkChain *chain, rkIKAttr *attr, int mask);
__ROKI_EXPORT rkIKCell *rkChainRegIKCellL2LAtt(rkChain *chain, rkIKAttr *attr, int mask);
__ROKI_EXPORT rkIKCell *rkChainRegIKCellCOM(rkChain *chain, rkIKAttr *attr, int mask);
__ROKI_EXPORT rkIKCell *rkChainRegIKCellAM(rkChain *chain, rkIKAttr *attr, int mask);
__ROKI_EXPORT rkIKCell *rkChainRegIKCellAMCOM(rkChain *chain, rkIKAttr *attr, int mask);

/*! \brief find a constraint cell.
 *
 * rkChainFindIKCellID() finds a constraint cell with an identifier \a id from
 * the constraint list registered in a kinematic chain \a chain.
 * \return
 * rkChainFindIKCellID() returns a pointer to the found cell, or the null pointer
 * if not found.
 */
__ROKI_EXPORT rkIKCell *rkChainFindIKCellID(rkChain *chain, int id);

/*! \brief deactivate and bind a constraint.
 *
 * rkChainDeactivateIK() deactivates all the constraints registered to the
 * inverse kinematics solver of a kinematic chain \a chain. In order to activate
 * each constraint cell, call rkIKSetRef family functions, or activate it manually
 * by rkIKCellOn().
 *
 * rkChainBindIK() sets the references of the constraints of \a chain for the
 * current values. It activates the all cells that binding functions are assigned.
 * \return
 * They return no values.
 */
__ROKI_EXPORT void rkChainDeactivateIK(rkChain *chain);
__ROKI_EXPORT void rkChainBindIK(rkChain *chain);
__ROKI_EXPORT void rkChainZeroIKAcm(rkChain *chain);

/*! \brief solve inverse kinematics.
 *
 * rkChainCreateIKEq() creates the motion rate constraint equation for the
 * inverse kinematics solver of a kinematic chain \a chain. It computes
 * coefficient matrix and strict referential vector of constrained values.
 *
 * rkChainIKRate() computes the joint vector by solving the motion constraint
 * equation J q = v. The solution method can be defined by rkIKSetEqSolver().
 * The following methods are predefined:
 *  - rkIKSolveEqMP() for Moore-Penrose generalized inverse
 *  - rkIKSolveEqSR() for Singularity-robust inverse
 *  - rkIKSolveEqED() for error-damped inverse (default)
 *
 * rkChainIK() solves the invserse kinematics of \a chain with numerical
 * iteration based on Levenberg-Marquardt method. It internally calls
 * rkChainIKOne() in an interative manner.
 * \return
 * rkChainCreateIKEq() does not return any value.
 * rkChainIKRate() returns a pointer to the joint rate vector.
 * rkChainIK() returns the actual number of iteration.
 */
__ROKI_EXPORT void rkChainCreateIKEq(rkChain *chain);

__ROKI_EXPORT zVec rkIKSolveEqMP(rkIK *ik);
__ROKI_EXPORT zVec rkIKSolveEqSR(rkIK *ik);
__ROKI_EXPORT zVec rkIKSolveEqED(rkIK *ik);
__ROKI_EXPORT zVec rkIKSolveEqSRED(rkIK *ik);

/*! \brief solve the motion constraint. */
#define rkIKSetEqSolver(ik,f)       ( (ik)->_solve_eq = (f) )
#define rkChainIKSetEqSolver(c,f)   rkIKSetEqSolver( (c)->_ik, f )
#define rkChainIKSetEqSolverMP(c)   rkChainIKSetEqSolver( c, rkIKSolveEqMP )
#define rkChainIKSetEqSolverSR(c)   rkChainIKSetEqSolver( c, rkIKSolveEqSR )
#define rkChainIKSetEqSolverED(c)   rkChainIKSetEqSolver( c, rkIKSolveEqED )
#define rkChainIKSetEqSolverSRED(c) rkChainIKSetEqSolver( c, rkIKSolveEqSRED )

__ROKI_EXPORT zVec rkChainIKSolveEq(rkChain *chain);
__ROKI_EXPORT zVec rkChainIKOne(rkChain *chain, zVec dis, double dt);
__ROKI_EXPORT zVec rkChainIKOneRJO(rkChain *chain, zVec dis, double dt);
__ROKI_EXPORT int rkChainIK(rkChain *chain, zVec dis, double tol, int iter);
__ROKI_EXPORT int rkChainIK_RJO(rkChain *chain, zVec dis, double tol, int iter);

/* ********************************************************** */
/* IK configuration file I/O
 * ********************************************************** */

#define RK_IK_JOINT_WEIGHT_DEFAULT 0.001

/* ZTK */

#define ZTK_TAG_RKIK "ik"

__ROKI_EXPORT rkChain *rkChainIKConfFromZTK(rkChain *chain, ZTK *ztk);
__ROKI_EXPORT rkChain *rkChainIKConfReadZTK(rkChain *chain, const char *filename);

__END_DECLS

#include <roki/rk_ik_seq.h> /* inverse kinematics sequence */
#include <roki/rk_ik_imp.h> /* inverse kinematics impedance control */

#endif /* __RK_IK_H__ */
