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

 1. Register cooperating joints by
     rkChainRegisterIKJoint( &chain, name, weight );
     ...
     \a name : name of a link which the joint is associated with.
    rkChainRegisterIKJoint() is also available for changing the weight on the
    joint displacement. rkChainRegisterIKJoint( &chain, name, weight2 ) changes
    the weight from \a weight to \a weight2.

 2. Register constraint cell of the inverse kinematics as
     cell = rkChainRegisterIKCell*( &chain, name, priority, &attr, mask );
    where
     \a name : name of the constraint
     \a priority: priority of the constraint (A larger number means higher priority.)
     \a attr : attributes of the inverse kinematics constraint (see 'rk_ik_cell.h')
     \a mask : mask for an attribute to be specified
    Note that * of the above rkChainRegisterIKCell* is the wild card.
    The following functions are avaible for real implementations of rkChainRegisterIKCell*:
     rkChainRegisterIKCellWldPos : position of a point on a link in the world frame
     rkChainRegisterIKCellWldAtt : attitude of a link in the world frame
     rkChainRegisterIKCellL2LPos : relative position of a point on a link with respect to a frame of another link
     rkChainRegisterIKCellL2LAtt : relative attitude of a link with respect to a frame of another link
     rkChainRegisterIKCellCOM : position of the center of mass in the world frame
     rkChainRegisterIKCellAM : angular momentum about the origin of the world frame
     rkChainRegisterIKCellAMCOM : angular momentum about the center of mass

 3. Initialize the posture of \a chain.

 4. Set the referential values of the constraints by rkIKCellSetRef() and so forth.
    rkChainBindIK() is also available to set the references for the current values.

 5. Solve the inverse kinematics by
     rkChainIK( &chain, dis, tol, iter );
    where
     \a dis : a vector to store the solution of inverse kinematics
     \a tol : tolerance of error
     \a iter : the maximum number of iterations (if 0, Z_MAX_ITER_NUM is applied).

 * ***********************************************************/

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIK ){
  bool *joint_is_enabled;   /*!< flag to check if each joint is enabled to cooperate */
  double *joint_weight;     /*!< joint cooperating weight */
  zVec joint_vec;           /*!< joint vector */
  rkIKCellList cell_list;   /*!< list of constraint cells */

  /*! \cond */
  double _eval;             /* evaluation value for optimization */
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
 * rkChainRegisterIKJointID() registers a joint assigned to \a id th link of a
 * kinematic chain \a chain as one of the cooperating joints.
 * \a weight is the weighting value to the joint displacement.
 * rkChainUnregisterIKJointID() unregisters the joint \a id from the cooperating
 * joints.
 * \return
 * rkChainRegisterIKJointID() returns the boolean value.
 * If \a id is a valid link identifier, it returns the true value.
 * Otherwise, the false value is returned.
 */
__ROKI_EXPORT bool rkChainRegisterIKJointID(rkChain *chain, int id, double weight);
__ROKI_EXPORT bool rkChainUnregisterIKJointID(rkChain *chain, int id);

#define rkChainRegisterIKJoint(chain,name,weight) \
  rkChainRegisterIKJointID( chain, rkChainFindLinkID( chain, name ), weight )
#define rkChainUnregisterIKJoint(chain,name) \
  rkChainUnregisterIKJointID( chain, rkChainFindLinkID( chain, name ) )

__ROKI_EXPORT bool rkChainRegisterIKJointAll(rkChain *chain, double weight);

/*! \brief register/unregister a constraint cell of the inverse kinematics.
 *
 * rkChainRegisterIKCell() registers a constraint cell denoted by \a priority, \a attr, and \a constraint
 * to the list of the inverse kinematics constraint cells of a kinematic chain \a chain.
 * \a priority is the priority of the constraint.
 * \a attr contains attributes of the attention property (link or point to be constrained). See
 * rk_ik_cell.h for detail. \a mask specifies the attributes of \a attr to be validated.
 * \a constraint is a set of functions that provide referential values, the constraint matrix,
 * the constraint vector, the reference bound to the current posture, and the accumulated error.
 * \a util is available for programmers' conveniences, which points any type of data chunk.
 *
 * rkChainUnregisterIKCell() unregisters a constraint cell \a cell of the inverse kinematics solver of \a chain.
 * \return
 * rkChainRegisterIKCell() returns a pointer to the registered cell, or the null pointer if it fails.
 * rkChainUnregisterIKCell() returns the true value if it succeeds, or the false value if it fails to
 * reallocate internal memory for the inverse kinematics.
 */
__ROKI_EXPORT rkIKCell *rkChainAddIKCell(rkChain *chain, rkIKCell *cell);
__ROKI_EXPORT rkIKCell *rkChainRegisterIKCell(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util);
__ROKI_EXPORT bool rkChainUnregisterIKCell(rkChain *chain, rkIKCell *cell);
__ROKI_EXPORT bool rkChainUnregisterAndDestroyIKCell(rkChain *chain, rkIKCell *cell);

/*! \brief register a constraint cell of the inverse kinematics. */
__ROKI_EXPORT rkIKCell *rkChainRegisterIKCellWldPos(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask);
__ROKI_EXPORT rkIKCell *rkChainRegisterIKCellWldAtt(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask);
__ROKI_EXPORT rkIKCell *rkChainRegisterIKCellL2LPos(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask);
__ROKI_EXPORT rkIKCell *rkChainRegisterIKCellL2LAtt(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask);
__ROKI_EXPORT rkIKCell *rkChainRegisterIKCellCOM(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask);
__ROKI_EXPORT rkIKCell *rkChainRegisterIKCellAM(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask);
__ROKI_EXPORT rkIKCell *rkChainRegisterIKCellAMCOM(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask);

/*! \brief find a constraint cell.
 *
 * rkChainFindIKCellByName() finds a constraint cell with a name \a name from the constraint list
 * registered in a kinematic chain \a chain.
 * \return
 * rkChainFindIKCellByName() returns a pointer to the found cell, or the null pointer if not found.
 */
__ROKI_EXPORT rkIKCell *rkChainFindIKCellByName(rkChain *chain, const char *name);

/*! \brief set priority of a constraint cell of an inverse kinematics solver.
 *
 * rkChainSetIKCellPriority() set priority of a constraint cell \a cell that is registered in a
 * kinematic chain \a chain for \a priority. The order of the internal list of constraint cells
 * is automatically rearranged.
 * \return
 * rkChainSetIKCellPriority() returns the true value if it succeeds to reorder the internal list
 * of constraint cells. If \a cell is the null pointer, it returns the false value.
 */
__ROKI_EXPORT bool rkChainSetIKCellPriority(rkChain *chain, rkIKCell *cell, int priority);

/*! \brief disable and bind a constraint.
 *
 * rkChainDisableIK() disables all the constraints registered to the
 * inverse kinematics solver of a kinematic chain \a chain. In order to enable
 * each constraint cell, call rkIKSetRef family functions, or enable it manually
 * by rkIKCellOn().
 *
 * rkChainBindIK() sets the references of the constraints of \a chain for the
 * current values. It enables the all cells that binding functions are assigned.
 * \return
 * They return no values.
 */
__ROKI_EXPORT void rkChainDisableIK(rkChain *chain);
__ROKI_EXPORT void rkChainBindIK(rkChain *chain);
__ROKI_EXPORT void rkChainZeroIKAcm(rkChain *chain);

/*! \brief methods to solve the constraint equation of the inverse kinematics.
 *
 * A family of functions to solve the constraint equation of the inverse kinematics is predefined as follows.
 *  - rkIKSolveEquationMP() for Moore-Penrose generalized inverse
 *  - rkIKSolveEquationSR() for Singularity-robust inverse
 *  - rkIKSolveEquationED() for error-damped inverse (default)
 * \a ik is an instance of the inverse kinematics solver.
 * The method can be set to the inverse kinematics solver by rkIKSetEquationSolver().
 *
 * Though those functions are publicly available in the library, it is not recommended to call them directly.
 * Use rkChainIKSetEquationSolverMP(), rkChainIKSetEquationSolverSR(), or rkChainIKSetEquationSolverED(), instead.
 * \return
 * These functions return a pointer to an internally allocated vector for the solution.
 */
__ROKI_EXPORT zVec rkIKSolveEquationMP(rkIK *ik);
__ROKI_EXPORT zVec rkIKSolveEquationSR(rkIK *ik);
__ROKI_EXPORT zVec rkIKSolveEquationED(rkIK *ik);
__ROKI_EXPORT zVec rkIKSolveEquationSRED(rkIK *ik);

/*! \brief solve the motion constraint. */
#define rkIKSetEquationSolver(ik,f)       ( (ik)->_solve_eq = (f) )
#define rkChainIKSetEquationSolver(c,f)   rkIKSetEquationSolver( (c)->_ik, f )
#define rkChainIKSetEquationSolverMP(c)   rkChainIKSetEquationSolver( c, rkIKSolveEquationMP )
#define rkChainIKSetEquationSolverSR(c)   rkChainIKSetEquationSolver( c, rkIKSolveEquationSR )
#define rkChainIKSetEquationSolverED(c)   rkChainIKSetEquationSolver( c, rkIKSolveEquationED )
#define rkChainIKSetEquationSolverSRED(c) rkChainIKSetEquationSolver( c, rkIKSolveEquationSRED )

/*! \brief solve inverse kinematics.
 *
 * rkChainIKOne() updates a joint displacement vector \a dis of a kinematic chain \a chain by solving
 * the motion constraint equation J dq = dp and concatenating the answer dq multiplied by \a dt.
 *
 * rkChainIK() numerically solves the invserse kinematics of \a chain based on Levenberg-Marquardt method.
 * It internally calls rkChainIKOne() in an interative manner.
 * \return
 * rkChainIKOne() returns a pointer to the updated joint displacement vector \a dis.
 * rkChainIK() returns the number of iterations.
 */
__ROKI_EXPORT zVec rkChainIKOne(rkChain *chain, zVec dis, double dt);
__ROKI_EXPORT int rkChainIK(rkChain *chain, zVec dis, double tol, int iter);
__ROKI_EXPORT int rkChainIK_RJO(rkChain *chain, zVec dis, double tol, int iter);

/* ********************************************************** */
/* IK configuration file I/O
 * ********************************************************** */

#define RK_IK_JOINT_WEIGHT_DEFAULT 0.001

/* ZTK */

#define ZTK_TAG_ROKI_CHAIN_IK "roki::chain::ik"

/* read the inverse kinematics configuration of a kinematic chain from ZTK. */
__ROKI_EXPORT rkChain *rkChainIKConfFromZTK(rkChain *chain, ZTK *ztk);
/* read the inverse kinematics configuration of a kinematic chain from a file. */
__ROKI_EXPORT rkChain *rkChainIKConfReadZTK(rkChain *chain, const char *filename);

/* print the inverse kinematics configuration of a kinematic chain out to the current position of a file. */
__ROKI_EXPORT void rkChainIKConfFPrintZTK(FILE *fp, rkChain *chain);
/* write the inverse kinematics configuration of a kinematic chain to a file in ZTK format. */
__ROKI_EXPORT bool rkChainIKConfWriteZTK(rkChain *chain, const char *filename);

__END_DECLS

#include <roki/rk_ik_seq.h> /* inverse kinematics sequence */
#include <roki/rk_ik_imp.h> /* inverse kinematics impedance control */

#endif /* __RK_IK_H__ */
