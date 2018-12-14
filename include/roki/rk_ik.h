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

 Inverse Kinematics Computation Procedure featuring rkIK

 suppose 'ik' is an instance of inverse kinematics solver.

 1. initialize a kinematic chain \a c and set the initial posture.

 2. rkIKCreate( &ik, c );

 3. register cooperating joints, using
     rkIKJointReg( &ik, id, weight );
     ...
    where 'id' is the identifier of a link which the joint
    is assigned to.
    Note that 'rkIKJointReg()' is utilized when changing
    the weight of the relative joint displacement norm.
    Simply calling rkIKJointReg( &ik, id, weight2 ),
    weight would be changed from 'weight' to 'weight2'.

 4. register constraint cell of inverse kinematics, using
     entry = rkIKCellReg( &ik, &attr, ref_fp, jacobi_fp, vel_fp, bind_fp, acm_fp, util );
     ...
    where 'attr' stores attributes of the attented property
    (see 'rk_ik_cell.h/c'), 'ref_fp' is a pointer to the
    function which composes reference from a set of three
    values, 'jacobi_fp' is a pointer to the function which
    computes Jacobian matrix relating joint velocity to
    the constrained property, 'vel_fp' is a pointer to the
    function which computes the constrained 3D vector
    (velocity or residual error of displacement in most
    cases), 'bind_fp' is a pointer to the function which
    computes the current amount of the constrained property,
    and 'util' is for programmers' utility to attach any type
    of data chunk.

    Note that the constraint is not activated just by being
    registered; calling rkIKSetRef family function activates
    it.

 5. deactivate all constraint cells by rkIKDeactivate( &ik );

 6. bind the current status of all constraints by calling
    rkIKBind( &ik ), if necessary.

 7. set the referential values of constrained properties
    by rkIKCellSetRef and so forth. Note that rkIKCellSetRef
    internally calls rkIKCellSetMask to activate the constraint.

 8. rkIKSolve( &ik, dis, tol, iter );
    where 'dis' is a vector to store the solution of inverse
    kinematics, 'tol' is the tolerance of error, and 'iter' is
    the maximum number of iteration. When 'iter' is zero, the
    default number is applied.

 9. rkIKDestroy( &ik );
    when terminating the program.

 * ***********************************************************/

typedef struct _rkIK{
  rkChain *chain;       /* a pointer to a kinematic chain */

  bool *joint_sw;       /* joint cooperation switch */
  double *joint_weight; /* joint cooperating weight */
  zVec joint_vel;       /* joint velocity */
  double eval;          /* evaluation function */

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
  /* working space for joint velocity computation */
  zMat __m;
  zVec __v, __s, __c;
  zIndex __idx;
} rkIK;

/*! \brief create and destroy inverse kinematics solver.
 *
 * rkIKCreate() creates an instance of inverse kinematics
 * solver \a ik, assigning a kinematic chain \a chain.
 * It prepares the internal working spaces to compute
 * constraint equation matrix and vectors.
 *
 * rkIKDestroy() destroys the internal working spaces of \a ik.
 * \return
 * rkIKCreate() returns a pointer \a ik if succeeding.
 * If failing to allocate the internal working spaces,
 * the null pointer is returned.
 *
 * rkIKDestroy() returns no value.
 */
__EXPORT rkIK *rkIKCreate(rkIK *ik, rkChain *chain);
__EXPORT void rkIKDestroy(rkIK *ik);

/*! \brief register/unregister cooperating joints and constraint cells.
 *
 * rkIKJointReg() registers a joint assigned to \a id'th
 * link of a kinematic chain as one of the cooperating
 * joints. \a weight is the weighting value to the norm of
 * relative joint displacement.
 * rkIKJointUnreg() unregisters the joint \a id from the
 * cooperating joints.
 *
 * rkIKCellReg() registers a constraint cell denoted by \a attr,
 * \a rf, \a mf, \a vf and \a bf, to the internal cell list of
 * \a ik.
 * \a attr is to denote attributes of the attented link or point
 * to the constraint, if necessary. See rk_ik_cell.h for the
 * detail.
 * \a rf is a pointer to the function which composes reference
 * from a set of the three values.
 * \a mf is a pointer to the function which computes Jacobian
 * matrix relating joint velocity to the constrained property.
 * \a vf is a pointer to the function which computes the
 * constrained 3D vector - velocity or residual error of
 * displacement, typically. Namely, for a constraint J q = v,
 * \a mf computes J and \a vf does v.
 * \a bf is a pointer to the function which computes the current
 * amount of the constrained property.
 * \a util is for programmers utility to attach any type of data
 * chunk, utilized in \a vf.
 * \return
 * rkIKJointReg() and rkIKCellReg() return the boolean value.
 * If \a id is a valid link identifier, they return the true
 * value. Otherwise, the false value is returned.
 *
 * rkIKCellReg() returns the entry number of the registered cell.
 */
__EXPORT bool rkIKJointReg(rkIK *ik, int id, double weight);
__EXPORT bool rkIKJointRegAll(rkIK *ik, double weight);
__EXPORT bool rkIKJointUnreg(rkIK *ik, int id);

/*! \brief register a joint into which the residual of constraints is resolved. */

__EXPORT rkIKCell *rkIKCellReg(rkIK *ik, rkIKCellAttr *attr, int mask, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKSRV_fp vf, rkIKBind_fp bf, rkIKAcm_fp af, void *util);
__EXPORT bool rkIKCellUnreg(rkIK *ik, rkIKCell *cell);

/*! \brief register a constraint cell of the inverse kinematics. */
__EXPORT rkIKCell *rkIKCellRegWldPos(rkIK *ik, rkIKCellAttr *attr, int mask);
__EXPORT rkIKCell *rkIKCellRegWldAtt(rkIK *ik, rkIKCellAttr *attr, int mask);
__EXPORT rkIKCell *rkIKCellRegL2LPos(rkIK *ik, rkIKCellAttr *attr, int mask);
__EXPORT rkIKCell *rkIKCellRegL2LAtt(rkIK *ik, rkIKCellAttr *attr, int mask);
__EXPORT rkIKCell *rkIKCellRegCOM(rkIK *ik, rkIKCellAttr *attr, int mask);
__EXPORT rkIKCell *rkIKCellRegAM(rkIK *ik, rkIKCellAttr *attr, int mask);
__EXPORT rkIKCell *rkIKCellRegAMCOM(rkIK *ik, rkIKCellAttr *attr, int mask);

__EXPORT rkIKCell *rkIKFindCell(rkIK *ik, int id);

/*! \brief deactivate and bind constraint properties.
 *
 * rkIKDeactivate() deactivates all the constraints registered
 * to the inverse kinematics solver \a ik. In order to activate
 * each constraint cell, call rkIKSetRef family functions, or
 * activate it manually by rkIKCellOn().
 *
 * rkIKBind() sets the references of the properties constrained
 * by \a ik for the current values. It activates the all cells
 * which binding functions are assigned.
 * \return
 * They return no values.
 */
__EXPORT void rkIKDeactivate(rkIK *ik);
__EXPORT void rkIKBind(rkIK *ik);
__EXPORT void rkIKAcmClear(rkIK *ik);

/*! \brief synchronize state of the virtual kinematic chain to an actual. */
#define rkIKSync(ik,c) rkChainCopyState( c, (ik)->chain )

/*! \brief resolve the motion rate. */
#define rkIKSetJointVelMethod(ik,f) ( (ik)->_jv = (f) )
__EXPORT zVec rkIKJointVelMP(rkIK *ik);
__EXPORT zVec rkIKJointVelSR(rkIK *ik);
__EXPORT zVec rkIKJointVelAD(rkIK *ik);

/*! \brief solve inverse kinematics.
 *
 * rkIKEq() forms the motion rate constraint equation for the
 * inverse kinematics solver \a ik, computing coefficient
 * matrix and strict referential vector of constrained values.
 *
 * rkIKSolveOne() computes the joint velocity vector by solving
 * the motion rate constraint equation J q = v with
 * singularity-robust inverse matrix of J, namely,
 *  q = J^T Wn ( J Wn J^T + We )^-1 v.
 *
 * rkIKSolve() solves the invserse kinematics with numerical
 * iteration based on Levenberg=Marquardt's method, repetitively
 * calling rkIKSolveOne().
 * \return
 * Neither rkIKEq() nor rkIKSolveOne() return any values.
 *
 * rkIKSolve() returns the number of iteration.
 */
__EXPORT void rkIKEq(rkIK *ik);
__EXPORT zVec rkIKSolveRate(rkIK *ik);
__EXPORT zVec rkIKSolveOne(rkIK *ik, zVec dis, double dt);
__EXPORT int rkIKSolve(rkIK *ik, zVec dis, double tol, int iter);

/* ********************************************************** */
/* IK configuration file I/O
 * ********************************************************** */

#define RK_IK_JOINT_WEIGHT_DEFAULT 0.001

/*! \brief read IK configuration file. */
__EXPORT bool rkIKConfFRead(FILE *fp, rkIK *ik, rkChain *chain);
/*! \brief read IK configuration from the current position of a file. */
__EXPORT bool rkIKConfReadFile(rkIK *ik, rkChain *chain, char *filename);

__END_DECLS

#include <roki/rk_ik_seq.h> /* inverse kinematics sequence */
#include <roki/rk_ik_imp.h> /* inverse kinematics impedance control */

#endif /* __RK_IK_H__ */
