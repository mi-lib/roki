/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_jacobi - basic manipulator Jacobian matrix
 */

#ifndef __RK_JACOBI_H__
#define __RK_JACOBI_H__

#include <roki/rk_chain.h>

__BEGIN_DECLS

/* Manipulator Jacobian matrix */

/*! \brief Jacobian matrix of a kinematic chain.
 *
 * rkChainLinkWldAngJacobi() and rkChainLinkWldLinJacobi() calculate Jacobian
 * matrices which map the whole joint velocity to the angular velocity of the
 * \a id'th link of a kinematic chain \a chain, and the linear velocity of \a p
 * attached to the \a i'th link of \a chain, respectively, with respect to the
 * world frame.
 *
 * rkChainLinkToLinkAngJacobi() and rkChainLinkToLinkLinJacobi() calculate
 * Jacobian matrices which map the whole joint velocity to the relative angular
 * velocity of the \a to'th link and the relative linear velocity of \a p
 * attached to the \a to'th link, respectively, to those of the \a from'th link
 * of a kinematic chain \a chain. The orientation of the relative velocity is with
 * respect to the world frame.
 *
 * For all these functions, the result is stored where is pointed by \a jacobi.
 * \return
 * rkChainLinkWldAngJacobi(), rkChainLinkWldLinJacobi(),
 * rkChainLinkToLinkAngJacobi() and
 * rkChainLinkToLinkLinJacobi()' return a pointer \a jacobi.
 */
__ROKI_EXPORT zMat rkChainLinkWldAngJacobi(rkChain *chain, int id, zMat jacobi);
__ROKI_EXPORT zMat rkChainLinkWldLinJacobi(rkChain *chain, int id, zVec3D *p, zMat jacobi);
__ROKI_EXPORT zMat rkChainLinkToLinkAngJacobi(rkChain *chain, int from, int to, zMat jacobi);
__ROKI_EXPORT zMat rkChainLinkToLinkLinJacobi(rkChain *chain, int from, int to, zVec3D *p, zMat jacobi);

/*! \brief COM Jacobian matrix
 *
 * rkChainCOMJacobi() calculates Jacobian matrices which maps the whole joint
 * velocity to the velocity of the center of mass of a kinematic chain \a chain
 * with respect to the world frame. The result is put into \a jacobi.
 * \return
 * rkChainCOMJacobi() returns a pointer \a jacobi.
 * \sa
 * rkChainLinkWldLinJacobi
 */
__ROKI_EXPORT zMat rkChainCOMJacobi(rkChain *chain, zMat jacobi);

/*! \brief angular momentum matrix.
 *
 * rkChainLinkAMMat() calculates a matrix that maps the joint velocity
 * vector of a kinematic chain \a chain to the angular momentum of the \a id'th
 * link of the chain about a given point \a p. \a p is with respect to the
 * world frame.
 *
 * rkChainAMMat() calculates a matrix that maps the joint velocity vector
 * of \a chain to the total angular momentum of the chain about the given point
 * \a p.
 * rkChainAMCOMMat() calculates a matrix that maps the joint velocity vector
 * to the total angular momentum of the chain about the center of mass.
 *
 * The results of all these three functions are put into \a m.
 *
 * Angular momentum of a kinematic chain can be represented in a form of a
 * product of a matrix and a joint velocity vector, where the coefficient
 * matrix depends on the kinematic chain posture. The matrix that maps the
 * joint velocity vector to the angular momentum is called the angular
 * momentum matrix. It is handled in a similar way to the manipulator
 * Jacobian matrix, though the angular momentum is not a derivative of any
 * quantity.
 * \return
 * rkChainLinkAMMat(), rkChainAMMat() and rkChainAMCOMMat() return a pointer
 * \a m.
 */
__ROKI_EXPORT zMat rkChainLinkAMMat(rkChain *chain, int id, zVec3D *p, zMat m);
__ROKI_EXPORT zMat rkChainAMMat(rkChain *chain, zVec3D *p, zMat m);
__ROKI_EXPORT zMat rkChainAMCOMMat(rkChain *chain, zMat m);

/*! \brief measure of manipulability.
 *
 * rkJacobiManip() computes the manipulability, proposed by T. Yoshikawa(1984),
 * of a physical quantity related to a Jacobian matrix \a jacobi. Typically,
 * \a jacobi is a manipulator Jacobian matrix of a kinematic chain as defined
 * originally by Yoshikawa. However, it does not concern this function.
 * \a jacobi could be COM Jacobian matrix, angular momentum matrix, etc.
 * In any case, the value calculated represents the manipulability in a certain
 * sense.
 *
 * rkJacobiManip() internally calls rkJacobiManipDST() with \a index and \a k,
 * which are used as workspaces. \a index and \a k have to have sizes of r x 1
 * and r x r, respectively, where r is the row size of \a jacobi.
 * \return
 * rkJacobiManip() and rkJacobiManipDST() return the calculated measure of
 * manipulability.
 */
__ROKI_EXPORT double rkJacobiManipDST(zMat jacobi, zIndex index, zMat k);
__ROKI_EXPORT double rkJacobiManip(zMat jacobi);

__END_DECLS

#endif /* __RK_JACOBI_H__ */
