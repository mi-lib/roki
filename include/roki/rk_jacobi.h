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

/* METHOD:
 * rkChainLinkToLinkAngJacobi, rkChainLinkToLinkLinJacobi,
 * rkChainLinkWldAngJacobi, rkChainLinkWldLinJacobi
 * - Jacobian matrix of a kinematic chain.
 *
 * 'rkChainLinkWldAngJacobi()' and 'rkChainLinkWldLinJacobi()'
 * calculate Jacobian matrices which map the whole joint
 * velocity to the angular velocity of the 'id'th link,
 * and the linear velocity of 'p' attached to the 'i'th
 * link of 'r', respectively, with respect to the world
 * frame.
 *
 * 'rkChainLinkToLinkAngJacobi()' and 'rkChainLinkToLinkLinJacobi()'
 * calculate Jacobian matrices which map the whole joint
 * velocity to the relative angular velocity of the
 * 'to'th link and the relative linear velocity of 'p'
 * attached to the 'to'th link, respectively, to those
 * of the 'from'th link of a kinematic chain 'r'. The orientation
 * of the relative velocity is with respect to the
 * world frame.
 *
 * For all these functions, the result is stored where
 * is pointed by 'jacobi'.
 * [RETURN VALUE]
 * 'rkChainLinkWldAngJacobi()', 'rkChainLinkWldLinJacobi()',
 * 'rkChainLinkToLinkAngJacobi()' and
 * 'rkChainLinkToLinkLinJacobi()' return a pointer 'jacobi'.
 */
__EXPORT zMat rkChainLinkWldAngJacobi(rkChain *c, int id, zMat jacobi);
__EXPORT zMat rkChainLinkWldLinJacobi(rkChain *c, int id, zVec3D *p, zMat jacobi);
__EXPORT zMat rkChainLinkToLinkAngJacobi(rkChain *c, int from, int to, zMat jacobi);
__EXPORT zMat rkChainLinkToLinkLinJacobi(rkChain *c, int from, int to, zVec3D *p, zMat jacobi);

/* METHOD:
 * rkChainCOMJacobi - COM Jacobian
 *
 * 'rkChainCOMJacobi()' calculates Jacobian matrices
 * which maps the whole joint velocity to the velocity
 * of the center of mass of a kinematic chain 'r' with respect
 * to the world frame.
 * The result is put into 'jacobi'.
 * [RETURN VALUE]
 * 'rkChainCOMJacobi()' returns a pointer 'jacobi'.
 * \sa
 * rkChainLinkWldLinJacobi
 */
__EXPORT zMat rkChainCOMJacobi(rkChain *c, zMat jacobi);

/* METHOD:
 * rkChainLinkAMJacobi, rkChainAMJacobi, rkChainAMCOMJacobi
 * - angular momentum Jacobian matrix.
 *
 * Angular momentum of a kinematic chain can be represented in a
 * form of a product of a matrix and a joint velocity
 * vector, where the coefficient matrix depends on
 * a kinematic chain posture. In this sense, the coefficient matrix
 * could be called quasi-Jacobian matrix. Of course,
 * it is not Jacobian matrix in accordance with the strict
 * mathematical definition, since it is not a derivative
 * of any function, namely, the integration of it does
 * not have any physical meanings. But, here, we call it
 * angular momentum Jacobian matrix just for convenience.
 * #
 * 'rkChainLinkAMJacobi()' calculates angular momentum
 * Jacobian matrix of the 'i'th link of a kinematic chain 'r'
 * about a given point 'p'. 'p' is with respect to the
 * world frame.
 * 'rkChainAMJacobi()' calculates the total angular
 * momentum Jacobian matrix of 'r' about 'p'.
 * 'rkChainAMCOMJacobi()' calculates the total angular
 * momentum Jacobian matrix of 'r' about COM.
 * The results of all these three functions are put into
 * 'jacobi'.
 * [RETURN VALUE]
 * 'rkChainLinkAMJacobi()', 'rkChainAMJacobi()' and
 * 'rkChainAMCOMJacobi()' return a pointer 'jacobi'.
 */
__EXPORT zMat rkChainLinkAMJacobi(rkChain *c, int id, zVec3D *p, zMat jacobi);
__EXPORT zMat rkChainAMJacobi(rkChain *c, zVec3D *p, zMat jacobi);
__EXPORT zMat rkChainAMCOMJacobi(rkChain *c, zMat jacobi);

/* METHOD:
 * rkJacobiManip
 * - measure of manipulability.
 *
 * 'rkJacobiManip()' computes the manipulability, proposed
 * by T. Yoshikawa(1984), of a physical amount related to
 * a Jacobian matrix 'jacobi'. Typically, 'jacobi' is a
 * manipulator Jacobian matrix of a kinematic chain as defined
 * originally by Yoshikawa. However, it does not concern
 * this function. 'jacobi' could be COM Jacobian matrix,
 * angular momentum Jacobian matrix, etc. In any case,
 * the value calculated represents the manipulability
 * in a certain sense.
 * [RETURN VALUE]
 * 'rkJacobiManip()' returns a value calculated.
 */
__EXPORT double rkJacobiManip(zMat jacobi);

__END_DECLS

#endif /* __RK_JACOBI_H__ */
