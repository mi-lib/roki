/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_abi - Articulated Body Inertia Method.
 * contributer: 2014-2015 Naoki Wakisaka
 */

#ifndef __RK_ABI_H__
#define __RK_ABI_H__

#include <roki/rk_chain.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkABI
 * ********************************************************** */

/* allocate memory for ABI of a link. */
__EXPORT rkLink *rkLinkAllocABI(rkLink *link);
/* allocate memory for ABI of a kinematic chain. */
__EXPORT rkChain *rkChainAllocABI(rkChain *chain);

/* destroy ABI of a link. */
__EXPORT void rkLinkDestroyABI(rkLink *link);
/* destroy ABI of a kinematic chain. */
__EXPORT void rkChainDestroyABI(rkChain *chain);

/* initialize ABI of a link for recursive computation. */
__EXPORT void rkLinkInitABI(rkLink *link, zVec6D *pvel);
/* backward computation to update ABI of a link. */
__EXPORT void rkLinkUpdateABIBackward(rkLink *link);
/* forward computation to update acceleration from ABI of a link. */
__EXPORT void rkLinkUpdateABIForward(rkLink *link, zVec6D *pa);
/* forward computation to update acceleration and wrench from ABI of a link. */
__EXPORT void rkLinkUpdateABIWrenchForward(rkLink *link, zVec6D *pa);

/* initialize ABI of a kinematic chain for recursive computation. */
__EXPORT void rkChainInitABI(rkChain *chain);
/* backward computation to update ABI of a kinematic chain. */
#define rkChainUpdateABIBackward(c) rkLinkUpdateABIBackward( rkChainRoot(c) )
/* forward computation to update acceleration from ABI of a kinematic chain. */
#define rkChainUpdateABIForward(c) rkLinkUpdateABIForward( rkChainRoot(c), ZVEC6DZERO )
/* forward computation to update acceleration and wrench from ABI of a kinematic chain. */
#define rkChainUpdateABIWrenchForward(c) rkLinkUpdateABIWrenchForward( rkChainRoot(c), ZVEC6DZERO )

/* update ABI and acceleration of a kinematic chain. */
__EXPORT void rkChainUpdateABI(rkChain *chain);
/* update ABI, acceleration and wrench of a kinematic chain. */
__EXPORT void rkChainUpdateABIWrench(rkChain *chain);

/* compute accleration of a kinematic chain based on ABI method. */
__EXPORT zVec rkChainFD_ABI(rkChain *chain, zVec dis, zVec vel, zVec acc);

__EXPORT void rkChainUpdateCachedABI(rkChain *chain);
__EXPORT void rkChainUpdateCachedABIWrench(rkChain *chain);

__EXPORT void rkChainSaveABIAccBias(rkChain *chain);
__EXPORT void rkChainRestoreABIAccBias(rkChain *chain);
__EXPORT void rkChainRestoreABIAccBiasPair(rkChain *chain, rkLink *link, rkLink *link2);
__EXPORT void rkChainUpdateCachedABIPair(rkChain *chain, rkLink *link, rkWrench *w, rkLink *link2, rkWrench *w2);

__END_DECLS

#endif /* __RK_ABI_H__ */
