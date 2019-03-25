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
__EXPORT void rkLinkABIAlloc(rkLink *link);
/* allocate memory for ABI of a kinematic chain. */
__EXPORT void rkChainABIAlloc(rkChain *chain);

/* destroy ABI of a link. */
__EXPORT void rkLinkABIDestroy(rkLink *link);
/* destroy ABI of a kinematic chain. */
__EXPORT void rkChainABIDestroy(rkChain *chain);

/* initialize ABI of a link for recursive computation. */
__EXPORT void rkLinkABIUpdateInit(rkLink *link, zVec6D *pvel);
/* backward computation to update ABI of a link. */
__EXPORT void rkLinkABIUpdateBackward(rkLink *link);
/* forward computation to update acceleration from ABI of a link. */
__EXPORT void rkLinkABIUpdateForward(rkLink *link, zVec6D *pa);
/* forward computation to update acceleration and wrench from ABI of a link. */
__EXPORT void rkLinkABIUpdateForwardGetWrench(rkLink *link, zVec6D *pa);

/* initialize ABI of a kinematic chain for recursive computation. */
__EXPORT void rkChainABIUpdateInit(rkChain *chain);
/* backward computation to update ABI of a kinematic chain. */
#define rkChainABIUpdateBackward(c) rkLinkABIUpdateBackward( rkChainRoot(c) )
/* forward computation to update acceleration from ABI of a kinematic chain. */
#define rkChainABIUpdateForward(c) rkLinkABIUpdateForward( rkChainRoot(c), ZVEC6DZERO )
/* forward computation to update acceleration and wrench from ABI of a kinematic chain. */
#define rkChainABIUpdateForwardGetWrench(c) rkLinkABIUpdateForwardGetWrench( rkChainRoot(c), ZVEC6DZERO )

/* update ABI and acceleration of a kinematic chain. */
__EXPORT void rkChainABIUpdate(rkChain *chain);
/* update ABI, acceleration and wrench of a kinematic chain. */
__EXPORT void rkChainABIUpdateGetWrench(rkChain *chain);
__EXPORT void rkChainABIUpdateAddExForce(rkChain *chain);
__EXPORT void rkChainABIUpdateAddExForceGetWrench(rkChain *chain);

/* compute accleration of a kinematic chain based on ABI method. */
__EXPORT zVec rkChainABI(rkChain *chain, zVec dis, zVec vel, zVec acc);

__EXPORT void rkChainABIPushPrpAccBias(rkChain *chain);
__EXPORT void rkChainABIPopPrpAccBias(rkChain *chain);
__EXPORT void rkChainABIPopPrpAccBiasAddExForceTwo(rkChain *chain, rkLink *link, rkLink *link2);
__EXPORT void rkChainABIUpdateAddExForceTwo(rkChain *chain, rkLink *link, rkWrench *w, rkLink *link2, rkWrench *w2);

__END_DECLS

#endif /* __RK_ABI_H__ */
