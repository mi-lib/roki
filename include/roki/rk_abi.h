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

__EXPORT void rkLinkABIInit(rkLink *link);
__EXPORT void rkChainABIInit(rkChain *chain);

__EXPORT void rkLinkABIDestroy(rkLink *link);
__EXPORT void rkChainABIDestroy(rkChain *chain);

__EXPORT void rkLinkABIUpdateInit(rkLink *link, zVec6D *pvel);
__EXPORT void rkLinkABIUpdateBackward(rkLink *link);
__EXPORT void rkLinkABIUpdateForward(rkLink *link, zVec6D *pa);
__EXPORT void rkLinkABIUpdateForwardGetWrench(rkLink *link, zVec6D *pa);

__EXPORT void rkChainABIUpdateInit(rkChain *chain);
#define rkChainABIUpdateBackward(c) rkLinkABIUpdateBackward( rkChainRoot(c) )
#define rkChainABIUpdateForward(c)  rkLinkABIUpdateForward( rkChainRoot(c), ZVEC6DZERO )
#define rkChainABIUpdateForwardGetWrench(c)  rkLinkABIUpdateForwardGetWrench( rkChainRoot(c), ZVEC6DZERO )

__EXPORT void rkChainABIUpdate(rkChain *chain);
__EXPORT void rkChainABIUpdateGetWrench(rkChain *chain);
__EXPORT void rkChainABIUpdateAddExForce(rkChain *chain);
__EXPORT void rkChainABIUpdateAddExForceGetWrench(rkChain *chain);

__EXPORT zVec rkChainABI(rkChain *chain, zVec dis, zVec vel, zVec acc);

__EXPORT void rkChainABIPushPrpAccBias(rkChain *chain);
__EXPORT void rkChainABIPopPrpAccBias(rkChain *chain);
__EXPORT void rkChainABIPopPrpAccBiasAddExForceTwo(rkChain *chain, rkLink *link, rkLink *link2);
__EXPORT void rkChainABIUpdateAddExForceTwo(rkChain *chain, rkLink *link, rkWrench *w, rkLink *link2, rkWrench *w2);

__END_DECLS

#endif /* __RK_ABI_H__ */
