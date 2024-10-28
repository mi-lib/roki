/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_chain_urdf - read a URDF file.
 */

#ifndef __RK_CHAIN_URDF_H__
#define __RK_CHAIN_URDF_H__

#include <roki/rk_chain.h>

__BEGIN_DECLS

/*! \brief read a URDF file and create an instance of rkChain. */
__ROKI_EXPORT rkChain *rkChainReadURDF(rkChain *chain, const char *filename);

/*! \brief directly convert a URDF file to a ZTK file. */
__ROKI_EXPORT bool rkURDF2ZTK(const char *inputfilename, const char *outputfilename);

__END_DECLS

#endif /* __RK_CHAIN_URDF_H__ */
