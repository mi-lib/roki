/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_g3d - 3D acceleration of gravity
 */

#ifndef __RK_G3D_H__
#define __RK_G3D_H__

#include <roki/rk_misc.h>
#include <roki/rk_g.h>

__BEGIN_DECLS

/*! \brief acceleration vector due to gravity. */
__ROKI_EXPORT const zVec6D rk_gravity6D;
#define RK_GRAVITY6D ( &rk_gravity6D )
#define RK_GRAVITY3D zVec6DLin( RK_GRAVITY6D )

__END_DECLS

#endif /* __RK_G3D_H__ */
