/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_g3d - 3D acceleration of gravity
 */

#ifndef __RK_G3D_H__
#define __RK_G3D_H__

#include <zm/zm.h>
#include <zeo/zeo.h>

#include <roki/rk_g.h>

__BEGIN_DECLS

/* OBJECT:
 * RK_GRAVITY3D, RK_GRAVITY6D
 * acceleration vector of gravity.
 */
extern zVec6D rk_gravity6D;
#define RK_GRAVITY6D ( &rk_gravity6D )
#define RK_GRAVITY3D zVec6DLin( RK_GRAVITY6D )

#define rkGravitySet(g)  zVec6DSetElem( RK_GRAVITY6D, zZ, g )
#define rkGravityReset() rkGravitySet( RK_G )

__END_DECLS

#endif /* __RK_G3D_H__ */
