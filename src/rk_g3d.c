/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_g3d - 3D acceleration of gravity
 */

#include <roki/rk_g3d.h>

/* OBJECT:
 * rk_gravity6D
 * acceleration vector of gravity.
 */
zVec6D rk_gravity6D = { { 0, 0, RK_G, 0, 0, 0 } };
