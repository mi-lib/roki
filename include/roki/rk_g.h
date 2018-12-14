/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_g - acceleration of gravity
 */

#ifndef __RK_G_H__
#define __RK_G_H__

/* MACRO: RK_G
 * acceleration of gravity
 * - it is nonsense to define the acceleration as more precise
 * value, since it depends on where it is measured.
 * RK_G_M  [m/s^2]
 * RK_G_MM [mm/s^2]
 * RK_G is a default macro(bound with RK_G_M).
 */
#define RK_G_M  9.806652
#define RK_G_MM 9806.652
#define RK_G    RK_G_M

/* METHOD:
 * rkKgf2N, rkN2Kgf - conversion between Kgf and Newton.
 *
 * 'rkKgf2N()' converts 'f' in Kgf to the equal Newton value.
 * 'rkN2Kgf()' converts 'f' in Newton to the equal Kgf value.
 * [RETURN VALUE]
 * The values converted are returned.
 */
#define rkKgf2N(f) ( (f) * RK_G_M )
#define rkN2Kgf(f) ( (f) / RK_G_M )

#endif /* __RK_G_H__ */
