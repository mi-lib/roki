/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_g - acceleration of gravity
 */

#ifndef __RK_G_H__
#define __RK_G_H__

/*! \brief acceleration of gravity
 *
 * RK_G_M  [m/s^2]
 * RK_G_MM [mm/s^2]
 * RK_G is a default macro(bound with RK_G_M).
 * \note
 * It does not make sense to define too much finely defined
 * acceleration of gravity since it varies depending on places.
 */
#define RK_G_M  9.806652
#define RK_G_MM 9806.652
#define RK_G    RK_G_M

/*! \brief conversion between Kgf and Newton.
 *
 * rkKgf2N() converts \a f in Kgf to the equal Newton value.
 * rkN2Kgf() converts \a f in Newton to the equal Kgf value.
 * \return
 * The values converted are returned.
 */
#define rkKgf2N(f) ( (f) * RK_G_M )
#define rkN2Kgf(f) ( (f) / RK_G_M )

#endif /* __RK_G_H__ */
