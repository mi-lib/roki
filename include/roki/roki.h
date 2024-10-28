/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 */

/*!
 * \mainpage

 RoKi is a library for computation of robot kinetics including:
 - forward kinematics
 - inverse kinematics
 - inverse dynamics
 - differential kinematics
 - articulated body inertia computation
 - collision detection
 */

#ifndef __ROKI_H__
#define __ROKI_H__

#include <roki/rk_abi.h>
#include <roki/rk_cd.h>

#if defined( __ZEDA_USE_LIBXML ) && defined( __ROKI_USE_URDF )
#include <roki/rk_chain_urdf.h>
#endif

#endif /* __ROKI_H__ */
