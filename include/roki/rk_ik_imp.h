/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_ik_imp - inverse kinematics: impedance control
 * NOTE!: This is only for testing.
 *        In practice, endpoint-designed impedance is preferable.
 */

#ifndef __RK_IK_IMP_H__
#define __RK_IK_IMP_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkIKImp
 * inverse kinematics impedance control parameter set class
 * ********************************************************** */

typedef struct{
  zVec3D k;  /*!< displacement compensation gain (stiffness) */
  zVec3D c;  /*!< velocity compensation gain (damper) */
} rkIKImp;

#define rkIKImpSetK(i,kx,ky,kz) zVec3DCreate( &(i)->k, kx, ky, kz )
#define rkIKImpSetC(i,cx,cy,cz) zVec3DCreate( &(i)->c, cx, cy, cz )

/* controller */
__ROKI_EXPORT zVec3D *rkIKImpTtlAtt(rkChain *chain, rkIKAttr *attr, void *priv, rkIKRef *ref, zVec3D *srv);
__ROKI_EXPORT zVec3D *rkIKImpTtlPos(rkChain *chain, rkIKAttr *attr, void *priv, rkIKRef *ref, zVec3D *srv);
__ROKI_EXPORT zVec3D *rkIKImpWldAtt(rkChain *chain, rkIKAttr *attr, void *priv, rkIKRef *ref, zVec3D *srv);
__ROKI_EXPORT zVec3D *rkIKImpWldPos(rkChain *chain, rkIKAttr *attr, void *priv, rkIKRef *ref, zVec3D *srv);
__ROKI_EXPORT zVec3D *rkIKImpWldCOM(rkChain *chain, rkIKAttr *attr, void *priv, rkIKRef *ref, zVec3D *srv);

__END_DECLS

#endif /* __RK_IK_IMP_H__ */
