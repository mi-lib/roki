/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_cd_brep_trunc - collision detection manager: truncation of B-Rep.
 */

#ifndef __RK_CD_BREP_TRUNC_H__
#define __RK_CD_BREP_TRUNC_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/*! \brief
 *
 * zBREPTrunc() directly truncates a B-Rep shape \a brep by
 * a cutting plane \a pl.
 *
 * Note that this operation is destructive.
 *
 * zBREPTruncPH3D() directly truncates \a brep by a polyhedron
 * \a ph. It internally truncates \a brep by all faces of \a ph.
 * \return
 * zBREPTrunc() returns a pointer \a brep if it succeeds to
 * create a truncated shape. If it fails to allocate internal
 * workspaces, the null pointer is returned.
 *
 * zBREPTruncPH3D() returns \a brep if it succeeds.
 * The null pointer is returned according to the same condition
 * with zBREPTrunc().
 */
__EXPORT rkBREP *rkBREPTrunc(rkBREP *brep, zPlane3D *pl);
__EXPORT rkBREP *rkBREPTruncPH3D(rkBREP *brep, zPH3D *ph);

__END_DECLS

#endif /* __RK_CD_BREP_TRUNC_H__ */
