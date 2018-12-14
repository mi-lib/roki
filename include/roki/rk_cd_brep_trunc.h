/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_brep_trunc - truncation of B-Rep.
 */

#ifndef __RK_CD_BREP_TRUNC_H__
#define __RK_CD_BREP_TRUNC_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* METHOD:
 * zBREPTrunc, zBREPTruncPH3D - truncate B-rep.
 * [SYNOPSIS]
 * zBREP *zBREPTrunc(zBREP *brep, zPlane3D *pl);
 * zBREP *zBREPTruncPH3D(zBREP *brep, zPH3D *ph);
 * [DESCRIPTION]
 * 'zBREPTrunc()' directly truncate a B-Rep shape 'brep'
 * by a cutting plane 'pl'. Note that this operation is
 * destructive.
 * #
 * 'zBREPTruncPH3D()' directly truncate 'brep' by a
 * polyhedron 'ph'. It internally truncate 'brep' by
 * all faces of 'ph'.
 * [RETURN VALUE]
 * 'zBREPTrunc()' returns a pointer 'brep' if succeeding
 * to create a truncated shape. If failing to allocate
 * internal workspaces, the null pointer is returned.
 * #
 * 'zBREPTruncPH3D()' returns 'brep' if succeeding.
 * The null pointer is returned according to the same
 * condition with 'zBREPTrunc()'.
 */
__EXPORT rkBREP *rkBREPTrunc(rkBREP *brep, zPlane3D *pl);
__EXPORT rkBREP *rkBREPTruncPH3D(rkBREP *brep, zPH3D *ph);

__END_DECLS

#endif /* __ZEO_BREP_TRUNC_H__ */
