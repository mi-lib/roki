/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_brep - B-Rep (boundary representation)
 */

#ifndef __RK_CD_BREP_H__
#define __RK_CD_BREP_H__

#include <zeo/zeo_ph.h>
#include <zeo/zeo_col.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkBREPVert, rkBREPEdge, rkBREPFace
 * B-Rep vertex, edge and face class
 * ********************************************************** */

typedef struct{
  zVec3D *p, _ps;
  double _d; /* for beneath-beyond test */
  void *_p; /* pointer to destination vector */
	bool org, discard;
} rkBREPVert;
zListClass( rkBREPVertList, rkBREPVertListCell, rkBREPVert );

#define rkBREPVertListCellInit(c,v) do{\
	(c)->data.p = v; \
  (c)->data._d = 0; /* dummy */\
  (c)->data._p = NULL;\
	} while(0)

/*! \brief find vertex cell from a list.
 *
 * rkBREPVertListFind() finds a cell which has the same vertex
 * with \a v from a list \a vlist.
 * \retval
 * rkBREPVertListFind() returns a pointer to the found
 * cell. Otherwise, the null pointer is returned.
 */
__EXPORT rkBREPVertListCell *rkBREPVertListFind(rkBREPVertList *vlist, zVec3D *v);

typedef struct{
  rkBREPVertListCell *v[2], *_v0[2];
  void *_v;
	bool org, discard;
} rkBREPEdge;
zListClass( rkBREPEdgeList, rkBREPEdgeListCell, rkBREPEdge );

#define rkBREPEdgeListCellInit(e,v1,v2) do{\
  (e)->data.v[0] = (v1);\
  (e)->data.v[1] = (v2);\
  (e)->data._v  = NULL;\
} while(0)

/*! \brief find edge cell from a list.
 *
 * rkBREPEdgeListFind() finds a cell which has an edge
 * consisting of vertices \a v1 and \a v2 from a list \a elist.
 * \retval
 * rkBREPEdgeListFind() returns a pointer to the found
 * cell. Otherwise, the null pointer is returned.
 */
__EXPORT rkBREPEdgeListCell *rkBREPEdgeListFind(rkBREPEdgeList *elist, rkBREPVertListCell *v1, rkBREPVertListCell *v2);

typedef struct{
  rkBREPVertListCell *v[3], *_v0[3];
  rkBREPEdgeListCell *e[3], *_e0[3];
  zVec3D norm; /* to be used. */
	bool org, discard;
} rkBREPFace;
zListClass( rkBREPFaceList, rkBREPFaceListCell, rkBREPFace );

/* ********************************************************** */
/* CLASS: rkBREP
 * B-Rep class
 * ********************************************************** */

typedef struct{
  rkBREPVertList vlist;
  rkBREPEdgeList elist;
  rkBREPFaceList flist;
	int discard;
} rkBREP;

__EXPORT void rkBREPInit(rkBREP *brep);

/*! \brief convert a polyhedron to a B-Rep solid.
 *
 * B-Rep solid, an instance of rkBREP class, has internal
 * lists of boundaries, namely, vertices, edges and faces.
 * The solid is represented by a set of faces, which
 * consist of three vertices and three edges bounded.
 *
 * zPH3D2BREP() converts a polyhedron \a ph to a B-Rep aolid.
 * The result is stored where \a brep points.
 * \retval
 * zPH3D2BREP() returns a pointer \a brep.
 */
__EXPORT rkBREP *rkPH3D2BREP(zPH3D *ph, rkBREP *brep);

/*! \brief convert polyhedron restricted in a box to B-Rep solid.
 *
 * zPH3D2BREPInBox() converts vertices and faces of a
 * polyhedron \a ph only intersecting with an axis-aligned
 * box \a box to a B-Rep aolid. The result is stored where
 * \a brep points.
 * \retval
 * zPH3D2BREPInBox() returns a pointer \a brep.
 */
__EXPORT rkBREP *rkPH3D2BREPInBox(zPH3D *ph, zAABox3D *box, rkBREP *brep);

/*! \brief convert a B-Rep solid to a polyhedron.
 *
 * rkBREP2PH3D() converts \a brep to a polyhedron \a ph.
 * The internal workspaces of \a ph and \a brep are
 * completely independent; destruction of one does not
 * harm the other.
 * \retval
 * rkBREP2PH3D() returns a pointer \a ph.
 */
__EXPORT zPH3D *rkBREP2PH3D(rkBREP *brep, zPH3D *ph);

/*! \brief destroy a B-Rep solid.
 *
 * rkBREPDestroy() destroys B-rep solid \a brep, freeing
 * internal boundary list.
 * \retval
 * rkBREPDestroy() returns no value.
 */
__EXPORT void rkBREPDestroy(rkBREP *brep);

__EXPORT void rkBREPReset(rkBREP *brep);

__END_DECLS

#include <roki/rk_cd_brep_trunc.h> /* B-Rep truncation */

#endif /* __RK_CD_BREP_H__ */
