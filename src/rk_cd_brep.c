/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_brep - B-Rep (boundary representation)
 */

#include <roki/rk_cd_brep.h>

/* ********************************************************** */
/* CLASS: zBREPVert, zBREPEdge, zBREPFace
 * B-Rep vertex, edge and face class
 * ********************************************************** */

static rkBREPVertListCell *_rkBREPVertListFindnReg(rkBREPVertList *vlist, zVec3D *v);
static rkBREPEdgeListCell *_rkBREPEdgeListFindnReg(rkBREPEdgeList *elist, rkBREPVertListCell *v1, rkBREPVertListCell *v2);
static bool _rkBREPFaceInsert(zTri3D *face, rkBREPFaceList *flist, rkBREPEdgeList *elist, rkBREPVertList *vlist);

void rkBREPInit(rkBREP *brep)
{
	zListInit( &brep->vlist );
  zListInit( &brep->elist );
  zListInit( &brep->flist );
}

/* rkBREPVertListFind
 * - find vertex list cell.
 */
rkBREPVertListCell *rkBREPVertListFind(rkBREPVertList *vlist, zVec3D *v)
{
  rkBREPVertListCell *vp;

  zListForEach( vlist, vp )
    if( zVec3DEqual( vp->data.p, v ) ) return vp;
  return NULL;
}

/* (static)
 * _rkBREPVertListFindnReg
 * - find or register a vertex list cell.
 */
rkBREPVertListCell *_rkBREPVertListFindnReg(rkBREPVertList *vlist, zVec3D *v)
{
  rkBREPVertListCell *vp;

  if( ( vp = rkBREPVertListFind( vlist, v ) ) ) return vp;
  if( !( vp = zAlloc( rkBREPVertListCell, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  rkBREPVertListCellInit( vp, v );
	vp->data.org = true;
	vp->data.discard = false;
  zListInsertHead( vlist, vp );
  return vp;
}

/* rkBREPEdgeListFind
 * - find edge list cell.
 */
rkBREPEdgeListCell *rkBREPEdgeListFind(rkBREPEdgeList *elist, rkBREPVertListCell *v1, rkBREPVertListCell *v2)
{
  rkBREPEdgeListCell *ep;

  zListForEach( elist, ep )
    if( ( ep->data.v[0] == v1 && ep->data.v[1] == v2 ) ||
        ( ep->data.v[0] == v2 && ep->data.v[1] == v1 ) ) return ep;
  return NULL;
}

/* (static)
 * _rkBREPEdgeListFindnReg
 * - find or register an edge list cell.
 */
rkBREPEdgeListCell *_rkBREPEdgeListFindnReg(rkBREPEdgeList *elist, rkBREPVertListCell *v1, rkBREPVertListCell *v2)
{
  rkBREPEdgeListCell *ep;

  if( ( ep = rkBREPEdgeListFind( elist, v1, v2 ) ) ) return ep;
  if( !( ep = zAlloc( rkBREPEdgeListCell, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  rkBREPEdgeListCellInit( ep, v1, v2 );
	ep->data._v0[0] = ep->data.v[0];
	ep->data._v0[1] = ep->data.v[1];
	ep->data.org = true;
	ep->data.discard = false;
  zListInsertHead( elist, ep );
  return ep;
}

/* (static)
 * _rkBREPFaceInsert
 * - insert face list cell.
 */
bool _rkBREPFaceInsert(zTri3D *face, rkBREPFaceList *flist, rkBREPEdgeList *elist, rkBREPVertList *vlist)
{
  rkBREPFaceListCell *f;
  register int i;

  if( !( f = zAlloc( rkBREPFaceListCell, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  for( i=0; i<3; i++ ){
    f->data._v0[i] = _rkBREPVertListFindnReg( vlist, zTri3DVert(face,i) );
		f->data.v[i] = f->data._v0[i];
	}
  for( i=0; i<3; i++ ){
    f->data._e0[i] = _rkBREPEdgeListFindnReg( elist, f->data.v[(i+1)%3], f->data.v[(i+2)%3] );
		f->data.e[i] = f->data._e0[i];
	}
  zVec3DCopy( zTri3DNorm(face), &f->data.norm );
	f->data.org = true;
	f->data.discard = false;
  zListInsertHead( flist, f );
  return true;
}

/* ********************************************************** */
/* CLASS: rkBREP
 * B-Rep class
 * ********************************************************** */

/* rkPH3D2BREP
 * - convert polyhedron to B-Rep solid.
 */
rkBREP *rkPH3D2BREP(zPH3D *ph, rkBREP *brep)
{
  register int i;

  zListInit( &brep->vlist );
  zListInit( &brep->elist );
  zListInit( &brep->flist );
  for( i=0; i<zPH3DFaceNum(ph); i++ )
    if( !_rkBREPFaceInsert( zPH3DFace(ph,i), &brep->flist, &brep->elist, &brep->vlist ) ){
      ZRUNERROR( ZEO_ERR_BREP_CONV );
      rkBREPDestroy( brep );
      return NULL;
    }
  return brep;
}

/* rkPH3D2BREPInBox
 * - convert polyhedron restricted in a box to B-Rep solid.
 */
rkBREP *rkPH3D2BREPInBox(zPH3D *ph, zAABox3D *box, rkBREP *brep)
{
  zTri3D *tri;
  register int i;

  /* zListInit( &brep->vlist ); */
  /* zListInit( &brep->elist ); */
  /* zListInit( &brep->flist ); */
  for( i=0; i<zPH3DFaceNum(ph); i++ ){
    if( zColChkTriAABox3D( ( tri = zPH3DFace(ph,i) ), box ) ){
      if( !_rkBREPFaceInsert( zPH3DFace(ph,i), &brep->flist, &brep->elist, &brep->vlist ) ){
        ZRUNERROR( ZEO_ERR_BREP_CONV );
        rkBREPDestroy( brep );
        return NULL;
      }
    }
  }
  return brep;
}

/* rkBREP2PH3D
 * - convert B-Rep solid to polyhedron.
 */
zPH3D *rkBREP2PH3D(rkBREP *brep, zPH3D *ph)
{
  register int i;
  rkBREPVertListCell *vp;
  rkBREPFaceListCell *fp;

  if( !zPH3DAlloc( ph, zListNum(&brep->vlist), zListNum(&brep->flist) ) ){
    ZALLOCERROR();
    return NULL;
  }
  i = 0;
  zListForEach( &brep->vlist, vp ){
    zVec3DCopy( &vp->data.p, zPH3DVert(ph,i) );
    vp->data._p = (void *)zPH3DVert(ph,i);
    i++;
  }
  i = 0;
  zListForEach( &brep->flist, fp ){
    zTri3DCreate( zPH3DFace(ph,i), fp->data.v[0]->data._p, fp->data.v[1]->data._p, fp->data.v[2]->data._p );
    i++;
  }
  return ph;
}

/* rkBREPDestroy
 * - destroy B-Rep solid.
 */
void rkBREPDestroy(rkBREP *brep)
{
  zListDestroy( rkBREPFaceListCell, &brep->flist );
  zListDestroy( rkBREPEdgeListCell, &brep->elist );
  zListDestroy( rkBREPVertListCell, &brep->vlist );
}

/* reset brep */
void rkBREPReset(rkBREP *brep)
{
	rkBREPVertListCell *vp, *vpp;
	rkBREPEdgeListCell *ep, *epp;
  rkBREPFaceListCell *fp, *fpp;
	register int i;

	brep->discard = 0;
	zListForEach( &brep->vlist, vp ){
		if( vp->data.org ){
			vp->data._d = 0;
			vp->data._p = NULL;
			vp->data.discard = false;
		} else {
			vp = zListCellPrev( vp );
      zListDeleteNext( &brep->vlist, vp, &vpp );
      zFree( vpp );
		}
	}
	zListForEach( &brep->elist, ep ){
		if( ep->data.org ){
			ep->data.v[0] = ep->data._v0[0];
			ep->data.v[1] = ep->data._v0[1];
			ep->data._v = NULL;
			ep->data.discard = false;
		} else {
			ep = zListCellPrev( ep );
      zListDeleteNext( &brep->elist, ep, &epp );
      zFree( epp );
		}
	}
	zListForEach( &brep->flist, fp ){
		if( fp->data.org ){
			for( i=0; i<3; i++ ){
				fp->data.v[i] = fp->data._v0[i];
				fp->data.e[i] = fp->data._e0[i];
			}
			fp->data.discard = false;
		} else {
			fp = zListCellPrev( fp );
      zListDeleteNext( &brep->flist, fp, &fpp );
      zFree( fpp );
		}
	}
}
