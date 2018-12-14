/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_brep_trunc - truncation of B-Rep.
 */

#include <roki/rk_cd_brep.h>

static int _rkBREPTruncBBCheck(rkBREPVertList *vlist, zPlane3D *pl);
static bool _rkBREPTruncIntersect(rkBREP *brep, zPlane3D *pl);

static bool _rkBREPTruncFaceA(rkBREP *brep, rkBREPFace *f, int i1, int i2, int i3);
static bool _rkBREPTruncFaceV(rkBREP *brep, rkBREPFace *f, int i1, int i2, int i3);
static bool _rkBREPTruncFaceE(rkBREP *brep, rkBREPFace *f, int i1, int i2, int i3);
static bool _rkBREPTruncFace(rkBREP *brep, rkBREPFaceListCell **fp);
static void _rkBREPTruncEdgeShrink(rkBREPEdgeList *elist);
static void _rkBREPTruncGR(rkBREP *brep);

/* (static)
 * _rkBREPTruncBBCheck
 * - beneath-beyond check with respect to a plane.
 */
int _rkBREPTruncBBCheck(rkBREPVertList *vlist, zPlane3D *pl)
{
  int count = 0;
  rkBREPVertListCell *vp;
	zVec3D tmp;

  zListForEach( vlist, vp ){
		if( vp->data.discard ) continue;
		zVec3DSub( vp->data.p, zPlane3DVert(pl), &tmp );
		vp->data._d = zVec3DInnerProd( &tmp, zPlane3DNorm(pl) );
    /* vp->data._d = zPlane3DPointDist( pl, &vp->data.p ); */
    if( zIsTiny( vp->data._d ) ) vp->data._d = 0;
    if( vp->data._d > 0 ) count++;
  }
  return count;
}

/* (static)
 * _rkBREPTruncIntersect
 * - compute intersections of edges and a cutting plane.
 */
bool _rkBREPTruncIntersect(rkBREP *brep, zPlane3D *pl)
{
  rkBREPEdgeListCell *ep;
  rkBREPVertListCell *vp;
  double d1, d2;

  zListForEach( &brep->elist, ep ){
		if( ep->data.discard ) continue;
    if( ep->data.v[0]->data._d * ep->data.v[1]->data._d < 0 ){
      if( !( vp = zAlloc( rkBREPVertListCell, 1 ) ) ){
        ZALLOCERROR();
        return false;
      }
			/* if( !( vp->data.p = zAlloc( zVec3D, 1 ) ) ){ */
			/* 	ZALLOCERROR(); */
			/* 	zFree( vp ); */
      /*   return false; */
			/* } */
			/* debug */
			/* eprintf("ev1\n"); */
			/* zVec3DFWrite(stderr,ep->data.v[0]->data.p); */
			/* eprintf("ev1\n"); */
			/* zVec3DFWrite(stderr,ep->data.v[1]->data.p); */
			/* eprintf("v\n"); */
			/* zVec3DFWrite(stderr,vp->data.p); */

      vp->data._d = 0;
      vp->data._p = NULL;
      d1 = fabs( ep->data.v[0]->data._d );
      d2 = fabs( ep->data.v[1]->data._d );
      zVec3DInterDiv( ep->data.v[0]->data.p, ep->data.v[1]->data.p,
        d1/(d1+d2), &vp->data._ps );
			vp->data.p = &vp->data._ps;
      zListInsertHead( &brep->vlist, vp );
      ep->data._v = vp;
    }
  }
  return true;
}

/* (static)
 * _rkBREPTruncFaceA
 * - A type face reconfiguration (2-beneath, 1-beyond).
 */
bool _rkBREPTruncFaceA(rkBREP *brep, rkBREPFace *f, int i1, int i2, int i3)
{
  rkBREPEdgeListCell *e4, *e5;
  rkBREPFaceListCell *fp;

  e4 = zAlloc( rkBREPEdgeListCell, 1 );
  e5 = zAlloc( rkBREPEdgeListCell, 1 );
  fp = zAlloc( rkBREPFaceListCell, 1 );
  if( !e4 || !e5 || !fp ){
    ZALLOCERROR();
    zFree( e4 );
    zFree( e5 );
    zFree( fp );
    return false;
  }
  /* trim edge */
  rkBREPEdgeListCellInit( e4, f->e[i2]->data._v, f->e[i3]->data._v );
	e4->data.org = false;
	e4->data.discard = false;
  zListInsertHead( &brep->elist, e4 );
  /* bracing edge */
  rkBREPEdgeListCellInit( e5, f->e[i2]->data._v, f->v[i2] );
	e5->data.org = false;
	e5->data.discard = false;
  zListInsertHead( &brep->elist, e5 );
  /* upper triangle */
  fp->data.v[0] = f->v[i2];
  fp->data.v[1] = f->e[i2]->data._v;
  fp->data.v[2] = f->e[i3]->data._v;
  fp->data.e[0] = e4;
  fp->data.e[1] = f->e[i3];
  fp->data.e[2] = e5;
	fp->data.org = false;
	fp->data.discard = false;
  zListInsertTail( &brep->flist, fp );
  /* lower triangle */
  f->e[i3] = e5;
  f->v[i1] = f->e[i2]->data._v;
  return true;
}

/* (static)
 * _rkBREPTruncFaceV
 * - V type face reconfiguration (1-beneath, 2-beyond).
 */
bool _rkBREPTruncFaceV(rkBREP *brep, rkBREPFace *f, int i1, int i2, int i3)
{
  rkBREPEdgeListCell *e4;

  if( !( e4 = zAlloc( rkBREPEdgeListCell, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  /* trim edge */
  rkBREPEdgeListCellInit( e4, f->e[i3]->data._v, f->e[i2]->data._v );
	e4->data.org = false;
	e4->data.discard = false;
  zListInsertHead( &brep->elist, e4 );
  /* renewal triangle */
  f->e[i1] = e4;
  f->v[i2] = f->e[i3]->data._v;
  f->v[i3] = f->e[i2]->data._v;
  return true;
}

/* (static)
 * _rkBREPTruncFaceE
 * - E type face reconfiguration (1-beneath, 1-beyond, 1-border).
 */
bool _rkBREPTruncFaceE(rkBREP *brep, rkBREPFace *f, int i1, int i2, int i3)
{
  rkBREPEdgeListCell *e4;

  if( !( e4 = zAlloc( rkBREPEdgeListCell, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  /* trim edge */
  rkBREPEdgeListCellInit( e4, f->v[i1], f->e[i1]->data._v );
	e4->data.org = false;
	e4->data.discard = false;
  zListInsertHead( &brep->elist, e4 );
  /* renewal triangle */
  f->v[i3] = f->e[i1]->data._v;
  f->e[i2] = e4;
  return true;
}

/* (static)
 * _rkBREPTruncEdgeShrink
 * - shrink intersecting edges.
 */
void _rkBREPTruncEdgeShrink(rkBREPEdgeList *elist)
{
  rkBREPEdgeListCell *ep;

  zListForEach( elist, ep ){
    if( ep->data.discard || !ep->data._v ) continue;
    if( ep->data.v[0]->data._d > 0 )
      ep->data.v[0] = ep->data._v;
    else
      ep->data.v[1] = ep->data._v;
    ep->data._v = NULL;
  }
}

/* (static)
 * _rkBREPTruncGR
 * - release garbages.
 */
void _rkBREPTruncGR(rkBREP *brep)
{
  rkBREPVertListCell *vp;
  rkBREPEdgeListCell *ep;
  rkBREPFaceListCell *fp;
  register int i;

  /* initialize mark */
  zListForEach( &brep->vlist, vp )
		if( !vp->data.discard )
			vp->data._p = NULL;
  zListForEach( &brep->elist, ep )
		if( !ep->data.discard )
			ep->data._v = NULL;
  /* mark */
  zListForEach( &brep->flist, fp ){
		if( fp->data.discard ) continue;
    for( i=0; i<3; i++ ){
      fp->data.v[i]->data._p = (void *)0xffff;
      fp->data.e[i]->data._v = (void *)0xffff;
    }
	}
	/* garbage release */
	zListForEach( &brep->vlist, vp ){
		if( vp->data.discard ) continue;
		if( vp->data._p == (void *)0xffff )
			vp->data._p = NULL;
		else{
			vp->data.discard = true;
			brep->discard++;
		}
	}
	zListForEach( &brep->elist, ep ){
		if( ep->data.discard ) continue;
		if( ep->data._v == (void *)0xffff )
			ep->data._v = NULL;
		else
			ep->data.discard = true;
	}
}

/* (static)
 * _rkBREPTruncFace
 * - face reconfiguration.
 */
#define __z_brep_trunc_pat(d) ( (d)<0 ? 3: ( (d)>0 ? 1 : 0 ) )
bool _rkBREPTruncFace(rkBREP *brep, rkBREPFaceListCell **fp)
{
  rkBREPFace *f;

  f = &(*fp)->data;
  switch( __z_brep_trunc_pat( f->v[0]->data._d )
        | __z_brep_trunc_pat( f->v[1]->data._d ) << 2
        | __z_brep_trunc_pat( f->v[2]->data._d ) << 4 ){
  case 0x01: case 0x04: case 0x05:
  case 0x10: case 0x11: case 0x14: case 0x15:
		f->discard = true;
  case 0x00: case 0x03: case 0x0c: case 0x0f:
  case 0x30: case 0x33: case 0x3c: case 0x3f:
    return true;
  case 0x1f: return _rkBREPTruncFaceA( brep, f, 2, 0, 1 );
  case 0x37: return _rkBREPTruncFaceA( brep, f, 1, 2, 0 );
  case 0x3d: return _rkBREPTruncFaceA( brep, f, 0, 1, 2 );
  case 0x07: return _rkBREPTruncFaceE( brep, f, 2, 0, 1 );
  case 0x0d: return _rkBREPTruncFaceE( brep, f, 2, 1, 0 );
  case 0x13: return _rkBREPTruncFaceE( brep, f, 1, 0, 2 );
  case 0x1c: return _rkBREPTruncFaceE( brep, f, 0, 1, 2 );
  case 0x31: return _rkBREPTruncFaceE( brep, f, 1, 2, 0 );
  case 0x34: return _rkBREPTruncFaceE( brep, f, 0, 2, 1 );
  case 0x17: return _rkBREPTruncFaceV( brep, f, 0, 1, 2 );
  case 0x1d: return _rkBREPTruncFaceV( brep, f, 1, 2, 0 );
  case 0x35: return _rkBREPTruncFaceV( brep, f, 2, 0, 1 );
  default:
    ZRUNERROR( ZEO_ERR_FATAL );
  }
  return false;
}

/* rkBREPTrunc
 * - truncate B-Rep by a plane.
 */
rkBREP *rkBREPTrunc(rkBREP *brep, zPlane3D *pl)
{
  rkBREPFaceListCell *fp;

  if( _rkBREPTruncBBCheck( &brep->vlist, pl ) == 0 )
    return brep; /* no need to truncate */
  if( !_rkBREPTruncIntersect( brep, pl ) ) return NULL;
  zListForEach( &brep->flist, fp )
		if( !fp->data.discard )
			if( !_rkBREPTruncFace( brep, &fp ) ) return NULL;

  _rkBREPTruncEdgeShrink( &brep->elist );
  _rkBREPTruncGR( brep );

  return brep;
}

/* rkBREPTruncPH3D
 * - truncate B-Rep by a polyhedron.
 */
rkBREP *rkBREPTruncPH3D(rkBREP *brep, zPH3D *ph)
{
  register int i;
  zPlane3D pl;

  for( i=0; i<zPH3DFaceNum(ph); i++ ){
    zTri3DToPlane3D( zPH3DFace(ph,i), &pl );
    if( !rkBREPTrunc( brep, &pl ) ) return NULL;
  }
  return brep;
}
