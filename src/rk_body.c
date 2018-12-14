/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_body - body with mass property
 */

#include <roki/rk_body.h>

/* ********************************************************** */
/* CLASS: rkMP
 * mass property class
 * ********************************************************** */

static zMat3D *_rkMPShiftInertia(zMat3D *src, double m, zVec3D *s, zMat3D *dest);

/* rkMPXfer
 * - transfer a mass property set to that with respect to a frame.
 */
rkMP *rkMPXfer(rkMP *src, zFrame3D *f, rkMP *dest)
{
  rkMPSetMass( dest, rkMPMass(src) );
  zXfer3D( f, rkMPCOM(src), rkMPCOM(dest) );
  zRotMat3D( zFrame3DAtt(f), rkMPInertia(src), rkMPInertia(dest) );
  return dest;
}

/* (static)
 * _rkMPShiftInertia
 * - shift mass property translationally.
 */
zMat3D *_rkMPShiftInertia(zMat3D *src, double m, zVec3D *s, zMat3D *dest)
{
  zMat3D tmp;
  double xx, xy, yy, yz, zz, zx;

  xx = zSqr(s->e[zX]);
  yy = zSqr(s->e[zY]);
  zz = zSqr(s->e[zZ]);
  xy = s->e[zX] * s->e[zY];
  yz = s->e[zY] * s->e[zZ];
  zx = s->e[zZ] * s->e[zX];
  zMat3DCreate( &tmp,
    yy+zz,   -xy,    -zx,
      -xy, zz+xx,    -yz,
      -zx,   -yz,  xx+yy );
  return zMat3DCat( src, m, &tmp, dest );
}

/* rkMPCombine
 * - combine two mass property sets in the same frame.
 */
rkMP *rkMPCombine(rkMP *mp1, rkMP *mp2, rkMP *mp)
{
  zVec3D r1, r2;
  zMat3D i1, i2;

  /* mass */
  rkMPSetMass( mp, rkMPMass(mp1) + rkMPMass(mp2) );
  /* COM */
  zVec3DMul( rkMPCOM(mp1), rkMPMass(mp1)/rkMPMass(mp), &r1 );
  zVec3DMul( rkMPCOM(mp2), rkMPMass(mp2)/rkMPMass(mp), &r2 );
  zVec3DAdd( &r1, &r2, rkMPCOM(mp) );
  /* inertia tensor */
  zVec3DSub( rkMPCOM(mp1), rkMPCOM(mp), &r1 );
  _rkMPShiftInertia( rkMPInertia(mp1), rkMPMass(mp1), &r1, &i1 );
  zVec3DSub( rkMPCOM(mp2), rkMPCOM(mp), &r2 );
  _rkMPShiftInertia( rkMPInertia(mp2), rkMPMass(mp2), &r2, &i2 );
  zMat3DAdd( &i1, &i2, rkMPInertia(mp) );
  return mp;
}

/* rkMPOrgInertia
 * - convert inertia tensor to that about the origin.
 */
zMat3D *rkMPOrgInertia(rkMP *mp, zMat3D *i)
{
  return _rkMPShiftInertia( rkMPInertia(mp), rkMPMass(mp), rkMPCOM(mp), i );
}

/* rkMPInertiaEllips
 * - compute the inertial ellipsoid from a mass property set.
 */
zEllips3D *rkMPInertiaEllips(rkMP *mp, zEllips3D *ie)
{
  zVec3D evec[3];
  double eval[3];

  zMat3DSymEig( rkMPInertia(mp), eval, evec );
  return zEllips3DCreate( ie, rkMPCOM(mp), &evec[0], &evec[1], &evec[2], eval[0], eval[1], eval[2], 0 );
}

/* rkMPFWrite
 * - output mass property.
 */
void rkMPFWrite(FILE *fp, rkMP *mp)
{
  fprintf( fp, "mass: %.10g\n", rkMPMass(mp) );
  fprintf( fp, "COM: " );
  zVec3DFWrite( fp, rkMPCOM(mp) );
  fprintf( fp, "inertia: " );
  zMat3DFWrite( fp, rkMPInertia(mp) );
}

/* ********************************************************** */
/* CLASS: rkBody
 * rigid body class
 * ********************************************************** */

/* rkBodyInit
 * - initialize a body.
 */
void rkBodyInit(rkBody *b)
{
  rkBodySetMass( b, 0 );
  rkBodySetCOM( b, ZVEC3DZERO );
  rkBodySetInertia( b, ZMAT3DZERO );
  rkBodySetFrame( b, ZFRAME3DIDENT );
  rkBodySetVel( b, ZVEC6DZERO );
  rkBodySetAcc( b, ZVEC6DZERO );
  rkBodySetWldCOM( b, ZVEC3DZERO );
  rkBodySetCOMVel( b, ZVEC3DZERO );
  rkBodySetCOMAcc( b, ZVEC3DZERO );

  zListInit( rkBodyExtWrench( b ) );
  zListInit( rkBodyShapeList( b ) );

  rkBodyStuff(b) = NULL;
}

/* rkBodyDestroy
 * - destroy a body.
 */
void rkBodyDestroy(rkBody *b)
{
  rkBodyExtWrenchDestroy( b );
  rkBodyShapeDestroy( b );
  rkBodyStuffDestroy( b );
  rkBodyInit( b );
}

/* rkBodyClone
 * - clone a body.
 */
rkBody *rkBodyClone(rkBody *org, rkBody *cln, zMShape3D *so, zMShape3D *sc)
{
  rkWrench *wp, *wpc;
  zShapeListCell *sp;

  rkMPCopy( &org->mp, &cln->mp );
  /* wrench list */
  zListInit( rkBodyExtWrench(cln) );
  zListForEach( rkBodyExtWrench(org), wp ){
    if( !( wpc = zAlloc( rkWrench, 1 ) ) ) return NULL;
    rkWrenchSetW( wpc, rkWrenchW(wp) );
    rkWrenchSetPos( wpc, rkWrenchPos(wp) );
    rkBodyExtWrenchPush( cln, wpc );
  }
  /* shape list */
  zListInit( rkBodyShapeList(cln) );
  zListForEach( rkBodyShapeList(org), sp )
    if( !rkBodyShapePush( cln, sp->data - zMShape3DShapeBuf(so) + zMShape3DShapeBuf(sc) ) ) return NULL;
  /* stuff */
  if( rkBodyStuff(org) && !rkBodySetStuff( cln, rkBodyStuff(org) ) ){
    ZALLOCERROR();
    return NULL;
  }
  return cln;
}

/* rkBodyCopyState
 * - copy body state.
 */
rkBody *rkBodyCopyState(rkBody *src, rkBody *dst)
{
  zFrame3DCopy( rkBodyFrame(src), rkBodyFrame(dst) );
  zVec6DCopy( rkBodyVel(src), rkBodyVel(dst) );
  zVec6DCopy( rkBodyAcc(src), rkBodyAcc(dst) );
  zVec3DCopy( rkBodyWldCOM(src), rkBodyWldCOM(dst) );
  zVec3DCopy( rkBodyCOMVel(src), rkBodyCOMVel(dst) );
  zVec3DCopy( rkBodyCOMAcc(src), rkBodyCOMAcc(dst) );
  return dst;
}

/* rkBodyCombine
 * - combine two bodies.
 */
rkBody *rkBodyCombine(rkBody *b1, rkBody *b2, zFrame3D *f, rkBody *b)
{
  rkMP mp1, mp2;
  zFrame3D df;

  zFrame3DXfer( f, rkBodyFrame(b1), &df );
  rkMPXfer( rkBodyMP(b1), &df, &mp1 );
  zFrame3DXfer( f, rkBodyFrame(b2), &df );
  rkMPXfer( rkBodyMP(b2), &df, &mp2 );
  rkMPCombine( &mp1, &mp2, rkBodyMP(b) );
  rkBodySetFrame( b, f );
  return b;
}

/* rkBodyCombineDRC
 * - combine a body directly to another.
 */
rkBody *rkBodyCombineDRC(rkBody *b, rkBody *sb)
{
  rkMP mp1, mp2;
  zFrame3D df;

  rkMPCopy( rkBodyMP(b), &mp1 );
  zFrame3DXfer( rkBodyFrame(b), rkBodyFrame(sb), &df );
  rkMPXfer( rkBodyMP(sb), &df, &mp2 );
  rkMPCombine( &mp1, &mp2, rkBodyMP(b) );
  return b;
}

/* rkBodyUpdateCOM
 * - update body COM position with respect to the world frame.
 */
zVec3D *rkBodyUpdateCOM(rkBody *body)
{
  return zXfer3D( rkBodyFrame(body), rkBodyCOM(body), rkBodyWldCOM(body) );
}

/* rkBodyUpdateCOMRate
 * - update body COM rate with respect to the inertial frame.
 */
void rkBodyUpdateCOMVel(rkBody *body)
{
	zVec3D tmp;

  /* COM velocity */
  zVec3DOuterProd( rkBodyAngVel(body), rkBodyCOM(body), &tmp ); /* w x p */
  zVec3DAdd( rkBodyLinVel(body), &tmp, rkBodyCOMVel(body) );
}

void rkBodyUpdateCOMAcc(rkBody *body)
{
	zVec3D tmp;
	/* COM acceleration */
	zVec3DTripleProd( rkBodyAngVel(body), rkBodyAngVel(body), rkBodyCOM(body), rkBodyCOMAcc(body) ); /* w x ( w x p ) */
  zVec3DOuterProd( rkBodyAngAcc(body), rkBodyCOM(body), &tmp ); /* a x p */
  zVec3DAddDRC( rkBodyCOMAcc(body), &tmp );
  zVec3DAddDRC( rkBodyCOMAcc(body), rkBodyLinAcc(body) );
}


void rkBodyUpdateCOMRate(rkBody *body)
{
	rkBodyUpdateCOMVel( body );
	rkBodyUpdateCOMAcc( body );
}

/* rkBodyNetWrench
 * - net wrench exerted on body.
 */
zVec6D *rkBodyNetWrench(rkBody *body, zVec6D *w)
{
  zVec3D tmp;

  zVec3DMul( rkBodyCOMAcc(body), rkBodyMass(body), zVec6DLin(w) );
  zMulMatVec3D( rkBodyInertia(body), rkBodyAngVel(body), &tmp );
  zVec3DOuterProd( rkBodyAngVel(body), &tmp, zVec6DAng(w) );
  zMulMatVec3D( rkBodyInertia(body), rkBodyAngAcc(body), &tmp );
  zVec3DAddDRC( zVec6DAng(w), &tmp );
  return w;
}

/* rkBodyAM
 * - angular momentum of body.
 */
zVec3D *rkBodyAM(rkBody *b, zVec3D *p, zVec3D *am)
{
  zVec3D tmp;

  zVec3DSub( rkBodyCOM(b), p, &tmp );
  zVec3DOuterProd( &tmp, rkBodyCOMVel(b), am );
  zVec3DMulDRC( am, rkBodyMass(b) );
  zMulMatVec3D( rkBodyInertia(b), rkBodyAngVel(b), &tmp );
  return zVec3DAddDRC( am, &tmp );
}

/* rkBodyKE
 * - kinematic energy of body.
 */
double rkBodyKE(rkBody *b)
{
  zVec3D tmp;
  double result;

  zMulMatVec3D( rkBodyInertia(b), rkBodyAngVel(b), &tmp );
  result = zVec3DInnerProd( rkBodyAngVel(b), &tmp );
  result += rkBodyMass(b)*zVec3DSqrNorm( rkBodyCOMVel(b) );
  return ( result *= 0.5 );
}

/* rkBodyContigVert
 * - contiguous vertex of a body to a point.
 */
zVec3D *rkBodyContigVert(rkBody *body, zVec3D *p, double *d)
{
  zVec3D pc;

  zXfer3DInv( rkBodyFrame(body), p, &pc );
  return zShapeListContigVert( rkBodyShapeList(body), &pc, d );
}
