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

/* convert mass properties in [g,mm] to that in [kg,m]. */
rkMP *rkMPgmm2kgm(rkMP *mp)
{
  rkMPMass(mp) *= 1.0e-3;
  zVec3DMulDRC( rkMPCOM(mp), 1.0e-3 );
  zMat3DMulDRC( rkMPInertia(mp), 1.0e-9 );
  return mp;
}

/* transform mass properties to that with respect to a frame. */
rkMP *rkMPXform(rkMP *src, zFrame3D *f, rkMP *dest)
{
  rkMPSetMass( dest, rkMPMass(src) );
  zXform3D( f, rkMPCOM(src), rkMPCOM(dest) );
  zRotMat3D( zFrame3DAtt(f), rkMPInertia(src), rkMPInertia(dest) );
  return dest;
}

/* transform mass properties to that with respect to the inverse of a frame. */
rkMP *rkMPXformInv(rkMP *src, zFrame3D *f, rkMP *dest)
{
  rkMPSetMass( dest, rkMPMass(src) );
  zXform3DInv( f, rkMPCOM(src), rkMPCOM(dest) );
  zRotMat3DInv( zFrame3DAtt(f), rkMPInertia(src), rkMPInertia(dest) );
  return dest;
}

/* combine two mass property sets in the same frame. */
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
  rkMPShiftInertia( mp1, &r1, &i1 );
  zVec3DSub( rkMPCOM(mp2), rkMPCOM(mp), &r2 );
  rkMPShiftInertia( mp2, &r2, &i2 );
  zMat3DAdd( &i1, &i2, rkMPInertia(mp) );
  return mp;
}

/* convert inertia tensor to that about the origin. */
zMat3D *rkMPOrgInertia(rkMP *mp, zMat3D *i)
{
  return rkMPShiftInertia( mp, rkMPCOM(mp), i );
}

/* compute the inertial ellipsoid from a mass property set. */
zEllips3D *rkMPInertiaEllips(rkMP *mp, zEllips3D *ie)
{
  zVec3D evec[3];
  double eval[3];

  zMat3DSymEig( rkMPInertia(mp), eval, evec );
  return zEllips3DCreate( ie, rkMPCOM(mp), &evec[0], &evec[1], &evec[2], eval[0], eval[1], eval[2], 0 );
}

/* print mass property out to a file. */
void rkMPFPrint(FILE *fp, rkMP *mp)
{
  fprintf( fp, "mass: %.10g\n", rkMPMass(mp) );
  fprintf( fp, "COM: " );
  zVec3DFPrint( fp, rkMPCOM(mp) );
  fprintf( fp, "inertia: " );
  zMat3DFPrint( fp, rkMPInertia(mp) );
}

/* ********************************************************** */
/* CLASS: rkBody
 * rigid body class
 * ********************************************************** */

/* initialize a body. */
void rkBodyInit(rkBody *body)
{
  rkBodySetMass( body, 0 );
  rkBodySetCOM( body, ZVEC3DZERO );
  rkBodySetInertia( body, ZMAT3DZERO );
  rkBodySetFrame( body, ZFRAME3DIDENT );
  rkBodySetVel( body, ZVEC6DZERO );
  rkBodySetAcc( body, ZVEC6DZERO );
  rkBodySetWldCOM( body, ZVEC3DZERO );
  rkBodySetCOMVel( body, ZVEC3DZERO );
  rkBodySetCOMAcc( body, ZVEC3DZERO );

  zListInit( rkBodyExtWrench( body ) );
  zListInit( rkBodyShapeList( body ) );

  rkBodyStuff(body) = NULL;
}

/* destroy a body. */
void rkBodyDestroy(rkBody *body)
{
  rkBodyExtWrenchDestroy( body );
  rkBodyShapeDestroy( body );
  rkBodyStuffDestroy( body );
  rkBodyInit( body );
}

/* clone a body. */
rkBody *rkBodyClone(rkBody *org, rkBody *cln, zMShape3D *shape_org, zMShape3D *shape_cln)
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
  zListForEachRew( rkBodyShapeList(org), sp )
    if( !rkBodyShapePush( cln, sp->data - zMShape3DShapeBuf(shape_org) + zMShape3DShapeBuf(shape_cln) ) ) return NULL;
  /* stuff */
  if( rkBodyStuff(org) && !rkBodySetStuff( cln, rkBodyStuff(org) ) ){
    ZALLOCERROR();
    return NULL;
  }
  return cln;
}

/* zero velocity and acceleration of a body. */
void rkBodyZeroRate(rkBody *body)
{
  rkBodySetVel( body, ZVEC6DZERO );
  rkBodySetAcc( body, ZVEC6DZERO );
}

/* copy state of a body. */
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

/* combine two bodies. */
rkBody *rkBodyCombine(rkBody *b1, rkBody *b2, zFrame3D *f, rkBody *b)
{
  rkMP mp1, mp2;
  zFrame3D df;

  zFrame3DXform( f, rkBodyFrame(b1), &df );
  rkMPXform( rkBodyMP(b1), &df, &mp1 );
  zFrame3DXform( f, rkBodyFrame(b2), &df );
  rkMPXform( rkBodyMP(b2), &df, &mp2 );
  rkMPCombine( &mp1, &mp2, rkBodyMP(b) );
  rkBodySetFrame( b, f );
  return b;
}

/* combine a body directly to another. */
rkBody *rkBodyCombineDRC(rkBody *b, rkBody *sb)
{
  rkMP mp1, mp2;
  zFrame3D df;

  rkMPCopy( rkBodyMP(b), &mp1 );
  zFrame3DXform( rkBodyFrame(b), rkBodyFrame(sb), &df );
  rkMPXform( rkBodyMP(sb), &df, &mp2 );
  rkMPCombine( &mp1, &mp2, rkBodyMP(b) );
  return b;
}

/* update COM position of a body with respect to the world frame. */
zVec3D *rkBodyUpdateCOM(rkBody *body)
{
  return zXform3D( rkBodyFrame(body), rkBodyCOM(body), rkBodyWldCOM(body) );
}

/* update COM rate of a body with respect to the inertial frame. */
void rkBodyUpdateCOMVel(rkBody *body)
{
  zVec3D tmp;

  /* COM velocity */
  zVec3DOuterProd( rkBodyAngVel(body), rkBodyCOM(body), &tmp ); /* w x p */
  zVec3DAdd( rkBodyLinVel(body), &tmp, rkBodyCOMVel(body) );
}

/* update COM acceleration of a body. */
void rkBodyUpdateCOMAcc(rkBody *body)
{
  zVec3D tmp;

  /* COM acceleration */
  zVec3DTripleProd( rkBodyAngVel(body), rkBodyAngVel(body), rkBodyCOM(body), rkBodyCOMAcc(body) ); /* w x ( w x p ) */
  zVec3DOuterProd( rkBodyAngAcc(body), rkBodyCOM(body), &tmp ); /* a x p */
  zVec3DAddDRC( rkBodyCOMAcc(body), &tmp );
  zVec3DAddDRC( rkBodyCOMAcc(body), rkBodyLinAcc(body) );
}

/* update COM rate of a body. */
void rkBodyUpdateCOMRate(rkBody *body)
{
  rkBodyUpdateCOMVel( body );
  rkBodyUpdateCOMAcc( body );
}

/* net wrench exerted on a body. */
zVec6D *rkBodyNetWrench(rkBody *body, zVec6D *w)
{
  zVec3D tmp;

  zVec3DMul( rkBodyCOMAcc(body), rkBodyMass(body), zVec6DLin(w) );
  zMulMat3DVec3D( rkBodyInertia(body), rkBodyAngVel(body), &tmp );
  zVec3DOuterProd( rkBodyAngVel(body), &tmp, zVec6DAng(w) );
  zMulMat3DVec3D( rkBodyInertia(body), rkBodyAngAcc(body), &tmp );
  zVec3DAddDRC( zVec6DAng(w), &tmp );
  return w;
}

/* angular momentum of a body. */
zVec3D *rkBodyAM(rkBody *b, zVec3D *p, zVec3D *am)
{
  zVec3D tmp;

  zVec3DSub( rkBodyCOM(b), p, &tmp );
  zVec3DOuterProd( &tmp, rkBodyCOMVel(b), am );
  zVec3DMulDRC( am, rkBodyMass(b) );
  zMulMat3DVec3D( rkBodyInertia(b), rkBodyAngVel(b), &tmp );
  return zVec3DAddDRC( am, &tmp );
}

/* kinematic energy of a body. */
double rkBodyKE(rkBody *b)
{
  zVec3D tmp;
  double result;

  zMulMat3DVec3D( rkBodyInertia(b), rkBodyAngVel(b), &tmp );
  result = zVec3DInnerProd( rkBodyAngVel(b), &tmp );
  result += rkBodyMass(b)*zVec3DSqrNorm( rkBodyCOMVel(b) );
  return ( result *= 0.5 );
}

/* contiguous vertex of a body to a point. */
zVec3D *rkBodyContigVert(rkBody *body, zVec3D *p, double *d)
{
  zVec3D pc;

  zXform3DInv( rkBodyFrame(body), p, &pc );
  return zShapeListContigVert( rkBodyShapeList(body), &pc, d );
}

/* compute volume of a body. */
double rkBodyShapeVolume(rkBody *body)
{
  zShapeListCell *cp;
  double v = 0;

  zListForEach( rkBodyShapeList(body), cp )
    v += zShape3DVolume( cp->data );
  return v;
}

/* compute mass property of a body. */
rkMP *rkBodyShapeMP(rkBody *body, double density, rkMP *mp)
{
  zShapeListCell *cp;
  double m;
  zVec3D c;
  zMat3D i;

  rkMPZero( mp );
  zListForEach( rkBodyShapeList(body), cp ){
    rkMPMass(mp) += ( m = density * zShape3DVolume( cp->data ) );
    zVec3DCatDRC( rkMPCOM(mp), m, zShape3DBarycenter( cp->data, &c ) );
    zShape3DInertia( cp->data, density, &i );
    zMat3DAddDRC( rkMPInertia(mp), &i );
  }
  zVec3DDivDRC( rkMPCOM(mp), rkMPMass(mp) );
  zMat3DCatVec3DDoubleOuterProdDRC( rkMPInertia(mp), rkMPMass(mp), rkMPCOM(mp) );
  return mp;
}
