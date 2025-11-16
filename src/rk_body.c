/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_body - body with mass property
 */

#include <roki/rk_body.h>

/* ********************************************************** */
/* mass property class
 * *//******************************************************* */

/* convert mass properties in [g,mm] to that in [kg,m]. */
rkMP *rkMPgmm2kgm(rkMP *mp)
{
  rkMPMass(mp) *= 1.0e-3;
  _zVec3DMulDRC( rkMPCOM(mp), 1.0e-3 );
  _zMat3DMulDRC( rkMPInertia(mp), 1.0e-9 );
  return mp;
}

/* check if two sets of mass properties are equal. */
bool rkMPEqual(rkMP *mp1, rkMP *mp2)
{
  return zEqual( rkMPMass(mp1), rkMPMass(mp2), zTOL ) &&
         zVec3DEqual( rkMPCOM(mp1), rkMPCOM(mp2) ) &&
         zMat3DEqual( rkMPInertia(mp1), rkMPInertia(mp2) );
}

/* transform mass properties to that with respect to a frame. */
rkMP *rkMPXform(const rkMP *src, const zFrame3D *f, rkMP *dest)
{
  rkMPSetMass( dest, rkMPMass(src) );
  _zXform3D( f, rkMPCOM(src), rkMPCOM(dest) );
  zRotMat3D( zFrame3DAtt(f), rkMPInertia(src), rkMPInertia(dest) );
  return dest;
}

/* transform mass properties to that with respect to the inverse of a frame. */
rkMP *rkMPXformInv(const rkMP *src, const zFrame3D *f, rkMP *dest)
{
  rkMPSetMass( dest, rkMPMass(src) );
  _zXform3DInv( f, rkMPCOM(src), rkMPCOM(dest) );
  zRotMat3DInv( zFrame3DAtt(f), rkMPInertia(src), rkMPInertia(dest) );
  return dest;
}

/* combine two mass property sets in the same frame. */
rkMP *rkMPCombine(const rkMP *mp1, const rkMP *mp2, rkMP *mp)
{
  zVec3D r1, r2, com_total;
  zMat3D i1, i2;
  double mass_total;

  /* mass */
  mass_total = rkMPMass(mp1) + rkMPMass(mp2);
  /* COM */
  zVec3DMul( rkMPCOM(mp1), rkMPMass(mp1)/mass_total, &r1 );
  zVec3DMul( rkMPCOM(mp2), rkMPMass(mp2)/mass_total, &r2 );
  _zVec3DAdd( &r1, &r2, &com_total );
  /* inertia tensor */
  _zVec3DSub( rkMPCOM(mp1), &com_total, &r1 );
  rkMPShiftInertia( mp1, &r1, &i1 );
  _zVec3DSub( rkMPCOM(mp2), &com_total, &r2 );
  rkMPShiftInertia( mp2, &r2, &i2 );
  _zMat3DAdd( &i1, &i2, rkMPInertia(mp) );
  rkMPSetMass( mp, mass_total );
  rkMPSetCOM( mp, &com_total );
  return mp;
}

/* merge mass property sets to another. */
rkMP *rkMPMerge(rkMP *mp, const rkMP *mp_sub)
{
  return rkMPCombine( mp, mp_sub, mp );
}

/* convert inertia tensor to that about the origin. */
zMat3D *rkMPOrgInertia(const rkMP *mp, zMat3D *i)
{
  return rkMPShiftInertia( mp, rkMPCOM(mp), i );
}

/* compute the inertial ellipsoid from a mass property set. */
zEllips3D *rkMPInertiaEllips(const rkMP *mp, zEllips3D *ie)
{
  zVec3D eigval;
  zMat3D eigbase;

  zMat3DSymEig( rkMPInertia(mp), &eigval, &eigbase );
  return zEllips3DCreate( ie, rkMPCOM(mp), &eigbase.v[0], &eigbase.v[1], &eigbase.v[2], eigval.e[0], eigval.e[1], eigval.e[2], 0 );
}

/* convert a raw vector to a mass property set. */
rkMP *rkMPFromRawVec(rkMP *mp, const double *mpvec)
{
  zMat3D org_inertia;

  rkMPSetMass( mp, mpvec[0] );
  zVec3DDiv( (zVec3D*)&mpvec[1], rkMPMass(mp), rkMPCOM(mp) );
  zMat3DCreate( &org_inertia,
    mpvec[4], mpvec[7], mpvec[8],
    mpvec[7], mpvec[5], mpvec[9],
    mpvec[8], mpvec[9], mpvec[6] );
  zMat3DCatVec3DDoubleOuterProd( &org_inertia, rkMPMass(mp), rkMPCOM(mp), rkMPInertia(mp) );
  return mp;
}

/* convert a 10-dim vector to a mass property set. */
rkMP *rkMPFromVec(rkMP *mp, const zVec mpvec)
{
  if( zVecSize(mpvec) < 10 ){
    ZRUNERROR( RK_ERR_MP_INVSIZEVEC, zVecSize(mpvec) );
    return NULL;
  }
  return rkMPFromRawVec( mp, zVecBufNC(mpvec) );
}

/* convert a mass property set to a raw vector. */
double *rkMPToRawVec(const rkMP *mp, double *mpvec)
{
  zMat3D org_inertia;

  mpvec[0] = rkMPMass(mp);
  zVec3DMul( rkMPCOM(mp), rkMPMass(mp), (zVec3D*)&mpvec[1] );
  rkMPOrgInertia( mp, &org_inertia );
  mpvec[4] = org_inertia.c.xx;
  mpvec[5] = org_inertia.c.yy;
  mpvec[6] = org_inertia.c.zz;
  mpvec[7] = org_inertia.c.xy;
  mpvec[8] = org_inertia.c.xz;
  mpvec[9] = org_inertia.c.yz;
  return mpvec;
}

/* convert a mass property set to a 10-dim vector. */
zVec rkMPToVec(const rkMP *mp, zVec mpvec)
{
  if( zVecSize(mpvec) < 10 ){
    ZRUNERROR( RK_ERR_MP_INVSIZEVEC, zVecSize(mpvec) );
    return NULL;
  }
  rkMPToRawVec( mp, zVecBufNC(mpvec) );
  return mpvec;
}

/* print mass property out to a file. */
void rkMPFPrint(FILE *fp, const rkMP *mp)
{
  fprintf( fp, "mass: %.10g\n", rkMPMass(mp) );
  fprintf( fp, "COM: " );
  zVec3DFPrint( fp, rkMPCOM(mp) );
  fprintf( fp, "inertia: " );
  zMat3DFPrint( fp, rkMPInertia(mp) );
}

/* ********************************************************** */
/* rigid body class
 * *//******************************************************* */

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
  rkBodySetExtWrench( body, ZVEC6DZERO );

  zListInit( rkBodyShapeList( body ) );
  rkBodyStuff(body) = NULL;
}

/* destroy a body. */
void rkBodyDestroy(rkBody *body)
{
  rkBodyShapeDestroy( body );
  rkBodyStuffDestroy( body );
  rkBodyInit( body );
}

/* clone a body. */
rkBody *rkBodyClone(const rkBody *org, rkBody *cln, const zMShape3D *shape_org, const zMShape3D *shape_cln)
{
  zShapeListCell *sp;

  rkMPCopy( &org->mp, &cln->mp );
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
rkBody *rkBodyCopyState(const rkBody *src, rkBody *dest)
{
  zFrame3DCopy( rkBodyFrame(src), rkBodyFrame(dest) );
  zVec6DCopy( rkBodyVel(src), rkBodyVel(dest) );
  zVec6DCopy( rkBodyAcc(src), rkBodyAcc(dest) );
  zVec3DCopy( rkBodyWldCOM(src), rkBodyWldCOM(dest) );
  zVec3DCopy( rkBodyCOMVel(src), rkBodyCOMVel(dest) );
  zVec3DCopy( rkBodyCOMAcc(src), rkBodyCOMAcc(dest) );
  return dest;
}

/* combine two bodies. */
rkBody *rkBodyCombine(const rkBody *body1, const rkBody *body2, const zFrame3D *frame, rkBody *body)
{
  rkMP mp1, mp2;
  zFrame3D df;

  _zFrame3DXform( frame, rkBodyFrame(body1), &df );
  rkMPXform( rkBodyMP(body1), &df, &mp1 );
  _zFrame3DXform( frame, rkBodyFrame(body2), &df );
  rkMPXform( rkBodyMP(body2), &df, &mp2 );
  rkMPCombine( &mp1, &mp2, rkBodyMP(body) );
  rkBodySetFrame( body, frame );
  return body;
}

/* combine a body directly to another. */
rkBody *rkBodyCombineDRC(rkBody *body, const rkBody *subbody)
{
  rkMP mp1, mp2;
  zFrame3D df;

  rkMPCopy( rkBodyMP(body), &mp1 );
  _zFrame3DXform( rkBodyFrame(body), rkBodyFrame(subbody), &df );
  rkMPXform( rkBodyMP(subbody), &df, &mp2 );
  rkMPCombine( &mp1, &mp2, rkBodyMP(body) );
  return body;
}

/* update COM position of a body with respect to the world frame. */
zVec3D *rkBodyUpdateCOM(rkBody *body)
{
  _zXform3D( rkBodyFrame(body), rkBodyCOM(body), rkBodyWldCOM(body) );
  return rkBodyWldCOM(body);
}

/* update COM rate of a body with respect to the inertial frame. */
void rkBodyUpdateCOMVel(rkBody *body)
{
  zVec3D tmp;

  _zVec3DOuterProd( rkBodyAngVel(body), rkBodyCOM(body), &tmp ); /* w x p */
  _zVec3DAdd( rkBodyLinVel(body), &tmp, rkBodyCOMVel(body) );
}

/* update COM acceleration of a body. */
void rkBodyUpdateCOMAcc(rkBody *body)
{
  zVec3D tmp;

  _zVec3DTripleProd( rkBodyAngVel(body), rkBodyAngVel(body), rkBodyCOM(body), rkBodyCOMAcc(body) ); /* w x ( w x p ) */
  _zVec3DOuterProd( rkBodyAngAcc(body), rkBodyCOM(body), &tmp ); /* a x p */
  _zVec3DAddDRC( rkBodyCOMAcc(body), &tmp );
  _zVec3DAddDRC( rkBodyCOMAcc(body), rkBodyLinAcc(body) );
}

/* update COM rate of a body. */
void rkBodyUpdateCOMRate(rkBody *body)
{
  rkBodyUpdateCOMVel( body );
  rkBodyUpdateCOMAcc( body );
}

/* set an external force of a body. */
zVec6D *rkBodySetExtForce(rkBody *body, const zVec3D *force, const zVec3D *pos)
{
  zVec3DCopy( force, zVec6DLin(rkBodyExtWrench(body)) );
  _zVec3DOuterProd( pos, force, zVec6DAng(rkBodyExtWrench(body)) );
  return rkBodyExtWrench(body);
}

/* add an external force to a body. */
zVec6D *rkBodyAddExtForce(rkBody *body, const zVec3D *force, const zVec3D *pos)
{
  zVec3D tmp;

  _zVec3DAddDRC( zVec6DLin(rkBodyExtWrench(body)), force );
  _zVec3DOuterProd( pos, force, &tmp );
  _zVec3DAddDRC( zVec6DAng(rkBodyExtWrench(body)), &tmp );
  return rkBodyExtWrench(body);
}

/* inertial wrench of a body. */
zVec6D *rkBodyInertialWrench(const rkBody *body, zVec6D *wrench)
{
  zVec3D tmp;

  _zVec3DMul( rkBodyCOMAcc(body), rkBodyMass(body), zVec6DLin(wrench) );
  _zMulMat3DVec3D( rkBodyInertia(body), rkBodyAngVel(body), &tmp );
  _zVec3DOuterProd( rkBodyAngVel(body), &tmp, zVec6DAng(wrench) );
  _zMulMat3DVec3D( rkBodyInertia(body), rkBodyAngAcc(body), &tmp );
  _zVec3DAddDRC( zVec6DAng(wrench), &tmp );
  return wrench;
}

/* linear momentum of a body. */
zVec3D *rkBodyLinearMomentum(const rkBody *body, zVec3D *momentum)
{
  _zVec3DMul( rkBodyCOMVel(body), rkBodyMass(body), momentum );
  return momentum;
}

/* angular momentum of a body. */
zVec3D *rkBodyAngularMomentum(const rkBody *body, const zVec3D *pos, zVec3D *am)
{
  zVec3D tmp;

  _zVec3DSub( rkBodyCOM(body), pos, &tmp );
  _zVec3DOuterProd( &tmp, rkBodyCOMVel(body), am );
  _zVec3DMulDRC( am, rkBodyMass(body) );
  _zMulMat3DVec3D( rkBodyInertia(body), rkBodyAngVel(body), &tmp );
  _zVec3DAddDRC( am, &tmp );
  return am;
}

/* kinetic energy of a body. */
double rkBodyKineticEnergy(const rkBody *body)
{
  zVec3D tmp;
  double result;

  _zMulMat3DVec3D( rkBodyInertia(body), rkBodyAngVel(body), &tmp );
  result = _zVec3DInnerProd( rkBodyAngVel(body), &tmp );
  result += rkBodyMass(body)*zVec3DSqrNorm( rkBodyCOMVel(body) );
  return ( result *= 0.5 );
}

/* contiguous vertex of a body to a point. */
const zVec3D *rkBodyContigVert(const rkBody *body, const zVec3D *point, double *distance)
{
  zVec3D pc;

  _zXform3DInv( rkBodyFrame(body), point, &pc );
  return zShapeListContigVert( rkBodyShapeList(body), &pc, distance );
}

/* compute volume of a body. */
double rkBodyShapeVolume(const rkBody *body)
{
  zShapeListCell *cp;
  double v = 0;

  zListForEach( rkBodyShapeList(body), cp )
    v += zShape3DVolume( cp->data );
  return v;
}

/* compute mass property of a body. */
rkMP *rkBodyShapeMP(const rkBody *body, double density, rkMP *mp)
{
  zShapeListCell *cp;
  double m;
  zVec3D c;
  zMat3D i;

  rkMPZero( mp );
  zListForEach( rkBodyShapeList(body), cp ){
    rkMPMass(mp) += ( m = density * zShape3DVolume( cp->data ) );
    _zVec3DCatDRC( rkMPCOM(mp), m, zShape3DBarycenter( cp->data, &c ) );
    zShape3DInertia( cp->data, density, &i );
    _zMat3DAddDRC( rkMPInertia(mp), &i );
  }
  zVec3DDivDRC( rkMPCOM(mp), rkMPMass(mp) );
  _zMat3DCatVec3DDoubleOuterProdDRC( rkMPInertia(mp), rkMPMass(mp), rkMPCOM(mp) );
  return mp;
}

/* compute a regressor matrix for mass property identification. */
zMat rkBodyMPRegressor(const rkBody *body, zMat regressor)
{
  double ox2, oy2, oz2, oxy, oyz, ozx;

  if( zMatRowSize(regressor) != 6 || zMatColSize(regressor) != 10 ){
    ZRUNERROR( RK_ERR_BODY_INVSIZE_REGRESSOR, zMatRowSize(regressor), zMatColSize(regressor) );
    return NULL;
  }
  ox2 = zSqr( rkBodyAngVel(body)->c.x );
  oy2 = zSqr( rkBodyAngVel(body)->c.y );
  oz2 = zSqr( rkBodyAngVel(body)->c.z );
  oxy = rkBodyAngVel(body)->c.x * rkBodyAngVel(body)->c.y;
  oyz = rkBodyAngVel(body)->c.y * rkBodyAngVel(body)->c.z;
  ozx = rkBodyAngVel(body)->c.z * rkBodyAngVel(body)->c.x;

  zMatSetElemNC( regressor, 0, 0, rkBodyLinAcc(body)->c.x );
  zMatSetElemNC( regressor, 1, 0, rkBodyLinAcc(body)->c.y );
  zMatSetElemNC( regressor, 2, 0, rkBodyLinAcc(body)->c.z );
  zMatSetElemNC( regressor, 3, 0, 0 );
  zMatSetElemNC( regressor, 4, 0, 0 );
  zMatSetElemNC( regressor, 5, 0, 0 );

  zMatSetElemNC( regressor, 0, 1, -oy2-oz2 );
  zMatSetElemNC( regressor, 1, 1, oxy+rkBodyAngAcc(body)->c.z );
  zMatSetElemNC( regressor, 2, 1, ozx-rkBodyAngAcc(body)->c.y );
  zMatSetElemNC( regressor, 3, 1, 0 );
  zMatSetElemNC( regressor, 4, 1,-rkBodyLinAcc(body)->c.z );
  zMatSetElemNC( regressor, 5, 1, rkBodyLinAcc(body)->c.y );

  zMatSetElemNC( regressor, 0, 2, oxy-rkBodyAngAcc(body)->c.z );
  zMatSetElemNC( regressor, 1, 2, -oz2-ox2 );
  zMatSetElemNC( regressor, 2, 2, oyz+rkBodyAngAcc(body)->c.x );
  zMatSetElemNC( regressor, 3, 2, rkBodyLinAcc(body)->c.z );
  zMatSetElemNC( regressor, 4, 2, 0 );
  zMatSetElemNC( regressor, 5, 2,-rkBodyLinAcc(body)->c.x );

  zMatSetElemNC( regressor, 0, 3, ozx+rkBodyAngAcc(body)->c.y );
  zMatSetElemNC( regressor, 1, 3, oyz-rkBodyAngAcc(body)->c.x );
  zMatSetElemNC( regressor, 2, 3, -ox2-oy2 );
  zMatSetElemNC( regressor, 3, 3,-rkBodyLinAcc(body)->c.y );
  zMatSetElemNC( regressor, 4, 3, rkBodyLinAcc(body)->c.x );
  zMatSetElemNC( regressor, 5, 3, 0 );

  zMatSetElemNC( regressor, 0, 4, 0 );
  zMatSetElemNC( regressor, 1, 4, 0 );
  zMatSetElemNC( regressor, 2, 4, 0 );
  zMatSetElemNC( regressor, 3, 4, rkBodyAngAcc(body)->c.x );
  zMatSetElemNC( regressor, 4, 4, ozx );
  zMatSetElemNC( regressor, 5, 4,-oxy );

  zMatSetElemNC( regressor, 0, 5, 0 );
  zMatSetElemNC( regressor, 1, 5, 0 );
  zMatSetElemNC( regressor, 2, 5, 0 );
  zMatSetElemNC( regressor, 3, 5,-oyz );
  zMatSetElemNC( regressor, 4, 5, rkBodyAngAcc(body)->c.y );
  zMatSetElemNC( regressor, 5, 5, oxy );

  zMatSetElemNC( regressor, 0, 6, 0 );
  zMatSetElemNC( regressor, 1, 6, 0 );
  zMatSetElemNC( regressor, 2, 6, 0 );
  zMatSetElemNC( regressor, 3, 6, oyz );
  zMatSetElemNC( regressor, 4, 6,-ozx );
  zMatSetElemNC( regressor, 5, 6, rkBodyAngAcc(body)->c.z );

  zMatSetElemNC( regressor, 0, 7, 0 );
  zMatSetElemNC( regressor, 1, 7, 0 );
  zMatSetElemNC( regressor, 2, 7, 0 );
  zMatSetElemNC( regressor, 3, 7,-ozx+rkBodyAngAcc(body)->c.y );
  zMatSetElemNC( regressor, 4, 7, oyz+rkBodyAngAcc(body)->c.x );
  zMatSetElemNC( regressor, 5, 7, ox2-oy2 );

  zMatSetElemNC( regressor, 0, 8, 0 );
  zMatSetElemNC( regressor, 1, 8, 0 );
  zMatSetElemNC( regressor, 2, 8, 0 );
  zMatSetElemNC( regressor, 3, 8, oxy+rkBodyAngAcc(body)->c.z );
  zMatSetElemNC( regressor, 4, 8, oz2-ox2 );
  zMatSetElemNC( regressor, 5, 8,-oyz+rkBodyAngAcc(body)->c.x );

  zMatSetElemNC( regressor, 0, 9, 0 );
  zMatSetElemNC( regressor, 1, 9, 0 );
  zMatSetElemNC( regressor, 2, 9, 0 );
  zMatSetElemNC( regressor, 3, 9, oy2-oz2 );
  zMatSetElemNC( regressor, 4, 9,-oxy+rkBodyAngAcc(body)->c.z );
  zMatSetElemNC( regressor, 5, 9, ozx+rkBodyAngAcc(body)->c.y );

  return regressor;
}
