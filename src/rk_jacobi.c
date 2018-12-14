/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_jacobi - basic manipulator Jacobian matrix
 */

#include <roki/rk_jacobi.h>

/* Manipulator Jacobian matrix */

static zVec3D *_rkJacobiLinCol(rkLink *l, int i, zFrame3D *f, zVec3D *p, zVec3D *s);
static zMat _rkChainLinkAMJacobi(rkChain *c, int id, zVec3D *p, zMat jacobi);

#define __rk_jacobi_set_vector(j,c,k,v) do{\
  zMatSetElem( j, zX, c, (v)->e[zX] );\
  zMatSetElem( j, zY, c, (v)->e[zY] );\
  zMatSetElem( j, zZ, c, (v)->e[zZ] );\
} while(0)

#define __rk_jacobi_add_vector(j,c,k,v) do{\
  zMatElem(j,zX,c) += (v)->e[zX];\
  zMatElem(j,zY,c) += (v)->e[zY];\
  zMatElem(j,zZ,c) += (v)->e[zZ];\
} while(0)

#define __rk_jacobi_sub_vector(j,c,k,v) do{\
  zMatElem(j,zX,c) -= (v)->e[zX];\
  zMatElem(j,zY,c) -= (v)->e[zY];\
  zMatElem(j,zZ,c) -= (v)->e[zZ];\
} while(0)

#define __rk_jacobi_cat_vector(j,c,k,v) do{\
  zMatElem(j,zX,c) += (k)*(v)->e[zX];\
  zMatElem(j,zY,c) += (k)*(v)->e[zY];\
  zMatElem(j,zZ,c) += (k)*(v)->e[zZ];\
} while(0)

#define __rk_jacobi_ang_col(l,f,m,op,s) do{\
  register int __i;\
\
  for( __i=0; __i<rkLinkJointSize(l); __i++ )\
    if( rkJointAngAxis( rkLinkJoint(l), __i, f, s ) )\
      op( m, rkLinkOffset(l)+__i, 0 /* dummy */, s );\
} while(0)

/* (static)
 * _rkJacobiLinCol
 * - column vector of linear Jacobian matrix.
 */
zVec3D *_rkJacobiLinCol(rkLink *l, int i, zFrame3D *f, zVec3D *p, zVec3D *s)
{
  zVec3D tmp, dp;

  if( rkJointLinAxis( rkLinkJoint(l), i, f, s ) ) return s;
  if( rkJointAngAxis( rkLinkJoint(l), i, f, &tmp ) ){
    zVec3DSub( p, zFrame3DPos(f), &dp );
    return zVec3DOuterProd( &tmp, &dp, s );
  }
  return NULL;
}

#define __rk_jacobi_lin_col(l,f,p,k,m,op,s) do{\
  register int __i;\
\
  for( __i=0; __i<rkLinkJointSize(l); __i++ )\
    if( _rkJacobiLinCol( l, __i, f, p, s ) )\
      op( m, rkLinkOffset(l)+__i, k, s );\
} while(0)

/* rkChainLinkWldAngJacobi
 * - Jacobian matrix about angular movement of a link with
 *   respect to the world frame.
 */
zMat rkChainLinkWldAngJacobi(rkChain *c, int id, zMat jacobi)
{
  rkLink *l;
  zVec3D s;

  zMatClear( jacobi );
  for( l=rkChainLink(c,id); ; l=rkLinkParent(l) ){
    __rk_jacobi_ang_col( l, rkLinkWldFrame(l), jacobi, __rk_jacobi_set_vector, &s );
    if( l == rkChainRoot(c) ) break;
  }
  return jacobi;
}

/* rkChainLinkWldLinJacobi
 * - Jacobian matrix about link translation with respect to
 *   the world frame.
 */
zMat rkChainLinkWldLinJacobi(rkChain *c, int id, zVec3D *p, zMat jacobi)
{
  rkLink *l;
  zVec3D s, tp;

  zMatClear( jacobi );
  zXfer3D( rkChainLinkWldFrame(c,id), p, &tp );
  for( l=rkChainLink(c,id); ; l=rkLinkParent(l) ){
    __rk_jacobi_lin_col( l, rkLinkWldFrame(l), &tp, 0, jacobi, __rk_jacobi_set_vector, &s );
    if( l == rkChainRoot(c) ) break;
  }
  return jacobi;
}

/* rkChainLinkToLinkAngJacobi
 * - Jacobian matrix about relative angular movement of a link
 *   to another with respect to the world frame.
 */
zMat rkChainLinkToLinkAngJacobi(rkChain *c, int from, int to, zMat jacobi)
{
  rkLink *l;
  zVec3D s;

  zMatClear( jacobi );
  for( l=rkChainLink(c,to); l!=rkChainRoot(c); l=rkLinkParent(l) ){
    if( l == rkChainLink(c,from) ) return jacobi;
    __rk_jacobi_ang_col( l, rkLinkWldFrame(l), jacobi, __rk_jacobi_set_vector, &s );
  }
  for( l=rkChainLink(c,from); l!=rkChainRoot(c); l=rkLinkParent(l) )
    __rk_jacobi_ang_col( l, rkLinkWldFrame(l), jacobi, __rk_jacobi_sub_vector, &s );
  return jacobi;
}

/* rkChainLinkToLinkLinJacobi
 * - Jacobian matrix about relative translation of a link to
 *   another with respect to the world frame.
 */
zMat rkChainLinkToLinkLinJacobi(rkChain *c, int from, int to, zVec3D *p, zMat jacobi)
{
  rkLink *l;
  zVec3D s, tp;

  zMatClear( jacobi );
  zXfer3D( rkChainLinkWldFrame(c,to), p, &tp );
  for( l=rkChainLink(c,to); ; l=rkLinkParent(l) ){
    if( l == rkChainLink(c,from) ) return jacobi;
    __rk_jacobi_lin_col( l, rkLinkWldFrame(l), &tp, 0, jacobi, __rk_jacobi_set_vector, &s );
    if( l == rkChainRoot(c) ) break;
  }
  for( l=rkChainLink(c,from); ; l=rkLinkParent(l) ){
    __rk_jacobi_lin_col( l, rkLinkWldFrame(l), &tp, 0, jacobi, __rk_jacobi_sub_vector, &s );
    if( l == rkChainRoot(c) ) break;
  }
  return jacobi;
}

/* rkChainCOMJacobi
 * - COM Jacobian matrix of a chain with respect to the world frame.
 */
zMat rkChainCOMJacobi(rkChain *c, zMat jacobi)
{
  register int i;
  rkLink *l;
  zVec3D s;
  double m;

  zMatClear( jacobi );
  for( i=0; i<rkChainNum(c); i++ ){
    if( rkChainLinkMass(c,i) == 0 ) continue;
    m = rkChainLinkMass(c,i) / rkChainMass(c);
    for( l=rkChainLink(c,i); ; l=rkLinkParent(l) ){
      __rk_jacobi_lin_col( l, rkLinkWldFrame(l), rkChainLinkWldCOM(c,i), m, jacobi, __rk_jacobi_cat_vector, &s );
      if( l == rkChainRoot(c) ) break;
    }
  }
  return jacobi;
}

/* (static)
 * _rkChainLinkAMJacobi
 * - link angular momentum Jacobian matrix.
 */
zMat _rkChainLinkAMJacobi(rkChain *c, int id, zVec3D *p, zMat jacobi)
{
  register int i;
  rkLink *l;
  zVec3D s, dp, v;
  zMat3D m;

  zVec3DSub( rkChainLinkWldCOM(c,id), p, &dp );
  rkLinkWldInertia( rkChainLink(c,id), &m );
  for( l=rkChainLink(c,id); ; l=rkLinkParent(l) ){
    for( i=0; i<rkLinkJointSize(l); i++ ){
      if( rkJointAngAxis( rkLinkJoint(l), i, rkLinkWldFrame(l), &s ) ){
        zMulMatVec3DDRC( &m, &s );
        __rk_jacobi_add_vector( jacobi, rkLinkOffset(l)+i, 0, &s );
      }
      if( _rkJacobiLinCol( l, i, rkLinkWldFrame(l), rkChainLinkWldCOM(c,id), &s ) ){
        zVec3DOuterProd( &dp, &s, &v );
        __rk_jacobi_cat_vector( jacobi, rkLinkOffset(l)+i, rkChainLinkMass(c,id), &v );
      }
    }
    if( l == rkChainRoot(c) ) break;
  }
  return jacobi;
}

/* rkChainLinkAMJacobi
 * - link angular momentum Jacobian matrix.
 */
zMat rkChainLinkAMJacobi(rkChain *c, int id, zVec3D *p, zMat jacobi)
{
  zMatClear( jacobi );
  return _rkChainLinkAMJacobi( c, id, p, jacobi );
}

/* rkChainAMJacobi
 * - angular momentum Jacobian matrix of a kinematic chain.
 */
zMat rkChainAMJacobi(rkChain *c, zVec3D *p, zMat jacobi)
{
  register int i;

  zMatClear( jacobi );
  for( i=0; i<rkChainNum(c); i++ )
    _rkChainLinkAMJacobi( c, i, p, jacobi );
  return jacobi;
}

/* rkChainAMCOMJacobi
 * - angular momentum Jacobian matrix about COM of a kinematic chain.
 */
zMat rkChainAMCOMJacobi(rkChain *c, zMat jacobi)
{
  return rkChainAMJacobi( c, rkChainWldCOM(c), jacobi );
}

/* rkJacobiManip
 * - measure of manipulability.
 */
double rkJacobiManip(zMat jacobi)
{
  zMat k;
  double result;

  if( !( k = zMatAllocSqr( zMatRowSizeNC(jacobi) ) ) ){
    ZALLOCERROR();
    return 0;
  }
  zMulMatMatT( jacobi, jacobi, k );
  result = sqrt( zMatDet( k ) );
  zMatFree( k );
  return result;
}
