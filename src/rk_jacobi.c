/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_jacobi - basic manipulator Jacobian matrix
 */

#include <roki/rk_jacobi.h>

/* Manipulator Jacobian matrix */

#define __rk_jacobi_set_vector(j,c,k,v) do{\
  zMatSetElemNC( j, zX, c, (v)->e[zX] );\
  zMatSetElemNC( j, zY, c, (v)->e[zY] );\
  zMatSetElemNC( j, zZ, c, (v)->e[zZ] );\
} while(0)

#define __rk_jacobi_add_vector(j,c,k,v) do{\
  zMatElemNC(j,zX,c) += (v)->e[zX];\
  zMatElemNC(j,zY,c) += (v)->e[zY];\
  zMatElemNC(j,zZ,c) += (v)->e[zZ];\
} while(0)

#define __rk_jacobi_sub_vector(j,c,k,v) do{\
  zMatElemNC(j,zX,c) -= (v)->e[zX];\
  zMatElemNC(j,zY,c) -= (v)->e[zY];\
  zMatElemNC(j,zZ,c) -= (v)->e[zZ];\
} while(0)

#define __rk_jacobi_cat_vector(j,c,k,v) do{\
  zMatElemNC(j,zX,c) += (k)*(v)->e[zX];\
  zMatElemNC(j,zY,c) += (k)*(v)->e[zY];\
  zMatElemNC(j,zZ,c) += (k)*(v)->e[zZ];\
} while(0)

#define __rk_jacobi_ang_col(l,f,m,op,s) do{\
  int __i;\
\
  for( __i=0; __i<rkLinkJointSize(l); __i++ )\
    if( rkJointAngAxis( rkLinkJoint(l), __i, f, s ) )\
      op( m, rkLinkOffset(l)+__i, 0 /* dummy */, s );\
} while(0)

/* column vector of linear Jacobian matrix. */
static zVec3D *_rkJacobiLinCol(rkLink *l, int i, zFrame3D *f, zVec3D *p, zVec3D *s)
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
  int __i;\
\
  for( __i=0; __i<rkLinkJointSize(l); __i++ )\
    if( _rkJacobiLinCol( l, __i, f, p, s ) )\
      op( m, rkLinkOffset(l)+__i, k, s );\
} while(0)

/* Jacobian matrix about angular movement of a link with respect to the world frame. */
zMat rkChainLinkWldAngJacobi(rkChain *c, int id, zMat jacobi)
{
  rkLink *l;
  zVec3D s;

  zMatZero( jacobi );
  for( l=rkChainLink(c,id); ; l=rkLinkParent(l) ){
    __rk_jacobi_ang_col( l, rkLinkWldFrame(l), jacobi, __rk_jacobi_set_vector, &s );
    if( l == rkChainRoot(c) ) break;
  }
  return jacobi;
}

/* Jacobian matrix about link translation with respect to the world frame. */
zMat rkChainLinkWldLinJacobi(rkChain *c, int id, zVec3D *p, zMat jacobi)
{
  rkLink *l;
  zVec3D s, tp;

  zMatZero( jacobi );
  zXform3D( rkChainLinkWldFrame(c,id), p, &tp );
  for( l=rkChainLink(c,id); ; l=rkLinkParent(l) ){
    __rk_jacobi_lin_col( l, rkLinkWldFrame(l), &tp, 0, jacobi, __rk_jacobi_set_vector, &s );
    if( l == rkChainRoot(c) ) break;
  }
  return jacobi;
}

/* Jacobian matrix about relative angular movement of a link to another
 * with respect to the world frame. */
zMat rkChainLinkToLinkAngJacobi(rkChain *c, int from, int to, zMat jacobi)
{
  rkLink *l;
  zVec3D s;

  zMatZero( jacobi );
  for( l=rkChainLink(c,to); l!=rkChainRoot(c); l=rkLinkParent(l) ){
    if( l == rkChainLink(c,from) ) return jacobi;
    __rk_jacobi_ang_col( l, rkLinkWldFrame(l), jacobi, __rk_jacobi_set_vector, &s );
  }
  for( l=rkChainLink(c,from); l!=rkChainRoot(c); l=rkLinkParent(l) )
    __rk_jacobi_ang_col( l, rkLinkWldFrame(l), jacobi, __rk_jacobi_sub_vector, &s );
  return jacobi;
}

/* Jacobian matrix about relative translation of a link to another
 * with respect to the world frame. */
zMat rkChainLinkToLinkLinJacobi(rkChain *c, int from, int to, zVec3D *p, zMat jacobi)
{
  rkLink *l;
  zVec3D s, tp;

  zMatZero( jacobi );
  zXform3D( rkChainLinkWldFrame(c,to), p, &tp );
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

/* COM Jacobian matrix of a chain with respect to the world frame. */
zMat rkChainCOMJacobi(rkChain *c, zMat jacobi)
{
  uint i;
  rkLink *l;
  zVec3D s;
  double m;

  zMatZero( jacobi );
  for( i=0; i<rkChainLinkNum(c); i++ ){
    if( rkChainLinkMass(c,i) == 0 ) continue;
    m = rkChainLinkMass(c,i) / rkChainMass(c);
    for( l=rkChainLink(c,i); ; l=rkLinkParent(l) ){
      __rk_jacobi_lin_col( l, rkLinkWldFrame(l), rkChainLinkWldCOM(c,i), m, jacobi, __rk_jacobi_cat_vector, &s );
      if( l == rkChainRoot(c) ) break;
    }
  }
  return jacobi;
}

/* link angular momentum matrix. */
static zMat _rkChainLinkAMMat(rkChain *c, int id, zVec3D *p, zMat mat)
{
  int i;
  rkLink *l;
  zVec3D s, dp, v;
  zMat3D m;

  zVec3DSub( rkChainLinkWldCOM(c,id), p, &dp );
  rkLinkWldInertia( rkChainLink(c,id), &m );
  for( l=rkChainLink(c,id); ; l=rkLinkParent(l) ){
    for( i=0; i<rkLinkJointSize(l); i++ ){
      if( rkJointAngAxis( rkLinkJoint(l), i, rkLinkWldFrame(l), &s ) ){
        zMulMat3DVec3DDRC( &m, &s );
        __rk_jacobi_add_vector( mat, rkLinkOffset(l)+i, 0, &s );
      }
      if( _rkJacobiLinCol( l, i, rkLinkWldFrame(l), rkChainLinkWldCOM(c,id), &s ) ){
        zVec3DOuterProd( &dp, &s, &v );
        __rk_jacobi_cat_vector( mat, rkLinkOffset(l)+i, rkChainLinkMass(c,id), &v );
      }
    }
    if( l == rkChainRoot(c) ) break;
  }
  return mat;
}

/* link angular momentum matrix. */
zMat rkChainLinkAMMat(rkChain *c, int id, zVec3D *p, zMat m)
{
  zMatZero( m );
  return _rkChainLinkAMMat( c, id, p, m );
}

/* angular momentum matrix of a kinematic chain. */
zMat rkChainAMMat(rkChain *c, zVec3D *p, zMat m)
{
  uint i;

  zMatZero( m );
  for( i=0; i<rkChainLinkNum(c); i++ )
    _rkChainLinkAMMat( c, i, p, m );
  return m;
}

/* angular momentum matrix about the center of mass of a kinematic chain. */
zMat rkChainAMCOMMat(rkChain *c, zMat m)
{
  return rkChainAMMat( c, rkChainWldCOM(c), m );
}

/* measure of manipulability. */
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
