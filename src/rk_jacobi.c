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
  for( __i=0; __i<rkLinkJointDOF(l); __i++ )\
    if( rkJointAngAxis( rkLinkJoint(l), __i, f, s ) )\
      op( m, rkLinkJointIDOffset(l)+__i, 0 /* dummy */, s );\
} while(0)

/* column vector of linear Jacobian matrix. */
static zVec3D *_rkJacobiLinCol(rkLink *link, int i, zFrame3D *f, zVec3D *p, zVec3D *s)
{
  zVec3D tmp, dp;

  if( rkJointLinAxis( rkLinkJoint(link), i, f, s ) ) return s;
  if( rkJointAngAxis( rkLinkJoint(link), i, f, &tmp ) ){
    zVec3DSub( p, zFrame3DPos(f), &dp );
    return zVec3DOuterProd( &tmp, &dp, s );
  }
  return NULL;
}

#define __rk_jacobi_lin_col(link,f,p,k,m,op,s) do{\
  int __i;\
\
  for( __i=0; __i<rkLinkJointDOF(link); __i++ )\
    if( _rkJacobiLinCol( link, __i, f, p, s ) )\
      op( m, rkLinkJointIDOffset(link)+__i, k, s );\
} while(0)

/* Jacobian matrix about angular movement of a link with respect to the world frame. */
zMat rkChainLinkWldAngJacobi(rkChain *chain, int id, zMat jacobi)
{
  rkLink *lp;
  zVec3D s;

  zMatZero( jacobi );
  for( lp=rkChainLink(chain,id); ; lp=rkLinkParent(lp) ){
    __rk_jacobi_ang_col( lp, rkLinkWldFrame(lp), jacobi, __rk_jacobi_set_vector, &s );
    if( lp == rkChainRoot(chain) ) break;
  }
  return jacobi;
}

/* Jacobian matrix about link translation with respect to the world frame. */
zMat rkChainLinkWldLinJacobi(rkChain *chain, int id, zVec3D *p, zMat jacobi)
{
  rkLink *lp;
  zVec3D s, tp;

  zMatZero( jacobi );
  zXform3D( rkChainLinkWldFrame(chain,id), p, &tp );
  for( lp=rkChainLink(chain,id); ; lp=rkLinkParent(lp) ){
    __rk_jacobi_lin_col( lp, rkLinkWldFrame(lp), &tp, 0, jacobi, __rk_jacobi_set_vector, &s );
    if( lp == rkChainRoot(chain) ) break;
  }
  return jacobi;
}

/* Jacobian matrix about relative angular movement of a link to another
 * with respect to the world frame. */
zMat rkChainLinkToLinkAngJacobi(rkChain *chain, int from, int to, zMat jacobi)
{
  rkLink *lp;
  zVec3D s;

  zMatZero( jacobi );
  for( lp=rkChainLink(chain,to); lp!=rkChainRoot(chain); lp=rkLinkParent(lp) ){
    if( lp == rkChainLink(chain,from) ) return jacobi;
    __rk_jacobi_ang_col( lp, rkLinkWldFrame(lp), jacobi, __rk_jacobi_set_vector, &s );
  }
  for( lp=rkChainLink(chain,from); lp!=rkChainRoot(chain); lp=rkLinkParent(lp) )
    __rk_jacobi_ang_col( lp, rkLinkWldFrame(lp), jacobi, __rk_jacobi_sub_vector, &s );
  return jacobi;
}

/* Jacobian matrix about relative translation of a link to another
 * with respect to the world frame. */
zMat rkChainLinkToLinkLinJacobi(rkChain *chain, int from, int to, zVec3D *p, zMat jacobi)
{
  rkLink *lp;
  zVec3D s, tp;

  zMatZero( jacobi );
  zXform3D( rkChainLinkWldFrame(chain,to), p, &tp );
  for( lp=rkChainLink(chain,to); ; lp=rkLinkParent(lp) ){
    if( lp == rkChainLink(chain,from) ) return jacobi;
    __rk_jacobi_lin_col( lp, rkLinkWldFrame(lp), &tp, 0, jacobi, __rk_jacobi_set_vector, &s );
    if( lp == rkChainRoot(chain) ) break;
  }
  for( lp=rkChainLink(chain,from); ; lp=rkLinkParent(lp) ){
    __rk_jacobi_lin_col( lp, rkLinkWldFrame(lp), &tp, 0, jacobi, __rk_jacobi_sub_vector, &s );
    if( lp == rkChainRoot(chain) ) break;
  }
  return jacobi;
}

/* COM Jacobian matrix of a chain with respect to the world frame. */
zMat rkChainCOMJacobi(rkChain *chain, zMat jacobi)
{
  int i;
  rkLink *lp;
  zVec3D s;
  double m;

  zMatZero( jacobi );
  for( i=0; i<rkChainLinkNum(chain); i++ ){
    if( rkChainLinkMass(chain,i) == 0 ) continue;
    m = rkChainLinkMass(chain,i) / rkChainMass(chain);
    for( lp=rkChainLink(chain,i); ; lp=rkLinkParent(lp) ){
      __rk_jacobi_lin_col( lp, rkLinkWldFrame(lp), rkChainLinkWldCOM(chain,i), m, jacobi, __rk_jacobi_cat_vector, &s );
      if( lp == rkChainRoot(chain) ) break;
    }
  }
  return jacobi;
}

/* link angular momentum matrix. */
static zMat _rkChainLinkAMMat(rkChain *chain, int id, zVec3D *p, zMat mat)
{
  int i;
  rkLink *lp;
  zVec3D s, dp, v;
  zMat3D m;

  zVec3DSub( rkChainLinkWldCOM(chain,id), p, &dp );
  rkLinkWldInertia( rkChainLink(chain,id), &m );
  for( lp=rkChainLink(chain,id); ; lp=rkLinkParent(lp) ){
    for( i=0; i<rkLinkJointDOF(lp); i++ ){
      if( rkJointAngAxis( rkLinkJoint(lp), i, rkLinkWldFrame(lp), &s ) ){
        zMulMat3DVec3DDRC( &m, &s );
        __rk_jacobi_add_vector( mat, rkLinkJointIDOffset(lp)+i, 0, &s );
      }
      if( _rkJacobiLinCol( lp, i, rkLinkWldFrame(lp), rkChainLinkWldCOM(chain,id), &s ) ){
        zVec3DOuterProd( &dp, &s, &v );
        __rk_jacobi_cat_vector( mat, rkLinkJointIDOffset(lp)+i, rkChainLinkMass(chain,id), &v );
      }
    }
    if( lp == rkChainRoot(chain) ) break;
  }
  return mat;
}

/* link angular momentum matrix. */
zMat rkChainLinkAMMat(rkChain *chain, int id, zVec3D *p, zMat m)
{
  zMatZero( m );
  return _rkChainLinkAMMat( chain, id, p, m );
}

/* angular momentum matrix of a kinematic chain. */
zMat rkChainAMMat(rkChain *chain, zVec3D *p, zMat m)
{
  int i;

  zMatZero( m );
  for( i=0; i<rkChainLinkNum(chain); i++ )
    _rkChainLinkAMMat( chain, i, p, m );
  return m;
}

/* angular momentum matrix about the center of mass of a kinematic chain. */
zMat rkChainAMCOMMat(rkChain *chain, zMat m)
{
  return rkChainAMMat( chain, rkChainWldCOM(chain), m );
}

/* measure of manipulability (direct computation). */
double rkJacobiManipDST(zMat jacobi, zIndex index, zMat k)
{
  zMulMatMatT( jacobi, jacobi, k );
  return sqrt( zMatDetDST( k, index ) );
}

/* measure of manipulability. */
double rkJacobiManip(zMat jacobi)
{
  zMat k;
  zIndex index;
  double result = 0;

  k = zMatAllocSqr( zMatRowSizeNC(jacobi) );
  index = zIndexCreate( zMatRowSizeNC(jacobi) );
  if( k && index )
    result = rkJacobiManipDST( jacobi, index, k );
  zMatFree( k );
  zIndexFree( index );
  return result;
}

#undef __rk_jacobi_set_vector
#undef __rk_jacobi_add_vector
#undef __rk_jacobi_sub_vector
#undef __rk_jacobi_cat_vector
#undef __rk_jacobi_ang_col

#undef __rk_jacobi_lin_col
