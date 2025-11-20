/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_jacobi - basic manipulator Jacobian matrix
 */

#include <roki/rk_jacobi.h>

/* Manipulator Jacobian matrix */

#define __rk_jacobi_set_vector(jacobi,col,k,vec) do{ \
  zMatSetElemNC( jacobi, zX, col, (vec)->c.x ); \
  zMatSetElemNC( jacobi, zY, col, (vec)->c.y ); \
  zMatSetElemNC( jacobi, zZ, col, (vec)->c.z ); \
} while(0)

#define __rk_jacobi_add_vector(jacobi,col,k,vec) do{ \
  zMatElemNC(jacobi,zX,col) += (vec)->c.x; \
  zMatElemNC(jacobi,zY,col) += (vec)->c.y; \
  zMatElemNC(jacobi,zZ,col) += (vec)->c.z; \
} while(0)

#define __rk_jacobi_sub_vector(jacobi,col,k,vec) do{ \
  zMatElemNC(jacobi,zX,col) -= (vec)->c.x; \
  zMatElemNC(jacobi,zY,col) -= (vec)->c.y; \
  zMatElemNC(jacobi,zZ,col) -= (vec)->c.z; \
} while(0)

#define __rk_jacobi_cat_vector(jacobi,col,k,vec) do{ \
  zMatElemNC(jacobi,zX,col) += (k)*(vec)->c.x; \
  zMatElemNC(jacobi,zY,col) += (k)*(vec)->c.y; \
  zMatElemNC(jacobi,zZ,col) += (k)*(vec)->c.z; \
} while(0)

#define __rk_jacobi_ang_col(link,frame,mat,op,vec) do{ \
  int __i; \
  for( __i=0; __i<rkLinkJointDOF(link); __i++ ) \
    if( rkJointAngAxis( rkLinkJoint(link), __i, frame, vec ) ) \
      op( mat, rkLinkJointIDOffset(link)+__i, 0 /* dummy */, vec ); \
} while(0)

/* column vector of linear Jacobian matrix. */
static zVec3D *_rkJacobiLinCol(rkLink *link, int i, zFrame3D *frame, zVec3D *p, zVec3D *vec)
{
  zVec3D tmp, dp;

  if( rkJointLinAxis( rkLinkJoint(link), i, frame, vec ) ) return vec;
  if( rkJointAngAxis( rkLinkJoint(link), i, frame, &tmp ) ){
    zVec3DSub( p, zFrame3DPos(frame), &dp );
    return zVec3DOuterProd( &tmp, &dp, vec );
  }
  return NULL;
}

#define __rk_jacobi_lin_col(link,frame,p,k,mat,op,vec) do{ \
  int __i; \
  for( __i=0; __i<rkLinkJointDOF(link); __i++ ) \
    if( _rkJacobiLinCol( link, __i, frame, p, vec ) ) \
      op( mat, rkLinkJointIDOffset(link)+__i, k, vec ); \
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
  zFrame3D frame_from;

  zMatZero( jacobi );
  for( lp=rkChainLink(chain,to); ; lp=rkLinkParent(lp) ){
    if( lp == rkChainLink(chain,from) ) return jacobi;
    _zFrame3DXform( rkChainLinkWldFrame(chain,from), rkLinkWldFrame(lp), &frame_from );
    __rk_jacobi_ang_col( lp, &frame_from, jacobi, __rk_jacobi_set_vector, &s );
    if( lp == rkChainRoot(chain) ) break;
  }
  for( lp=rkChainLink(chain,from); ; lp=rkLinkParent(lp) ){
    _zFrame3DXform( rkChainLinkWldFrame(chain,from), rkLinkWldFrame(lp), &frame_from );
    __rk_jacobi_ang_col( lp, &frame_from, jacobi, __rk_jacobi_sub_vector, &s );
    if( lp == rkChainRoot(chain) ) break;
  }
  return jacobi;
}

/* Jacobian matrix about relative translation of a link to another
 * with respect to the world frame. */
zMat rkChainLinkToLinkLinJacobi(rkChain *chain, int from, int to, zVec3D *p, zMat jacobi)
{
  rkLink *lp;
  zVec3D s, p_to, p_from_to;
  zFrame3D frame_from;

  zMatZero( jacobi );
  _zXform3D( rkChainLinkWldFrame(chain,to), p, &p_to );
  _zXform3DInv( rkChainLinkWldFrame(chain,from), &p_to, &p_from_to );
  for( lp=rkChainLink(chain,to); ; lp=rkLinkParent(lp) ){
    if( lp == rkChainLink(chain,from) ) return jacobi;
    _zFrame3DXform( rkChainLinkWldFrame(chain,from), rkLinkWldFrame(lp), &frame_from );
    __rk_jacobi_lin_col( lp, &frame_from, &p_from_to, 0, jacobi, __rk_jacobi_set_vector, &s );
    if( lp == rkChainRoot(chain) ) break;
  }
  for( lp=rkChainLink(chain,from); ; lp=rkLinkParent(lp) ){
    _zFrame3DXform( rkChainLinkWldFrame(chain,from), rkLinkWldFrame(lp), &frame_from );
    __rk_jacobi_lin_col( lp, &frame_from, &p_from_to, 0, jacobi, __rk_jacobi_sub_vector, &s );
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
