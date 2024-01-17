/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_chain - kinematic chain, kinematics and dynamics
 */

#include <roki/rk_chain.h>

/* ********************************************************** */
/* CLASS: rkChain
 * kinematic chain class
 * ********************************************************** */

/* initialize a kinematic chain. */
void rkChainInit(rkChain *c)
{
  zNameSetPtr( c, NULL );
  zArrayInit( &c->link );
  rkChainSetShape( c, NULL );
  rkChainSetMotor( c, NULL );
  rkChainSetWldCOM( c, ZVEC3DZERO );
  rkChainSetCOMVel( c, ZVEC3DZERO );
  rkChainSetCOMAcc( c, ZVEC3DZERO );
  c->_ik = NULL;
}

/* destroy a kinematic chain. */
void rkChainDestroy(rkChain *c)
{
  int i;

  if( !c ) return;
  zNameFree(c);
  for( i=0; i<rkChainLinkNum(c); i++ )
    rkLinkDestroy( rkChainLink(c,i) );
  zArrayFree( &c->link );
  zMShape3DDestroy( rkChainShape(c) );
  zFree( rkChainShape(c) );
  if( rkChainMotor(c) ){
    zArrayFree( rkChainMotor(c) );
    zFree( rkChainMotor(c) );
  }
  rkChainDestroyIK( c );
  rkChainInit( c );
}

/* clone a kinematic chain. */
rkChain *rkChainClone(rkChain *org, rkChain *cln)
{
  char name[BUFSIZ];
  int i;

  if( !cln || !org ){
    ZRUNERROR( RK_WARN_CHAIN_NULL );
    return NULL;
  }
  if( rkChainLinkNum(org) == 0 ){
    ZRUNERROR( RK_WARN_CHAIN_EMPTY );
    return NULL;
  }
  rkChainInit( cln );
  sprintf( name, "%s_clone", zName(org) );
  if( !zNameSet( cln, name ) ||
      ( rkChainShape(org) && !( rkChainShape(cln) = zMShape3DClone( rkChainShape(org) ) ) ) ||
      ( rkChainMotor(org) && !( rkChainMotor(cln) = rkMotorArrayClone( rkChainMotor(org) ) ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  zArrayAlloc( &cln->link, rkLink, rkChainLinkNum(org) );
  if( rkChainLinkNum(cln) != rkChainLinkNum(org) ) return NULL;
  for( i=0; i<rkChainLinkNum(cln); i++ )
    if( !rkLinkClone( rkChainLink(org,i), rkChainLink(cln,i), rkChainShape(org), rkChainShape(cln) ) )
      return NULL;
  rkChainCopyState( org, cln );
  /* TODO: clone IK */
  return cln;
}

/* copy state of a kinematic chain. */
rkChain *rkChainCopyState(rkChain *src, rkChain *dst)
{
  int i;

  for( i=0; i<rkChainLinkNum(src); i++ )
    rkLinkCopyState( rkChainLink(src,i), rkChainLink(dst,i) );
  zVec3DCopy( rkChainWldCOM(src), rkChainWldCOM(dst) );
  zVec3DCopy( rkChainCOMVel(src), rkChainCOMVel(dst) );
  zVec3DCopy( rkChainCOMAcc(src), rkChainCOMAcc(dst) );
  return dst;
}

/* count the total number of joint of a kinematic chain. */
int rkChainJointSize(rkChain *c)
{
  int i, size;

  for( size=0, i=0; i<rkChainLinkNum(c); i++ )
    size += rkChainLinkJointSize(c,i);
  return size;
}

/* create default joint index of a kinematic chain. */
zIndex rkChainCreateDefaultJointIndex(rkChain *c)
{
  int i, count;
  zIndex index;

  for( count=0, i=0; i<rkChainLinkNum(c); i++ )
    if( rkChainLinkJointSize(c,i) > 0 ) count++;
  if( !( index = zIndexCreate( count ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  for( count=0, i=0; i<rkChainLinkNum(c); i++ )
    if( rkChainLinkJointSize(c,i) > 0 ){
      zIndexSetElemNC( index, count, i );
      count++;
    }
  return index;
}

/* count the total joint size indicated in a kinematic chain. */
int rkChainJointIndexSize(rkChain *c, zIndex idx)
{
  int i, size;

  for( size=0, i=0; i<zArraySize(idx); i++ )
    size += rkChainLinkJointSize(c,zIndexElemNC(idx,i));
  return size;
}

/* find a link of a kinematic chain from name. */
rkLink *rkChainFindLink(rkChain *chain, char *name)
{
  rkLink *l;

  zArrayFindName( rkChainLinkArray(chain), name, l );
  if( !l ) ZRUNERROR( RK_ERR_LINK_UNKNOWN, name );
  return l;
}

/* find a link identifier of a kinematic chain from name. */
int rkChainFindLinkID(rkChain *chain, char *name)
{
  rkLink *l;
  return ( l = rkChainFindLink( chain, name ) ) ? (int)( l - rkChainRoot(chain) ) : -1;
}

/* find a joint identifier offset of a link of a kinematic chain from name. */
int rkChainFindLinkJointIDOffset(rkChain *chain, char *name)
{
  rkLink *l;
  return ( l = rkChainFindLink( chain, name ) ) ? rkLinkJointIDOffset(l) : -1;
}

/* set joint displacements of a kinematic chain. */
void rkChainSetJointDis(rkChain *c, zIndex idx, zVec dis)
{
  int i;
  double *dp;

  for( dp=zVecBuf(dis), i=0; i<zArraySize(idx); i++ ){
    rkChainLinkJointSetDis( c, zIndexElemNC(idx,i), dp );
    dp += rkChainLinkJointSize(c,zIndexElemNC(idx,i));
  }
}

/* continuously update joint displacements of a kinematic chain. */
void rkChainSetJointDisCNT(rkChain *c, zIndex idx, zVec dis, double dt)
{
  int i;
  double *dp;

  for( dp=zVecBuf(dis), i=0; i<zArraySize(idx); i++ ){
    rkChainLinkJointSetDisCNT( c, zIndexElemNC(idx,i), dp, dt );
    dp += rkChainLinkJointSize(c,zIndexElemNC(idx,i));
  }
}

/* set joint velocities of a kinematic chain. */
void rkChainSetJointVel(rkChain *c, zIndex idx, zVec vel)
{
  int i;
  double *vp;

  for( vp=zVecBuf(vel), i=0; i<zArraySize(idx); i++ ){
    rkJointSetVel( rkChainLinkJoint(c,zIndexElemNC(idx,i)), vp );
    vp += rkChainLinkJointSize(c,zIndexElemNC(idx,i));
  }
}

/* set joint accelerations of a kinematic chain. */
void rkChainSetJointAcc(rkChain *c, zIndex idx, zVec acc)
{
  int i;
  double *vp;

  for( vp=zVecBuf(acc), i=0; i<zArraySize(idx); i++ ){
    rkJointSetAcc( rkChainLinkJoint(c,zIndexElemNC(idx,i)), vp );
    vp += rkChainLinkJointSize(c,zIndexElemNC(idx,i));
  }
}

/* set joint velocities and accelerations of a kinematic chain. */
void rkChainSetJointRate(rkChain *c, zIndex idx, zVec vel, zVec acc)
{
  int i;
  double *vp, *ap;

  for( vp=zVecBuf(vel), ap=zVecBuf(acc), i=0; i<zArraySize(idx); i++ ){
    rkJointSetVel( rkChainLinkJoint(c,zIndexElemNC(idx,i)), vp );
    rkJointSetAcc( rkChainLinkJoint(c,zIndexElemNC(idx,i)), ap );
    vp += rkChainLinkJointSize(c,zIndexElemNC(idx,i));
    ap += rkChainLinkJointSize(c,zIndexElemNC(idx,i));
  }
}

/* get joint displacements of a kinematic chain. */
zVec rkChainGetJointDis(rkChain *c, zIndex idx, zVec dis)
{
  int i;
  double *dp;

  for( dp=zVecBuf(dis), i=0; i<zArraySize(idx); i++ ){
    rkChainLinkJointGetDis( c, zIndexElemNC(idx,i), dp );
    dp += rkChainLinkJointSize(c,zIndexElemNC(idx,i));
  }
  return dis;
}

/* get joint velocities of a kinematic chain. */
zVec rkChainGetJointVel(rkChain *c, zIndex idx, zVec vel)
{
  int i;
  double *dp;

  for( dp=zVecBuf(vel), i=0; i<zArraySize(idx); i++ ){
    rkChainLinkJointGetVel( c, zIndexElemNC(idx,i), dp );
    dp += rkChainLinkJointSize(c,zIndexElemNC(idx,i));
  }
  return vel;
}

/* get joint accelerations of a kinematic chain. */
zVec rkChainGetJointAcc(rkChain *c, zIndex idx, zVec acc)
{
  int i;
  double *dp;

  for( dp=zVecBuf(acc), i=0; i<zArraySize(idx); i++ ){
    rkChainLinkJointGetAcc( c, zIndexElemNC(idx,i), dp );
    dp += rkChainLinkJointSize(c,zIndexElemNC(idx,i));
  }
  return acc;
}

/* set all joint displacements of a kinematic chain. */
void rkChainSetJointDisAll(rkChain *c, zVec dis)
{
  int i;

  if( dis ){
    for( i=0; i<rkChainLinkNum(c); i++ )
      if( rkChainLinkJointIDOffset(c,i) >= 0 )
        rkChainLinkJointSetDis( c, i, &zVecElemNC(dis,rkChainLinkJointIDOffset(c,i)) );
  } else{
    for( i=0; i<rkChainLinkNum(c); i++ )
      if( rkChainLinkJointIDOffset(c,i) >= 0 )
        rkChainLinkJointSetDis( c, i, ZVEC6DZERO->e );
  }
}

/* concatenate all joint displacements of a kinematic chain. */
void rkChainCatJointDisAll(rkChain *c, zVec dis, double k, zVec v)
{
  int i;

  for( i=0; i<rkChainLinkNum(c); i++ )
    if( rkChainLinkJointIDOffset(c,i) >= 0 )
      rkJointCatDis( rkChainLinkJoint(c,i), &zVecElemNC(dis,rkChainLinkJointIDOffset(c,i)), k, &zVecElemNC(v,rkChainLinkJointIDOffset(c,i)) );
}

/* subtract all joint displacements of a kinematic chain. */
void rkChainSubJointDisAll(rkChain *c, zVec dis, zVec sdis)
{
  int i;

  for( i=0; i<rkChainLinkNum(c); i++ )
    if( rkChainLinkJointIDOffset(c,i) >= 0 )
      rkJointSubDis( rkChainLinkJoint(c,i), &zVecElemNC(dis,rkChainLinkJointIDOffset(c,i)), &zVecElemNC(sdis,rkChainLinkJointIDOffset(c,i)) );
}

/* continuously update all joint displacements of a kinematic chain. */
void rkChainSetJointDisCNTAll(rkChain *c, zVec dis, double dt)
{
  int i;

  for( i=0; i<rkChainLinkNum(c); i++ )
    if( rkChainLinkJointIDOffset(c,i) >= 0 )
      rkChainLinkJointSetDisCNT( c, i, &zVecElemNC(dis,rkChainLinkJointIDOffset(c,i)), dt );
}

/* set all joint velocities of a kinematic chain. */
void rkChainSetJointVelAll(rkChain *c, zVec vel)
{
  int i;

  if( vel ){
    for( i=0; i<rkChainLinkNum(c); i++ )
      if( rkChainLinkJointIDOffset(c,i) >= 0 )
        rkJointSetVel( rkChainLinkJoint(c,i), &zVecElemNC(vel,rkChainLinkJointIDOffset(c,i)) );
  } else{
    for( i=0; i<rkChainLinkNum(c); i++ )
      if( rkChainLinkJointIDOffset(c,i) >= 0 )
        rkJointSetVel( rkChainLinkJoint(c,i), ZVEC6DZERO->e );
  }
}

/* set all joint accelerations of a kinematic chain. */
void rkChainSetJointAccAll(rkChain *c, zVec acc)
{
  int i;

  if( acc ){
    for( i=0; i<rkChainLinkNum(c); i++ )
      if( rkChainLinkJointIDOffset(c,i) >= 0 )
        rkJointSetAcc( rkChainLinkJoint(c,i), &zVecElemNC(acc,rkChainLinkJointIDOffset(c,i)) );
  } else{
    for( i=0; i<rkChainLinkNum(c); i++ )
      if( rkChainLinkJointIDOffset(c,i) >= 0 )
        rkJointSetAcc( rkChainLinkJoint(c,i), ZVEC6DZERO->e );
  }
}

/* set all joint velocities and accelerations of a kinematic chain. */
void rkChainSetJointRateAll(rkChain *c, zVec vel, zVec acc)
{
  int i;

  for( i=0; i<rkChainLinkNum(c); i++ )
    if( rkChainLinkJointIDOffset(c,i) >= 0 ){
      rkJointSetVel( rkChainLinkJoint(c,i), &zVecElemNC(vel,rkChainLinkJointIDOffset(c,i)) );
      rkJointSetAcc( rkChainLinkJoint(c,i), &zVecElemNC(acc,rkChainLinkJointIDOffset(c,i)) );
    }
}

/* set all joint torques of a kinematic chain. */
void rkChainSetJointTrqAll(rkChain *c, zVec trq)
{
  int i;

  for( i=0; i<rkChainLinkNum(c); i++ )
    if( rkChainLinkJointIDOffset(c,i) >= 0 )
      rkChainLinkJointSetTrq( c, i, &zVecElemNC(trq,rkChainLinkJointIDOffset(c,i)) );
}

/* get all joint displacements of a kinematic chain. */
zVec rkChainGetJointDisAll(rkChain *c, zVec dis)
{
  int i;

  for( i=0; i<rkChainLinkNum(c); i++ )
    if( rkChainLinkJointIDOffset(c,i) >= 0 )
      rkChainLinkJointGetDis( c, i, &zVecElemNC(dis,rkChainLinkJointIDOffset(c,i)) );
  return dis;
}

/* get all joint velocities of a kinematic chain. */
zVec rkChainGetJointVelAll(rkChain *c, zVec vel)
{
  int i;

  for( i=0; i<rkChainLinkNum(c); i++ )
    if( rkChainLinkJointIDOffset(c,i) >= 0 )
      rkChainLinkJointGetVel( c, i, &zVecElemNC(vel,rkChainLinkJointIDOffset(c,i)) );
  return vel;
}

/* get all joint accelerations of a kinematic chain. */
zVec rkChainGetJointAccAll(rkChain *c, zVec acc)
{
  int i;

  for( i=0; i<rkChainLinkNum(c); i++ )
    if( rkChainLinkJointIDOffset(c,i) >= 0 )
      rkChainLinkJointGetAcc( c, i, &zVecElemNC(acc,rkChainLinkJointIDOffset(c,i)) );
  return acc;
}

/* get all joint torques of a kinematic chain. */
zVec rkChainGetJointTrqAll(rkChain *c, zVec trq)
{
  int i;

  for( i=0; i<rkChainLinkNum(c); i++ )
    if( rkChainLinkJointIDOffset(c,i) >= 0 )
      rkChainLinkJointGetTrq( c, i, &zVecElemNC(trq,rkChainLinkJointIDOffset(c,i)) );
  return trq;
}

/* get all link configurations of a kinematic chain. */
zVec rkChainGetConf(rkChain *chain, zVec conf)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    zFrame3DToArrayAA( rkChainLinkWldFrame(chain,i), &zVecElemNC(conf,i*6) );
  return conf;
}

/* set all link configurations of a kinematic chain. */
void rkChainSetConf(rkChain *chain, zVec conf)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    zArrayToFrame3DAA( &zVecElemNC(conf,i*6), rkChainLinkWldFrame(chain,i) );
  rkLinkConfToJointDis( rkChainRoot(chain) );
}

/* set all joint motor trq of a kinematic chain. */
void rkChainSetMotorInputAll(rkChain *c, zVec input)
{
  int i;

  for( i=0; i<rkChainLinkNum(c); i++ )
    if( rkChainLinkJointIDOffset(c,i) >= 0 )
      rkChainLinkJointMotorSetInput(c,i,&zVecElemNC(input,rkChainLinkJointIDOffset(c,i)));
}

/* update link frames of a kinematic chain via forward kinematics. */
void rkChainUpdateFK(rkChain *c)
{
  rkChainUpdateFrame( c );
  rkChainUpdateCOM( c );
}

/* solve forward kinematics of a kinematic chain. */
void rkChainFK(rkChain *c, zVec dis)
{
  rkChainSetJointDisAll( c, dis );
  rkChainUpdateFK( c );
}

/* neutralize all joints of a kinematic chain. */
void rkChainNeutral(rkChain *chain)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkChainLinkJointNeutral( chain, i );
  rkChainUpdateFK( chain );
}

/* update link states and joint torques of a kinematic chain via inverse dynamics. */
void rkChainUpdateID_G(rkChain *c, zVec6D *g)
{
  rkChainUpdateRateG( c, g );
  rkChainUpdateWrench( c );
  rkChainUpdateCOMVel( c );
  rkChainUpdateCOMAcc( c );
}

/* solve inverse dynamics of a kinematic chain. */
void rkChainID_G(rkChain *c, zVec vel, zVec acc, zVec6D *g)
{
  rkChainSetJointRateAll( c, vel, acc );
  rkChainUpdateID_G( c, g );
}

/* continuously update joint displacements of a kinematic chain over a time step. */
void rkChainFKCNT(rkChain *c, zVec dis, double dt)
{
  rkChainSetJointDisCNTAll( c, dis, dt );
  rkChainUpdateFK( c );
  rkChainUpdateID( c );
}

/* link acceleration at zero joint acceleration. */
zVec6D *rkChainLinkZeroAccG(rkChain *c, int id, zVec3D *p, zVec6D *g, zVec6D *a0)
{
  zVec3D tmp;

  rkChainSetJointAccAll( c, NULL );
  rkChainUpdateRateG( c, g );
  rkChainLinkPointAcc( c, id, p, &tmp );
  _zMulMat3DVec3D( rkChainLinkWldAtt(c,id), &tmp, zVec6DLin(a0) );
  _zMulMat3DVec3D( rkChainLinkWldAtt(c,id), rkChainLinkAngAcc(c,id), zVec6DAng(a0) );
  return a0;
}

/* the center of mass of a kinematic chain with respect to the total/world frame. */
zVec3D *rkChainUpdateCOM(rkChain *c)
{
  int i;

  rkChainSetWldCOM( c, ZVEC3DZERO );
  for( i=0; i<rkChainLinkNum(c); i++ )
    zVec3DCatDRC( rkChainWldCOM(c), rkChainLinkMass(c,i), rkChainLinkWldCOM(c,i) );
  return zVec3DDivDRC( rkChainWldCOM(c), rkChainMass(c) );
}

/* velocity of the center of mass of a kinematic chain with respect to the world frame. */
zVec3D *rkChainUpdateCOMVel(rkChain *c)
{
  int i;
  zVec3D v;

  rkChainSetCOMVel( c, ZVEC3DZERO );
  for( i=0; i<rkChainLinkNum(c); i++ ){
    /* COM velocity of link is with respect to the local frame,
       while COM velocity of a kinematic chain is with respect to
       the world frame. */
    zMulMat3DVec3D( rkChainLinkWldAtt(c,i), rkChainLinkCOMVel(c,i), &v );
    zVec3DCatDRC( rkChainCOMVel(c), rkChainLinkMass(c,i)/rkChainMass(c), &v );
  }
  return rkChainCOMVel(c);
}

/* acceleration of the center of mass of a kinematic chain with respect to the world frame. */
zVec3D *rkChainUpdateCOMAcc(rkChain *c)
{
  int i;
  zVec3D a;

  rkChainSetCOMAcc( c, ZVEC3DZERO );
  for( i=0; i<rkChainLinkNum(c); i++ ){
    /* COM acceleration of a link is with respect to the local frame,
       while COM acceleration of a kinematic chain is with respect to
       the world frame. */
    zMulMat3DVec3D( rkChainLinkWldAtt(c,i), rkChainLinkCOMAcc(c,i), &a );
    zVec3DCatDRC( rkChainCOMAcc(c), rkChainLinkMass(c,i)/rkChainMass(c), &a );
  }
  return rkChainCOMAcc(c);
}

/* Zero Moment Point of a kinematic chain. */
zVec3D *rkChainZMP(rkChain *c, double z, zVec3D *zmp)
{
  zVec3D dz;
  double f;

  rkChainGravityDir( c, &dz );
  if( zIsTiny( ( f = zVec3DInnerProd( &dz, rkChainRootForce(c) ) ) ) )
    return NULL; /* ZMP does not exist (floating). */
  zVec3DOuterProd( &dz, rkChainRootTorque(c), zmp );
  zVec3DCatDRC( zmp, z-rkChainRootPos(c)->e[zZ], rkChainRootForce(c) );
  zVec3DDivDRC( zmp, f );
  return zXform3DDRC( rkChainRootFrame(c), zmp );
}

/* net torque around vertical axis exerted to a kinematic chain. */
double rkChainYawTorque(rkChain *c)
{
  zVec3D dz;

  rkChainGravityDir( c, &dz );
  return zVec3DInnerProd(rkChainRootTorque(c),rkChainRootForce(c)) / zVec3DInnerProd(rkChainRootTorque(c),&dz);
}

/* angular momentum of a kinematic chain. */
zVec3D *rkChainAM(rkChain *c, zVec3D *p, zVec3D *am)
{
  int i;
  zVec3D tp, tmp;

  zVec3DZero( am );
  for( i=0; i<rkChainLinkNum(c); i++ ){
    zXform3DInv( rkChainLinkWldFrame(c,i), p, &tp );
    rkLinkAM( rkChainLink(c,i), &tp, &tmp );
    zMulMat3DVec3DDRC( rkChainLinkWldAtt(c,i), &tmp );
    zVec3DAddDRC( am, &tmp );
  }
  return am;
}

/* kinetic energy of a kinematic chain. */
double rkChainKE(rkChain *c)
{
  int i;
  double energy = 0;

  for( i=0; i<rkChainLinkNum(c); i++ )
    energy += rkLinkKE( rkChainLink(c,i) );
  return energy;
}

/* bias force vector of a kinematic chain. */
static void _rkChainBiasVec(rkChain *chain, zVec bias)
{
  rkChainSetJointAccAll( chain, NULL );
  rkChainUpdateID( chain );
  rkChainGetJointTrqAll( chain, bias );
}

/* bias force vector of a kinematic chain by the unit vector method. */
bool rkChainBiasVec(rkChain *chain, zVec bias)
{
  if( zVecSizeNC(bias) != rkChainJointSize(chain) ){
    ZRUNERROR( RK_ERR_MAT_VEC_SIZMISMATCH );
    return false;
  }
  _rkChainBiasVec( chain, bias );
  return true;
}

/* inertia matrix of a kinematic chain from momentum Jacobian matrices. */
zMat rkChainInertiaMatMJ(rkChain *chain, zMat inertia)
{
  zMat jacobi, inertia_tmp, tmp;
  zMatStruct mp;
  zMat3D mpe;
  int i, n;

  n = rkChainJointSize( chain );
  inertia_tmp = zMatAllocSqr( n );
  jacobi = zMatAlloc( 3, n );
  tmp = zMatAlloc( 3, n );
  zMatSetSizeNC( &mp, 3, 3 );
  zMatBufNC(&mp) = (double *)&mpe;
  zMatZero( inertia );
  for( i=rkChainLinkNum(chain)-1; i>=0; i-- ){
    /* linear component */
    rkChainLinkWldLinJacobi( chain, i, rkChainLinkCOM(chain,i), jacobi );
    zMatMulNC( jacobi, rkChainLinkMass(chain,i), tmp );
    zMulMatTMatNC( jacobi, tmp, inertia_tmp );
    zMatAddNCDRC( inertia, inertia_tmp );
    /* angular component */
    rkChainLinkWldAngJacobi( chain, i, jacobi );
    rkLinkWldInertia( rkChainLink(chain, i), &mpe );
    zMulMatMatNC( &mp, jacobi, tmp );
    zMulMatTMatNC( jacobi, tmp, inertia_tmp );
    zMatAddNCDRC( inertia, inertia_tmp );
  }
  zMatFreeAO( 3, inertia_tmp, jacobi, tmp );
  return inertia;
}

/* inertia matrix of a kinematic chain by the unit vector method. */
/* note: bias is an input and has to be precomputed. */
static void _rkChainInertiaMatUV(rkChain *chain, zVec bias, zMat inertia)
{
  int i, j;
  int k;
  zVecStruct h;
  double acc[] = { 0, 0, 0, 0, 0, 0 };

  rkChainSetJointAccAll( chain, NULL );
  /* inertia matrix */
  h.size = zMatRowSizeNC(inertia);
  for( i=j=0; j<rkChainLinkNum(chain); j++ )
    for( k=0; k<rkChainLinkJointSize(chain,j); k++, i++ ){
      h.buf = zMatRowBuf( inertia, i );
      acc[k] = 1;
      rkChainLinkJointSetAcc( chain, j, acc );
      rkChainUpdateID( chain );
      rkChainGetJointTrqAll( chain, &h );
      zVecSubDRC( &h, bias );
      acc[k] = 0;
      rkChainLinkJointSetAcc( chain, j, acc );
    }
}

/* inertia matrix of a kinematic chain by the unit vector method. */
bool rkChainInertiaMatUV(rkChain *chain, zMat inertia)
{
  zVec b;

  if( !zMatIsSqr( inertia ) || zMatColSizeNC(inertia) != rkChainJointSize(chain) ){
    ZRUNERROR( RK_ERR_MAT_VEC_SIZMISMATCH );
    return false;
  }
  if( !( b = zVecAlloc( zMatRowSizeNC(inertia) ) ) ) return false;
  _rkChainBiasVec( chain, b );
  _rkChainInertiaMatUV( chain, b, inertia );
  zVecFree( b );
  return true;
}

/* compute a cell of the inertia matrix of a kinematic chain based on the composite rigid body method. */
static void _rkChainLinkInertiaMatCRB(rkLink *link, zVec6D wi[], zVec6D si[], zMat hij, zMat inertia)
{
  rkLink *lp;
  zFrame3D f;
  int i, j;

  /* diagonal block */
  rkJointCRBWrench( rkLinkJoint(link), rkLinkCRB(link), wi );
  rkJointCRBXform( rkLinkJoint(link), ZFRAME3DIDENT, si );
  zMatSetSizeNC( hij, rkLinkJointSize(link), rkLinkJointSize(link) );
  for( i=0; i<rkLinkJointSize(link); i++ )
    for( j=0; j<rkLinkJointSize(link); j++ )
      zMatElemNC(hij,i,j) = zVec6DInnerProd( &si[i], &wi[j] );
  zMatPutNC( inertia, rkLinkJointIDOffset(link), rkLinkJointIDOffset(link), hij );
  /* non-diagonal block */
  for( lp=rkLinkParent(link); lp; lp=rkLinkParent(lp) ){
    _zFrame3DXform( rkLinkWldFrame(link), rkLinkWldFrame(lp), &f );
    rkJointCRBXform( rkLinkJoint(lp), &f, si );
    zMatSetSizeNC( hij, rkLinkJointSize(lp), rkLinkJointSize(link) );
    for( i=0; i<rkLinkJointSize(lp); i++ )
      for( j=0; j<rkLinkJointSize(link); j++ )
        zMatElemNC(hij,i,j) = zVec6DInnerProd( &si[i], &wi[j] );
    zMatPutNC(  inertia, rkLinkJointIDOffset(lp), rkLinkJointIDOffset(link), hij );
    zMatTPutNC( inertia, rkLinkJointIDOffset(link), rkLinkJointIDOffset(lp), hij );
  }

  if( rkLinkChild(link) )
    _rkChainLinkInertiaMatCRB( rkLinkChild(link), wi, si, hij, inertia );
  if( rkLinkSibl(link) )
    _rkChainLinkInertiaMatCRB( rkLinkSibl(link), wi, si, hij, inertia );
}

/* compute the inertia matrix of a kinematic chain based on the composite rigid body method. */
bool rkChainInertiaMatCRB(rkChain *chain, zMat inertia)
{
  zMatStruct hij;
  zVec6D wi[6], si[6];
  double _e[36];

  rkChainUpdateCRB( chain );
  zMatBufNC(&hij) = _e;
  _rkChainLinkInertiaMatCRB( rkChainRoot(chain), wi, si, &hij, inertia );
  return true;
}

bool (* rkChainInertiaMat)(rkChain*,zMat) = rkChainInertiaMatCRB;

/* inertia matrix and bias force vector of a kinematic chain by the unit vector method. */
bool rkChainInertiaMatBiasVecUV(rkChain *chain, zMat inertia, zVec bias)
{
  if( !zMatIsSqr( inertia ) || !zMatColVecSizeIsEqual( inertia, bias ) ||
      zVecSizeNC(bias) != rkChainJointSize(chain) ){
    ZRUNERROR( RK_ERR_MAT_VEC_SIZMISMATCH );
    return false;
  }
  _rkChainBiasVec( chain, bias );
  _rkChainInertiaMatUV( chain, bias, inertia );
  return true;
}

/* inertia matrix and bias force vector of a kinematic chain by the composite rigid body method. */
bool rkChainInertiaMatBiasVecCRB(rkChain *chain, zMat inertia, zVec bias)
{
  if( !zMatIsSqr( inertia ) || !zMatColVecSizeIsEqual( inertia, bias ) ||
      zVecSizeNC(bias) != rkChainJointSize(chain) ){
    ZRUNERROR( RK_ERR_MAT_VEC_SIZMISMATCH );
    return false;
  }
  _rkChainBiasVec( chain, bias );
  rkChainInertiaMatCRB( chain, inertia );
  return true;
}

bool (* rkChainInertiaMatBiasVec)(rkChain*,zMat,zVec) = rkChainInertiaMatBiasVecCRB;

/* net external wrench applied to a kinematic chain. */
zVec6D *rkChainNetExtWrench(rkChain *c, zVec6D *w)
{
  int i;
  zVec6D ew;

  zVec6DZero( w );
  for( i=0; i<rkChainLinkNum(c); i++ ){
    rkLinkNetExtWrench( rkChainLink(c,i), &ew );
    if( zVec6DEqual( &ew, ZVEC6DZERO ) ) continue;
    zMulMat3DVec6DDRC( rkChainLinkWldAtt(c,i), &ew );
    zVec6DAngShiftDRC( &ew, rkChainLinkWldPos(c,i) );
    zVec6DAddDRC( w, &ew );
  }
  return w;
}

/* destroy external wrench list attached to a kinematic chain. */
void rkChainExtWrenchDestroy(rkChain *c)
{
  int i;

  for( i=0; i<rkChainLinkNum(c); i++ )
    rkLinkExtWrenchDestroy( rkChainLink(c,i) );
}

/* set joint identifier offset values of links of a kinematic chain. */
void rkChainSetJointIDOffset(rkChain *c)
{
  int i, s;

  for( i=0, s=0; i<rkChainLinkNum(c); i++ )
    if( rkChainLinkJointSize(c,i) > 0 ){
      rkLinkSetJointIDOffset( rkChainLink(c,i), s );
      s += rkChainLinkJointSize(c,i);
    } else
      rkLinkSetJointIDOffset( rkChainLink(c,i), -1 );
}

/* make a list of vertices of a kinematic chain. */
zVec3DList *rkChainVertList(rkChain *chain, zVec3DList *vl)
{
  rkLink *l;
  zShapeListCell *sc;
  zShape3D s;
  zVec3D v;
  int i, j;

  zListInit( vl );
  for( i=0; i<rkChainLinkNum(chain); i++ ){
    l = rkChainLink(chain,i);
    zListForEach( rkLinkShapeList(l), sc ){
      if( sc->data->com == &zeo_shape3d_ph_com ){
        for( j=0; j<zShape3DVertNum(sc->data); j++ ){
          zXform3D( rkLinkWldFrame(l), zShape3DVert(sc->data,j), &v );
          if( !zVec3DListAdd( vl, &v ) ) return NULL;
        }
      } else{
        zShape3DClone( sc->data, &s, NULL );
        zShape3DXform( sc->data, rkLinkWldFrame(l), &s );
        if( !zShape3DToPH( &s ) ) return NULL;
        if( !zVec3DListAppendArray( vl, &zShape3DPH(&s)->vert ) ) vl = NULL;
        zShape3DDestroy( &s );
        if( !vl ) return NULL;
      }
    }
  }
  return vl;
}

/* generate the bounding ball of a kinematic chain. */
zSphere3D *rkChainBBall(rkChain *chain, zSphere3D *bb)
{
  zVec3DList pl;

  if( rkChainVertList( chain, &pl ) )
    zBBall3DPL( bb, &pl, NULL );
  else
    bb = NULL;
  zVec3DListDestroy( &pl );
  return bb;
}

/* ZTK */

static void *_rkChainNameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return zNameSet( (rkChain*)obj, ZTKVal(ztk) ) ? obj : NULL;
}

static void _rkChainNameFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName((rkChain*)obj) );
}

static ZTKPrp __ztk_prp_rkchain_chain[] = {
  { "name", 1, _rkChainNameFromZTK, _rkChainNameFPrintZTK },
};

static void *_rkChainChainFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return ZTKEvalKey( obj, arg, ztk, __ztk_prp_rkchain_chain );
}
static void *_rkChainMotorFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkMotorFromZTK( zArrayElemNC(rkChainMotor((rkChain*)obj),i), ztk );
}
static void *_rkChainLinkFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkLinkFromZTK( rkChainLink((rkChain*)obj,i),
    &((rkChain*)obj)->link,
    rkChainShape((rkChain*)obj) ? &rkChainShape((rkChain*)obj)->shape : NULL,
    rkChainMotor((rkChain*)obj), ztk ) ? obj : NULL;
}
static void *_rkChainLinkConnectFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkLinkConnectFromZTK( rkChainLink((rkChain*)obj,i),
    &((rkChain*)obj)->link, ztk ) ? obj : NULL;
}

static void _rkChainChainFPrintZTK(FILE *fp, int i, void *obj){
  ZTKPrpKeyFPrint( fp, obj, __ztk_prp_rkchain_chain );
}

static void *_rkChainInitPosFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zVec3DFromZTK( rkChainOrgPos((rkChain*)obj), ztk );
  return obj;
}
static void *_rkChainInitAttFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zMat3DFromZTK( rkChainOrgAtt((rkChain*)obj), ztk );
  return obj;
}
static void *_rkChainInitFrameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zFrame3DFromZTK( rkChainOrgFrame((rkChain*)obj), ztk );
  return obj;
}
static void *_rkChainInitJointFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkLink *link;
  if( !( link = rkChainFindLink( (rkChain*)obj, ZTKVal(ztk) ) ) ){
    ZRUNERROR( RK_ERR_LINK_UNKNOWN, ZTKVal(ztk) );
    return NULL;
  }
  ZTKValNext( ztk );
  rkLinkJoint(link)->com->_dis_fromZTK( rkLinkJoint(link), 0, NULL, ztk );
  return obj;
}

static void _rkChainInitPosFPrintZTK(FILE *fp, int i, void *obj){
  zVec3DFPrint( fp, rkChainOrgPos((rkChain*)obj) );
}
static void _rkChainInitAttFPrintZTK(FILE *fp, int i, void *obj){
  zMat3DFPrint( fp, rkChainOrgAtt((rkChain*)obj) );
}

static ZTKPrp __ztk_prp_rkchain_initkey[] = {
  { "pos", 1, _rkChainInitPosFromZTK, _rkChainInitPosFPrintZTK },
  { "att", 1, _rkChainInitAttFromZTK, _rkChainInitAttFPrintZTK },
  { "frame", 1, _rkChainInitFrameFromZTK, NULL },
  { "joint", -1, _rkChainInitJointFromZTK, NULL },
};

static void *_rkChainInitFromZTK(void *obj, int i, void *arg, ZTK *ztk)
{
  if( !ZTKEvalKey( obj, NULL, ztk, __ztk_prp_rkchain_initkey ) ) return NULL;
  return obj;
}

static void _rkChainInitFPrintZTK(FILE *fp, int i, void *obj)
{
  int k;
  rkLink *link;

  ZTKPrpKeyFPrint( fp, obj, __ztk_prp_rkchain_initkey );
  for( k=0; k<rkChainLinkNum((rkChain*)obj); k++ ){
    link = rkChainLink((rkChain*)obj,k);
    if( rkLinkJointSize(link) == 0 || rkJointIsNeutral( rkLinkJoint(link) ) ) continue;
    fprintf( fp, "joint: %s ", zName(link) );
    rkLinkJoint(link)->com->_dis_fprintZTK( fp, 0, rkLinkJoint(link) );
  }
}

static ZTKPrp __ztk_prp_tag_rkchain_optic[] = {
  { ZTK_TAG_OPTIC, -1, NULL, NULL },
};

static ZTKPrp __ztk_prp_tag_rkchain_shape[] = {
  { ZTK_TAG_SHAPE, -1, NULL, NULL },
};

static ZTKPrp __ztk_prp_tag_rkchain_motor[] = {
  { ZTK_TAG_RKMOTOR, -1, _rkChainMotorFromZTK, NULL },
};

static ZTKPrp __ztk_prp_tag_rkchain_link[] = {
  { ZTK_TAG_RKLINK, -1, _rkChainLinkFromZTK, NULL },
};

static ZTKPrp __ztk_prp_tag_rkchain_connection[] = {
  { ZTK_TAG_RKLINK, -1, _rkChainLinkConnectFromZTK, NULL },
};

static ZTKPrp __ztk_prp_tag_rkchain[] = {
  { ZTK_TAG_RKCHAIN, 1, _rkChainChainFromZTK, _rkChainChainFPrintZTK },
  { ZTK_TAG_INIT, 1, _rkChainInitFromZTK, NULL },
};

rkChain *rkChainFromZTK(rkChain *chain, ZTK *ztk)
{
  int num_motor, num_link;

  if( ( num_motor = ZTKCountTag( ztk, ZTK_TAG_RKMOTOR ) ) > 0 ){
    if( !( rkChainMotor(chain) = zAlloc( rkMotorArray, 1 ) ) ){
      ZALLOCERROR();
      return NULL;
    }
    zArrayAlloc( rkChainMotor(chain), rkMotor, num_motor );
    if( zArraySize(rkChainMotor(chain)) != num_motor ) return NULL;
  }
  num_link = ZTKCountTag( ztk, ZTK_TAG_RKLINK );
  zArrayAlloc( &chain->link, rkLink, num_link );
  if( rkChainLinkNum(chain) != num_link ) return NULL;
  if( rkChainLinkNum(chain) == 0 ){
    ZRUNWARN( RK_WARN_CHAIN_EMPTY );
    return NULL;
  }
  ZTKEvalTag( chain, NULL, ztk, __ztk_prp_tag_rkchain_optic );
  ZTKEvalTag( chain, NULL, ztk, __ztk_prp_tag_rkchain_shape );
  ZTKEvalTag( chain, NULL, ztk, __ztk_prp_tag_rkchain_motor );
  ZTKEvalTag( chain, NULL, ztk, __ztk_prp_tag_rkchain_link );
  ZTKEvalTag( chain, NULL, ztk, __ztk_prp_tag_rkchain_connection );
  ZTKEvalTag( chain, NULL, ztk, __ztk_prp_tag_rkchain );
  rkChainSetJointIDOffset( chain ); /* joint identifier offset value */
  rkChainUpdateCRBMass( chain );
  if( rkChainMass(chain) == 0 )
    rkChainSetMass( chain, 1.0 ); /* dummy weight */
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  return chain;
}

/* print information of a kinematic chain out to the current position of a file. */
void rkChainFPrintZTK(FILE *fp, rkChain *chain)
{
  int i;

  ZTKPrpTagFPrint( fp, chain, __ztk_prp_tag_rkchain );
  fprintf( fp, "\n" );
  if( rkChainShape(chain) )
    zMShape3DFPrintZTK( fp, rkChainShape(chain) );
  if( rkChainMotor(chain) )
    rkMotorArrayFPrintZTK( fp, rkChainMotor(chain) );
  for( i=0; i<rkChainLinkNum(chain); i++ ){
    fprintf( fp, "[%s]\n", ZTK_TAG_RKLINK );
    rkLinkFPrintZTK( fp, rkChainLink(chain,i) );
  }
  fprintf( fp, "[%s]\n", ZTK_TAG_INIT );
  _rkChainInitFPrintZTK( fp, 0, chain );
}

/* read a ZTK file and create a new kinematic chain. */
rkChain *rkChainReadZTK(rkChain *chain, const char *filename)
{
  ZTK ztk;

  ZTKInit( &ztk );
  if( ZTKParse( &ztk, (char *)filename ) ){
    /* read optical infos and shapes */
    rkChainInit( chain );
    if( ZTKCountTag( &ztk, ZTK_TAG_SHAPE ) > 0 ){
      if( !( rkChainShape(chain) = zAlloc( zMShape3D, 1 ) ) ){
        ZALLOCERROR();
        return NULL;
      }
      if( !zMShape3DFromZTK( rkChainShape(chain), &ztk ) ) return NULL;
    }
    /* read robot name, motors and links */
    chain = rkChainFromZTK( chain, &ztk );
  } else
    chain = NULL;
  ZTKDestroy( &ztk );
  return chain;
}

/* write information of a kinematic chain to a file in ZTK format. */
bool rkChainWriteZTK(rkChain *c, const char *filename)
{
  FILE *fp;

  if( !( fp = zOpenZTKFile( (char *)filename, "w" ) ) ) return false;
  rkChainFPrintZTK( fp, c );
  fclose(fp);
  return true;
}

static ZTKPrp __ztk_prp_tag_rkchain_init[] = {
  { ZTK_TAG_INIT, 1, _rkChainInitFromZTK, _rkChainInitFPrintZTK },
};

rkChain *rkChainInitFromZTK(rkChain *chain, ZTK *ztk)
{
  ZTKEvalTag( chain, NULL, ztk, __ztk_prp_tag_rkchain_init );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  return chain;
}

void rkChainInitFPrintZTK(FILE *fp, rkChain *chain)
{
  ZTKPrpTagFPrint( fp, chain, __ztk_prp_tag_rkchain_init );
  fprintf( fp, "\n" );
}

rkChain *rkChainInitReadZTK(rkChain *chain, const char *filename)
{
  ZTK ztk;

  ZTKInit( &ztk );
  ZTKParse( &ztk, (char *)filename );
  chain = rkChainInitFromZTK( chain, &ztk );
  ZTKDestroy( &ztk );
  return chain;
}

bool rkChainInitWriteZTK(rkChain *chain, const char *filename)
{
  FILE *fp;

  if( !( fp = zOpenZTKFile( (char *)filename, "w" ) ) ) return false;
  rkChainInitFPrintZTK( fp, chain );
  fclose( fp );
  return true;
}

/* print current posture of a kinematic chain out to a file. */
void rkChainPostureFPrint(FILE *fp, rkChain *c)
{
  int i;

  fprintf( fp, "Chain : %s\n", zName(c) );
  for( i=0; i<rkChainLinkNum(c); i++ )
    rkLinkPostureFPrint( fp, rkChainLink(c,i) );
}

/* print connection of a kinematic chain out to a file. */
void rkChainConnectionFPrint(FILE *fp, rkChain *c)
{
  fprintf( fp, "Chain : %s\n", zName(c) );
  rkLinkConnectionFPrint( fp, rkChainRoot(c), 0 );
}

/* print external wrench exerted to a kinematic chain out to a file. */
void rkChainExtWrenchFPrint(FILE *fp, rkChain *c)
{
  int i;

  for( i=0; i<rkChainLinkNum(c); i++ )
    if( zListSize(rkChainLinkExtWrench(c,i)) != 0 ){
      fprintf( fp, "[%s]\n", rkChainLinkName(c,i) );
      rkLinkExtWrenchFPrint( fp, rkChainLink(c,i) );
    }
}
