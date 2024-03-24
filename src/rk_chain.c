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
void rkChainInit(rkChain *chain)
{
  zNameSetPtr( chain, NULL );
  zArrayInit( rkChainLinkArray(chain) );
  rkChainSetShape( chain, NULL );
  zArrayInit( rkChainMotorSpecArray(chain) );
  rkChainSetWldCOM( chain, ZVEC3DZERO );
  rkChainSetCOMVel( chain, ZVEC3DZERO );
  rkChainSetCOMAcc( chain, ZVEC3DZERO );
  chain->_ik = NULL;
}

/* destroy a kinematic chain. */
void rkChainDestroy(rkChain *chain)
{
  if( !chain ) return;
  zNameFree( chain );
  rkLinkArrayDestroy( rkChainLinkArray(chain) );
  zMShape3DDestroy( rkChainShape(chain) );
  zFree( rkChainShape(chain) );
  rkMotorSpecArrayDestroy( rkChainMotorSpecArray(chain) );
  rkChainDestroyIK( chain );
  rkChainInit( chain );
}

/* clone a kinematic chain. */
rkChain *rkChainClone(rkChain *org, rkChain *cln)
{
  char name[BUFSIZ];

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
  if( !zNameSet( cln, name ) ){
    ZALLOCERROR();
    return NULL;
  }
  if( rkChainShape(org) && !( rkChainShape(cln) = zMShape3DClone( rkChainShape(org) ) ) )
    return NULL;
  if( !rkMotorSpecArrayClone( rkChainMotorSpecArray(org), rkChainMotorSpecArray(cln) ) )
    return NULL;
  if( !rkLinkArrayClone( rkChainLinkArray(org), rkChainLinkArray(cln), rkChainShape(org), rkChainShape(cln), rkChainMotorSpecArray(org), rkChainMotorSpecArray(cln) ) )
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
int rkChainJointSize(rkChain *chain)
{
  int i, size;

  for( size=0, i=0; i<rkChainLinkNum(chain); i++ )
    size += rkChainLinkJointDOF(chain,i);
  return size;
}

/* create default joint index of a kinematic chain. */
zIndex rkChainCreateDefaultJointIndex(rkChain *chain)
{
  int i, count;
  zIndex index;

  for( count=0, i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointDOF(chain,i) > 0 ) count++;
  if( !( index = zIndexCreate( count ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  for( count=0, i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointDOF(chain,i) > 0 ){
      zIndexSetElemNC( index, count, i );
      count++;
    }
  return index;
}

/* count the total joint size indicated in a kinematic chain. */
int rkChainJointIndexSize(rkChain *chain, zIndex idx)
{
  int i, size;

  for( size=0, i=0; i<zArraySize(idx); i++ )
    size += rkChainLinkJointDOF(chain,zIndexElemNC(idx,i));
  return size;
}

/* find a link of a kinematic chain from name. */
rkLink *rkChainFindLink(rkChain *chain, const char *name)
{
  rkLink *l;

  zArrayFindName( rkChainLinkArray(chain), name, l );
  if( !l ) ZRUNERROR( RK_ERR_LINK_UNKNOWN, name );
  return l;
}

/* find a link identifier of a kinematic chain from name. */
int rkChainFindLinkID(rkChain *chain, const char *name)
{
  rkLink *l;
  return ( l = rkChainFindLink( chain, name ) ) ? (int)( l - rkChainRoot(chain) ) : -1;
}

/* find a joint identifier offset of a link of a kinematic chain from name. */
int rkChainFindLinkJointIDOffset(rkChain *chain, const char *name)
{
  rkLink *l;
  return ( l = rkChainFindLink( chain, name ) ) ? rkLinkJointIDOffset(l) : -1;
}

/* set joint displacements of a kinematic chain. */
void rkChainSetJointDis(rkChain *chain, zIndex idx, zVec dis)
{
  int i;
  double *dp;

  for( dp=zVecBuf(dis), i=0; i<zArraySize(idx); i++ ){
    rkChainLinkJointSetDis( chain, zIndexElemNC(idx,i), dp );
    dp += rkChainLinkJointDOF(chain,zIndexElemNC(idx,i));
  }
}

/* continuously update joint displacements of a kinematic chain. */
void rkChainSetJointDisCNT(rkChain *chain, zIndex idx, zVec dis, double dt)
{
  int i;
  double *dp;

  for( dp=zVecBuf(dis), i=0; i<zArraySize(idx); i++ ){
    rkChainLinkJointSetDisCNT( chain, zIndexElemNC(idx,i), dp, dt );
    dp += rkChainLinkJointDOF(chain,zIndexElemNC(idx,i));
  }
}

/* set joint velocities of a kinematic chain. */
void rkChainSetJointVel(rkChain *chain, zIndex idx, zVec vel)
{
  int i;
  double *vp;

  for( vp=zVecBuf(vel), i=0; i<zArraySize(idx); i++ ){
    rkJointSetVel( rkChainLinkJoint(chain,zIndexElemNC(idx,i)), vp );
    vp += rkChainLinkJointDOF(chain,zIndexElemNC(idx,i));
  }
}

/* set joint accelerations of a kinematic chain. */
void rkChainSetJointAcc(rkChain *chain, zIndex idx, zVec acc)
{
  int i;
  double *vp;

  for( vp=zVecBuf(acc), i=0; i<zArraySize(idx); i++ ){
    rkJointSetAcc( rkChainLinkJoint(chain,zIndexElemNC(idx,i)), vp );
    vp += rkChainLinkJointDOF(chain,zIndexElemNC(idx,i));
  }
}

/* set joint velocities and accelerations of a kinematic chain. */
void rkChainSetJointRate(rkChain *chain, zIndex idx, zVec vel, zVec acc)
{
  int i;
  double *vp, *ap;

  for( vp=zVecBuf(vel), ap=zVecBuf(acc), i=0; i<zArraySize(idx); i++ ){
    rkJointSetVel( rkChainLinkJoint(chain,zIndexElemNC(idx,i)), vp );
    rkJointSetAcc( rkChainLinkJoint(chain,zIndexElemNC(idx,i)), ap );
    vp += rkChainLinkJointDOF(chain,zIndexElemNC(idx,i));
    ap += rkChainLinkJointDOF(chain,zIndexElemNC(idx,i));
  }
}

/* get joint displacements of a kinematic chain. */
zVec rkChainGetJointDis(rkChain *chain, zIndex idx, zVec dis)
{
  int i;
  double *dp;

  for( dp=zVecBuf(dis), i=0; i<zArraySize(idx); i++ ){
    rkChainLinkJointGetDis( chain, zIndexElemNC(idx,i), dp );
    dp += rkChainLinkJointDOF(chain,zIndexElemNC(idx,i));
  }
  return dis;
}

/* get joint velocities of a kinematic chain. */
zVec rkChainGetJointVel(rkChain *chain, zIndex idx, zVec vel)
{
  int i;
  double *dp;

  for( dp=zVecBuf(vel), i=0; i<zArraySize(idx); i++ ){
    rkChainLinkJointGetVel( chain, zIndexElemNC(idx,i), dp );
    dp += rkChainLinkJointDOF(chain,zIndexElemNC(idx,i));
  }
  return vel;
}

/* get joint accelerations of a kinematic chain. */
zVec rkChainGetJointAcc(rkChain *chain, zIndex idx, zVec acc)
{
  int i;
  double *dp;

  for( dp=zVecBuf(acc), i=0; i<zArraySize(idx); i++ ){
    rkChainLinkJointGetAcc( chain, zIndexElemNC(idx,i), dp );
    dp += rkChainLinkJointDOF(chain,zIndexElemNC(idx,i));
  }
  return acc;
}

/* set all joint displacements of a kinematic chain. */
void rkChainSetJointDisAll(rkChain *chain, zVec dis)
{
  int i;

  if( dis ){
    for( i=0; i<rkChainLinkNum(chain); i++ )
      if( rkChainLinkJointIDOffset(chain,i) >= 0 )
        rkChainLinkJointSetDis( chain, i, &zVecElemNC(dis,rkChainLinkJointIDOffset(chain,i)) );
  } else{
    for( i=0; i<rkChainLinkNum(chain); i++ )
      if( rkChainLinkJointIDOffset(chain,i) >= 0 )
        rkChainLinkJointSetDis( chain, i, ZVEC6DZERO->e );
  }
}

/* concatenate all joint displacements of a kinematic chain. */
void rkChainCatJointDisAll(rkChain *chain, zVec dis, double k, zVec v)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointIDOffset(chain,i) >= 0 )
      rkJointCatDis( rkChainLinkJoint(chain,i), &zVecElemNC(dis,rkChainLinkJointIDOffset(chain,i)), k, &zVecElemNC(v,rkChainLinkJointIDOffset(chain,i)) );
}

/* subtract all joint displacements of a kinematic chain. */
void rkChainSubJointDisAll(rkChain *chain, zVec dis, zVec sdis)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointIDOffset(chain,i) >= 0 )
      rkJointSubDis( rkChainLinkJoint(chain,i), &zVecElemNC(dis,rkChainLinkJointIDOffset(chain,i)), &zVecElemNC(sdis,rkChainLinkJointIDOffset(chain,i)) );
}

/* continuously update all joint displacements of a kinematic chain. */
void rkChainSetJointDisCNTAll(rkChain *chain, zVec dis, double dt)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointIDOffset(chain,i) >= 0 )
      rkChainLinkJointSetDisCNT( chain, i, &zVecElemNC(dis,rkChainLinkJointIDOffset(chain,i)), dt );
}

/* set all joint velocities of a kinematic chain. */
void rkChainSetJointVelAll(rkChain *chain, zVec vel)
{
  int i;

  if( vel ){
    for( i=0; i<rkChainLinkNum(chain); i++ )
      if( rkChainLinkJointIDOffset(chain,i) >= 0 )
        rkJointSetVel( rkChainLinkJoint(chain,i), &zVecElemNC(vel,rkChainLinkJointIDOffset(chain,i)) );
  } else{
    for( i=0; i<rkChainLinkNum(chain); i++ )
      if( rkChainLinkJointIDOffset(chain,i) >= 0 )
        rkJointSetVel( rkChainLinkJoint(chain,i), ZVEC6DZERO->e );
  }
}

/* set all joint accelerations of a kinematic chain. */
void rkChainSetJointAccAll(rkChain *chain, zVec acc)
{
  int i;

  if( acc ){
    for( i=0; i<rkChainLinkNum(chain); i++ )
      if( rkChainLinkJointIDOffset(chain,i) >= 0 )
        rkJointSetAcc( rkChainLinkJoint(chain,i), &zVecElemNC(acc,rkChainLinkJointIDOffset(chain,i)) );
  } else{
    for( i=0; i<rkChainLinkNum(chain); i++ )
      if( rkChainLinkJointIDOffset(chain,i) >= 0 )
        rkJointSetAcc( rkChainLinkJoint(chain,i), ZVEC6DZERO->e );
  }
}

/* set all joint velocities and accelerations of a kinematic chain. */
void rkChainSetJointRateAll(rkChain *chain, zVec vel, zVec acc)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointIDOffset(chain,i) >= 0 ){
      rkJointSetVel( rkChainLinkJoint(chain,i), &zVecElemNC(vel,rkChainLinkJointIDOffset(chain,i)) );
      rkJointSetAcc( rkChainLinkJoint(chain,i), &zVecElemNC(acc,rkChainLinkJointIDOffset(chain,i)) );
    }
}

/* set all joint torques of a kinematic chain. */
void rkChainSetJointTrqAll(rkChain *chain, zVec trq)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointIDOffset(chain,i) >= 0 )
      rkChainLinkJointSetTrq( chain, i, &zVecElemNC(trq,rkChainLinkJointIDOffset(chain,i)) );
}

/* get all joint displacements of a kinematic chain. */
zVec rkChainGetJointDisAll(rkChain *chain, zVec dis)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointIDOffset(chain,i) >= 0 )
      rkChainLinkJointGetDis( chain, i, &zVecElemNC(dis,rkChainLinkJointIDOffset(chain,i)) );
  return dis;
}

/* get all joint velocities of a kinematic chain. */
zVec rkChainGetJointVelAll(rkChain *chain, zVec vel)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointIDOffset(chain,i) >= 0 )
      rkChainLinkJointGetVel( chain, i, &zVecElemNC(vel,rkChainLinkJointIDOffset(chain,i)) );
  return vel;
}

/* get all joint accelerations of a kinematic chain. */
zVec rkChainGetJointAccAll(rkChain *chain, zVec acc)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointIDOffset(chain,i) >= 0 )
      rkChainLinkJointGetAcc( chain, i, &zVecElemNC(acc,rkChainLinkJointIDOffset(chain,i)) );
  return acc;
}

/* get all joint torques of a kinematic chain. */
zVec rkChainGetJointTrqAll(rkChain *chain, zVec trq)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointIDOffset(chain,i) >= 0 )
      rkChainLinkJointGetTrq( chain, i, &zVecElemNC(trq,rkChainLinkJointIDOffset(chain,i)) );
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
void rkChainSetMotorInputAll(rkChain *chain, zVec input)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointIDOffset(chain,i) >= 0 )
      rkChainLinkJointMotorSetInput(chain,i,&zVecElemNC(input,rkChainLinkJointIDOffset(chain,i)));
}

/* update link frames of a kinematic chain via forward kinematics. */
void rkChainUpdateFK(rkChain *chain)
{
  rkChainUpdateFrame( chain );
  rkChainUpdateCOM( chain );
}

/* solve forward kinematics of a kinematic chain. */
void rkChainFK(rkChain *chain, zVec dis)
{
  rkChainSetJointDisAll( chain, dis );
  rkChainUpdateFK( chain );
}

/* neutralize all joints of a kinematic chain. */
void rkChainNeutralize(rkChain *chain)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkChainLinkJointNeutralize( chain, i );
  rkChainUpdateFK( chain );
}

/* update link states and joint torques of a kinematic chain via inverse dynamics. */
void rkChainUpdateID_G(rkChain *chain, zVec6D *g)
{
  rkChainUpdateRateG( chain, g );
  rkChainUpdateWrench( chain );
  rkChainUpdateCOMVel( chain );
  rkChainUpdateCOMAcc( chain );
}

/* solve inverse dynamics of a kinematic chain. */
void rkChainID_G(rkChain *chain, zVec vel, zVec acc, zVec6D *g)
{
  rkChainSetJointRateAll( chain, vel, acc );
  rkChainUpdateID_G( chain, g );
}

/* continuously update joint displacements of a kinematic chain over a time step. */
void rkChainFKCNT(rkChain *chain, zVec dis, double dt)
{
  rkChainSetJointDisCNTAll( chain, dis, dt );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
}

/* link acceleration at zero joint acceleration. */
zVec6D *rkChainLinkZeroAccG(rkChain *chain, int id, zVec3D *p, zVec6D *g, zVec6D *a0)
{
  zVec3D tmp;

  rkChainSetJointAccAll( chain, NULL );
  rkChainUpdateRateG( chain, g );
  rkChainLinkPointAcc( chain, id, p, &tmp );
  _zMulMat3DVec3D( rkChainLinkWldAtt(chain,id), &tmp, zVec6DLin(a0) );
  _zMulMat3DVec3D( rkChainLinkWldAtt(chain,id), rkChainLinkAngAcc(chain,id), zVec6DAng(a0) );
  return a0;
}

/* the center of mass of a kinematic chain with respect to the total/world frame. */
zVec3D *rkChainUpdateCOM(rkChain *chain)
{
  int i;

  rkChainSetWldCOM( chain, ZVEC3DZERO );
  for( i=0; i<rkChainLinkNum(chain); i++ )
    zVec3DCatDRC( rkChainWldCOM(chain), rkChainLinkMass(chain,i), rkChainLinkWldCOM(chain,i) );
  return zVec3DDivDRC( rkChainWldCOM(chain), rkChainMass(chain) );
}

/* velocity of the center of mass of a kinematic chain with respect to the world frame. */
zVec3D *rkChainUpdateCOMVel(rkChain *chain)
{
  int i;
  zVec3D v;

  rkChainSetCOMVel( chain, ZVEC3DZERO );
  for( i=0; i<rkChainLinkNum(chain); i++ ){
    /* COM velocity of link is with respect to the local frame,
       while COM velocity of a kinematic chain is with respect to
       the world frame. */
    zMulMat3DVec3D( rkChainLinkWldAtt(chain,i), rkChainLinkCOMVel(chain,i), &v );
    zVec3DCatDRC( rkChainCOMVel(chain), rkChainLinkMass(chain,i)/rkChainMass(chain), &v );
  }
  return rkChainCOMVel(chain);
}

/* acceleration of the center of mass of a kinematic chain with respect to the world frame. */
zVec3D *rkChainUpdateCOMAcc(rkChain *chain)
{
  int i;
  zVec3D a;

  rkChainSetCOMAcc( chain, ZVEC3DZERO );
  for( i=0; i<rkChainLinkNum(chain); i++ ){
    /* COM acceleration of a link is with respect to the local frame,
       while COM acceleration of a kinematic chain is with respect to
       the world frame. */
    zMulMat3DVec3D( rkChainLinkWldAtt(chain,i), rkChainLinkCOMAcc(chain,i), &a );
    zVec3DCatDRC( rkChainCOMAcc(chain), rkChainLinkMass(chain,i)/rkChainMass(chain), &a );
  }
  return rkChainCOMAcc(chain);
}

/* Zero Moment Point of a kinematic chain. */
zVec3D *rkChainZMP(rkChain *chain, double z, zVec3D *zmp)
{
  zVec3D dz;
  double f;

  rkChainGravityDir( chain, &dz );
  if( zIsTiny( ( f = zVec3DInnerProd( &dz, rkChainRootForce(chain) ) ) ) )
    return NULL; /* ZMP does not exist (floating). */
  zVec3DOuterProd( &dz, rkChainRootTorque(chain), zmp );
  zVec3DCatDRC( zmp, z-rkChainRootPos(chain)->e[zZ], rkChainRootForce(chain) );
  zVec3DDivDRC( zmp, f );
  return zXform3DDRC( rkChainRootFrame(chain), zmp );
}

/* net torque around vertical axis exerted to a kinematic chain. */
double rkChainYawTorque(rkChain *chain)
{
  zVec3D dz;

  rkChainGravityDir( chain, &dz );
  return zVec3DInnerProd(rkChainRootTorque(chain),rkChainRootForce(chain)) / zVec3DInnerProd(rkChainRootTorque(chain),&dz);
}

/* angular momentum of a kinematic chain. */
zVec3D *rkChainAM(rkChain *chain, zVec3D *p, zVec3D *am)
{
  int i;
  zVec3D tp, tmp;

  zVec3DZero( am );
  for( i=0; i<rkChainLinkNum(chain); i++ ){
    zXform3DInv( rkChainLinkWldFrame(chain,i), p, &tp );
    rkLinkAM( rkChainLink(chain,i), &tp, &tmp );
    zMulMat3DVec3DDRC( rkChainLinkWldAtt(chain,i), &tmp );
    zVec3DAddDRC( am, &tmp );
  }
  return am;
}

/* kinetic energy of a kinematic chain. */
double rkChainKE(rkChain *chain)
{
  int i;
  double energy = 0;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    energy += rkLinkKE( rkChainLink(chain,i) );
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
    for( k=0; k<rkChainLinkJointDOF(chain,j); k++, i++ ){
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
  zMatSetSizeNC( hij, rkLinkJointDOF(link), rkLinkJointDOF(link) );
  for( i=0; i<rkLinkJointDOF(link); i++ )
    for( j=0; j<rkLinkJointDOF(link); j++ )
      zMatElemNC(hij,i,j) = zVec6DInnerProd( &si[i], &wi[j] );
  zMatPutNC( inertia, rkLinkJointIDOffset(link), rkLinkJointIDOffset(link), hij );
  /* non-diagonal block */
  for( lp=rkLinkParent(link); lp; lp=rkLinkParent(lp) ){
    _zFrame3DXform( rkLinkWldFrame(link), rkLinkWldFrame(lp), &f );
    rkJointCRBXform( rkLinkJoint(lp), &f, si );
    zMatSetSizeNC( hij, rkLinkJointDOF(lp), rkLinkJointDOF(link) );
    for( i=0; i<rkLinkJointDOF(lp); i++ )
      for( j=0; j<rkLinkJointDOF(link); j++ )
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
zVec6D *rkChainNetExtWrench(rkChain *chain, zVec6D *w)
{
  int i;
  zVec6D ew;

  zVec6DZero( w );
  for( i=0; i<rkChainLinkNum(chain); i++ ){
    rkLinkNetExtWrench( rkChainLink(chain,i), &ew );
    if( zVec6DEqual( &ew, ZVEC6DZERO ) ) continue;
    zMulMat3DVec6DDRC( rkChainLinkWldAtt(chain,i), &ew );
    zVec6DAngShiftDRC( &ew, rkChainLinkWldPos(chain,i) );
    zVec6DAddDRC( w, &ew );
  }
  return w;
}

/* destroy external wrench list attached to a kinematic chain. */
void rkChainExtWrenchDestroy(rkChain *chain)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkLinkExtWrenchDestroy( rkChainLink(chain,i) );
}

/* set joint identifier offset values of links of a kinematic chain. */
void rkChainSetJointIDOffset(rkChain *chain)
{
  int i, s;

  for( i=0, s=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointDOF(chain,i) > 0 ){
      rkLinkSetJointIDOffset( rkChainLink(chain,i), s );
      s += rkChainLinkJointDOF(chain,i);
    } else
      rkLinkSetJointIDOffset( rkChainLink(chain,i), -1 );
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
static void *_rkChainMotorSpecFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkMotorSpecFromZTK( zArrayElemNC(rkChainMotorSpecArray((rkChain*)obj),i), ztk );
}
static void *_rkChainLinkFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkLinkFromZTK( rkChainLink((rkChain*)obj,i),
    rkChainLinkArray((rkChain*)obj),
    rkChainShape((rkChain*)obj) ? &rkChainShape((rkChain*)obj)->shape : NULL,
    rkChainMotorSpecArray((rkChain*)obj), ztk ) ? obj : NULL;
}
static void *_rkChainLinkConnectFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkLinkConnectFromZTK( rkChainLink((rkChain*)obj,i),
    rkChainLinkArray((rkChain*)obj), ztk ) ? obj : NULL;
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
    if( rkLinkJointDOF(link) == 0 || rkJointIsNeutral( rkLinkJoint(link) ) ) continue;
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
  { ZTK_TAG_RKMOTOR, -1, _rkChainMotorSpecFromZTK, NULL },
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

  if( ( num_motor = ZTKCountTag( ztk, ZTK_TAG_RKMOTOR ) ) > 0 )
    if( !rkMotorSpecArrayAlloc( rkChainMotorSpecArray(chain), num_motor ) ) return NULL;
  if( ( num_link = ZTKCountTag( ztk, ZTK_TAG_RKLINK ) ) > 0 ){
    if( !rkLinkArrayAlloc( rkChainLinkArray(chain), num_link ) ) return NULL;
  } else{
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
  ZTKPrpTagFPrint( fp, chain, __ztk_prp_tag_rkchain );
  fprintf( fp, "\n" );
  if( rkChainShape(chain) )
    zMShape3DFPrintZTK( fp, rkChainShape(chain) );
  if( zArraySize(rkChainMotorSpecArray(chain)) > 0 )
    rkMotorSpecArrayFPrintZTK( fp, rkChainMotorSpecArray(chain) );
  rkLinkArrayFPrintZTK( fp, rkChainLinkArray(chain) );
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
void rkChainPostureFPrint(FILE *fp, rkChain *chain)
{
  int i;

  fprintf( fp, "Chain : %s\n", zName(chain) );
  for( i=0; i<rkChainLinkNum(chain); i++ )
    rkLinkPostureFPrint( fp, rkChainLink(chain,i) );
}

/* print connection of a kinematic chain out to a file. */
void rkChainConnectionFPrint(FILE *fp, rkChain *chain)
{
  fprintf( fp, "Chain : %s\n", zName(chain) );
  rkLinkConnectionFPrint( fp, rkChainRoot(chain), 0 );
}

/* print external wrench exerted to a kinematic chain out to a file. */
void rkChainExtWrenchFPrint(FILE *fp, rkChain *chain)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( zListSize(rkChainLinkExtWrench(chain,i)) != 0 ){
      fprintf( fp, "[%s]\n", rkChainLinkName(chain,i) );
      rkLinkExtWrenchFPrint( fp, rkChainLink(chain,i) );
    }
}
