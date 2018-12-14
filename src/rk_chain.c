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

/* rkChainInit
 * - initialize a kinematic chain.
 */
void rkChainInit(rkChain *c)
{
  zNameSetPtr( c, NULL );
  zArrayInit( &c->link );
  rkChainSetShape( c, NULL );
  rkChainSetMotor( c, NULL );
  rkChainSetMass( c, 0 );
  rkChainSetWldCOM( c, ZVEC3DZERO );
  rkChainSetCOMVel( c, ZVEC3DZERO );
  rkChainSetCOMAcc( c, ZVEC3DZERO );
}

/* rkChainDestroy
 * - destroy a kinematic chain.
 */
void rkChainDestroy(rkChain *c)
{
  register int i;

  if( !c ) return;
  zNameDestroy(c);
  for( i=0; i<rkChainNum(c); i++ )
    rkLinkDestroy( rkChainLink(c,i) );
  zArrayFree( &c->link );
  zMShape3DDestroy( rkChainShape(c) );
  zFree( rkChainShape(c) );
  if( rkChainMotor(c) ){
    zArrayFree( rkChainMotor(c) );
    zFree( rkChainMotor(c) );
  }
  rkChainInit( c );
}

/* rkChainClone
 * - clone a kinematic chain.
 */
rkChain *rkChainClone(rkChain *org, rkChain *cln)
{
  char name[BUFSIZ];
  int i;

  rkChainInit( cln );
  sprintf( name, "%s_clone", zName(org) );
  if( !zNameSet( cln, name ) ||
      !( rkChainShape(cln) = zMShape3DClone( rkChainShape(org) ) ) ||
			!( rkChainMotor(cln) = rkMotorArrayClone( rkChainMotor(org) ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  zArrayAlloc( &cln->link, rkLink, rkChainNum(org) );
  if( rkChainNum(cln) != rkChainNum(org) ) return NULL;
  for( i=0; i<rkChainNum(cln); i++ )
    if( !rkLinkClone( rkChainLink(org,i), rkChainLink(cln,i), rkChainShape(org), rkChainShape(cln) ) )
      return NULL;
  rkChainSetMass( cln, rkChainMass(org) );
  rkChainCopyState( org, cln );
  return cln;
}

/* rkChainCopyState
 * - copy state of a kinematic chain.
 */
rkChain *rkChainCopyState(rkChain *src, rkChain *dst)
{
  register int i;

  for( i=0; i<rkChainNum(src); i++ )
    rkLinkCopyState( rkChainLink(src,i), rkChainLink(dst,i) );
  zVec3DCopy( rkChainWldCOM(src), rkChainWldCOM(dst) );
  zVec3DCopy( rkChainCOMVel(src), rkChainCOMVel(dst) );
  zVec3DCopy( rkChainCOMAcc(src), rkChainCOMAcc(dst) );
  return dst;
}

/* rkChainJointSize
 * - count the total number of joint of a kinematic chain.
 */
int rkChainJointSize(rkChain *c)
{
  register int i, size;

  for( size=0, i=0; i<rkChainNum(c); i++ )
    size += rkChainLinkJointSize(c,i);
  return size;
}

/* rkChainCreateDefaultJointIndex
 * - create default joint index of a kinematic chain.
 */
zIndex rkChainCreateDefaultJointIndex(rkChain *c)
{
  register int i, count;
  zIndex index;

  for( count=0, i=0; i<rkChainNum(c); i++ )
    if( rkChainLinkJointType(c,i) != RK_JOINT_FIXED ) count++;
  if( !( index = zIndexCreate( count ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  for( count=0, i=0; i<rkChainNum(c); i++ )
    if( rkChainLinkJointType(c,i) != RK_JOINT_FIXED ){
      zIndexSetElem( index, count, i );
      count++;
    }
  return index;
}

/* rkChainJointIndexSize
 * - count the total joint size indicated on a kinematic chain.
 */
int rkChainJointIndexSize(rkChain *c, zIndex idx)
{
  register int i, size;

  for( size=0, i=0; i<zArrayNum(idx); i++ )
    size += rkChainLinkJointSize(c,zIndexElem(idx,i));
  return size;
}

/* rkChainSetJointDis
 * - set joint displacement.
 */
void rkChainSetJointDis(rkChain *c, zIndex idx, zVec dis)
{
  register int i;
  double *dp;

  for( dp=zVecBuf(dis), i=0; i<zArrayNum(idx); i++ ){
    rkChainLinkSetJointDis( c, zIndexElem(idx,i), dp );
    dp += rkChainLinkJointSize(c,zIndexElem(idx,i));
  }
}

/* rkChainSetJointDisCNT
 * - continuously update joint displacement.
 */
void rkChainSetJointDisCNT(rkChain *c, zIndex idx, zVec dis, double dt)
{
  register int i;
  double *dp;

  for( dp=zVecBuf(dis), i=0; i<zArrayNum(idx); i++ ){
    rkChainLinkSetJointDisCNT( c, zIndexElem(idx,i), dp, dt );
    dp += rkChainLinkJointSize(c,zIndexElem(idx,i));
  }
}

/* rkChainSetJointVel
 * - set joint velocity.
 */
void rkChainSetJointVel(rkChain *c, zIndex idx, zVec vel)
{
  register int i;
  double *vp;

  for( vp=zVecBuf(vel), i=0; i<zArrayNum(idx); i++ ){
    rkJointSetVel( rkChainLinkJoint(c,zIndexElem(idx,i)), vp );
    vp += rkChainLinkJointSize(c,zIndexElem(idx,i));
  }
}

/* rkChainSetJointRate
 * - set joint velocity and acceleration.
 */
void rkChainSetJointRate(rkChain *c, zIndex idx, zVec vel, zVec acc)
{
  register int i;
  double *vp, *ap;

  for( vp=zVecBuf(vel), ap=zVecBuf(acc), i=0; i<zArrayNum(idx); i++ ){
    rkJointSetVel( rkChainLinkJoint(c,zIndexElem(idx,i)), vp );
    rkJointSetAcc( rkChainLinkJoint(c,zIndexElem(idx,i)), ap );
    vp += rkChainLinkJointSize(c,zIndexElem(idx,i));
    ap += rkChainLinkJointSize(c,zIndexElem(idx,i));
  }
}

/* rkChainGetJointDis
 * - get joint displacement.
 */
zVec rkChainGetJointDis(rkChain *c, zIndex idx, zVec dis)
{
  register int i;
  double *dp;

  for( dp=zVecBuf(dis), i=0; i<zArrayNum(idx); i++ ){
    rkChainLinkGetJointDis( c, zIndexElem(idx,i), dp );
    dp += rkChainLinkJointSize(c,zIndexElem(idx,i));
  }
  return dis;
}

/* rkChainGetJointVel
 * - get joint velocity.
 */
zVec rkChainGetJointVel(rkChain *c, zIndex idx, zVec vel)
{
  register int i;
  double *dp;

  for( dp=zVecBuf(vel), i=0; i<zArrayNum(idx); i++ ){
    rkChainLinkGetJointVel( c, zIndexElem(idx,i), dp );
    dp += rkChainLinkJointSize(c,zIndexElem(idx,i));
  }
  return vel;
}

/* rkChainGetJointAcc
 * - get joint acceleration.
 */
zVec rkChainGetJointAcc(rkChain *c, zIndex idx, zVec acc)
{
  register int i;
  double *dp;

  for( dp=zVecBuf(acc), i=0; i<zArrayNum(idx); i++ ){
    rkChainLinkGetJointAcc( c, zIndexElem(idx,i), dp );
    dp += rkChainLinkJointSize(c,zIndexElem(idx,i));
  }
  return acc;
}

/* rkChainSetJointDisAll
 * - set all joint displacements.
 */
void rkChainSetJointDisAll(rkChain *c, zVec dis)
{
  register int i;

  for( i=0; i<rkChainNum(c); i++ )
    if( rkChainLinkOffset(c,i) >= 0 )
      rkChainLinkSetJointDis( c, i, &zVecElem(dis,rkChainLinkOffset(c,i)) );
}

/* rkChainCatJointDisAll
 * - concatenate all joint displacement.
 */
void rkChainCatJointDisAll(rkChain *c, zVec dis, double k, zVec v)
{
  register int i;

  for( i=0; i<rkChainNum(c); i++ )
    if( rkChainLinkOffset(c,i) >= 0 )
      rkJointCatDis( rkChainLinkJoint(c,i), &zVecElem(dis,rkChainLinkOffset(c,i)), k, &zVecElem(v,rkChainLinkOffset(c,i)) );
}

/* rkChainSubJointDisAll
 * - subtract all joint displacement.
 */
void rkChainSubJointDisAll(rkChain *c, zVec dis, zVec sdis)
{
  register int i;

  for( i=0; i<rkChainNum(c); i++ )
    if( rkChainLinkOffset(c,i) >= 0 )
      rkJointSubDis( rkChainLinkJoint(c,i), &zVecElem(dis,rkChainLinkOffset(c,i)), &zVecElem(sdis,rkChainLinkOffset(c,i)) );
}

/* rkChainSetJointDisCNTAll
 * - continuously update all joint displacements.
 */
void rkChainSetJointDisCNTAll(rkChain *c, zVec dis, double dt)
{
  register int i;

  for( i=0; i<rkChainNum(c); i++ )
    if( rkChainLinkOffset(c,i) >= 0 )
      rkChainLinkSetJointDisCNT( c, i, &zVecElem(dis,rkChainLinkOffset(c,i)), dt );
}

/* rkChainSetJointVelAll
 * - set all joint velocities.
 */
void rkChainSetJointVelAll(rkChain *c, zVec vel)
{
  register int i;

  for( i=0; i<rkChainNum(c); i++ )
    if( rkChainLinkOffset(c,i) >= 0 )
      rkJointSetVel( rkChainLinkJoint(c,i), &zVecElem(vel,rkChainLinkOffset(c,i)) );
}

/* rkChainSetJointRateAll
 * - set all joint velocities and accelerations.
 */
void rkChainSetJointRateAll(rkChain *c, zVec vel, zVec acc)
{
  register int i;

  for( i=0; i<rkChainNum(c); i++ )
    if( rkChainLinkOffset(c,i) >= 0 ){
      rkJointSetVel( rkChainLinkJoint(c,i), &zVecElem(vel,rkChainLinkOffset(c,i)) );
      rkJointSetAcc( rkChainLinkJoint(c,i), &zVecElem(acc,rkChainLinkOffset(c,i)) );
    }
}

/* rkChainGetJointDisAll
 * - get all joint displacements.
 */
zVec rkChainGetJointDisAll(rkChain *c, zVec dis)
{
  register int i;

  for( i=0; i<rkChainNum(c); i++ )
    if( rkChainLinkOffset(c,i) >= 0 )
      rkChainLinkGetJointDis( c, i, &zVecElem(dis,rkChainLinkOffset(c,i)) );
  return dis;
}

/* rkChainGetJointVelAll
 * - get all joint velocities.
 */
zVec rkChainGetJointVelAll(rkChain *c, zVec vel)
{
  register int i;

  for( i=0; i<rkChainNum(c); i++ )
    if( rkChainLinkOffset(c,i) >= 0 )
      rkChainLinkGetJointVel( c, i, &zVecElem(vel,rkChainLinkOffset(c,i)) );
  return vel;
}

/* rkChainGetJointAccAll
 * - get all joint accelerations.
 */
zVec rkChainGetJointAccAll(rkChain *c, zVec acc)
{
  register int i;

  for( i=0; i<rkChainNum(c); i++ )
    if( rkChainLinkOffset(c,i) >= 0 )
      rkChainLinkGetJointAcc( c, i, &zVecElem(acc,rkChainLinkOffset(c,i)) );
  return acc;
}

/* rkChainGetJointTrqAll
 * - get all joint torque.
 */
zVec rkChainGetJointTrqAll(rkChain *c, zVec trq)
{
  register int i;

  for( i=0; i<rkChainNum(c); i++ )
    if( rkChainLinkOffset(c,i) >= 0 )
      rkChainLinkGetJointTrq( c, i, &zVecElem(trq,rkChainLinkOffset(c,i)) );
  return trq;
}

/* rkChainGetConf
 * - get all link configuration.
 */
zVec rkChainGetConf(rkChain *chain, zVec conf)
{
  register int i;

  for( i=0; i<rkChainNum(chain); i++ )
    zFrame3DToArrayAA( rkChainLinkWldFrame(chain,i), &zVecElem(conf,i*6) );
  return conf;
}

/* rkChainSetConf
 * - set all link configuration.
 */
void rkChainSetConf(rkChain *chain, zVec conf)
{
  register int i;

  for( i=0; i<rkChainNum(chain); i++ )
    zArrayToFrame3DAA( &zVecElem(conf,i*6), rkChainLinkWldFrame(chain,i) );
  rkLinkConfToJointDis( rkChainRoot(chain) );
}

/* rkChainGravityDir
 * - direction vector of gravity with respect to the body frame.
 */
zVec3D *rkChainGravityDir(rkChain *c, zVec3D *v)
{
  return zMat3DRow( rkChainRootAtt(c), 2, v );
}

/* rkChainUpdateFK, rkChainFK
 * - forward kinematics of a kinematic chain.
 */
void rkChainUpdateFK(rkChain *c)
{
  rkChainUpdateFrame( c );
  rkChainCalcCOM( c );
}

void rkChainFK(rkChain *c, zVec dis)
{
  rkChainSetJointDisAll( c, dis );
  rkChainUpdateFK( c );
}

/* rkChainUpdateID, rkChainID
 * - inverse dynamics of a kinematic chain.
 */
void rkChainUpdateID(rkChain *c)
{
  rkChainUpdateRate( c );
  rkChainUpdateWrench( c );
  rkChainCalcCOMVel( c );
  rkChainCalcCOMAcc( c );
}

void rkChainID(rkChain *c, zVec vel, zVec acc)
{
  rkChainSetJointRateAll( c, vel, acc );
  rkChainUpdateID( c );
}

/* rkChainFKCNT
 * - continuously update joint displacement over a time step.
 */
void rkChainFKCNT(rkChain *c, zVec dis, double dt)
{
  rkChainSetJointDisCNTAll( c, dis, dt );
  rkChainUpdateFK( c );
  rkChainUpdateID( c );
}

/* rkChainCalcCOM
 * - the center of mass of a kinematic chain with respect to the total/world frame.
 */
zVec3D *rkChainCalcCOM(rkChain *c)
{
  register int i;

  rkChainSetWldCOM( c, ZVEC3DZERO );
  for( i=0; i<rkChainNum(c); i++ )
    zVec3DCatDRC( rkChainWldCOM(c),
      rkChainLinkMass(c,i), rkChainLinkWldCOM(c,i) );
  return zVec3DDivDRC( rkChainWldCOM(c), rkChainMass(c) );
}

/* rkChainCalcCOMVel
 * - COM velocity of a kinematic chain with respect to the world frame.
 */
zVec3D *rkChainCalcCOMVel(rkChain *c)
{
  register int i;
  zVec3D v;

  rkChainSetCOMVel( c, ZVEC3DZERO );
  for( i=0; i<rkChainNum(c); i++ ){
    /* COM velocity of link is with respect to the local frame,
       while COM velocity of a kinematic chain is with respect to
       the world frame. */
    zMulMatVec3D( rkChainLinkWldAtt(c,i), rkChainLinkCOMVel(c,i), &v );
    zVec3DCatDRC( rkChainCOMVel(c), rkChainLinkMass(c,i)/rkChainMass(c), &v );
  }
  return rkChainCOMVel(c);
}

/* rkChainCalcCOMAcc
 * - COM acceleration of a kinematic chain with respect to the world frame.
 */
zVec3D *rkChainCalcCOMAcc(rkChain *c)
{
  register int i;
  zVec3D a;

  rkChainSetCOMAcc( c, ZVEC3DZERO );
  for( i=0; i<rkChainNum(c); i++ ){
    /* COM acceleration of a link is with respect to the local frame,
       while COM acceleration of a kinematic chain is with respect to
       the world frame. */
    zMulMatVec3D( rkChainLinkWldAtt(c,i), rkChainLinkCOMAcc(c,i), &a );
    zVec3DCatDRC( rkChainCOMAcc(c), rkChainLinkMass(c,i)/rkChainMass(c), &a );
  }
  return rkChainCOMAcc(c);
}

/* rkChainZMP
 * - Zero Moment Point of a kinematic chain.
 */
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
  return zXfer3DDRC( rkChainRootFrame(c), zmp );
}

/* rkChainYawTorque
 * - torque around vertical axis.
 */
double rkChainYawTorque(rkChain *c)
{
  zVec3D dz;

  rkChainGravityDir( c, &dz );
  return zVec3DInnerProd(rkChainRootTorque(c),rkChainRootForce(c)) / zVec3DInnerProd(rkChainRootTorque(c),&dz);
}

/* rkChainAM
 * - angular momentum of a kinematic chain.
 */
zVec3D *rkChainAM(rkChain *c, zVec3D *p, zVec3D *am)
{
  register int i;
  zVec3D tp, tmp;

  zVec3DClear( am );
  for( i=0; i<rkChainNum(c); i++ ){
    zXfer3DInv( rkChainLinkWldFrame(c,i), p, &tp );
    rkLinkAM( rkChainLink(c,i), &tp, &tmp );
    zMulMatVec3DDRC( rkChainLinkWldAtt(c,i), &tmp );
    zVec3DAddDRC( am, &tmp );
  }
  return am;
}

/* rkChainKE
 * - kinematic energy of a kinematic chain.
 */
double rkChainKE(rkChain *c)
{
  register int i;
  double energy = 0;

  for( i=0; i<rkChainNum(c); i++ )
    energy += rkLinkKE( rkChainLink(c,i) );
  return energy;
}

/* rkChainCalcExtWrench
 * - calculation of total external wrench applied to a kinematic chain.
 */
zVec6D *rkChainCalcExtWrench(rkChain *c, zVec6D *w)
{
  register int i;
  zVec6D ew;

  zVec6DClear( w );
  for( i=0; i<rkChainNum(c); i++ ){
    rkLinkCalcExtWrench( rkChainLink(c,i), &ew );
    if( zVec6DEqual( &ew, ZVEC6DZERO ) ) continue;
    zMulMatVec6DDRC( rkChainLinkWldAtt(c,i), &ew );
    zVec6DAngShiftDRC( &ew, rkChainLinkWldPos(c,i) );
    zVec6DAddDRC( w, &ew );
  }
  return w;
}

/* rkChainExtWrenchDestroy
 * - destroy external wrench list applied to a kinematic chain.
 */
void rkChainExtWrenchDestroy(rkChain *c)
{
  register int i;

  for( i=0; i<rkChainNum(c); i++ )
    rkLinkExtWrenchDestroy( rkChainLink(c,i) );
}

/* rkChainSetOffset
 * - set offset value of each link.
 */
void rkChainSetOffset(rkChain *c)
{
  register int i, s;

  for( i=0, s=0; i<rkChainNum(c); i++ )
    if( rkChainLinkJointSize(c,i) > 0 ){
      rkLinkSetOffset( rkChainLink(c,i), s );
      s += rkChainLinkJointSize(c,i);
    } else
      rkLinkSetOffset( rkChainLink(c,i), -1 );
}

/* rkChain2VertList
 * - make a list of vertices of a chain.
 */
zVec3DList *rkChain2VertList(rkChain *chain, zVec3DList *vl)
{
  zVec3D v;
  zShapeListCell *sc;
  rkLink *l;
  register int i, j;

  zListInit( vl );
  for( i=0; i<rkChainNum(chain); i++ ){
    l = rkChainLink(chain,i);
    zListForEach( rkLinkShapeList(l), sc ){
      for( j=0; j<zShape3DVertNum(sc->data); j++ ){
        zXfer3D( rkLinkWldFrame(l), zShape3DVert(sc->data,j), &v );
        if( !zVec3DListInsert( vl, &v ) ){
          zVec3DListDestroy( vl );
          return NULL;
        }
      }
    }
  }
  return vl;
}

static rkChain *_rkChainLinkFAlloc(FILE *fp, rkChain *c);
static bool __rkChainNameFRead(FILE *fp, void *instance, char *buf, bool *success);
static rkChain *_rkChainNameFRead(FILE *fp, rkChain *c);
static rkChain *_rkChainLinkFRead(FILE *fp, rkChain *c, int i);

/* rkChainReadFile
 * - generate a kinematic chain instance reading properties from a file.
 */
bool rkChainReadFile(rkChain *c, char filename[])
{
  FILE *fp;
  rkChain *result;

  if( !( fp = zOpenFile( filename, RK_CHAIN_SUFFIX, "r" ) ) )
    return false;
  result = rkChainFRead( fp, c );
  fclose( fp );
  return result != NULL;
}

typedef struct{
  rkChain *c;
  int lc;
} _rkChainParam;

/* (static)
 * _rkChainFRead
 * - generate a kinematic chain instance reading properties from a file.
 */
bool _rkChainFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  _rkChainParam *prm;

  prm = instance;
  if( strcmp( buf, "chain" ) == 0 ){
    if( !_rkChainNameFRead( fp, prm->c ) )
      return ( *success = false );
  } else if( strcmp( buf, "link" ) == 0 ){
    if( !_rkChainLinkFRead( fp, prm->c, prm->lc++ ) )
      return ( *success = false );
  } else if( strcmp( buf, "init" ) == 0 ){
    if( !rkChainInitFRead( fp, prm->c ) )
      return ( *success = false );
  }
  return true;
}

/* rkChainFRead
 * - generate a kinematic chain instance reading properties from a file.
 */
rkChain *rkChainFRead(FILE *fp, rkChain *c)
{
  _rkChainParam prm;

  rkChainInit( c );
  if( !_rkChainLinkFAlloc(fp,c) ) return NULL;
  if( !( rkChainShape(c) = zAlloc( zMShape3D, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  if( !zMShape3DFRead( fp, rkChainShape(c) ) ){
    ZRUNERROR( "invalid model file" );
    return NULL;
  }
  rewind(fp);
  if( !( rkChainMotor(c) = zAlloc( rkMotorArray, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  if( !rkMotorArrayFRead( fp, rkChainMotor(c) ) ){
    ZRUNERROR( "invalid model file" );
    return NULL;
  }
  rewind( fp );
  prm.c = c;
  prm.lc = 0;
  if( !zTagFRead( fp, _rkChainFRead, &prm ) ){
    rkChainDestroy( c );
    return NULL;
  }
  if( zIsTiny( rkChainMass(c) ) )
    rkChainSetMass( c, 1.0 ); /* dummy weight */
  rkChainSetOffset( c ); /* offset value arrangement */
  rkChainUpdateFK( c );
  rkChainUpdateID( c );
  return c;
}

/* rkChainInitReadFile
 * read an information about the initial state of a kinematic chain
 * from file.
 */
bool rkChainInitReadFile(rkChain *c, char filename[])
{
  FILE *fp;
  rkChain *result;

  if( !( fp = zOpenFile( filename, RK_CHAIN_INIT_SUFFIX, "r" ) ) )
    return false;
  result = rkChainInitFRead( fp, c );
  fclose( fp );
  return result != NULL;
}

/* (static)
 * _rkChainInitFRead
 * - set initial position of a kinematic chain.
 *   The initial position of root link is written in the file.
 */
bool _rkChainInitFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  rkChain *c;
  rkLink *l;

  c = instance;
  if( strcmp( buf, "pos" ) == 0 )
    zVec3DFRead( fp, rkChainOrgPos(c) );
  else if( strcmp( buf, "att" ) == 0 )
    zMat3DFRead( fp, rkChainOrgAtt(c) );
  else if( strcmp( buf, "frame" ) == 0 )
    zFrame3DFRead( fp, rkChainOrgFrame(c) );
  else{
    zNameFind( rkChainRoot(c), rkChainNum(c), buf, l );
    if( !l ) return false;
    rkJointQueryFRead( fp, "dis", rkLinkJoint(l), zArrayBuf(rkChainMotor(c)), zArrayNum(rkChainMotor(c)) );
  }
  return true;
}

/* rkChainInitFRead
 * - set initial position of a kinematic chain.
 *   The initial position of root link is written in the file.
 */
rkChain *rkChainInitFRead(FILE *fp, rkChain *c)
{
  zFieldFRead( fp, _rkChainInitFRead, c );
  rkChainUpdateFK( c );
  rkChainUpdateID( c );
  return c;
}

/* (static)
 * __rkChainNameFRead
 * - read name of a kinematic chain from a file and set properties.
 */
bool __rkChainNameFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  if( strcmp( buf, "name" ) == 0 ){
    if( strlen( zFToken( fp, buf, BUFSIZ ) ) >= BUFSIZ ){
      buf[BUFSIZ-1] = '\0';
      ZRUNWARN( "too long name, truncated to %s", buf );
    }
    zNameSet( (rkChain *)instance, buf );
  } else
    return false;
  return true;
}

/* (static)
 * _rkChainNameFRead
 * - read name of a kinematic chain from a file and set properties.
 */
rkChain *_rkChainNameFRead(FILE *fp, rkChain *c)
{
  zFieldFRead( fp, __rkChainNameFRead, c );
  return c;
}

/* (static)
 * _rkChainLinkFAlloc
 * prepare and initialize the whole links of a kinematic chain.
 * the information is given from a file.
 */
rkChain *_rkChainLinkFAlloc(FILE *fp, rkChain *c)
{
  register int i;
  int n;

  n = zFCountTag( fp, RK_LINK_TAG );
  zArrayAlloc( &c->link, rkLink, n );
  if( !rkChainRoot(c) ){
    ZALLOCERROR();
    return NULL;
  }
  for( i=0; i<n; i++ )
    rkLinkInit( rkChainLink(c,i) );
  return c;
}

/* (static)
 * _rkChainLinkFRead
 * - read properties of a link of a kinematic chain from a file.
 */
rkChain *_rkChainLinkFRead(FILE *fp, rkChain *c, int i)
{
  if( i >= rkChainNum(c) ){
    ZRUNERROR( "too many links specified" );
    return NULL;
  }
  if( !rkLinkFRead( fp, rkChainLink(c,i), rkChainRoot(c), rkChainNum(c),
        zMShape3DShapeBuf( rkChainShape(c) ),
        zMShape3DShapeNum( rkChainShape(c) ),
        zArrayBuf( rkChainMotor(c) ),
        zArrayNum( rkChainMotor(c) ) ) ){
    ZRUNERROR( "invalid description for the link" );
    return NULL;
  }
  if( !zNamePtr(rkChainLink(c,i)) ){
    ZRUNERROR( "unnamed link exists" );
    return NULL;
  }
  rkChainMass(c) += rkChainLinkMass(c,i);
  return c;
}

/* rkChainMShape3DReadFile
 * - read 3D multiple-shapes from a file and create a mono-link chain.
 */
bool rkChainMShape3DReadFile(rkChain *chain, char filename[])
{
  register int i;
  zMShape3D *ms;
  char basename[BUFSIZ];

  rkChainInit( chain );
  zGetBasename( filename, basename, BUFSIZ );
  zNameSet( chain, basename );
  if( !( ms = rkChainShape(chain) = zAlloc( zMShape3D, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  if( !zMShape3DReadFile( ms, filename ) ){
    ZRUNERROR( "invalid model file" );
    return false;
  }
  zArrayAlloc( &chain->link, rkLink, 1 );
  if( !rkChainRoot(chain) ){
    ZALLOCERROR();
    return false;
  }
  rkLinkInit( rkChainRoot(chain) );
  zNameSet( rkChainRoot(chain), "base" );
  rkJointCreate( rkLinkJoint(rkChainRoot(chain)), RK_JOINT_FIXED );
  for( i=0; i<zMShape3DShapeNum(ms); i++ ){
    rkLinkShapePush( rkChainRoot(chain), zMShape3DShape(ms,i) );
  }
  return true;
}

/* rkChainWriteFile
 * - output the information of a kinematic chain to a file.
 */
bool rkChainWriteFile(rkChain *c, char filename[])
{
  char name[BUFSIZ];
  FILE *fp;

  zAddSuffix( filename, RK_CHAIN_SUFFIX, name, BUFSIZ );
  if( !( fp = fopen( name, "w" ) ) ) return false;
  rkChainFWrite( fp, c );
	fclose(fp);
  return true;
}

/* rkChainFWrite
 * - output the information of a kinematic chain to a file.
 */
void rkChainFWrite(FILE *fp, rkChain *c)
{
  register int i;

  fprintf( fp, "[chain]\n" );
  if( zNamePtr(c) )
    fprintf( fp, "name : %s\n\n", zName(c) );
  if( rkChainShape(c) )
    zMShape3DFWrite( fp, rkChainShape(c) );
  if( rkChainMotor(c) )
    rkMotorArrayFWrite( fp, rkChainMotor(c) );

  for( i=0; i<rkChainNum(c); i++ ){
    fprintf( fp, "[link]\n" );
    rkLinkFWrite( fp, rkChainLink(c,i) );
  }
  if( rkChainRoot(c) ){
    fprintf( fp, "[init]\n" );
    rkChainInitFWrite( fp, c );
  }
}

bool rkChainInitWriteFile(rkChain *c, char filename[])
{
  char name[BUFSIZ];
  FILE *fp;

  zAddSuffix( filename, RK_CHAIN_INIT_SUFFIX, name, BUFSIZ );
  if( !( fp = fopen( name, "w" ) ) ){
    ZOPENERROR( name );
    return false;
  }
  rkChainInitFWrite( fp, c );
  fclose( fp );
  return true;
}

/* _rkChainInitFWrite
 * output the information about initial posture of a kinematic chain to a file.
 */
void rkChainInitFWrite(FILE *fp, rkChain *c)
{
  register int i;

  fprintf( fp, "frame : " );
  zFrame3DFWrite( fp, rkChainOrgFrame(c) );
  for( i=0; i<rkChainNum(c); i++ )
    if( !rkJointIsNeutral( rkChainLinkJoint(c,i) ) )
      rkJointFWrite( fp, rkChainLinkJoint(c,i), rkChainLinkName(c,i) );
}

/* rkChainPostureFWrite
 * show the current posture of a kinematic chain to a file.
 */
void rkChainPostureFWrite(FILE *fp, rkChain *c)
{
  register int i;

  fprintf( fp, "Chain : %s\n", zName(c) );
  for( i=0; i<rkChainNum(c); i++ )
    rkLinkPostureFWrite( fp, rkChainLink(c,i) );
}

/* rkChainConnectionFWrite
 * show the connection of a kinematic chain to a file.
 */
void rkChainConnectionFWrite(FILE *fp, rkChain *c)
{
  fprintf( fp, "Chain : %s\n", zName(c) );
  rkLinkConnectionFWrite( fp, rkChainRoot(c), 0 );
}

void rkChainExtWrenchFWrite(FILE *fp, rkChain *c)
{
  register int i;

  for( i=0; i<rkChainNum(c); i++ )
    if( zListNum(rkChainLinkExtWrench(c,i)) != 0 ){
      fprintf( fp, "[%s]\n", rkChainLinkName(c,i) );
      rkLinkExtWrenchFWrite( fp, rkChainLink(c,i) );
    }
}
