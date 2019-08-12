/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor - motor model
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

/* ********************************************************** */
/* a class of motor model
 * ********************************************************** */

rkMotor *rkMotorAssign(rkMotor *m, rkMotorCom *com)
{
  if( ( m->prp = ( m->com = com )->_alloc() ) )
    m->com->_init( m->prp );
  return m;
}

rkMotor *rkMotorQueryAssign(rkMotor *m, char *str)
{
  rkMotorCom *com[] = {
    &rk_motor_none,
    &rk_motor_dc,
    &rk_motor_trq,
    NULL,
  };
  register int i;

  for( i=0; com[i]; i++ )
    if( strcmp( str, com[i]->typestr ) == 0 )
      return rkMotorAssign( m, com[i] );
  ZRUNERROR( RK_ERR_MOTOR_UNKNOWNTYPE, str );
  return NULL;
}

void rkMotorDestroy(rkMotor *m)
{
  zNameFree( m );
  zFree( m->prp );
  rkMotorInit( m );
}

rkMotor *rkMotorClone(rkMotor *org, rkMotor *cln)
{
  rkMotorAssign( cln, org->com );
  zNameSet( cln, zName(org) );
  org->com->_copy( org->prp, cln->prp );
  return cln;
}

static void *_rkMotorNameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zNameSet( (rkMotor*)obj, ZTKVal(ztk) );
  return zNamePtr((rkMotor*)obj) ? obj : NULL;
}
static void *_rkMotorTypeFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkMotorQueryAssign( obj, ZTKVal(ztk) );
}

static void _rkMotorNameFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName( (rkMotor*)obj ) );
}
static void _rkMotorTypeFPrint(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", rkMotorTypeStr( (rkMotor*)obj ) );
}

static ZTKPrp __ztk_prp_rkmotor[] = {
  { "name", 1, _rkMotorNameFromZTK, _rkMotorNameFPrint },
  { "type", 1, _rkMotorTypeFromZTK, _rkMotorTypeFPrint },
};

bool rkMotorRegZTK(ZTK *ztk)
{
  return ZTKDefRegPrp( ztk, ZTK_TAG_RKMOTOR, __ztk_prp_rkmotor ) &&
         rkMotorRegZTKTrq( ztk, ZTK_TAG_RKMOTOR ) &&
         rkMotorRegZTKDC( ztk, ZTK_TAG_RKMOTOR );
}

rkMotor *rkMotorFromZTK(rkMotor *motor, ZTK *ztk)
{
  rkMotorInit( motor );
  if( !ZTKEncodeKey( motor, NULL, ztk, __ztk_prp_rkmotor ) ) return NULL;
  motor->com->_fromZTK( motor->prp, ztk );
  return motor;
}

void rkMotorFPrint(FILE *fp, rkMotor *motor)
{
  if( !motor ) return;
  ZTKPrpKeyFPrint( fp, motor, __ztk_prp_rkmotor );
  motor->com->_fprint( fp, motor->prp );
  fprintf( fp, "\n" );
}

/* ********************************************************** */
/* array of motors
 * ********************************************************** */

rkMotorArray *rkMotorArrayClone(rkMotorArray *org)
{
  rkMotorArray *cln;
  register int i;

  if( !( cln = zAlloc( rkMotorArray, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  zArrayAlloc( cln, rkMotor, zArraySize(org) );
  if( zArraySize(cln) != zArraySize(org) ) return NULL;
  for( i=0; i<zArraySize(cln); i++ ){
    if( !rkMotorClone( zArrayElemNC(org,i), zArrayElemNC(cln,i) ) )
      return NULL;
  }
  return cln;
}

rkMotor *rkMotorArrayFind(rkMotorArray *marray, char *name)
{
  rkMotor *mp;
  zArrayFindName( marray, name, mp );
  if( !mp ){
    ZRUNERROR( RK_ERR_MOTOR_UNKNOWN, name );
    return NULL;
  }
  return mp;
}

rkMotorArray *_rkMotorArrayFAlloc(FILE *fp, rkMotorArray *m)
{
  register int i;
  int n;

  n = zFCountTag( fp, ZTK_TAG_RKMOTOR );
  zArrayAlloc( m, rkMotor, n );
  if( n > 0 && !zArrayBuf(m) ) return NULL;
  for( i=0; i<n; i++ )
    rkMotorInit( zArrayElemNC(m,i) );
  return m;
}

typedef struct{
  rkMotor *m;
  rkMotorArray *marray;
} _rkMotorArrayMotorParam;

bool __rkMotorArrayMotorFScan(FILE *fp, void *instance, char *buf, bool *success)
{
  _rkMotorArrayMotorParam *prm;
  rkMotor *cm;

  prm = instance;
  if( strcmp( buf, "name" ) == 0 ){
    if( strlen( zFToken( fp, buf, BUFSIZ ) ) >= BUFSIZ ){
      buf[BUFSIZ-1] = '\0';
      ZRUNWARN( RK_WARN_TOOLNG_NAME, buf );
    }
    zNameFind( zArrayBuf(prm->marray), zArraySize(prm->marray), buf, cm );
    if( cm ){
      ZRUNERROR( RK_WARN_TWOFOLDNAME, buf );
      return false;
    }
    if( !( zNameSet( prm->m, buf ) ) ){
      ZALLOCERROR();
      return ( *success = false );
    }
  } else if( strcmp( buf, "type" ) == 0 ){
    if( !rkMotorQueryAssign( prm->m, zFToken( fp, buf, BUFSIZ ) ) )
      return ( *success = false );
  } else if( !rkMotorQueryFScan( fp, buf, prm->m ) )
    return false;
  return true;
}

rkMotorArray *_rkMotorArrayMotorFScan(FILE *fp, rkMotorArray *marray, int i)
{
  _rkMotorArrayMotorParam prm;

  if( i >= zArraySize(marray) ){
    ZRUNERROR( RK_ERR_MOTOR_OUTOFRANGE, i, zArraySize(marray) );
    return NULL;
  }
  prm.m = zArrayElemNC(marray,i);
  prm.marray = marray;
  if( !zFieldFScan( fp, __rkMotorArrayMotorFScan, &prm ) ) return NULL;
  if( !zNamePtr( prm.m ) ){
    ZRUNERROR( RK_ERR_MOTOR_UNNAMED );
    return NULL;
  }
  return marray;
}

typedef struct{
  rkMotorArray *m;
  int nm;
} _rkMotorArrayParam;

bool _rkMotorArrayFScan(FILE *fp, void *instance, char *buf, bool *success)
{
  _rkMotorArrayParam *prm;

  prm = instance;
  if( strcmp( buf, ZTK_TAG_RKMOTOR ) == 0 ){
    if( !_rkMotorArrayMotorFScan( fp, prm->m, prm->nm++ ) )
      return ( *success = false );
  }
  return true;
}

rkMotorArray *rkMotorArrayFScan(FILE *fp, rkMotorArray *m)
{
  _rkMotorArrayParam prm;

  prm.m = m;
  prm.nm = 0;
  zArrayInit( m );
  if( !_rkMotorArrayFAlloc(fp,m) ) return NULL;
  if( !zTagFScan( fp, _rkMotorArrayFScan, &prm ) ){
    zArrayFree( m );
    return NULL;
  }
  return m;
}

void rkMotorArrayFPrint(FILE *fp, rkMotorArray *m)
{
  register int i;

  for( i=0; i<zArraySize(m); i++ ){
    fprintf( fp, "[%s]\n", ZTK_TAG_RKMOTOR );
    rkMotorFPrint( fp, zArrayElemNC(m,i) );
  }
}
