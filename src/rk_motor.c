/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor - motor model
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

/* ********************************************************** */
/* CLASS: rkMotor
 * geared DC motor model class
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

void rkMotorFPrint(FILE *fp, rkMotor *m)
{
  fprintf( fp, "name: %s\n", zName(m) );
  fprintf( fp, "type: %s\n", rkMotorTypeStr(m) );
  (m)->com->_fprint( fp, (m)->prp );
}

/* ********************************************************** */
/* CLASS: rkMotorArray
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

rkMotorArray *_rkMotorArrayFAlloc(FILE *fp, rkMotorArray *m)
{
  register int i;
  int n;

  n = zFCountTag( fp, RK_MOTOR_TAG );
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
  if( strcmp( buf, RK_MOTOR_TAG ) == 0 ){
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
    fprintf( fp, "[%s]\n", RK_MOTOR_TAG );
    rkMotorFPrint( fp, zArrayElemNC(m,i) );
  }
}
