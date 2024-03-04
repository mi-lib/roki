/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_motor - motor model
 * contributer: 2014-2015 Naoki Wakisaka
 */

#include <roki/rk_motor.h>

/* ********************************************************** */
/* motor specification class
 * ********************************************************** */

/* assign a motor type to motor specification instance. */
rkMotorSpec *rkMotorSpecAssign(rkMotorSpec *ms, rkMotorCom *com)
{
  if( ( ms->prp = ( ms->com = com )->_alloc_prp() ) )
    ms->com->_init_prp( ms );
  return ms;
}

/* query motor specification by a string. */
rkMotorSpec *rkMotorSpecQuery(rkMotorSpec *ms, const char *str)
{
  RK_MOTOR_COM_ARRAY;
  int i;

  for( i=0; _rk_motor_com[i]; i++ )
    if( strcmp( str, _rk_motor_com[i]->typestr ) == 0 )
      return rkMotorSpecAssign( ms, _rk_motor_com[i] );
  ZRUNERROR( RK_ERR_MOTOR_UNKNOWNTYPE, str );
  return NULL;
}

/* destroy a motor specification instance. */
void rkMotorSpecDestroy(rkMotorSpec *ms)
{
  zNameFree( ms );
  zFree( ms->prp );
  rkMotorSpecInit( ms );
}

/* clone a motor specification instance. */
rkMotorSpec *rkMotorSpecClone(rkMotorSpec *org, rkMotorSpec *cln)
{
  rkMotorSpecAssign( cln, org->com );
  zNameSet( cln, zName(org) );
  org->com->_copy_prp( org, cln );
  return cln;
}

/* ZTK */

static void *_rkMotorSpecNameFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  zNameSet( (rkMotorSpec*)obj, ZTKVal(ztk) );
  return zNamePtr((rkMotorSpec*)obj) ? obj : NULL;
}
static void *_rkMotorSpecTypeFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  return rkMotorSpecQuery( (rkMotorSpec *)obj, ZTKVal(ztk) );
}

static void _rkMotorSpecNameFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName( (rkMotorSpec*)obj ) );
}
static void _rkMotorSpecTypeFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", rkMotorSpecTypeStr( (rkMotorSpec*)obj ) );
}

static ZTKPrp __ztk_prp_rkmotorspec[] = {
  { "name", 1, _rkMotorSpecNameFromZTK, _rkMotorSpecNameFPrintZTK },
  { "type", 1, _rkMotorSpecTypeFromZTK, _rkMotorSpecTypeFPrintZTK },
};

rkMotorSpec *rkMotorSpecFromZTK(rkMotorSpec *ms, ZTK *ztk)
{
  rkMotorSpecInit( ms );
  if( !ZTKEvalKey( ms, NULL, ztk, __ztk_prp_rkmotorspec ) ) return NULL;
  ms->com->_fromZTK( ms, ztk );
  return ms;
}

void rkMotorSpecFPrintZTK(FILE *fp, rkMotorSpec *ms)
{
  if( !ms ) return;
  ZTKPrpKeyFPrint( fp, ms, __ztk_prp_rkmotorspec );
  ms->com->_fprintZTK( fp, ms );
  fprintf( fp, "\n" );
}

/* ********************************************************** */
/* array of motor specifications
 * ********************************************************** */

rkMotorSpecArray *rkMotorSpecArrayAlloc(rkMotorSpecArray *msarray, int size)
{
  zArrayAlloc( msarray, rkMotorSpec, size );
  return zArraySize(msarray) == size ? msarray : NULL;
}

void rkMotorSpecArrayDestroy(rkMotorSpecArray *msarray)
{
  int i;

  for( i=0; i<zArraySize(msarray); i++ )
    rkMotorSpecDestroy( zArrayElem(msarray,i) );
  zArrayFree( msarray );
}

rkMotorSpecArray *rkMotorSpecArrayClone(rkMotorSpecArray *org, rkMotorSpecArray *cln)
{
  int i;

  if( zArraySize(org) > 0 ){
    zArrayAlloc( cln, rkMotorSpec, zArraySize(org) );
    if( zArraySize(cln) != zArraySize(org) ) return NULL;
    for( i=0; i<zArraySize(cln); i++ ){
      if( !rkMotorSpecClone( zArrayElemNC(org,i), zArrayElemNC(cln,i) ) )
        return NULL;
    }
  } else
    zArrayInit( cln );
  return cln;
}

rkMotorSpec *rkMotorSpecArrayFind(rkMotorSpecArray *msarray, const char *name)
{
  rkMotorSpec *ms;

  zArrayFindName( msarray, name, ms );
  if( !ms ){
    ZRUNERROR( RK_ERR_MOTOR_UNKNOWN, name );
    return NULL;
  }
  return ms;
}

void rkMotorSpecArrayFPrintZTK(FILE *fp, rkMotorSpecArray *msarray)
{
  int i;

  for( i=0; i<zArraySize(msarray); i++ ){
    fprintf( fp, "[%s]\n", ZTK_TAG_RKMOTOR );
    rkMotorSpecFPrintZTK( fp, zArrayElemNC(msarray,i) );
  }
}

/* ********************************************************** */
/* motor class
 * ********************************************************** */

rkMotor *rkMotorCreate(rkMotor *motor, rkMotorSpec *ms)
{
  if( !( motor->state = ms->com->_alloc_state() ) ){
    ZALLOCERROR();
    return NULL;
  }
  motor->spec = ms;
  ms->com->_init_state( motor );
  return motor;
}

void rkMotorDestroy(rkMotor *motor)
{
  motor->spec = NULL;
  zFree( motor->state );
}

rkMotor *rkMotorClone(rkMotor *org, rkMotor *cln, rkMotorSpecArray *msarray_org, rkMotorSpecArray *msarray_cln)
{
  if( org->spec ){
    cln->spec = org->spec - zArrayBuf(msarray_org) + zArrayBuf(msarray_cln);
    if( !rkMotorCreate( cln, cln->spec ) ) return NULL;
    cln->spec->com->_copy_state( org, cln );
  } else
    rkMotorInit( cln );
  return cln;
}
