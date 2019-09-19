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
  RK_MOTOR_COM_ARRAY;
  register int i;

  for( i=0; _rk_motor_com[i]; i++ )
    if( strcmp( str, _rk_motor_com[i]->typestr ) == 0 )
      return rkMotorAssign( m, _rk_motor_com[i] );
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

static void _rkMotorNameFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", zName( (rkMotor*)obj ) );
}
static void _rkMotorTypeFPrintZTK(FILE *fp, int i, void *obj){
  fprintf( fp, "%s\n", rkMotorTypeStr( (rkMotor*)obj ) );
}

static ZTKPrp __ztk_prp_rkmotor[] = {
  { "name", 1, _rkMotorNameFromZTK, _rkMotorNameFPrintZTK },
  { "type", 1, _rkMotorTypeFromZTK, _rkMotorTypeFPrintZTK },
};

bool rkMotorRegZTK(ZTK *ztk)
{
  RK_MOTOR_COM_ARRAY;
  register int i;

  if( !ZTKDefRegPrp( ztk, ZTK_TAG_RKMOTOR, __ztk_prp_rkmotor ) ) return false;
  for( i=0; _rk_motor_com[i]; i++ )
    if( !_rk_motor_com[i]->_regZTK( ztk, ZTK_TAG_RKMOTOR ) ) return false;
  return true;
}

rkMotor *rkMotorFromZTK(rkMotor *motor, ZTK *ztk)
{
  rkMotorInit( motor );
  if( !ZTKEvalKey( motor, NULL, ztk, __ztk_prp_rkmotor ) ) return NULL;
  motor->com->_fromZTK( motor->prp, ztk );
  return motor;
}

void rkMotorFPrintZTK(FILE *fp, rkMotor *motor)
{
  if( !motor ) return;
  ZTKPrpKeyFPrint( fp, motor, __ztk_prp_rkmotor );
  motor->com->_fprintZTK( fp, motor->prp );
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

void rkMotorArrayFPrintZTK(FILE *fp, rkMotorArray *m)
{
  register int i;

  for( i=0; i<zArraySize(m); i++ ){
    fprintf( fp, "[%s]\n", ZTK_TAG_RKMOTOR );
    rkMotorFPrintZTK( fp, zArrayElemNC(m,i) );
  }
}
