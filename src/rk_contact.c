/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_contact - physical properties of contact
 */

#include <roki/rk_contact.h>

/* ********************************************************** */
/* CLASS: rkContactInfo
 * contact model class
 * ********************************************************** */

/* create a rigid contact model. */
rkContactInfo *rkContactInfoRigidCreate(rkContactInfo *ci, double k, double l, double sf, double kf, const char *stf1, const char *stf2)
{
  rkContactInfoSetType( ci, RK_CONTACT_RIGID );
  rkContactInfoSetK( ci, k );
  rkContactInfoSetL( ci, l );
  rkContactInfoSetSF( ci, sf );
  rkContactInfoSetKF( ci, kf );
  ci->__stf[0] = zStrClone( stf1 );
  ci->__stf[1] = zStrClone( stf2 );
  return ci;
}

/* create an elastic contact model. */
rkContactInfo *rkContactInfoElasticCreate(rkContactInfo *ci, double e, double v, double sf, double kf, const char *stf1, const char *stf2)
{
  rkContactInfoSetType( ci, RK_CONTACT_ELASTIC );
  rkContactInfoSetE( ci, e );
  rkContactInfoSetV( ci, v );
  rkContactInfoSetSF( ci, sf );
  rkContactInfoSetKF( ci, kf );
  ci->__stf[0] = zStrClone( stf1 );
  ci->__stf[1] = zStrClone( stf2 );
  return ci;
}

/* associate contact info with a pair of stuff. */
rkContactInfo *rkContactInfoAssoc(rkContactInfo *ci, const char *stf1, const char *stf2)
{
  if( ( strcmp( ci->__stf[0], stf1 ) == 0 && strcmp( ci->__stf[1], stf2 ) == 0 ) ||
      ( strcmp( ci->__stf[0], stf2 ) == 0 && strcmp( ci->__stf[1], stf1 ) == 0 ) )
    return ci;
  return NULL;
}

/* ZTK */

static void *_rkContactInfoBindFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  const char *stf0, *stf1;
  stf0 = ZTKVal(ztk);
  ZTKValNext( ztk );
  stf1 = ZTKVal(ztk);
  if( !( ((rkContactInfo*)obj)->__stf[0] = zStrClone( stf0 ) ) ||
      !( ((rkContactInfo*)obj)->__stf[1] = zStrClone( stf1 ) ) ) return NULL;
  if( rkContactInfoArrayAssoc( (rkContactInfoArray*)arg, stf0, stf1 ) != obj )
    ZRUNWARN( RK_WARN_CONTACT_DUPLICATE_KEY, stf0, stf1 );
  return obj;
}
static void *_rkContactInfoStaticFricFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkContactInfoSetSF( (rkContactInfo*)obj, ZTKDouble(ztk) );
  return obj;
}
static void *_rkContactInfoKineticFricFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkContactInfoSetKF( (rkContactInfo*)obj, ZTKDouble(ztk) );
  return obj;
}
static void *_rkContactInfoCompensationFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkContactInfoSetK( (rkContactInfo*)obj, ZTKDouble(ztk) );
  rkContactInfoSetType( (rkContactInfo*)obj, RK_CONTACT_RIGID );
  return obj;
}
static void *_rkContactInfoRelaxationFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkContactInfoSetL( (rkContactInfo*)obj, ZTKDouble(ztk) );
  rkContactInfoSetType( (rkContactInfo*)obj, RK_CONTACT_RIGID );
  return obj;
}
static void *_rkContactInfoElasticityFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkContactInfoSetE( (rkContactInfo*)obj, ZTKDouble(ztk) );
  rkContactInfoSetType( (rkContactInfo*)obj, RK_CONTACT_ELASTIC );
  return obj;
}
static void *_rkContactInfoViscosityFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkContactInfoSetV( (rkContactInfo*)obj, ZTKDouble(ztk) );
  rkContactInfoSetType( (rkContactInfo*)obj, RK_CONTACT_ELASTIC );
  return obj;
}

static const ZTKPrp __ztk_prp_rkcontactinfo[] = {
  { ZTK_KEY_ROKI_CONTACT_BIND,            1, _rkContactInfoBindFromZTK, NULL },
  { ZTK_KEY_ROKI_CONTACT_STATICFRICTION,  1, _rkContactInfoStaticFricFromZTK, NULL },
  { ZTK_KEY_ROKI_CONTACT_KINETICFRICTION, 1, _rkContactInfoKineticFricFromZTK, NULL },
  { ZTK_KEY_ROKI_CONTACT_COMPENSATION,    1, _rkContactInfoCompensationFromZTK, NULL },
  { ZTK_KEY_ROKI_CONTACT_RELAXATION,      1, _rkContactInfoRelaxationFromZTK, NULL },
  { ZTK_KEY_ROKI_CONTACT_ELASTICITY,      1, _rkContactInfoElasticityFromZTK, NULL },
  { ZTK_KEY_ROKI_CONTACT_VISCOSITY,       1, _rkContactInfoViscosityFromZTK, NULL },
};

static void *_rkContactInfoFromZTK(void *obj, int i, void *arg, ZTK *ztk)
{
  if( !ZTKEvalKey( zArrayElem((rkContactInfoArray*)obj,i), arg, ztk, __ztk_prp_rkcontactinfo ) ) return NULL;
  return obj;
}

/* print information of a contact model. */
void rkContactInfoFPrintZTK(FILE *fp, rkContactInfo *ci)
{
  fprintf( fp, "%s: %s %s\n", ZTK_KEY_ROKI_CONTACT_BIND, ci->__stf[0], ci->__stf[1] );
  switch( rkContactInfoType(ci) ){
  case RK_CONTACT_RIGID:
    fprintf( fp, "%s: %.10g\n", ZTK_KEY_ROKI_CONTACT_COMPENSATION, rkContactInfoK(ci) );
    fprintf( fp, "%s: %.10g\n", ZTK_KEY_ROKI_CONTACT_RELAXATION, rkContactInfoL(ci) );
    break;
  case RK_CONTACT_ELASTIC:
    fprintf( fp, "%s: %.10g\n", ZTK_KEY_ROKI_CONTACT_ELASTICITY, rkContactInfoE(ci) );
    fprintf( fp, "%s: %.10g\n", ZTK_KEY_ROKI_CONTACT_VISCOSITY, rkContactInfoV(ci) );
    break;
  default: ;
  }
  fprintf( fp, "%s: %.10g\n", ZTK_KEY_ROKI_CONTACT_STATICFRICTION, rkContactInfoSF(ci) );
  fprintf( fp, "%s: %.10g\n", ZTK_KEY_ROKI_CONTACT_KINETICFRICTION, rkContactInfoKF(ci) );
  fprintf( fp, "\n" );
}

/* ********************************************************** */
/* CLASS: rkContactInfoArray
 * contact model array class
 * ********************************************************** */

/* destroy contact information array */
void rkContactInfoArrayDestroy(rkContactInfoArray *carray)
{
  int i;

  if( !carray ) return;
  for( i=0; i<zArraySize(carray); i++ )
    rkContactInfoDestroy( zArrayElemNC(carray,i) );
  zArrayFree( carray );
}

/* associate contact information with a pair of keys. */
rkContactInfo *rkContactInfoArrayAssoc(rkContactInfoArray *carray, const char *stf1, const char *stf2)
{
  int i;

  if( !stf1 || !stf2 ) return NULL;
  for( i=0; i<zArraySize(carray); i++ )
    if( rkContactInfoAssoc( zArrayElemNC(carray,i), stf1, stf2 ) )
      return zArrayElemNC(carray,i);
  return NULL;
}

/* associate contact information that matches specified type with a pair of keys. */
rkContactInfo *rkContactInfoArrayAssocType(rkContactInfoArray *carray, const char *stf1, const char *stf2, const char type)
{
  int i;

  if( !stf1 || !stf2) return NULL;
  for( i=0; i<zArraySize(carray); i++ )
    if( rkContactInfoType( zArrayElemNC(carray,i) ) == type && rkContactInfoAssoc( zArrayElemNC(carray,i), stf1, stf2 ) )
      return zArrayElemNC(carray,i);
  return NULL;
}

/* ZTK */

static const ZTKPrp __ztk_prp_tag_roki_contactinfo[] = {
  { ZTK_TAG_ROKI_CONTACTINFO, -1, _rkContactInfoFromZTK, NULL },
};

/* scan contact information array from a ZTK format processor. */
rkContactInfoArray *rkContactInfoArrayFromZTK(rkContactInfoArray *carray, ZTK *ztk)
{
  int n;

  zArrayInit( carray );
  if( ( n = ZTKCountTag( ztk, ZTK_TAG_ROKI_CONTACTINFO ) ) == 0 ){
    ZRUNWARN( RK_WARN_CONTACT_EMPTY );
    return NULL;
  }
  zArrayAlloc( carray, rkContactInfo, n );
  ZTKEvalTag( carray, carray, ztk, __ztk_prp_tag_roki_contactinfo );
  return carray;
}

/* print contact information array out to the current position of a ZTK format file. */
void rkContactInfoArrayFPrintZTK(FILE *fp, rkContactInfoArray *carray)
{
  int i;

  for( i=0; i<zArraySize(carray); i++ ){
    fprintf( fp, "[%s]\n", ZTK_TAG_ROKI_CONTACTINFO );
    rkContactInfoFPrintZTK( fp, zArrayElemNC(carray,i) );
  }
}

/* read contact information array from a ZTK file. */
rkContactInfoArray *rkContactInfoArrayReadZTK(rkContactInfoArray *carray, const char filename[])
{
  ZTK ztk;

  ZTKInit( &ztk );
  ZTKParse( &ztk, filename );
  carray = rkContactInfoArrayFromZTK( carray, &ztk );
  ZTKDestroy( &ztk );
  return carray;
}

/* write contact information array to a ZTK file. */
bool rkContactInfoArrayWriteZTK(rkContactInfoArray *carray, const char filename[])
{
  FILE *fp;

  if( !( fp = zOpenZTKFile( filename, "w" ) ) ) return false;
  rkContactInfoArrayFPrintZTK( fp, carray );
  return true;
}
