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

static bool _rkContactInfoFScan(FILE *fp, void *instance, char *buf, bool *success);

/* create a rigid contact model. */
rkContactInfo *rkContactInfoRigidCreate(rkContactInfo *ci, double k, double l, double sf, double kf, char *stf1, char *stf2)
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
rkContactInfo *rkContactInfoElasticCreate(rkContactInfo *ci, double e, double v, double sf, double kf, char *stf1, char *stf2)
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
rkContactInfo *rkContactInfoAssoc(rkContactInfo *ci, char *stf1, char *stf2)
{
  if( ( strcmp( ci->__stf[0], stf1 ) == 0 && strcmp( ci->__stf[1], stf2 ) == 0 ) ||
      ( strcmp( ci->__stf[0], stf2 ) == 0 && strcmp( ci->__stf[1], stf1 ) == 0 ) )
    return ci;
  return NULL;
}

/* scan information of a contact model (internal operation). */
bool _rkContactInfoFScan(FILE *fp, void *instance, char *buf, bool *success)
{
  rkContactInfo *ci;

  ci = instance;
  if( strcmp( buf, "bind" ) == 0 ){
    ci->__stf[0] = zStrClone( zFToken( fp, buf, BUFSIZ ) );
    ci->__stf[1] = zStrClone( zFToken( fp, buf, BUFSIZ ) );
  } else if( strcmp( buf, "staticfriction" ) == 0 ){
    rkContactInfoSetSF( ci, zFDouble( fp ) );
  } else if( strcmp( buf, "kineticfriction" ) == 0 ){
    rkContactInfoSetKF( ci, zFDouble( fp ) );
  } else if( strcmp( buf, "compensation" ) == 0 ){
    rkContactInfoSetK( ci, zFDouble( fp ) );
    rkContactInfoSetType( ci, RK_CONTACT_RIGID );
  } else if( strcmp( buf, "relaxation" ) == 0 ){
    rkContactInfoSetL( ci, zFDouble( fp ) );
    rkContactInfoSetType( ci, RK_CONTACT_RIGID );
  } else if( strcmp( buf, "elasticity" ) == 0 ){
    rkContactInfoSetE( ci, zFDouble( fp ) );
    rkContactInfoSetType( ci, RK_CONTACT_ELASTIC );
  } else if( strcmp( buf, "viscosity" ) == 0 ){
    rkContactInfoSetV( ci, zFDouble( fp ) );
    rkContactInfoSetType( ci, RK_CONTACT_ELASTIC );
  } else
    return false;
  return true;
}

/* scan information of a contact model. */
rkContactInfo *rkContactInfoFScan(FILE *fp, rkContactInfo *ci)
{
  rkContactInfoInit( ci );
  zFieldFScan( fp, _rkContactInfoFScan, ci );
  if( ci->__stf[0] && ci->__stf[1] ) return ci;
  ZRUNERROR( RK_ERR_CONTACT_UNBOUND );
  return NULL;
}

/* print information of a contact model. */
void rkContactInfoFPrint(FILE *fp, rkContactInfo *ci)
{
  fprintf( fp, "bind: %s %s\n", ci->__stf[0], ci->__stf[1] );
  switch( rkContactInfoType(ci) ){
  case RK_CONTACT_RIGID:
    fprintf( fp, "compensation: %.10g\n", rkContactInfoK(ci) );
    fprintf( fp, "relaxation: %.10g\n", rkContactInfoL(ci) );
    break;
  case RK_CONTACT_ELASTIC:
    fprintf( fp, "elasticity: %.10g\n", rkContactInfoE(ci) );
    fprintf( fp, "viscosity: %.10g\n", rkContactInfoV(ci) );
    break;
  default: ;
  }
  fprintf( fp, "staticfriction: %.10g\n", rkContactInfoSF(ci) );
  fprintf( fp, "kineticfriction: %.10g\n", rkContactInfoKF(ci) );
  fprintf( fp, "\n" );
}

/* ZTK */

static void *_rkContactInfoBindFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  char *stf0, *stf1;
  stf0 = ZTKVal(ztk);
  ZTKValNext( ztk );
  stf1 = ZTKVal(ztk);
  if( !( ((rkContactInfo*)obj)->__stf[0] = zStrClone( stf0 ) ) ||
      !( ((rkContactInfo*)obj)->__stf[1] = zStrClone( stf1 ) ) ) return NULL;
  if( rkContactInfoArrayAssoc( (rkContactInfoArray*)arg, stf0, stf1 ) != obj )
    ZRUNWARN( RK_WARN_CONTACT_DUPKEY, stf0, stf1 );
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

static ZTKPrp __ztk_prp_rkcontactinfo[] = {
  { "bind", 1, _rkContactInfoBindFromZTK, NULL },
  { "staticfriction", 1, _rkContactInfoStaticFricFromZTK, NULL },
  { "kineticfriction", 1, _rkContactInfoKineticFricFromZTK, NULL },
  { "compensation", 1, _rkContactInfoCompensationFromZTK, NULL },
  { "relaxation", 1, _rkContactInfoRelaxationFromZTK, NULL },
  { "elasticity", 1, _rkContactInfoElasticityFromZTK, NULL },
  { "viscosity", 1, _rkContactInfoViscosityFromZTK, NULL },
};

static void *_rkContactInfoFromZTK(void *obj, int i, void *arg, ZTK *ztk)
{
  if( !ZTKEncodeKey( zArrayElem((rkContactInfoArray*)obj,i), arg, ztk, __ztk_prp_rkcontactinfo ) ) return NULL;
  return obj;
}

/* ********************************************************** */
/* CLASS: rkContactInfoArray
 * contact model array class
 * ********************************************************** */

static bool _rkContactInfoArrayFScan(FILE *fp, void *instance, char *buf, bool *success);

/* destroy contact information array */
void rkContactInfoArrayDestroy(rkContactInfoArray *carray)
{
  register int i;

  if( !carray ) return;
  for( i=0; i<zArraySize(carray); i++ )
    rkContactInfoDestroy( zArrayElemNC(carray,i) );
  zArrayFree( carray );
}

/* associate contact information with a pair of keys. */
rkContactInfo *rkContactInfoArrayAssoc(rkContactInfoArray *carray, char *stf1, char *stf2)
{
  register int i;

  if( !stf1 || !stf2 ) return NULL;
  for( i=0; i<zArraySize(carray); i++ )
    if( rkContactInfoAssoc( zArrayElemNC(carray,i), stf1, stf2 ) )
      return zArrayElemNC(carray,i);
  return NULL;
}

/* associate contact information that matches specified type with a pair of keys. */
rkContactInfo *rkContactInfoArrayAssocType(rkContactInfoArray *carray, char *stf1, char *stf2, char type)
{
  register int i;

  if( !stf1 || !stf2) return NULL;

  for( i=0; i<zArraySize(carray); i++ )
    if( rkContactInfoType( zArrayElemNC(carray,i) ) == type && rkContactInfoAssoc( zArrayElemNC(carray,i), stf1, stf2 ) )
      return zArrayElemNC(carray,i);
  return NULL;
}

/* scan contact information array from a file. */
bool rkContactInfoArrayScanFile(rkContactInfoArray *carray, char filename[])
{
  FILE *fp;
  rkContactInfoArray *result;

  if( !( fp = zOpenZTKFile( filename, "r" ) ) )
    return false;
  result = rkContactInfoArrayFScan( fp, carray );
  fclose( fp );
  return result != NULL;
}

typedef struct{
  rkContactInfoArray *carray;
  int c;
} _rkContactInfoArrayParam;

bool _rkContactInfoArrayFScan(FILE *fp, void *instance, char *buf, bool *success)
{
  _rkContactInfoArrayParam *prm;
  rkContactInfo *ci;

  prm = instance;
  ci = zArrayElemNC( prm->carray, prm->c++ );
  if( strcmp( buf, ZTK_TAG_RKCONTACTINFO ) == 0 )
    if( !rkContactInfoFScan( fp, ci ) )
      return ( *success = false );
  if( rkContactInfoArrayAssoc( prm->carray, ci->__stf[0], ci->__stf[1] ) != ci )
    ZRUNWARN( RK_WARN_CONTACT_DUPKEY, ci->__stf[0], ci->__stf[1] );
  return true;
}

/* scan contact information array from a file. */
rkContactInfoArray *rkContactInfoArrayFScan(FILE *fp, rkContactInfoArray *carray)
{
  _rkContactInfoArrayParam prm;

  zArrayAlloc( carray, rkContactInfo, zFCountTag( fp, ZTK_TAG_RKCONTACTINFO ) );
  if( !zArrayBuf(carray) ){
    ZALLOCERROR();
    return NULL;
  }
  prm.carray = carray;
  prm.c = 0;
  if( zTagFScan( fp, _rkContactInfoArrayFScan, &prm ) )
    return carray;
  zArrayFree( carray );
  return NULL;
}

/* ZTK */

static ZTKPrp __ztk_prp_tag_rkcontactinfo[] = {
  { ZTK_TAG_RKCONTACTINFO, -1, _rkContactInfoFromZTK, NULL },
};

rkContactInfoArray *rkContactInfoArrayFromZTK(rkContactInfoArray *carray, ZTK *ztk)
{
  int n;

  zArrayInit( carray );
  if( ( n = ZTKCountTag( ztk, ZTK_TAG_RKCONTACTINFO ) ) == 0 ){
    ZRUNWARN( RK_WARN_CONTACT_EMPTY );
    return NULL;
  }
  zArrayAlloc( carray, rkContactInfo, n );
  ZTKEncodeTag( carray, carray, ztk, __ztk_prp_tag_rkcontactinfo );
  return carray;
}

rkContactInfoArray *rkContactInfoArrayScanZTK(rkContactInfoArray *carray, char filename[])
{
  ZTK ztk;

  ZTKInit( &ztk );
  if( !ZTKDefRegPrp( &ztk, ZTK_TAG_RKCONTACTINFO, __ztk_prp_tag_rkcontactinfo ) ||
      !ZTKDefRegPrp( &ztk, ZTK_TAG_RKCONTACTINFO, __ztk_prp_rkcontactinfo ) )
    return NULL;
  ZTKParse( &ztk, filename );
  carray = rkContactInfoArrayFromZTK( carray, &ztk );
  ZTKDestroy( &ztk );
  return carray;
}

/* print contact information array out to a file. */
void rkContactInfoArrayFPrint(FILE *fp, rkContactInfoArray *carray)
{
  register int i;

  for( i=0; i<zArraySize(carray); i++ ){
    fprintf( fp, "[%s]\n", ZTK_TAG_RKCONTACTINFO );
    rkContactInfoFPrint( fp, zArrayElemNC(carray,i) );
  }
}

/* print contact information array out to a ZTK file. */
bool rkContactInfoArrayPrintZTK(rkContactInfoArray *carray, char filename[])
{
  char name[BUFSIZ];
  FILE *fp;

  if( !( fp = zOpenZTKFile( name, "w" ) ) ) return false;
  rkContactInfoArrayFPrint( fp, carray );
  return true;
}
