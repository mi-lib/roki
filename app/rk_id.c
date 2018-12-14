#include <zm/zm_seq.h>
#include <roki/rk_chain.h>

static rkChain chain;
static zSeq seq;
static double zfloor = 0;

static char seqfilebase[BUFSIZ];

/* 0 - COM
 * 1 - COM velocity
 * 2 - COM acceleration
 * 3 - ZMP
 * 4 - base attitude
 * 5 - AM around COM
 * 6 - joint torques
 */
#define RK_ID_PARAM_NUM 7
static FILE *fp[RK_ID_PARAM_NUM];

enum{
  RK_ID_CHAINFILE = 0,
  RK_ID_SEQFILE,
  RK_ID_INITFILE,
  RK_ID_HELP,
  RK_ID_INVALID
};
zOption option[] = {
  { "chain", NULL, "<.zkc file>",      "kinematic chain model file", NULL, false },
  { "seq",   NULL, "<.zvs/.zus file>", "sequence file", NULL, false },
  { "init",  NULL, "<.zkci file>",     "initial state file", NULL, false },
  { "help",  NULL, NULL,               "show this message", NULL, false },
  { NULL, NULL, NULL, NULL, NULL, false },
};

void rk_idUsage(void)
{
  ZECHO( "Usage: rk_id [options] <chainfile> [seqfile]" );
  zOptionHelp( option );
  exit( 0 );
}

bool rk_idLoadSequence(void)
{
  if( option[RK_ID_SEQFILE].flag ){
    if( !zSeqReadFile( &seq, option[RK_ID_SEQFILE].arg ) )
      return false;
  } else{
    option[RK_ID_SEQFILE].arg = "id.out";
    if( !zSeqRead( &seq ) )
      return false;
  }
  zGetBasename( option[RK_ID_SEQFILE].arg, seqfilebase, BUFSIZ );
  return true;
}

bool rk_idOpenLogfile(void)
{
  register int i;
  char filename[BUFSIZ];
  char *type[] = { "com", "comv", "coma", "zmp", "att", "am", "trq" };

  for( i=0; i<RK_ID_PARAM_NUM; i++ ){
    sprintf( filename, "%s.%s", seqfilebase, type[i] );
    if( ( fp[i] = fopen( filename, "w" ) ) == NULL ){
      ZOPENERROR( filename );
      return false;
    }
  }
  return true;
}

bool rk_idCommandArgs(int argc, char *argv[])
{
  zStrList arglist;
  char *chainfile, *seqfile;

  if( argc <= 1 ) rk_idUsage();
  zOptionRead( option, argv, &arglist );
  if( option[RK_ID_HELP].flag ) rk_idUsage();
  zStrListGetPtr( &arglist, 2, &chainfile, &seqfile );
  if( chainfile ){
    option[RK_ID_CHAINFILE].flag = true;
    option[RK_ID_CHAINFILE].arg  = chainfile;
  } else{
    ZRUNERROR( "kinematic chain model unspecified" );
    rk_idUsage();
  }
  if( seqfile ){
    option[RK_ID_SEQFILE].flag = true;
    option[RK_ID_SEQFILE].arg  = seqfile;
  }

  if( !rkChainReadFile( &chain, option[RK_ID_CHAINFILE].arg ) ){
    ZOPENERROR( option[RK_ID_CHAINFILE].arg );
    return false;
  }
  if( !rk_idLoadSequence() || !rk_idOpenLogfile() ) return false;
  if( option[RK_ID_INITFILE].flag &&
      !rkChainInitReadFile( &chain, option[RK_ID_INITFILE].arg ) ){
    ZOPENERROR( option[RK_ID_INITFILE].arg );
    return false;
  }
  zStrListDestroy( &arglist, false );
  return true;
}

/* log */

void rk_idCloseLogfile(void)
{
  register int i;

  for( i=0; i<RK_ID_PARAM_NUM; i++ )
    fclose( fp[i] );
}

void rk_idOutput(double t, zVec trq)
{
  zVec3D a, zmp, rpy, am;

  zVec3DSub( rkChainCOMAcc(&chain), RK_GRAVITY3D, &a );
  rkChainZMP( &chain, zfloor, &zmp );
  zMat3DToZYX( rkChainRootAtt(&chain), &rpy );
  rkChainAM( &chain, rkChainWldCOM(&chain), &am );
  rkChainGetJointTrqAll( &chain, trq );

  fprintf( fp[0], "%f ", t ); zVec3DDataFWrite( fp[0], rkChainWldCOM(&chain) );
  fprintf( fp[1], "%f ", t ); zVec3DDataFWrite( fp[1], rkChainCOMVel(&chain) );
  fprintf( fp[2], "%f ", t ); zVec3DDataFWrite( fp[2], &a );
  fprintf( fp[3], "%f ", t ); zVec3DDataFWrite( fp[3], &zmp );
  fprintf( fp[4], "%f ", t ); zVec3DDataFWrite( fp[4], &rpy );
  fprintf( fp[5], "%f ", t ); zVec3DDataFWrite( fp[5], &am );
  fprintf( fp[6], "%f ", t ); zVecDataFWrite( fp[6], trq );
}

/* ******************************************************* */

int main(int argc, char *argv[])
{
  register int i = 0;
  double t = 0;
  zSeqListCell *cp;
  zVec trq;

  if( !rk_idCommandArgs( argc, argv+1 ) ) return 1;
  if( !( trq = zVecAlloc( rkChainJointSize(&chain) ) ) ){
    ZALLOCERROR();
    return 1;
  }
  printf( "exec Inverse Dynamics\n" );

  rkChainFK( &chain, zListHead(&seq)->data.v );
  zListForEachRew( &seq, cp ){
    printf( "step %d/%d\n", i, zListNum(&seq)-1 );
    rkChainFKCNT( &chain, cp->data.v, cp->data.dt );
    rk_idOutput( t, trq );
    i++;
    t += cp->data.dt;
  }

  zVecFree( trq );
  rk_idCloseLogfile();
  rkChainDestroy( &chain );
  zSeqFree( &seq );
  return 0;
}
