#include <zm/zm_seq.h>
#include <roki/rk_chain.h>

static rkChain chain;
static zSeq seq;
static rkLink *link;

enum{
  RK_FK_CHAINFILE = 0,
  RK_FK_SEQFILE,
  RK_FK_INITFILE,
  RK_FK_LINKNAME,
  RK_FK_HELP,
  RK_FK_INVALID
};
zOption option[] = {
  { "chain", NULL, "<.ztk file>", "kinematic chain model file", NULL, false },
  { "seq",   NULL, "<.zvs file>", "sequence file", NULL, false },
  { "init",  NULL, "<.zkci file>","initial state file", NULL, false },
  { "link",  NULL, "<link name>", "link name", NULL, false },
  { "help",  NULL, NULL,          "show this message", NULL, false },
  { NULL, NULL, NULL, NULL, NULL, false },
};

void rk_fkUsage(void)
{
  ZECHO( "Usage: rk_fk [options] <chainfile> <link name> [seqfile]" );
  zOptionHelp( option );
  exit( 0 );
}

FILE *rk_fkOpenLogfile(char *linkname)
{
  char filename[BUFSIZ];
  char *seqfilebase;
  FILE *fp;

  if( option[RK_FK_SEQFILE].flag ){
    if( !zSeqScanFile( &seq, option[RK_FK_SEQFILE].arg ) )
      return false;
  } else{
    option[RK_FK_SEQFILE].arg = "fk.out";
    if( !zSeqScan( &seq ) )
      return false;
  }
  if( !( seqfilebase = zStrClone( option[RK_FK_SEQFILE].arg ) ) )
    return false;
  zGetBasenameDRC( seqfilebase );
  sprintf( filename, "%s.%s", seqfilebase, linkname );
  if( ( fp = fopen( filename, "w" ) ) == NULL )
    ZOPENERROR( filename );
  free( seqfilebase );
  return fp;
}

FILE *rk_fkCommandArgs(int argc, char *argv[])
{
  zStrAddrList arglist;
  char *chainfile, *linkname, *seqfile;
  FILE *fp;

  if( argc <= 1 ) rk_fkUsage();
  zOptionRead( option, argv, &arglist );
  if( option[RK_FK_HELP].flag ) rk_fkUsage();
  zStrListGetPtr( &arglist, 3, &chainfile, &linkname, &seqfile );
  if( chainfile ){
    option[RK_FK_CHAINFILE].flag = true;
    option[RK_FK_CHAINFILE].arg  = chainfile;
  } else{
    ZRUNERROR( "kinematic chain model unspecified" );
    rk_fkUsage();
  }
  if( linkname ){
    option[RK_FK_LINKNAME].flag = true;
    option[RK_FK_LINKNAME].arg  = linkname;
  } else{
    ZRUNERROR( "link unspecified" );
    rk_fkUsage();
  }
  if( seqfile ){
    option[RK_FK_SEQFILE].flag = true;
    option[RK_FK_SEQFILE].arg  = seqfile;
  }

  if( !rkChainReadZTK( &chain, option[RK_FK_CHAINFILE].arg ) ){
    ZOPENERROR( option[RK_FK_CHAINFILE].arg );
    return NULL;
  }
  zNameFind( rkChainRoot(&chain), rkChainLinkNum(&chain), option[RK_FK_LINKNAME].arg, link );
  if( !link ){
    ZRUNERROR( "unknown link name %s", option[RK_FK_LINKNAME].arg );
    return NULL;
  }
  if( !( fp = rk_fkOpenLogfile( option[RK_FK_LINKNAME].arg ) ) ) return NULL;
  if( option[RK_FK_INITFILE].flag &&
      !rkChainInitReadZTK( &chain, option[RK_FK_INITFILE].arg ) ){
    ZOPENERROR( option[RK_FK_INITFILE].arg );
    return NULL;
  }
  zStrAddrListDestroy( &arglist );
  return fp;
}

void rk_fkOutput(FILE *fp, double t)
{
  zVec3D *p, rpy;

  p = rkLinkWldPos(link);
  zMat3DToZYX( rkLinkWldAtt(link), &rpy );
  fprintf( fp, "%f %.10f %.10f %.10f %.10f %.10f %.10f\n", t,
    p->e[zX], p->e[zY], p->e[zZ], rpy.e[zX], rpy.e[zY], rpy.e[zZ] );
}

/* ******************************************************* */

int main(int argc, char *argv[])
{
  int i = 0;
  double t = 0;
  zSeqCell *cp;
  FILE *fp;

  if( !( fp = rk_fkCommandArgs( argc, argv+1 ) ) ) return 1;

  printf( "exec forward kinematics\n" );

  rkChainFK( &chain, zListHead(&seq)->data.v );
  zListForEachRew( &seq, cp ){
    printf( "step %d/%d\n", i, zListSize(&seq)-1 );
    rkChainFK( &chain, cp->data.v );
    rk_fkOutput( fp, t );
    i++;
    t += cp->data.dt;
  }
  fclose( fp );
  rkChainDestroy( &chain );
  zSeqFree( &seq );
  return 0;
}
