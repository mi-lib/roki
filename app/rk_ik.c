/* inverse kinematics solver */

#include <roki/rk_ik.h>

static rkChain chain;
static rkIKSeq ik_seq;

enum{
  RK_IK_MODELFILE=0, RK_IK_CONFFILE,
  RK_IK_ENTRYFILE, RK_IK_OUTPUTFILE,
  RK_IK_ITERNUM, RK_IK_TOL,
  RK_IK_VERBOSE,
  RK_IK_HELP,
  RK_IK_INVALID,
};
zOption option[] = {
  { "m", "model", "<.ztk file>", "kinematic chain model file", NULL, false },
  { "c", "conf", "<.ztk file>", "IK configuration file", NULL, false },
  { "e", "entry", "<entry file>", "motion constraint entry data file", NULL, false },
  { "o", "out", "<.zvs file>", "output joint sequence file", NULL, false },
  { "i", "iternum", "<n>", "number of the maximum iteration steps", "1000", false },
  { "t", "tol", "<n>", "tolerance of error", "1.0e-10", false },
  { "v", "verbose", NULL, "run this program verbosely", NULL, false },
  { "h", "help", NULL, "show this message", NULL, false },
  { NULL, NULL, NULL, NULL, NULL, false },
};

void rk_ik_usage(void)
{
  eprintf( "Usage: rk_ik <options> [model file] [config file] [entry file] [output file]\n" );
  zOptionHelp( option );

  eprintf( "\nWhen [entry file] is omitted, data are read through the standard input.\n" );
  eprintf( "\nWhen [output file] is omitted, result is output to the standard output.\n" );
  exit( 0 );
}

void rk_ik_message(const char *str)
{
  if( !option[RK_IK_VERBOSE].flag ) return;
  eprintf( "%s", str );
}

void rk_ik_verbose_filename(void)
{
  if( !option[RK_IK_VERBOSE].flag ) return;
  eprintf( "kinematic chain model file: %s\n", option[RK_IK_MODELFILE].arg );
  eprintf( "IK configration file      : %s\n", option[RK_IK_CONFFILE].arg );
  eprintf( "IK entry data file        : %s\n", option[RK_IK_ENTRYFILE].arg );
  eprintf( "output file               : %s\n", option[RK_IK_OUTPUTFILE].arg );
}

FILE *rk_ik_command_args(int argc, char *argv[])
{
  zStrAddrList arglist;
  char *modelfile, *conffile, *entryfile, *outputfile;
  FILE *fout;

  if( argc < 3 ) rk_ik_usage();
  zOptionRead( option, argv, &arglist );
  if( option[RK_IK_HELP].flag ) rk_ik_usage();
  zStrListGetPtr( &arglist, 4, &modelfile, &conffile, &entryfile, &outputfile );
  if( modelfile ){
    option[RK_IK_MODELFILE].flag = true;
    option[RK_IK_MODELFILE].arg  = modelfile;
  }
  if( conffile ){
    option[RK_IK_CONFFILE].flag = true;
    option[RK_IK_CONFFILE].arg  = conffile;
  }
  if( entryfile ){
    option[RK_IK_ENTRYFILE].flag = true;
    option[RK_IK_ENTRYFILE].arg  = entryfile;
  }
  if( outputfile ){
    option[RK_IK_OUTPUTFILE].flag = true;
    option[RK_IK_OUTPUTFILE].arg  = outputfile;
  }
  rk_ik_verbose_filename();
  rk_ik_message( "Read a kinematic chain model ..." );
  if( !rkChainReadZTK( &chain, option[RK_IK_MODELFILE].arg ) ) exit( 1 );
  rk_ik_message( "done.\n" );
  rk_ik_message( "Read an IK configuration file ..." );
  if( !rkChainIKConfReadZTK( &chain, option[RK_IK_CONFFILE].arg ) ) exit( 1 );
  rk_ik_message( "done.\n" );
  rk_ik_message( "Read an IK entry file ..." );
  if( option[RK_IK_ENTRYFILE].flag )
    rkIKSeqScanFile( &ik_seq, option[RK_IK_ENTRYFILE].arg );
  else
    rkIKSeqScan( &ik_seq );
  rk_ik_message( "done.\n" );

  if( !option[RK_IK_OUTPUTFILE].flag ){
    rk_ik_message( "Output the result to standard output.\n" );
    return stdout;
  }
  if( !( fout = fopen( option[RK_IK_OUTPUTFILE].arg, "w" ) ) ){
    ZOPENERROR( option[RK_IK_OUTPUTFILE].arg );
    exit( 1 );
  }
  rk_ik_message( "Output the result to " );
  rk_ik_message( option[RK_IK_OUTPUTFILE].arg );
  rk_ik_message( "\n" );
  zStrAddrListDestroy( &arglist );
  return fout;
}

void rk_ik_solve(FILE *fout)
{
  rkIKSeqListCell *cp;
  zVec v;
  int iter;
  int i = 0;
  double tol;

  if( !( v = zVecAlloc( rkChainJointSize(&chain) ) ) ){
    ZALLOCERROR();
    exit( 1 );
  }
  iter = atoi( option[RK_IK_ITERNUM].arg );
  tol = atof( option[RK_IK_TOL].arg );
  while( !zListIsEmpty(&ik_seq) ){
    zQueueDequeue( &ik_seq, &cp );
    rkChainDeactivateIK( &chain );
    rkChainBindIK( &chain ); /* bind current status to the reference. */
    rkChainSetIKSeqCell( &chain, &cp->data );
    eprintf( "output: %d\n", ++i );
    rkChainIK( &chain, v, tol, iter );
    /* output */
    fprintf( fout, "%.10f ", cp->data.dt );
    zVecFPrint( fout, v ); fflush( fout );
    rkIKSeqListCellFree( cp );
  }
  zVecFree( v );
}

int main(int argc, char *argv[])
{
  FILE *fout;

  fout = rk_ik_command_args( argc, argv+1 );
  rk_ik_solve( fout );
  if( !option[RK_IK_OUTPUTFILE].flag ) fclose( fout );
  rkIKSeqFree( &ik_seq );
  rkChainDestroy( &chain );
  return 0;
}
