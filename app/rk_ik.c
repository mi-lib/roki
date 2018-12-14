/* inverse kinematics solver */

#include <roki/rk_ik.h>

static rkChain chain;
static rkIK ik;
static rkIKSeq ik_seq;

enum{
  RK_IK_MODELFILE=0, RK_IK_CONFFILE,
  RK_IK_ENTRYFILE, RK_IK_OUTPUTFILE,
  RK_IK_ITERNUM, RK_IK_TOL,
  RK_IK_HELP,
  RK_IK_INVALID
};
zOption option[] = {
  { "m", "model", "<.zkc file>", "kinematic chain model file", NULL, false },
  { "c", "conf", "<.zik file>", "IK configuration file", NULL, false },
  { "e", "entry", "<entry file>", "motion constraint entry data file", NULL, false },
  { "o", "out", "<.zvs file>", "output joint sequence file", NULL, false },
  { "i", "iternum", "<n>", "number of the maximum iteration steps", "10000", false },
  { "t", "tol", "<n>", "tolerance of error", "1.0e-12", false },
  { "h", "help", NULL, "show this message", NULL, false },
  { NULL, NULL, NULL, NULL, NULL, false },
};

void rk_ik_usage(void)
{
  eprintf( "Usage: rk_ik <options> [.zkc file] [.zik file] [entry file] [output file]\n" );
  zOptionHelp( option );

  eprintf( "\nWhen [entry file] is omitted, data are read through the standard input.\n" );
  eprintf( "\nWhen [output file] is omitted, result is output to the standard output.\n" );
  exit( 0 );
}

FILE *rk_ik_command_args(int argc, char *argv[])
{
  zStrList arglist;
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

  if( !rkChainReadFile( &chain, option[RK_IK_MODELFILE].arg ) ) exit( 1 );
  if( !rkIKConfReadFile( &ik, &chain, option[RK_IK_CONFFILE].arg ) ) exit( 1 );
  if( option[RK_IK_ENTRYFILE].flag )
    rkIKSeqReadFile( &ik_seq, option[RK_IK_ENTRYFILE].arg );
  else
    rkIKSeqRead( &ik_seq );

  if( !option[RK_IK_OUTPUTFILE].flag ) return stdout;
  if( !( fout = fopen( option[RK_IK_OUTPUTFILE].arg, "w" ) ) ){
    ZOPENERROR( option[RK_IK_OUTPUTFILE].arg );
    exit( 1 );
  }
  zStrListDestroy( &arglist, false );
  return fout;
}

void rk_ik_solve(rkIKSeq *seq, FILE *fout, rkIK *ik)
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
  while( !zListIsEmpty(seq) ){
    zQueueDequeue( seq, &cp );
    rkIKDeactivate( ik );
    rkIKBind( ik ); /* bind current status to the reference. */
    rkIKSeqCellSet( ik, &cp->data );
    eprintf( "output: %d\n", ++i );
    rkIKSolve( ik, v, tol, iter );
    /* output */
    fprintf( fout, "%.10f ", cp->data.dt );
    zVecFWrite( fout, v ); fflush( fout );
    rkIKSeqListCellFree( cp );
  }
  zVecFree( v );
}

int main(int argc, char *argv[])
{
  FILE *fout;

  fout = rk_ik_command_args( argc, argv+1 );
  rk_ik_solve( &ik_seq, fout, &ik );
  if( !option[RK_IK_OUTPUTFILE].flag ) fclose( fout );
  rkIKSeqFree( &ik_seq );
  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  return 0;
}
