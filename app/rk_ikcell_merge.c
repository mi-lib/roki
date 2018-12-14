#include <zeda/zeda.h>

typedef struct{
  FILE *fp;
  int num;
} ikcell_file_t;

void ikcell_merge_usage(void)
{
  eprintf( "Usage: ikcell_merge [file1] [file2] ...\n" );
  eprintf( " file?\tinverse kinematics seed file\n" );
  eprintf( "      \tare merged and output to the standard output\n" );
  exit( 0 );
}

int ikcell_merge_open(ikcell_file_t **file, int argc, char *argv[])
{
  register int i;
  int num;

  num = argc - 1;
  if( !( *file = zAlloc( ikcell_file_t, num ) ) ){
    ZALLOCERROR();
    exit( 1 );
  }
  for( i=0; i<num; i++ )
    if( !( (*file)[i].fp = fopen( argv[i+1], "r" ) ) ){
      ZOPENERROR( argv[i+1] );
      exit( 1 );
    }
  return num;
}

void ikcell_merge_close(int num, ikcell_file_t *file)
{
  register int i;

  for( i=0; i<num; i++ ) fclose( file[i].fp );
  free( file );
}

void ikcell_merge_exec(int fpnum, ikcell_file_t *file)
{
  register int i, j;
  int num, entry;
  double w, v[3];

  while( 1 ){
    for( num=0, i=0; i<fpnum; i++ ){
      if( !zFSkipDefaultComment( file[i].fp ) ) return;
      num += ( file[i].num = zFInt( file[i].fp ) );
    }
    printf( "%d", num );
    for( i=0; i<fpnum; i++ )
      for( j=0; j<file[i].num; j++ ){
        entry = zFInt( file[i].fp );
        w    = zFDouble( file[i].fp );
        v[0] = zFDouble( file[i].fp );
        v[1] = zFDouble( file[i].fp );
        v[2] = zFDouble( file[i].fp );
        printf( " %d %.10g %.10g %.10g %.10g", entry, w, v[0], v[1], v[2] );
      }
    printf( "\n" );
  }
}

int main(int argc, char *argv[])
{
  int fpnum;
  ikcell_file_t *file;

  if( argc < 2 ) ikcell_merge_usage();
  fpnum = ikcell_merge_open( &file, argc, argv );
  ikcell_merge_exec( fpnum, file );
  ikcell_merge_close( fpnum, file );
  return 0;
}
