#include <roki/roki.h>

void urdf2ztk_usage(char *argv)
{
  eprintf( "Usage: %s <URDF file>\n", argv );
  exit( EXIT_FAILURE );
}

int main(int argc, char *argv[])
{
  char outputfilename[BUFSIZ];

  if( argc < 2 ) urdf2ztk_usage( argv[0] );
  zReplaceSuffix( argv[1], ZEDA_ZTK_SUFFIX, outputfilename, BUFSIZ );
  rkURDFWriteZTK( argv[1], outputfilename );
  return EXIT_SUCCESS;
}
