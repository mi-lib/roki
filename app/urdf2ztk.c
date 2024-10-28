#include <roki/roki.h>

void urdf2ztk_usage(char *argv)
{
  eprintf( "Usage: %s <URDF file>\n", argv );
  exit( EXIT_FAILURE );
}

int main(int argc, char *argv[])
{
  char filename[BUFSIZ];

  if( argc < 2 ) urdf2ztk_usage( argv[0] );
  zReplaceSuffix( argv[1], "ztk", filename, BUFSIZ );
  rkURDF2ZTK( argv[1], filename );
  return EXIT_SUCCESS;
}
