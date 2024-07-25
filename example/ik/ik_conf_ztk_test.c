#include <roki/rk_chain.h>

#define CHAIN_ZTK  "../model/H5.ztk"
#define IKCONF_ZTK "../model/H5.ztk"

int main(int argc, char *argv[])
{
  rkChain robot;

  rkChainReadZTK( &robot, argc > 1 ? argv[1] : CHAIN_ZTK );
  rkChainIKConfReadZTK( &robot, argc > 2 ? argv[2] : IKCONF_ZTK );
  rkChainIKConfWriteZTK( &robot, "ik_output.ztk" );
  rkChainDestroy( &robot );
  return 0;
}
