/* rk_chain_convexify
 *
 * 2023. 6. 2. Originally developed by Kenta Imanishi.
 * 2023. 6. 4. Modified by Tom Sugihara.
 * 2024.12. 9. Last updated by Tom Sugihara.
 */

#include <roki/roki.h>

enum{
  RCC_SRCFILE = 0,
  RCC_OUTPUTFILE,
  RCC_VERBOSE,
  RCC_HELP,
  RCC_INVALID
};
zOption option[] = {
  { "s", "source", "<.ztk file>", "original kinematic chain model file",    NULL, false },
  { "o", "output", "<.ztk file>", "convexified kinematic chain model file", NULL, false },
  { "v", "verbose", NULL,         "output messages", NULL, false },
  { "help",  NULL, NULL,          "show this message", NULL, false },
  { NULL, NULL, NULL, NULL, NULL, false },
};

void rcc_usage(void)
{
  ZECHO( "Usage: rk_chain_convexify [options] <source .ztk file> [source .ztk file]" );
  zOptionHelp( option );
  exit( 0 );
}

bool rcc_read_command_arg(rkChain *chain, int argc, char *argv[])
{
  zStrAddrList arglist;
  char *srcfile, *outputfile;

  if( argc <= 1 ) rcc_usage();
  zOptionRead( option, argv, &arglist );
  if( option[RCC_HELP].flag ) rcc_usage();
  zStrListGetPtr( &arglist, 2, &srcfile, &outputfile );
  if( srcfile ){
    option[RCC_SRCFILE].flag = true;
    option[RCC_SRCFILE].arg  = srcfile;
  } else{
    ZRUNERROR( "source kinematic chain model unspecified" );
    rcc_usage();
  }
  if( outputfile ){
    option[RCC_OUTPUTFILE].flag = true;
    option[RCC_OUTPUTFILE].arg  = outputfile;
  }
  if( !rkChainReadZTK( chain, option[RCC_SRCFILE].arg ) ){
    ZOPENERROR( option[RCC_SRCFILE].arg );
    return false;
  }
  zMShape3DToPH( rkChainShape(chain) );
  zArrayFree( &rkChainShape(chain)->optic );
  zArrayFree( &rkChainShape(chain)->texture );
  zStrAddrListDestroy( &arglist );
  return true;
}

bool rcc_is_fixed_link(rkLink *link)
{
  return strcmp( rkLinkJointTypeStr(link), "fixed" ) == 0;
}

int rcc_add_link_vert(rkLink *link, zFrame3D *frame, zVec3DData *data)
{
  zShapeListCell *sp;
  rkLink *l;
  zFrame3D f;
  zVec3D v;
  int i;

  zListForEach( rkLinkShapeList(link), sp ){
    for( i=0; i<zPH3DVertNum(zShape3DPH(sp->data)); i++ )
      zVec3DDataAdd( data, zXform3D( frame, zPH3DVert(zShape3DPH(sp->data),i), &v ) );
  }
  for( l=rkLinkChild(link); l; l=rkLinkSibl(l) ){
    if( !rcc_is_fixed_link( l ) ) continue;
    if( option[RCC_VERBOSE].flag ) eprintf( "merge %s to %s.\n", zName(l), zName(link) );
    zFrame3DCascade( frame, rkLinkAdjFrame(l), &f );
    rcc_add_link_vert( l, &f, data );
  }
  return zVec3DDataSize( data );
}

bool rcc_replace_link_shape(rkChain *chain)
{
  zShape3DArray shape_array;
  zShape3D shape, *sp;
  zVec3DData data;
  int i;

  zArrayInit( &shape_array );
  for( i=0; i<rkChainLinkNum(chain); i++ ){
    if( zListIsEmpty( rkChainLinkShapeList(chain,i) ) ) continue;
    if( rcc_is_fixed_link( rkChainLink(chain,i) ) && i != 0 ) continue;
    zVec3DDataInitList( &data );
    if( rcc_add_link_vert( rkChainLink(chain,i), ZFRAME3DIDENT, &data ) == 0 ) continue;
    /* allocate a shape for the convex hull */
    zShape3DInit( &shape );
    shape.com = &zeo_shape3d_ph_com;
    if( !( shape.body = shape.com->_alloc() ) ) return false;
    zNameSet( &shape, rkChainLinkName(chain,i) );
    /* create the convex hull from vertices. */
    if( option[RCC_VERBOSE].flag ) eprintf( "comvexify %s.\n", rkChainLinkName(chain,i) );
    zVec3DDataConvexHull( &data, zShape3DPH(&shape) );
    zVec3DDataDestroy( &data );
    zArrayAdd( &shape_array, zShape3D, &shape );
  }
  /* replace the original shape of the link. */
  for( i=0; i<rkChainLinkNum(chain); i++ ){
    zShapeListDestroy( rkChainLinkShapeList(chain,i) );
    zArrayFindName( &shape_array, rkChainLinkName(chain,i), sp );
    if( !sp ) continue;
    if( option[RCC_VERBOSE].flag ) eprintf( "associate link shape to %s.\n", rkChainLinkName(chain,i) );
    rkLinkShapePush( rkChainLink(chain,i), sp );
  }
  zArrayMove( &shape_array, &rkChainShape(chain)->shape );
  return true;
}

void rcc_output(rkChain *chain)
{
  char outputfile_default[BUFSIZ];

  if( !option[RCC_OUTPUTFILE].flag ){
    strcpy( outputfile_default, option[RCC_SRCFILE].arg );
    zGetBasenameDRC( outputfile_default );
    zCutSuffix( outputfile_default );
    zStrCat( outputfile_default, "_convex.ztk", BUFSIZ );
    option[RCC_OUTPUTFILE].arg  = outputfile_default;
  }
  if( option[RCC_VERBOSE].flag ) eprintf( "output result to %s.\n", option[RCC_OUTPUTFILE].arg );
  rkChainWriteZTK( chain, option[RCC_OUTPUTFILE].arg );
  rkChainDestroy( chain );
}

int main(int argc, char *argv[])
{
  rkChain chain;

  if( !rcc_read_command_arg( &chain, argc, argv + 1 ) )
    return EXIT_FAILURE;
  if( !rcc_replace_link_shape( &chain ) );
  rcc_output( &chain );
  return EXIT_SUCCESS;
}
