#include <roki/roki.h>

#define DEBUG 0

int bvh2ztk_usage(char *argv, zOption *option)
{
  eprintf( "Usage: %s <.bvh file>\n", argv );
  zOptionHelp( option );
  exit( EXIT_SUCCESS );
}

ZDEF_STRUCT( stream_t ){
  FILE *fp;
  char buf[BUFSIZ];
  char *str;
  char token[BUFSIZ];
};

bool bvh2ztk_stream_open(stream_t *stream, char *filename)
{
  if( !( stream->fp = fopen( filename, "rt" ) ) ){
    ZOPENERROR( filename );
    return false;
  }
  return true;
}

void bvh2ztk_stream_close(stream_t *stream)
{
  fclose( stream->fp );
}

char *bvh2ztk_stream_getline(stream_t *stream)
{
  if( !fgets( stream->buf, BUFSIZ, stream->fp ) ) return NULL;
  stream->str = zSSkipWS( stream->buf );
  return stream->str;
}

char *bvh2ztk_stream_gettoken(stream_t *stream)
{
  return zSToken( stream->str, stream->token, BUFSIZ );
}

bool bvh2ztk_cmp_token(stream_t *stream, const char *word)
{
  return strncmp( stream->token, word, strlen(word) ) == 0;
}

ZDEF_STRUCT( joint_t ){
  Z_NAMED_CLASS
  zFrame3D orgframe;
  zFrame3D adjframe;
  zFrame3D wldframe;
  int num_axis;
  zAxis axis[6];
  joint_t *child;
  joint_t *sibl;
};

joint_t *bvh2ztk_joint_create(char *name)
{
  joint_t *joint;

  if( !( joint = zAlloc( joint_t, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  zNameSet( joint, name );
  zFrame3DIdent( &joint->orgframe );
  zFrame3DIdent( &joint->wldframe );
  joint->num_axis = 0;
  joint->child = joint->sibl = NULL;
  return joint;
}

void bvh2ztk_joint_destroy(joint_t *joint)
{
  if( joint->child ) bvh2ztk_joint_destroy( joint->child );
  if( joint->sibl ) bvh2ztk_joint_destroy( joint->sibl );
  zNameFree( joint );
  free( joint );
}

void bvh2ztk_joint_add(joint_t *joint, joint_t *child)
{
  joint_t *jp;

  if( !joint->child ){
    joint->child = child;
  } else{
    for( jp=joint->child; jp->sibl; jp=jp->sibl );
    jp->sibl = child;
  }
}

int bvh2ztk_joint_count(joint_t *joint)
{
  return joint ? bvh2ztk_joint_count( joint->child ) + bvh2ztk_joint_count( joint->sibl ) + 1 : 0;
}

int bvh2ztk_joint_count_dis(joint_t *joint)
{
  int count;

  count = joint->num_axis;
  if( joint->child )
    count += bvh2ztk_joint_count_dis( joint->child );
  if( joint->sibl )
    count += bvh2ztk_joint_count_dis( joint->sibl );
  return count;
}

void bvh2ztk_joint_update_frame(joint_t *joint, joint_t *parent)
{
  if( parent )
    zFrame3DCascade( &parent->wldframe, &joint->adjframe, &joint->wldframe );
  else{
    static zFrame3D f = { { { 0, 0, 0 } }, { { { 0, 1, 0 }, { 0, 0, 1 }, { 1, 0, 0 } } } };
    zFrame3DCascade( &f, &joint->adjframe, &joint->wldframe );
  }
  if( joint->child )
    bvh2ztk_joint_update_frame( joint->child, joint );
  if( joint->sibl )
    bvh2ztk_joint_update_frame( joint->sibl, parent );
}

#if DEBUG == 1
void bvh2ztk_joint_print(joint_t *joint, int indent)
{
  int i;

  zIndent( indent );
  printf( "name: %s\n", zName(joint) );
  zIndent( indent );
  printf( "offset : " ); zVec3DPrint( zFrame3DPos(&joint->orgframe) );
  zIndent( indent );
  printf( "channels : %d\t", joint->num_axis );
  for( i=0; i<joint->num_axis; i++ )
    printf( "%d ", joint->axis[i] );
  printf( "\n" );
  if( joint->child ){
    zIndent( indent );
    printf( "<child>\n" );
    bvh2ztk_joint_print( joint->child, indent + 2 );
  }
  if( joint->sibl )
    bvh2ztk_joint_print( joint->sibl, indent );
}
#endif

bool bvh2ztk_read_offset(stream_t *stream, joint_t *joint)
{
  if( !bvh2ztk_stream_gettoken( stream ) ) return false;
  zFrame3DPos(&joint->orgframe)->c.x = 0.01 * atof( stream->token );
  if( !bvh2ztk_stream_gettoken( stream ) ) return false;
  zFrame3DPos(&joint->orgframe)->c.y = 0.01 * atof( stream->token );
  if( !bvh2ztk_stream_gettoken( stream ) ) return false;
  zFrame3DPos(&joint->orgframe)->c.z = 0.01 * atof( stream->token );
  return true;
}

enum{ BVH2ZTK_XPOS, BVH2ZTK_YPOS, BVH2ZTK_ZPOS, BVH2ZTK_XROT, BVH2ZTK_YROT, BVH2ZTK_ZROT };
static const char *bvh2ztk_axis_name[] = {
  "Xposition",
  "Yposition",
  "Zposition",
  "Xrotation",
  "Yrotation",
  "Zrotation",
};

bool bvh2ztk_read_channels(stream_t *stream, joint_t *joint)
{
  int i, j;

  bvh2ztk_stream_gettoken( stream ); joint->num_axis = atoi( stream->token );
  for( i=0; i<joint->num_axis; i++ ){
    if( !bvh2ztk_stream_gettoken( stream ) ) return false;
    for( j=BVH2ZTK_XPOS; j<=BVH2ZTK_ZROT; j++ ){
      if( bvh2ztk_cmp_token( stream, bvh2ztk_axis_name[j] ) ){
        joint->axis[i] = j;
        break;
      }
    }
    if( j > BVH2ZTK_ZROT ){
      ZRUNERROR( "unidentified channel type: %s", stream->token );
      return false;
    }
  }
  return true;
}

bool bvh2ztk_read_end(stream_t *stream, joint_t *joint)
{
  joint_t *end;
  char endname[BUFSIZ];

  sprintf( endname, "%s_end", zName(joint) );
  if( !bvh2ztk_stream_getline( stream ) ) return false;
  if( stream->str[0] != '{' ){
    ZRUNERROR( "empty joint field" );
    return false;
  }
  if( !( end = bvh2ztk_joint_create( endname ) ) ) return false;
  bvh2ztk_joint_add( joint, end );
  if( !bvh2ztk_stream_getline( stream ) ) return false;
  bvh2ztk_stream_gettoken( stream );
  if( bvh2ztk_cmp_token( stream, "OFFSET" ) ){
    bvh2ztk_read_offset( stream, end );
  }
  if( !bvh2ztk_stream_getline( stream ) ) return false;
  if( stream->str[0] != '}' ){
    ZRUNERROR( "unclosed end field" );
    return false;
  }
  return true;
}

bool bvh2ztk_read_joint(stream_t *stream, joint_t *joint)
{
  joint_t *child;

  if( !bvh2ztk_stream_getline( stream ) ) return -1;
  if( stream->str[0] != '{' ){
    ZRUNERROR( "empty joint field" );
    return false;
  }
  while( bvh2ztk_stream_getline( stream ) ){
    if( stream->str[0] == '}' ) break;
    bvh2ztk_stream_gettoken( stream );
    if( bvh2ztk_cmp_token( stream, "OFFSET" ) ){
      bvh2ztk_read_offset( stream, joint );
    } else
    if( bvh2ztk_cmp_token( stream, "CHANNELS" ) ){
      bvh2ztk_read_channels( stream, joint );
    } else
    if( bvh2ztk_cmp_token( stream, "JOINT" ) ){
      if( !bvh2ztk_stream_gettoken( stream ) ) return false;
      if( !( child = bvh2ztk_joint_create( stream->token ) ) ) return false;
      bvh2ztk_joint_add( joint, child );
      if( !bvh2ztk_read_joint( stream, child ) ) return false;
    } else
    if( bvh2ztk_cmp_token( stream, "End" ) ){
      if( !bvh2ztk_read_end( stream, joint ) ) return false;
    } else{
      ZRUNERROR( "unidentified keyword: %s", stream->token );
    }
  }
  return true;
}

joint_t *bvh2ztk_read_hierarchy(stream_t *stream)
{
  joint_t *root;

  /* header */
  if( !bvh2ztk_stream_getline( stream ) ) return NULL;
  if( !bvh2ztk_stream_gettoken( stream ) ) return NULL;
  if( !bvh2ztk_cmp_token( stream, "HIERARCHY" ) ) return NULL;
  if( !bvh2ztk_stream_getline( stream ) ) return NULL;
  if( !bvh2ztk_stream_gettoken( stream ) ) return NULL;
  if( !bvh2ztk_cmp_token( stream, "ROOT" ) ) return NULL;
  if( !bvh2ztk_stream_gettoken( stream ) ) return NULL;
  if( !( root = bvh2ztk_joint_create( stream->token ) ) ) return NULL;
  bvh2ztk_read_joint( stream, root );
#if DEBUG == 1
  bvh2ztk_joint_print( root, 0 );
#endif
  return root;
}

void bvh2ztk_write_joint(FILE *fp, joint_t *joint, joint_t *parent)
{
  fprintf( fp, "[link]\n" );
  fprintf( fp, "name: %s\n", zName(joint) );
  fprintf( fp, "jointtype: %s\n", joint->num_axis == 6 ? "float" : ( joint->child ? "spherical" : "fixed" ) );
  if( parent )
    fprintf( fp, "parent: %s\n", zName(parent) );
  fprintf( fp, "\n" );
  if( joint->child )
    bvh2ztk_write_joint( fp, joint->child, joint );
  if( joint->sibl )
    bvh2ztk_write_joint( fp, joint->sibl, parent );
}

bool bvh2ztk_write_hierarchy(joint_t *root, const char *filename)
{
  FILE *fp;

  if( !( fp = fopen( filename, "w" ) ) ){
    ZOPENERROR( filename );
    return false;
  }
  bvh2ztk_write_joint( fp, root, NULL );
  fclose( fp );
  return true;
}

bool bvh2ztk_read_posture(stream_t *stream, joint_t *joint)
{
  int i;
  double val[6];
  zFrame3D f;

  for( i=0; i<joint->num_axis; i++ ){
    if( !bvh2ztk_stream_gettoken( stream ) ) return false;
    val[i] = atof( stream->token );
  }
  zFrame3DIdent( &f );
  for( i=joint->num_axis-1; i>=0; i-- ){
    switch( joint->axis[i] ){
     case 0: /* X position */ zFrame3DPos(&f)->c.x += 0.01 * val[i]; break;
     case 1: /* Y position */ zFrame3DPos(&f)->c.y += 0.01 * val[i]; break;
     case 2: /* Z position */ zFrame3DPos(&f)->c.z += 0.01 * val[i]; break;
     case 3: /* X rotation */ zMat3DRotRollDRC(  zFrame3DAtt(&f), zDeg2Rad( val[i] ) ); break;
     case 4: /* Y rotation */ zMat3DRotPitchDRC( zFrame3DAtt(&f), zDeg2Rad( val[i] ) ); break;
     case 5: /* Z rotation */ zMat3DRotYawDRC(   zFrame3DAtt(&f), zDeg2Rad( val[i] ) ); break;
     default: ;
    }
  }
  zFrame3DCascade( &f, &joint->orgframe, &joint->adjframe );
  if( joint->child )
    if( !bvh2ztk_read_posture( stream, joint->child ) ) return false;
  if( joint->sibl )
    if( !bvh2ztk_read_posture( stream, joint->sibl ) ) return false;
  return true;
}

void bvh2ztk_write_joint_motion(FILE *fp, joint_t *joint)
{
  zVec6D v;

  zFrame3DToVec6DAA( &joint->wldframe, &v );
  zVec6DDataFPrint( fp, &v );
  if( joint->child )
    bvh2ztk_write_joint_motion( fp, joint->child );
  if( joint->sibl )
    bvh2ztk_write_joint_motion( fp, joint->sibl );
}

void bvh2ztk_write_motion(FILE *fp, joint_t *joint, double dt)
{
  fprintf( fp, "%g %d {", dt, bvh2ztk_joint_count( joint ) * 6 );
  bvh2ztk_write_joint_motion( fp, joint );
  fprintf( fp, " }\n" );
}

bool bvh2ztk_conv_motion(stream_t *stream, joint_t *root, const char *filename)
{
  int i, num_frame;
  double dt;
  FILE *fp;

  /* header */
  if( !bvh2ztk_stream_getline( stream ) ) return false;
  if( !bvh2ztk_stream_gettoken( stream ) ) return false;
  if( !bvh2ztk_cmp_token( stream, "MOTION" ) ) return false;
  /* number of frames */
  if( !bvh2ztk_stream_getline( stream ) ) return false;
  if( !bvh2ztk_stream_gettoken( stream ) ) return false;
  if( !bvh2ztk_cmp_token( stream, "Frames" ) ) return false;
  if( !bvh2ztk_stream_gettoken( stream ) ) return false;
  num_frame = atoi( stream->token );
  /* frame interval */
  if( !bvh2ztk_stream_getline( stream ) ) return false;
  if( !bvh2ztk_stream_gettoken( stream ) ) return false;
  if( !bvh2ztk_cmp_token( stream, "Frame" ) ) return false;
  if( !bvh2ztk_stream_gettoken( stream ) ) return false;
  if( !bvh2ztk_cmp_token( stream, "Time" ) ) return false;
  if( !bvh2ztk_stream_gettoken( stream ) ) return false;
  dt = atof( stream->token );
  /* output zkcs file */
  if( !( fp = fopen( filename, "w" ) ) ){
    ZOPENERROR( filename );
    return false;
  }
  for( i=0; i<num_frame; i++ ){
    if( !bvh2ztk_stream_getline( stream ) ) break;
    bvh2ztk_read_posture( stream, root );
    bvh2ztk_joint_update_frame( root, NULL );
    bvh2ztk_write_motion( fp, root, dt );
  }
  fclose( fp );
  return true;
}

enum{
  BVH2ZTK_MODEL_FILE=0,
  BVH2ZTK_ZKCS_FILE,
  BVH2ZTK_HELP,
  BVH2ZTK_INVALID,
};
zOption bvh2ztk_option[] = {
  { "m", "model", "<.ztk file>", "chain model file", "hierarchy.ztk", false },
  { "s", "seq", "<.zkcs file>", "configuration sequence file", "motion.zkcs", false },
  { "h", "help", NULL, "show this message", NULL, false },
  { NULL, NULL, NULL, NULL, NULL, false },
};

bool bvh2ztk_read_option(stream_t *stream, int argc, char *argv[])
{
  zStrAddrList arglist;

  zOptionRead( bvh2ztk_option, argv, &arglist );
  if( bvh2ztk_option[BVH2ZTK_HELP].flag ) bvh2ztk_usage( "bvh2ztk", bvh2ztk_option );
  if( !bvh2ztk_stream_open( stream, zListTail(&arglist)->data ) ) return false;
  zStrAddrListDestroy( &arglist );
  return true;
}

int main(int argc, char *argv[])
{
  stream_t stream;
  joint_t *root;

  if( argc <= 1 )
    bvh2ztk_usage( argv[0], bvh2ztk_option );
  if( !bvh2ztk_read_option( &stream, argc, argv + 1 ) )
    return EXIT_FAILURE;
  if( !( root = bvh2ztk_read_hierarchy( &stream ) ) )
    return EXIT_FAILURE;
  bvh2ztk_write_hierarchy( root, bvh2ztk_option[BVH2ZTK_MODEL_FILE].arg );
  bvh2ztk_conv_motion( &stream, root, bvh2ztk_option[BVH2ZTK_ZKCS_FILE].arg );
  bvh2ztk_joint_destroy( root );
  bvh2ztk_stream_close( &stream );
  return EXIT_SUCCESS;
}
