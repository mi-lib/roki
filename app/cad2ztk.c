#include <roki/roki.h>

#define MAX_BUFSIZ 256

/* link information */

typedef struct link_t{
  char name[MAX_BUFSIZ];
  char parentname[MAX_BUFSIZ];
  char jointtype[MAX_BUFSIZ];
  struct link_t *parent;
  zFrame3D frame;
  rkMP mp;
} link_t;

zArrayClass( link_array_t, link_t );

link_t *link_array_find_link(link_array_t *la, char *name)
{
  int i;

  for( i=0; i<zArraySize(la); i++ )
    if( strcmp( zArrayElem(la,i)->name, name ) == 0 ) return zArrayElem(la,i);
  return NULL;
}

bool link_array_assoc_parent(link_array_t *la)
{
  int i;
  link_t *link;
  bool ret = true;

  for( i=0; i<zArraySize(la); i++ ){
    link = zArrayElem(la,i);
    if( strcmp( link->parentname, "nil" ) == 0 )
      link->parent = NULL;
    else if( !( link->parent = link_array_find_link( la, link->parentname ) ) ){
      ZRUNERROR( "link %s (parent of %s) not found.", link->parentname, link->name );
      ret = false;
    }
  }
  return ret;
}

int link_array_read_csv(zCSV *csv, link_array_t *la)
{
  int i;
  link_t *link;

  zArrayAlloc( la, link_t, zCSVLineNum( csv ) );
  if( zArraySize(la) == 0 ){
    ZALLOCERROR();
    return 0;
  }
  eprintf( "number of links = %d\n", zArraySize(la) );
  zCSVRewind( csv );
  for( i=0; i<zArraySize(la); i++ ){
    if( !zCSVGetLine( csv ) ) break;
    link = zArrayElem(la,i);
    zCSVGetField( csv, link->name, MAX_BUFSIZ );
    zCSVGetField( csv, link->parentname, MAX_BUFSIZ );
    link->parent = NULL;
    zCSVGetField( csv, link->jointtype, MAX_BUFSIZ );
    zCSVGetDoubleN( csv, link->frame.pos.e, 3 );
    zCSVGetDoubleN( csv, (double *)link->frame.att.e, 9 );
    zCSVGetDouble( csv, &link->mp.mass );
    zCSVGetDoubleN( csv, link->mp.com.e, 3 );
    zCSVGetDouble( csv, &link->mp.inertia.c.xx );
    zCSVGetDouble( csv, &link->mp.inertia.c.xy );
    zCSVGetDouble( csv, &link->mp.inertia.c.xz );
    zCSVGetDouble( csv, &link->mp.inertia.c.yy );
    zCSVGetDouble( csv, &link->mp.inertia.c.yz );
    zCSVGetDouble( csv, &link->mp.inertia.c.zz );
    link->mp.inertia.c.yx = link->mp.inertia.c.xy;
    link->mp.inertia.c.zx = link->mp.inertia.c.xz;
    link->mp.inertia.c.zy = link->mp.inertia.c.yz;

    eprintf( "Read link %s (parent=%s)\n", link->name, link->parentname );
  }
  return zArraySize(la);
}

/* shape information */

typedef struct{
  char name[MAX_BUFSIZ];
  char stlname[MAX_BUFSIZ];
  link_t *link;
  zFrame3D frame;
} shape_t;

zArrayClass( shape_array_t, shape_t );

bool shape_array_read_csv(zCSV *csv, shape_array_t *sa, link_array_t *la)
{
  char linkname[MAX_BUFSIZ];
  int i;
  shape_t *shape;
  bool ret = true;

  zArrayAlloc( sa, shape_t, zCSVLineNum( csv ) );
  if( zArraySize(sa) == 0 ){
    ZALLOCERROR();
    return 0;
  }
  eprintf( "number of shapes = %d\n", zArraySize(sa) );
  zCSVRewind( csv );
  for( i=0; i<zArraySize(sa); i++ ){
    if( !zCSVGetLine( csv ) ) break;
    shape = zArrayElem(sa,i);
    zCSVGetField( csv, shape->name, MAX_BUFSIZ );
    zCSVGetField( csv, shape->stlname, MAX_BUFSIZ );
    zCSVGetField( csv, linkname, MAX_BUFSIZ );
    if( !( shape->link = link_array_find_link( la, linkname ) ) ){
      ZRUNERROR( "link %s (of %s) not found.", linkname, shape->name );
      ret = false;
    }
    zCSVGetDoubleN( csv, shape->frame.pos.e, 3 );
    zCSVGetDoubleN( csv, (double *)shape->frame.att.e, 9 );

    eprintf( "Read shape %s (link=%s)\n", shape->name, shape->link->name );
  }
  return ret;
}

bool shape_array_output_stlconv_sh(shape_array_t *sa, char *filename, char *stlorgdir, char *stldir)
{
  FILE *fp;
  char tstr[MAX_BUFSIZ];
  char rstr[MAX_BUFSIZ];
  zFrame3D f;
  shape_t *shape;
  int i;

  if( !( fp = fopen( filename, "w" ) ) ){
    ZOPENERROR( filename );
    return false;
  }
  fprintf( fp, "test -d %s || echo \"%s\" does not exist.\n", stlorgdir, stlorgdir );
  fprintf( fp, "mkdir %s\n", stldir );
  for( i=0; i<zArraySize(sa); i++ ){
    shape = zArrayElem(sa,i);
    zFrame3DXform( &shape->link->frame, &shape->frame, &f );
    sprintf( tstr, "\"%.10g %.10g %.10g\"", f.pos.c.x, f.pos.c.y, f.pos.c.z );
    sprintf( rstr, "\"%.10g %.10g %.10g %.10g %.10g %.10g %.10g %.10g %.10g\"",
      f.att.c.xx, f.att.c.xy, f.att.c.xz,
      f.att.c.yx, f.att.c.yy, f.att.c.yz,
      f.att.c.zx, f.att.c.zy, f.att.c.zz );
    fprintf( fp, "zeo_phconv -b -s 0.001 -t %s %s -r %s %s \"%s/%s.STL\" \"%s/%s.STL\"\n",
      tstr[1] == '-' ? "--" : "", tstr,
      rstr[1] == '-' ? "--" : "", rstr,
      stlorgdir, shape->stlname,
      stldir, shape->name );
  }
  fclose( fp );
  return true;
}

/* command line options */

enum{
  CAD2ZTK_NAME = 0,
  CAD2ZTK_LINK,
  CAD2ZTK_SHAPE,
  CAD2ZTK_STL_DIR,
  CAD2ZTK_STL_ORG_DIR,
  CAD2ZTK_CONV_SH,
  CAD2ZTK_URDF,
  CAD2ZTK_DOC,
  CAD2ZTK_HELP,
  CAD2ZTK_INVALID
};
zOption option[] = {
  { "n", "name",   "<model name>",  "name of model",                   NULL,         false },
  { "l", "link",   NULL,            "link information CSV file",       "link.csv",   false },
  { "s", "shape",  NULL,            "shape information CSV file",      "shape.csv",  false },
  { "d", "dir",    NULL,            "directory of output STL files",   "STL",        false },
  { "o", "orgdir", NULL,            "directory of original STL files", "STL_org",    false },
  { "c", "script", NULL,            "script file for STL conversion",  "stlconv.sh", false },
  { "urdf", "urdf", NULL,           "output URDF",                     NULL,         false },
  { "doc", "doc",  NULL,            "output a document of how to use", NULL,         false },
  { "h", "help",   NULL,            "show this message", NULL, false },
  { NULL, NULL, NULL, NULL, NULL, false },
};

void cad2ztk_usage(char *arg)
{
  eprintf( "Usage: %s [options] <model name>\n", arg );
  zOptionHelp( option );
  exit( EXIT_SUCCESS );
}

void cad2ztk_how_to_use(void)
{
  printf( "# How to make a ZTK/URDF file from Solidworks data in a systematic way\n\n" );
  printf( "## Process to create a ZTK file\n\n" );
  printf( " - Define link names, coordinate frames, and the home configuration\n" );
  printf( " - Export STL files\n" );
  printf( " - Make link.csv and share.csv\n" );
  printf( " - Run %% cad2ztk <model name>\n" );
  printf( " - Edit <model name>.ztk if you want\n\n" );
  printf( "## Links and coordinate frames\n\n" );
  printf( "The first thing you have to do is to define the names and connections of links.\n" );
  printf( "**Note that the model to be made is an open kinematic chain.**\n" );
  printf( "The world coordinate frame basically conforms to that defined in a CAD (e.g., Solidworks).\n" );
  printf( "The link coordinate frames have to be defined by yourself. Two rules for it are:\n\n" );
  printf( " - right-hand system\n" );
  printf( " - z-axis of each link aligned with the positive direction of the joint axis (the other axes may be defined at your convenience)\n\n" );
  printf( "## STL files\n\n" );
  printf( "The shapes of the links designed with the CAD can be usually exported to STL files.\n" );
  printf( "A good idea is to replace them by simplified versions rather than to directly use the original shapes.\n" );
  printf( "**IMPORTANT:** the STL files have to be output in the model coordinate system.\n\n" );
  printf( "## link.csv and shape.csv\n\n" );
  printf( "Make the following two CSV files.\n\n" );
  printf( "### link.csv:\n\n" );
  printf( "Each row of the file consists of the following fields.\n\n" );
  printf( " - Name of link\n" );
  printf( " - Name of parent link\n" );
  printf( " - type of joint (fixed/revolute/prismatic/cylindrical/hooke/spherical/float)\n" );
  printf( " - Xo, Yo, Zo: position coordinates of the origin of the link frame in the world frame\n" );
  printf( " - Rxx ... Rzz: attitude matrix of the link frame with respect to the world frame\n" );
  printf( "   - for example, Ryx stands for the x component of the y base vector\n" );
  printf( " - m: mass of the link\n" );
  printf( " - X, Y, Z: position coordinates of the center of mass of the link in the world frame\n" );
  printf( " - Lxx, Lxy, Lxz, Lyy, Lyz, Lzz: components of the inertial tensor about the center of mass with respect to the world frame\n\n" );
  printf( "**Note:** Zo has a degree of freedom since it can translate along the z-axis.\n" );
  printf( "You may choose any point on the axis as the origin of the link frame.\n" );
  printf( "Rxx - Rzz components have to be found by yourself.\n\n" );
  printf( "### shape.csv:\n\n" );
  printf( "Each row consists of the following fields.\n\n" );
  printf( " - Name of shape: to be attached to the links in ZTK file\n" );
  printf( " - original STL: a file exported from the CAD\n" );
  printf( " - Name of link: a link that the shape to be attached to\n" );
  printf( " - Xo, Yo, Zo: position of the origin of the part in the world frame\n" );
  printf( " - Rxx - Rzz: attitude matrix of the part with respect to the world frame\n\n" );
  printf( "You may define any \"Name of shape\" as long as their uniquenesses remain.\n" );
  printf( "On the other hand, \"original STL\" has to be the same with the names of the exported STL files, where the suffix \".STL\" has to be omitted.\n\n" );
  printf( "## cad2ztk\n\n" );
  printf( "Put link.csv, shape.csv and STL_org/ in the same directory, where the original STL files are stored under STL_org/.\n" );
  printf( "Then, run %% cad2ztk <name>\n" );
  printf( "As the result, <name>.ztk and a shellscript stlconv.sh are created in the same directory.\n" );
  printf( "The original STL files under STL_org/ are converted to ones referred from <name>.ztk under a directory STL/, which is also newly created, by running stlconv.sh.\n" );
  exit( 0 );
}

bool cad2ztk_read_cmdarg(char **argv)
{
  zStrAddrList arglist;
  char *chainname;

  zOptionRead( option, argv+1, &arglist );
  if( option[CAD2ZTK_HELP].flag ) cad2ztk_usage( argv[0] );
  if( option[CAD2ZTK_DOC].flag ) cad2ztk_how_to_use();
  zStrListGetPtr( &arglist, 1, &chainname );
  if( chainname ){
    option[CAD2ZTK_NAME].flag = true;
    option[CAD2ZTK_NAME].arg  = chainname;
  }
  zStrAddrListDestroy( &arglist );
  if( !option[CAD2ZTK_NAME].flag ){
    ZRUNERROR( "Model name not specified." );
    return false;
  }
  return true;
}

/* output URDF */

void cad2ztk_output_urdf_vec(FILE *fp, zVec3D *v)
{
  fprintf( fp, "\"%.10g %.10g %.10g\"", v->c.x, v->c.y, v->c.z );
}

char *cad2ztk_output_urdf_jointtype(char *str)
{
  static char *jointtype[] = {
    "fixed",
    "revolute",
    "prismatic",
    "float",
    NULL,
  };
  char **cp;

  for( cp=jointtype; *cp; cp++ ){
    if( strcmp( *cp, str ) == 0 ) return *cp;
  }
  return NULL;
}

void cad2ztk_output_urdf_link_shape(FILE *fp, shape_array_t *sa, link_t *link)
{
  int i;
  shape_t *shape;

  for( i=0; i<zArraySize(sa); i++ ){
    shape = zArrayElem(sa,i);
    if( shape->link != link ) continue;
    fprintf( fp, "    <visual>\n" );
    fprintf( fp, "      <geometry>\n" );
    fprintf( fp, "        <mesh filename=\"%s/%s.STL\"/>\n", option[CAD2ZTK_STL_DIR].arg, shape->name );
    fprintf( fp, "      </geometry>\n" );
    fprintf( fp, "      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n" );
    fprintf( fp, "    </visual>\n" );
  }
}

void cad2ztk_output_urdf_link(FILE *fp, link_array_t *la, shape_array_t *sa)
{
  int i;
  link_t *link;
  rkMP mp;

  for( i=0; i<zArraySize(la); i++ ){
    link = zArrayElem(la,i);
    fprintf( fp, "  <link name=\"%s\">\n", link->name );
    cad2ztk_output_urdf_link_shape( fp, sa, link );
    rkMPXformInv( &link->mp, &link->frame, &mp );
    rkMPgmm2kgm( &mp );
    fprintf( fp, "    <inertial>\n" );
    fprintf( fp, "      <mass value=\"%.10g\"/>\n", mp.mass );
    fprintf( fp, "      <origin xyz=" );
    cad2ztk_output_urdf_vec( fp, &mp.com );
    fprintf( fp, "/>\n" );
    fprintf( fp, "      <inertia ixx=\"%.10g\" ixy=\"%.10g\" ixz=\"%.10g\" iyy=\"%.10g\" iyz=\"%.10g\" izz=\"%.10g\"/>\n",
      mp.inertia.c.xx, mp.inertia.c.xy, mp.inertia.c.xz, mp.inertia.c.yy, mp.inertia.c.yz, mp.inertia.c.zz );
    fprintf( fp, "    </inertial>\n" );
    fprintf( fp, "  </link>\n" );
  }
}

void cad2ztk_output_urdf_joint(FILE *fp, link_array_t *la)
{
  int i;
  link_t *link;
  zFrame3D f;
  zVec3D xyz, rpy;

  for( i=0; i<zArraySize(la); i++ ){
    link = zArrayElem(la,i);
    fprintf( fp, "  <joint name=\"%s\" type=\"%s\">\n", link->name, cad2ztk_output_urdf_jointtype( link->jointtype ) );
    fprintf( fp, "    <parent link=\"%s\"/>\n", link->parentname );
    fprintf( fp, "    <child link=\"%s\"/>\n", link->name );
    if( link->parent )
      zFrame3DXform( &link->parent->frame, &link->frame, &f );
    else
      zFrame3DCopy( &link->frame, &f );
    zVec3DMul( zFrame3DPos(&f), 1.0e-3, &xyz );
    zMat3DToZYX( zFrame3DAtt(&f), &rpy );
    fprintf( fp, "    <origin xyz=" );
    cad2ztk_output_urdf_vec( fp, &xyz );
    fprintf( fp, " rpy=" );
    cad2ztk_output_urdf_vec( fp, &rpy );
    fprintf( fp, "/>\n" );
    fprintf( fp, "    <axis xyz=\"0 0 1\"/>\n" );
    fprintf( fp, "  </joint>\n" );
  }
}

bool cad2ztk_output_urdf(char *name, link_array_t *la, shape_array_t *sa)
{
  FILE *fp;
  char filename[BUFSIZ];

  sprintf( filename, "%s.urdf", name );
  if( !( fp = fopen( filename, "w" ) ) ){
    ZOPENERROR( filename );
    return false;
  }
  fprintf( fp, "<?xml version=\"1.0\"?>\n" );
  fprintf( fp, "<robot name=\"%s\">\n", name );
  cad2ztk_output_urdf_link( fp, la, sa );
  cad2ztk_output_urdf_joint( fp, la );
  fprintf( fp, "</robot>\n" );
  fclose( fp );
  return true;
}

/* output ZTK */

void cad2ztk_output_ztk_shape(FILE *fp, shape_array_t *sa)
{
  shape_t *shape;
  int i;

  for( i=0; i<zArraySize(sa); i++ ){
    shape = zArrayElem(sa,i);
    fprintf( fp, "[shape]\n" );
    fprintf( fp, "name: \"%s\"\n", shape->name );
    fprintf( fp, "optic: white\n" );
    fprintf( fp, "import: \"%s/%s.STL\" 1.0\n\n", option[CAD2ZTK_STL_DIR].arg, shape->name );
  }
}

void cad2ztk_output_ztk_link_shape(FILE *fp, shape_array_t *sa, link_t *link)
{
  int i;
  shape_t *shape;

  for( i=0; i<zArraySize(sa); i++ ){
    shape = zArrayElem(sa,i);
    if( shape->link != link ) continue;
    fprintf( fp, "shape: \"%s\"\n", shape->name );
  }
}

void cad2ztk_output_ztk_link(FILE *fp, link_array_t *la, shape_array_t *sa)
{
  int i;
  link_t *link;
  zFrame3D f;
  rkMP mp;

  for( i=0; i<zArraySize(la); i++ ){
    link = zArrayElem(la,i);
    fprintf( fp, "[link]\n" );
    fprintf( fp, "name: %s\n", link->name );
    fprintf( fp, "jointtype: %s\n", link->jointtype );
    fprintf( fp, "frame: " );
    if( link->parent )
      zFrame3DXform( &link->parent->frame, &link->frame, &f );
    else
      zFrame3DCopy( &link->frame, &f );
    zVec3DMulDRC( zFrame3DPos(&f), 1.0e-3 );
    zFrame3DFPrint( fp, &f );
    rkMPXformInv( &link->mp, &link->frame, &mp );
    rkMPgmm2kgm( &mp );
    rkMPFPrint( fp, &mp );
    cad2ztk_output_ztk_link_shape( fp, sa, link );
    if( link->parent )
      fprintf( fp, "parent: %s\n", link->parent->name );
    fprintf( fp, "\n" );
  }
}

bool cad2ztk_output_ztk(char *name, link_array_t *la, shape_array_t *sa)
{
  FILE *fp;
  char filename[BUFSIZ];

  sprintf( filename, "%s.ztk", name );
  if( !( fp = fopen( filename, "w" ) ) ){
    ZOPENERROR( filename );
    return false;
  }
  fprintf( fp, "[chain]\n" );
  fprintf( fp, "name: %s\n\n", name );
  fprintf( fp, "[optic]\n" );
  fprintf( fp, "name: white\n" );
  fprintf( fp, "ambient:  1.0, 1.0, 1.0\n" );
  fprintf( fp, "diffuse:  1.0, 1.0, 1.0\n" );
  fprintf( fp, "specular: 0.0, 0.0, 0.0\n" );
  fprintf( fp, "esr: 1.0\n\n" );
  cad2ztk_output_ztk_shape( fp, sa );
  cad2ztk_output_ztk_link( fp, la, sa );
  fclose( fp );
  return true;
}

int main(int argc, char *argv[])
{
  zCSV csv;
  link_array_t la;
  shape_array_t sa;

  cad2ztk_how_to_use();

  if( argc < 2 ) cad2ztk_usage( argv[0] );
  if( !cad2ztk_read_cmdarg( argv ) ) return EXIT_FAILURE;

  if( !zCSVOpen( &csv, option[CAD2ZTK_LINK].arg ) ) return EXIT_FAILURE;
  zArrayInit( &la );
  if( link_array_read_csv( &csv, &la ) == 0 ) return EXIT_FAILURE;
  zCSVClose( &csv );
  if( !link_array_assoc_parent( &la ) ) goto TERMINATE;

  if( !zCSVOpen( &csv, option[CAD2ZTK_SHAPE].arg ) ) return EXIT_FAILURE;
  zArrayInit( &sa );
  shape_array_read_csv( &csv, &sa, &la );
  zCSVClose( &csv );
  shape_array_output_stlconv_sh( &sa, option[CAD2ZTK_CONV_SH].arg, option[CAD2ZTK_STL_ORG_DIR].arg, option[CAD2ZTK_STL_DIR].arg );

  if( option[CAD2ZTK_URDF].flag )
    cad2ztk_output_urdf( option[CAD2ZTK_NAME].arg, &la, &sa );
  else
    cad2ztk_output_ztk( option[CAD2ZTK_NAME].arg, &la, &sa );
  if( zArraySize(&sa) > 0 ) zArrayFree( &sa );

 TERMINATE:
  zArrayFree( &la );
  return EXIT_SUCCESS;
}
