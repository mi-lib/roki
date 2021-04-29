#include <roki/roki.h>

/* *** structures to store information *** */

#define list_find(type) \
  type##_t *type##_list_find(type##_list_t *list, char *name){\
    type##_cell_t *cp;\
    zListForEach( list, cp ){\
      if( strcmp( cp->data.name, name ) == 0 ) return &cp->data;\
    }\
    return NULL;\
  }

typedef zOpticalInfo material_t;
zListClass( material_list_t, material_cell_t, material_t );
list_find( material )

typedef struct{
  char *name;
  char *type;
  zFrame3D fl, fw;
  char *property1;
  char *property2;
  char *material;
} shape_t;
zListClass( shape_list_t, shape_cell_t, shape_t );
list_find( shape )
zListClass( shape_p_list_t, shape_p_cell_t, shape_t* );

void shape_list_destroy(shape_list_t *list)
{
  shape_cell_t *sc;

  while( !zListIsEmpty( list ) ){
    zListDeleteHead( list, &sc );
    free( sc->data.name );
    free( sc );
  }
}

struct _joint_t;

typedef struct{
  char *name;
  rkMP mp;
  zMat3D _iw;
  shape_p_list_t shape_p_list;
  struct _joint_t *joint;
  struct _joint_t *child;
} link_t;
zListClass( link_list_t, link_cell_t, link_t );
list_find( link )

void link_list_destroy(link_list_t *list)
{
  link_cell_t *lc;

  while( !zListIsEmpty(list) ){
    zListDeleteHead( list, &lc );
    zListDestroy( shape_p_cell_t, &lc->data.shape_p_list );
    free( lc );
  }
}

typedef struct _joint_t{
  char *name;
  char *type;
  zFrame3D fl, fw;
  zVec3D axis;
  char *damping;
  char *friction;
  char *lower;
  char *upper;
  link_t *parent;
  link_t *child;
  struct _joint_t *sibling;
} joint_t;
zListClass( joint_list_t, joint_cell_t, joint_t );

typedef struct{
  char *name;
  material_list_t material_list;
  shape_list_t shape_list;
  link_list_t link_list;
  joint_list_t joint_list;
} robot_info_t;

void robot_info_init(robot_info_t *robot_info)
{
  robot_info->name = NULL;
  zListInit( &robot_info->material_list );
  zListInit( &robot_info->shape_list );
  zListInit( &robot_info->link_list );
  zListInit( &robot_info->joint_list );
}

void robot_info_destroy(robot_info_t *robot_info)
{
  robot_info->name = NULL;
  zListDestroy( material_cell_t, &robot_info->material_list );
  shape_list_destroy( &robot_info->shape_list );
  link_list_destroy( &robot_info->link_list );
  zListDestroy( joint_cell_t, &robot_info->joint_list );
}

/* evaluate frame information */

void urdf2ztk_get_vec(char *str, zVec3D *v)
{
  sscanf( str, "%lf %lf %lf", &v->c.x, &v->c.y, &v->c.z );
}

void urdf2ztk_get_att(char *str, zMat3D *m)
{
  double roll, pitch, yaw;

  sscanf( str, "%lf %lf %lf", &roll, &pitch, &yaw );
  zMat3DFromZYX( m, yaw, pitch, roll );
}

bool urdf2ztk_eval_origin(xmlNode *node, zFrame3D *f)
{
  xmlAttr *attr;

  zXMLForEachAttr( node, attr ){
    zXMLCheckAttrAndExec( attr, "xyz", urdf2ztk_get_vec( zXMLGetAttrVal( attr ), zFrame3DPos(f) ) ) else
    zXMLCheckAttrAndExec( attr, "rpy", urdf2ztk_get_att( zXMLGetAttrVal( attr ), zFrame3DAtt(f) ) );
  }
  return true;
}

/* *** evaluate material information *** */

void urdf2ztk_material_get_rgba(char *str, material_cell_t *mc)
{
  sscanf( str, "%f %f %f %lf", &mc->data.dif.r, &mc->data.dif.g, &mc->data.dif.b, &mc->data.alpha );
}

bool urdf2ztk_eval_color(xmlNode *node, material_cell_t *mc)
{
  urdf2ztk_material_get_rgba( zXMLFindNodeAttr( node, "rgba" ), mc );
  return true;
}

bool urdf2ztk_eval_material(xmlNode *node, robot_info_t *robot_info)
{
  xmlNode *np;
  material_cell_t *mc;

  if( !( mc = zAlloc( material_cell_t, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  zOpticalInfoInit( &mc->data );
  zRGBSet( &mc->data.amb, 0.5, 0.5, 0.5 );
  zListInsertHead( &robot_info->material_list, mc );
  zNameSetPtr( &mc->data, zXMLFindNodeAttr( node, "name" ) );
  zXMLForEachNode( node->children, np ){
    zXMLCheckElementAndExec( np, "color", urdf2ztk_eval_color( np, mc ) );
    /* NOTE: texture unsupported. */
  }
  return true;
}

/* *** evaluate link information *** */

/* evaluate mass property */

bool urdf2ztk_eval_link_mass(xmlNode *node, link_cell_t *lc)
{
  lc->data.mp.mass = atof( zXMLFindNodeAttr( node, "value" ) );
  return true;
}

bool urdf2ztk_eval_link_inertia(xmlNode *node, link_cell_t *lc)
{
  xmlAttr *attr;
  double ixx, ixy, ixz, iyy, iyz, izz;

  ixx = ixy = ixz = iyy = iyz = izz = 0;
  zXMLForEachAttr( node, attr ){
    zXMLCheckAttrAndExec( attr, "ixx", ixx = zXMLGetAttrDouble( attr ) ) else
    zXMLCheckAttrAndExec( attr, "ixy", ixy = zXMLGetAttrDouble( attr ) ) else
    zXMLCheckAttrAndExec( attr, "ixz", ixz = zXMLGetAttrDouble( attr ) ) else
    zXMLCheckAttrAndExec( attr, "iyy", iyy = zXMLGetAttrDouble( attr ) ) else
    zXMLCheckAttrAndExec( attr, "iyz", iyz = zXMLGetAttrDouble( attr ) ) else
    zXMLCheckAttrAndExec( attr, "izz", izz = zXMLGetAttrDouble( attr ) );
  }
  zMat3DCreate( &lc->data.mp.inertia, ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz );
  return true;
}

bool urdf2ztk_eval_link_inertial(xmlNode *node, link_cell_t *lc)
{
  xmlNode *np;
  zFrame3D f;

  zFrame3DIdent( &f );
  zXMLForEachNode( node->children, np ){
    zXMLCheckElementAndExec( np, "origin",  urdf2ztk_eval_origin( np, &f ) ) else
    zXMLCheckElementAndExec( np, "mass",    urdf2ztk_eval_link_mass( np, lc ) ) else
    zXMLCheckElementAndExec( np, "inertia", urdf2ztk_eval_link_inertia( np, lc ) );
  }
  zVec3DCopy( zFrame3DPos(&f), &lc->data.mp.com );
  zRotMat3DDRC( zFrame3DAtt(&f), &lc->data.mp.inertia );
  return true;
}

/* evaluate shape geometry */

bool urdf2ztk_eval_shape_box(xmlNode *node, shape_cell_t *sc)
{
  sc->data.type = (char *)node->name;
  sc->data.property1 = zXMLFindNodeAttr( node, "size" );
  return true;
}

bool urdf2ztk_eval_shape_sphere(xmlNode *node, shape_cell_t *sc)
{
  sc->data.type = (char *)node->name;
  sc->data.property1 = zXMLFindNodeAttr( node, "radius" );
  return true;
}

bool urdf2ztk_eval_shape_cylinder(xmlNode *node, shape_cell_t *sc)
{
  xmlAttr *attr;

  sc->data.type = (char *)node->name;
  zXMLForEachAttr( node, attr ){
    zXMLCheckAttrAndExec( attr, "radius", sc->data.property1 = zXMLGetAttrVal( attr ) ) else
    zXMLCheckAttrAndExec( attr, "length", sc->data.property2 = zXMLGetAttrVal( attr ) );
  }
  return true;
}

bool urdf2ztk_eval_shape_mesh(xmlNode *node, shape_cell_t *sc)
{
  xmlAttr *attr;

  sc->data.type = (char *)node->name;
  zXMLForEachAttr( node, attr ){
    zXMLCheckAttrAndExec( attr, "filename", sc->data.property1 = zXMLGetAttrVal( attr ) ) else
    zXMLCheckAttrAndExec( attr, "scale",    sc->data.property2 = zXMLGetAttrVal( attr ) );
  }
  return true;
}

bool urdf2ztk_eval_shape_geometry(xmlNode *node, shape_cell_t *sc)
{
  xmlNode *np;

  zXMLForEachNode( node, np ){
    zXMLCheckElementAndExec( np, "box",      urdf2ztk_eval_shape_box( np, sc ) ) else
    zXMLCheckElementAndExec( np, "sphere",   urdf2ztk_eval_shape_sphere( np, sc ) ) else
    zXMLCheckElementAndExec( np, "cylinder", urdf2ztk_eval_shape_cylinder( np, sc ) ) else
    zXMLCheckElementAndExec( np, "mesh",     urdf2ztk_eval_shape_mesh( np, sc ) );
  }
  return true;
}

bool urdf2ztk_eval_shape_material(xmlNode *node, robot_info_t *robot_info, shape_cell_t *sc)
{
  sc->data.material = zXMLFindNodeAttr( node, "name" );
  return node->children ? urdf2ztk_eval_material( node, robot_info ) : true;
}

/* evaluate visual property of a link */

bool urdf2ztk_eval_link_visual(xmlNode *node, robot_info_t *robot_info, link_cell_t *lc)
{
  xmlNode *np;
  shape_cell_t *sc;
  shape_p_cell_t *spc;
  char buf[BUFSIZ];

  if( !( sc = zAlloc( shape_cell_t, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  sc->data.name = NULL;
  zFrame3DIdent( &sc->data.fl );
  zFrame3DIdent( &sc->data.fw );
  sc->data.property1 = sc->data.property2 = NULL;
  sc->data.material = NULL;
  zListInsertHead( &robot_info->shape_list, sc );
  sc->data.name = zStrClone( zXMLFindNodeAttr( node, "name" ) );
  if( !sc->data.name ){
    sprintf( buf, "%s_%d", lc->data.name, zListSize(&lc->data.shape_p_list) );
    if( !( sc->data.name = zStrClone( buf ) ) ){
      ZALLOCERROR();
      return false;
    }
  }
  if( !( spc = zAlloc( shape_p_cell_t, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  spc->data = &sc->data;
  zListInsertHead( &lc->data.shape_p_list, spc );
  zXMLForEachNode( node->children, np ){
    zXMLCheckElementAndExec( np, "origin",   urdf2ztk_eval_origin( np, &sc->data.fl ) ) else
    zXMLCheckElementAndExec( np, "geometry", urdf2ztk_eval_shape_geometry( np->children, sc ) ) else
    zXMLCheckElementAndExec( np, "material", urdf2ztk_eval_shape_material( np, robot_info, sc ) );
  }
  return true;
}

bool urdf2ztk_eval_link(xmlNode *node, robot_info_t *robot_info)
{
  xmlNode *np;
  link_cell_t *lc;

  if( !( lc = zAlloc( link_cell_t, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  lc->data.name = NULL;
  zListInit( &lc->data.shape_p_list );
  lc->data.joint = NULL;
  lc->data.child = NULL;
  zListInsertHead( &robot_info->link_list, lc );
  lc->data.name = zXMLFindNodeAttr( node, "name" );
  zXMLForEachNode( node->children, np ){
    zXMLCheckElementAndExec( np, "inertial", urdf2ztk_eval_link_inertial( np, lc ) ) else
    zXMLCheckElementAndExec( np, "visual",   urdf2ztk_eval_link_visual( np, robot_info, lc ) );
    /* collision and contact are ignored. */
  }
  return true;
}

/* *** evaluate joint information *** */

joint_t *urdf2ztk_add_joint_sibling(joint_t *joint, joint_t *sibling)
{
  if( !joint->sibling ) return joint->sibling = sibling;
  return urdf2ztk_add_joint_sibling( joint->sibling, sibling );
}

joint_t *urdf2ztk_add_link_child(link_t *link, joint_t *child)
{
  if( !link->child ) return link->child = child;
  return urdf2ztk_add_joint_sibling( link->child, child );
}

void urdf2ztk_bind_joint_parent(robot_info_t *robot_info, char *str, joint_cell_t *jc)
{
  jc->data.parent = link_list_find( &robot_info->link_list, str );
  urdf2ztk_add_link_child( jc->data.parent, &jc->data );
}

void urdf2ztk_bind_joint_child(robot_info_t *robot_info, char *str, joint_cell_t *jc)
{
  if( ( jc->data.child = link_list_find( &robot_info->link_list, str ) ) )
    jc->data.child->joint = &jc->data;
}

bool  urdf2ztk_eval_joint_parent(xmlNode *node, robot_info_t *robot_info, joint_cell_t *jc)
{
  urdf2ztk_bind_joint_parent( robot_info, zXMLFindNodeAttr( node, "link" ), jc );
  return true;
}

bool  urdf2ztk_eval_joint_child(xmlNode *node, robot_info_t *robot_info, joint_cell_t *jc)
{
  urdf2ztk_bind_joint_child( robot_info, zXMLFindNodeAttr( node, "link" ), jc );
  return true;
}

bool  urdf2ztk_eval_joint_axis(xmlNode *node, joint_cell_t *jc)
{
  urdf2ztk_get_vec( zXMLFindNodeAttr( node, "xyz" ), &jc->data.axis );
  return true;
}

bool  urdf2ztk_eval_joint_dynamic(xmlNode *node, joint_cell_t *jc)
{
  xmlAttr *attr;

  zXMLForEachAttr( node, attr ){
    zXMLCheckAttrAndExec( attr, "damping",  jc->data.damping = zXMLGetAttrVal( attr ) ) else
    zXMLCheckAttrAndExec( attr, "friction", jc->data.friction = zXMLGetAttrVal( attr ) );
  }
  return true;
}

bool  urdf2ztk_eval_joint_limit(xmlNode *node, joint_cell_t *jc)
{
  xmlAttr *attr;

  zXMLForEachAttr( node, attr ){
    zXMLCheckAttrAndExec( attr, "lower", jc->data.lower = zXMLGetAttrVal( attr ) ) else
    zXMLCheckAttrAndExec( attr, "upper", jc->data.upper = zXMLGetAttrVal( attr ) );
    /* effort and velocity are ignored. */
  }
  return true;
}

bool urdf2ztk_eval_joint(xmlNode *node, robot_info_t *robot_info)
{
  xmlNode *np;
  xmlAttr *attr;
  joint_cell_t *jc;

  if( !( jc = zAlloc( joint_cell_t, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  jc->data.name = NULL;
  jc->data.type = NULL;
  zFrame3DIdent( &jc->data.fl );
  zFrame3DIdent( &jc->data.fw );
  zVec3DCopy( ZVEC3DX, &jc->data.axis );
  jc->data.damping = NULL;
  jc->data.friction = NULL;
  jc->data.lower = NULL;
  jc->data.upper = NULL;
  jc->data.parent = jc->data.child = NULL;
  jc->data.sibling = NULL;
  zListInsertHead( &robot_info->joint_list, jc );
  zXMLForEachAttr( node, attr ){
    zXMLCheckAttrAndExec( attr, "name", jc->data.name = zXMLGetAttrVal( attr ) ) else
    zXMLCheckAttrAndExec( attr, "type", jc->data.type = zXMLGetAttrVal( attr ) );
  }
  zXMLForEachNode( node->children, np ){
    zXMLCheckElementAndExec( np, "origin",  urdf2ztk_eval_origin( np, &jc->data.fl ) ) else
    zXMLCheckElementAndExec( np, "parent",  urdf2ztk_eval_joint_parent( np, robot_info, jc ) ) else
    zXMLCheckElementAndExec( np, "child",   urdf2ztk_eval_joint_child( np, robot_info, jc ) ) else
    zXMLCheckElementAndExec( np, "axis",    urdf2ztk_eval_joint_axis( np, jc ) ) else
    zXMLCheckElementAndExec( np, "dynamic", urdf2ztk_eval_joint_dynamic( np, jc ) ) else
    zXMLCheckElementAndExec( np, "limit",   urdf2ztk_eval_joint_limit( np, jc ) );
  }
  return true;
}

/* *** evaluate robot model information *** */

bool urdf2ztk_eval_robot(xmlNode *node, robot_info_t *robot_info)
{
  xmlNode *np;

  zXMLForEachNode( node, np ){
    zXMLCheckElementAndExec( np, "material",
      urdf2ztk_eval_material( np, robot_info ) );
  }
  zXMLForEachNode( node, np ){
    zXMLCheckElementAndExec( np, "link",
      urdf2ztk_eval_link( np, robot_info ) );
  }
  zXMLForEachNode( node, np ){
    zXMLCheckElementAndExec( np, "joint",
      urdf2ztk_eval_joint( np, robot_info ) );
  }
  return true;
}

bool urdf2ztk_eval(xmlNode *node, robot_info_t *robot_info)
{
  xmlNode *np;

  if( !( np = zXMLFindNodeElement( node, "robot" ) ) )
    return false;
  robot_info->name = zXMLFindNodeAttr( np, "name" );
  return urdf2ztk_eval_robot( np->children, robot_info );
}

/* *** calculate transformations *** */

bool urdf2ztk_xform(joint_t *joint)
{
  shape_p_cell_t *spc;

  if( !joint->parent || !joint->child ){
    ZRUNERROR( "unconnected joint %s", joint->name );
    return false;
  }
  if( !joint->parent->joint ) /* root link */
    zFrame3DCopy( &joint->fl, &joint->fw );
  else
    zFrame3DCascade( &joint->parent->joint->fw, &joint->fl, &joint->fw );
  zRotMat3D( zFrame3DAtt(&joint->fw), &joint->child->mp.inertia, &joint->child->_iw );
  zListForEach( &joint->child->shape_p_list, spc ){
    zFrame3DCascade( &joint->fw, &spc->data->fl, &spc->data->fw );
  }
  if( joint->child->child )
    if( !urdf2ztk_xform( joint->child->child ) ) return false;
  return joint->sibling ? urdf2ztk_xform( joint->sibling ) : true;
}

bool urdf2ztk_xform_inv(joint_t *joint)
{
  shape_p_cell_t *spc;

  if( !joint->parent || !joint->child ){
    ZRUNERROR( "unconnected joint %s", joint->name );
    return false;
  }
  if( joint->parent->joint )
    zFrame3DXform( &joint->parent->joint->fw, &joint->fw, &joint->fl );
  zRotMat3DInv( zFrame3DAtt(&joint->fl), &joint->child->_iw, &joint->child->mp.inertia );
  zListForEach( &joint->child->shape_p_list, spc ){
    zFrame3DXform( &joint->fw, &spc->data->fw, &spc->data->fl );
  }
  if( joint->child->child )
    if( !urdf2ztk_xform_inv( joint->child->child ) ) return false;
  return joint->sibling ? urdf2ztk_xform_inv( joint->sibling ) : true;
}

link_t *urdf2ztk_find_root(robot_info_t *robot_info)
{
  link_cell_t *lc;

  zListForEach( &robot_info->link_list, lc )
    if( !lc->data.joint ) return &lc->data;
  return NULL;
}

bool urdf2ztk_joint_is_fixed(joint_t *joint)
{
  return strcmp( joint->type, "fixed" ) == 0;
}

bool urdf2ztk_joint_is_revol(joint_t *joint)
{
  return strcmp( joint->type, "revolute" ) == 0 || strcmp( joint->type, "continuous" ) == 0;
}

bool urdf2ztk_joint_is_prism(joint_t *joint)
{
  return strcmp( joint->type, "prismatic" ) == 0;
}

bool urdf2ztk_joint_is_float(joint_t *joint)
{
  return strcmp( joint->type, "float" ) == 0;
}

bool urdf2ztk_correct_frame(robot_info_t *robot_info)
{
  link_t *root;
  joint_cell_t *jc;
  zVec3D aa;

  if( !( root = urdf2ztk_find_root( robot_info ) ) ){
    ZRUNERROR( "root link not found" );
    return false;
  }
  if( root->child ) urdf2ztk_xform( root->child );
  zListForEach( &robot_info->joint_list, jc ){
    if( urdf2ztk_joint_is_revol( &jc->data ) || urdf2ztk_joint_is_prism( &jc->data ) ){
      zVec3DAAError( ZVEC3DZ, &jc->data.axis, &aa );
      zMat3DRotDRC( zFrame3DAtt(&jc->data.fw), &aa );
    }
  }
  if( root->child ) urdf2ztk_xform_inv( root->child );
  return true;
}

/* *** output ZTK file *** */

void output_ztk_material(FILE *fp, material_list_t *material_list)
{
  material_cell_t *mc;

  zListForEach( material_list, mc ){
    fprintf( fp, "[%s]\n", ZTK_TAG_OPTIC );
    zOpticalInfoFPrintZTK( fp, &mc->data );
  }
}

void output_ztk_shape_box(FILE *fp, shape_cell_t *sc)
{
  double width, height, depth;

  fprintf( fp, "type: box\n" );
  sscanf( sc->data.property1, "%lf %lf %lf", &depth, &width, &height );
  fprintf( fp, "depth: %.10g\n", depth );
  fprintf( fp, "width: %.10g\n", width );
  fprintf( fp, "height: %.10g\n", height );
}

void output_ztk_shape_sphere(FILE *fp, shape_cell_t *sc)
{
  double radius;

  fprintf( fp, "type: sphere\n" );
  sscanf( sc->data.property1, "%lf", &radius );
  fprintf( fp, "radius: %.10g\n", radius );
}

void output_ztk_shape_cylinder(FILE *fp, shape_cell_t *sc)
{
  double radius, length;

  fprintf( fp, "type: cylinder\n" );
  sscanf( sc->data.property1, "%lf", &radius );
  sscanf( sc->data.property2, "%lf", &length );
  fprintf( fp, "center: ( 0, 0, %.10g )\n", length/2 );
  fprintf( fp, "center: ( 0, 0,-%.10g )\n", length/2 );
  fprintf( fp, "radius: %.10g\n", radius );
}

void output_ztk_shape_mesh(FILE *fp, shape_cell_t *sc)
{
  fprintf( fp, "import: %s", sc->data.property1 );
  if( sc->data.property2 ) /* scale */
    fprintf( fp, " %s", sc->data.property2 );
  fprintf( fp, "\n" );
}

void output_ztk_shape(FILE *fp, shape_list_t *shape_list)
{
  shape_cell_t *sc;

  zListForEach( shape_list, sc ){
    if( !sc->data.type ){
      ZRUNWARN( "Unnamed shape. Skipped" );
      continue;
    }
    fprintf( fp, "[shape]\n" );
    fprintf( fp, "name: %s\n", sc->data.name );
    if( !zVec3DIsTiny( zFrame3DPos(&sc->data.fl) ) ){
      fprintf( fp, "pos: " );
      zVec3DFPrint( fp, zFrame3DPos(&sc->data.fl) );
    }
    if( !zMat3DIsIdent( zFrame3DAtt(&sc->data.fl) ) ){
      fprintf( fp, "att: " );
      zMat3DFPrint( fp, zFrame3DAtt(&sc->data.fl) );
    }
    if( sc->data.material ) fprintf( fp, "optic: %s\n", sc->data.material );
    if( strcmp( sc->data.type, "box" ) == 0 ) output_ztk_shape_box( fp, sc ); else
    if( strcmp( sc->data.type, "sphere" ) == 0 ) output_ztk_shape_sphere( fp, sc ); else
    if( strcmp( sc->data.type, "cylinder" ) == 0 ) output_ztk_shape_cylinder( fp, sc ); else
    if( strcmp( sc->data.type, "mesh" ) == 0 ) output_ztk_shape_mesh( fp, sc ); else{
      ZRUNWARN( "Unknown type of a shape: %s", sc->data.type );
    }
    fprintf( fp, "\n" );
  }
}

/* only for debug */
void output_ztk_joint_child(FILE *fp, joint_t *child)
{
  fprintf( fp, "child: %s\n", child->name );
  if( child->sibling )
    output_ztk_joint_child( fp, child->sibling );
}

void output_ztk_joint(FILE *fp, joint_t *joint)
{
  double lower, upper;

  lower = upper = 0;
  if( joint->type ){
    if( urdf2ztk_joint_is_revol( joint ) ){
      fprintf( fp, "jointtype: revolute\n" );
      if( joint->lower ) lower = zRad2Deg( atof( joint->lower ) );
      if( joint->upper ) upper = zRad2Deg( atof( joint->upper ) );
    } else
    if( urdf2ztk_joint_is_prism( joint ) ){
      fprintf( fp, "jointtype: prismatic\n" );
      if( joint->lower ) lower = atof( joint->lower );
      if( joint->upper ) upper = atof( joint->upper );
    } else
    if( urdf2ztk_joint_is_float( joint ) )
      fprintf( fp, "jointtype: float\n" );
    else
      fprintf( fp, "jointtype: fixed\n" );
  }
  if( joint->lower )    fprintf( fp, "min: %.10g\n",    lower );
  if( joint->upper )    fprintf( fp, "max: %.10g\n",    upper );
  if( joint->damping )  fprintf( fp, "viscosity: %s\n", joint->damping );
  if( joint->friction ) fprintf( fp, "coulomb: %s\n",   joint->friction );
  if( !zVec3DIsTiny( zFrame3DPos(&joint->fl) ) ){
    fprintf( fp, "pos: " );
    zVec3DFPrint( fp, zFrame3DPos(&joint->fl) );
  }
  if( !zMat3DIsIdent( zFrame3DAtt(&joint->fl) ) ){
    fprintf( fp, "att: " );
    zMat3DFPrint( fp, zFrame3DAtt(&joint->fl) );
  }
}

void output_ztk_link(FILE *fp, link_list_t *link_list)
{
  link_cell_t *lc;
  shape_p_cell_t *spc;

  zListForEach( link_list, lc ){
    fprintf( fp, "[link]\n" );
    if( lc->data.name )   fprintf( fp, "name: %s\n", lc->data.name );
    if( lc->data.joint )
      output_ztk_joint( fp, lc->data.joint );
    if( lc->data.mp.mass != 0 ){
      fprintf( fp, "mass: %.10g\n", lc->data.mp.mass );
      if( !zVec3DIsTiny( &lc->data.mp.com ) ){
        fprintf( fp, "COM: " );
        zVec3DFPrint( fp, &lc->data.mp.com );
      }
      fprintf( fp, "inertia: " ); zMat3DFPrint( fp, &lc->data.mp.inertia );
    }
    zListForEach( &lc->data.shape_p_list, spc ){
      fprintf( fp, "shape: %s\n", spc->data->name );
    }
    if( lc->data.joint && lc->data.joint->parent )
      fprintf( fp, "parent: %s\n", lc->data.joint->parent->name );
    fprintf( fp, "\n" );
  }
}

void output_ztk(FILE *fp, robot_info_t *robot_info)
{
  fprintf( fp, "[chain]\n" );
  fprintf( fp, "name: %s\n\n", robot_info->name );
  if( !zListIsEmpty( &robot_info->material_list ) )
    output_ztk_material( fp, &robot_info->material_list );
  if( !zListIsEmpty( &robot_info->shape_list ) )
    output_ztk_shape( fp, &robot_info->shape_list );
  if( !zListIsEmpty( &robot_info->link_list ) )
    output_ztk_link( fp, &robot_info->link_list );
}

bool urdf2ztk_output(robot_info_t *robot_info, char *filename)
{
  char buf[BUFSIZ];
  FILE *fp;

  zReplaceSuffix( filename, "ztk", buf, BUFSIZ );
  if( !( fp = fopen( buf, "w" ) ) ){
    ZOPENERROR( buf );
    return false;
  }
  output_ztk( fp, robot_info );
  fclose( fp );
  return true;
}

int main(int argc, char *argv[])
{
  xmlDoc *doc;
  robot_info_t robot_info;

  if( argc < 2 ) return EXIT_FAILURE;
  zXMLInit();
  if( !( doc = xmlReadFile( argv[1], NULL, XML_PARSE_RECOVER | XML_PARSE_COMPACT ) ) ){
    ZOPENERROR( argv[1] );
    return EXIT_FAILURE;
  }
  robot_info_init( &robot_info );
  if( !urdf2ztk_eval( xmlDocGetRootElement( doc ), &robot_info ) ){
    ZRUNERROR( "invalid URDF file" );
  } else{
    urdf2ztk_correct_frame( &robot_info );
    urdf2ztk_output( &robot_info, argv[1] );
  }
  robot_info_destroy( &robot_info );

  xmlFreeDoc( doc );
  xmlCleanupParser();
  return EXIT_SUCCESS;
}
