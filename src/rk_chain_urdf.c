/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_chain_urdf - read a URDF file.
 */

#include <zeda/zeda_xml.h>
#include <roki/rk_chain.h>

/* optical info list */
zListClass( zOpticalInfoList, zOpticalInfoListCell, zOpticalInfo );

/* shape list */
typedef struct{
  char *name;
  char *type;
  zFrame3D fl, fw;
  char *property1;
  char *property2;
  char *material;
} rkURDFShape;
zListClass( rkURDFShapeList, rkURDFShapeListCell, rkURDFShape );
zListClass( rkURDFShapePList, rkURDFShapePListCell, rkURDFShape* );

static void _rkURDFShapeListDestroy(rkURDFShapeList *list)
{
  rkURDFShapeListCell *sc;

  while( !zListIsEmpty( list ) ){
    zListDeleteHead( list, &sc );
    free( sc->data.name );
    free( sc );
  }
}

/* link list */
struct _rkURDFJoint;
typedef struct{
  char *name;
  rkMP mp;
  zMat3D _iw;
  zVec3D _cw;
  rkURDFShapePList shape_p_list;
  struct _rkURDFJoint *joint;
  struct _rkURDFJoint *child;
} rkURDFLink;
zListClass( rkURDFLinkList, rkURDFLinkListCell, rkURDFLink );

static void _rkURDFLinkListDestroy(rkURDFLinkList *list)
{
  rkURDFLinkListCell *lc;

  while( !zListIsEmpty(list) ){
    zListDeleteHead( list, &lc );
    zListDestroy( rkURDFShapePListCell, &lc->data.shape_p_list );
    free( lc );
  }
}

/* joint list */
typedef struct _rkURDFJoint{
  char *name;
  char *type;
  zFrame3D fl, fw;
  zVec3D axis;
  char *damping;
  char *friction;
  char *lower;
  char *upper;
  rkURDFLink *parent;
  rkURDFLink *child;
  struct _rkURDFJoint *sibling;
} rkURDFJoint;
zListClass( rkURDFJointList, rkURDFJointListCell, rkURDFJoint );

/* robot info */
typedef struct{
  char *name;
  zOpticalInfoList material_list;
  rkURDFShapeList shape_list;
  rkURDFLinkList link_list;
  rkURDFJointList joint_list;
} rkURDFRobotInfo;

static void _rkURDFRobotInfoInit(rkURDFRobotInfo *robot_info)
{
  robot_info->name = NULL;
  zListInit( &robot_info->material_list );
  zListInit( &robot_info->shape_list );
  zListInit( &robot_info->link_list );
  zListInit( &robot_info->joint_list );
}

static void _rkURDFRobotInfoDestroy(rkURDFRobotInfo *robot_info)
{
  robot_info->name = NULL;
  zListDestroy( zOpticalInfoListCell, &robot_info->material_list );
  _rkURDFShapeListDestroy( &robot_info->shape_list );
  _rkURDFLinkListDestroy( &robot_info->link_list );
  zListDestroy( rkURDFJointListCell, &robot_info->joint_list );
}

/* evaluate frame information */

static void _rkURDFReadVec(char *str, zVec3D *v)
{
  sscanf( str, "%lf %lf %lf", &v->c.x, &v->c.y, &v->c.z );
}

static void _rkURDFReadAtt(char *str, zMat3D *m)
{
  double roll, pitch, yaw;

  sscanf( str, "%lf %lf %lf", &roll, &pitch, &yaw );
  zMat3DFromZYX( m, yaw, pitch, roll );
}

static bool _rkURDFEvalOrigin(xmlNode *node, zFrame3D *f)
{
  xmlAttr *attr;

  zXMLForEachAttr( node, attr ){
    zXMLCheckAttrAndExec( attr, "xyz", _rkURDFReadVec( zXMLGetAttrVal( attr ), zFrame3DPos(f) ) ) else
    zXMLCheckAttrAndExec( attr, "rpy", _rkURDFReadAtt( zXMLGetAttrVal( attr ), zFrame3DAtt(f) ) );
  }
  return true;
}

/* evaluate material information */

static void _rkURDFReadRGBA(char *str, zOpticalInfoListCell *mc)
{
  sscanf( str, "%f %f %f %lf", &mc->data.diffuse.r, &mc->data.diffuse.g, &mc->data.diffuse.b, &mc->data.alpha );
}

static bool _rkURDFEvalColor(xmlNode *node, zOpticalInfoListCell *mc)
{
  _rkURDFReadRGBA( zXMLFindNodeAttr( node, "rgba" ), mc );
  return true;
}

static bool _rkURDFEvalMaterial(xmlNode *node, rkURDFRobotInfo *robot_info)
{
  xmlNode *np;
  zOpticalInfoListCell *mc;

  if( !( mc = zAlloc( zOpticalInfoListCell, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  zOpticalInfoInit( &mc->data );
  zRGBSet( &mc->data.ambient, 0.5, 0.5, 0.5 );
  zListInsertHead( &robot_info->material_list, mc );
  zNameSetPtr( &mc->data, zXMLFindNodeAttr( node, "name" ) );
  zXMLForEachNode( node->children, np ){
    zXMLCheckElementAndExec( np, "color", _rkURDFEvalColor( np, mc ) );
    /* NOTE: texture unsupported yet. */
  }
  return true;
}

/* evaluate mass property */

static bool _rkURDFEvalLinkMass(xmlNode *node, rkURDFLinkListCell *lc)
{
  lc->data.mp.mass = atof( zXMLFindNodeAttr( node, "value" ) );
  return true;
}

static bool _rkURDFEvalLinkInertia(xmlNode *node, rkURDFLinkListCell *lc)
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

static bool _rkURDFEvalLinkMP(xmlNode *node, rkURDFLinkListCell *lc)
{
  xmlNode *np;
  zFrame3D f;

  zFrame3DIdent( &f );
  zXMLForEachNode( node->children, np ){
    zXMLCheckElementAndExec( np, "origin",  _rkURDFEvalOrigin( np, &f ) )   else
    zXMLCheckElementAndExec( np, "mass",    _rkURDFEvalLinkMass( np, lc ) ) else
    zXMLCheckElementAndExec( np, "inertia", _rkURDFEvalLinkInertia( np, lc ) );
  }
  zVec3DCopy( zFrame3DPos(&f), &lc->data.mp.com );
  zRotMat3DDRC( zFrame3DAtt(&f), &lc->data.mp.inertia );
  return true;
}

/* evaluate shape geometry */

static bool _rkURDFEvalShapeBox(xmlNode *node, rkURDFShapeListCell *sc)
{
  sc->data.type = (char *)node->name;
  sc->data.property1 = zXMLFindNodeAttr( node, "size" );
  return true;
}

static bool _rkURDFEvalShapeSphere(xmlNode *node, rkURDFShapeListCell *sc)
{
  sc->data.type = (char *)node->name;
  sc->data.property1 = zXMLFindNodeAttr( node, "radius" );
  return true;
}

static bool _rkURDFEvalShapeCylinderCapsule(xmlNode *node, rkURDFShapeListCell *sc)
{
  xmlAttr *attr;

  sc->data.type = (char *)node->name;
  zXMLForEachAttr( node, attr ){
    zXMLCheckAttrAndExec( attr, "radius", sc->data.property1 = zXMLGetAttrVal( attr ) ) else
    zXMLCheckAttrAndExec( attr, "length", sc->data.property2 = zXMLGetAttrVal( attr ) );
  }
  return true;
}

static bool _rkURDFEvalShapeMesh(xmlNode *node, rkURDFShapeListCell *sc)
{
  xmlAttr *attr;

  sc->data.type = (char *)node->name;
  zXMLForEachAttr( node, attr ){
    zXMLCheckAttrAndExec( attr, "filename", sc->data.property1 = zXMLGetAttrVal( attr ) ) else
    zXMLCheckAttrAndExec( attr, "scale",    sc->data.property2 = zXMLGetAttrVal( attr ) );
  }
  return true;
}

static bool _rkURDFEvalShapeGeometry(xmlNode *node, rkURDFShapeListCell *sc)
{
  xmlNode *np;

  zXMLForEachNode( node, np ){
    zXMLCheckElementAndExec( np, "box",      _rkURDFEvalShapeBox( np, sc ) )      else
    zXMLCheckElementAndExec( np, "sphere",   _rkURDFEvalShapeSphere( np, sc ) )   else
    zXMLCheckElementAndExec( np, "cylinder", _rkURDFEvalShapeCylinderCapsule( np, sc ) ) else
    zXMLCheckElementAndExec( np, "capsule",  _rkURDFEvalShapeCylinderCapsule( np, sc ) ) else
    zXMLCheckElementAndExec( np, "mesh",     _rkURDFEvalShapeMesh( np, sc ) );
  }
  return true;
}

static bool _rkURDFEvalShapeMaterial(xmlNode *node, rkURDFRobotInfo *robot_info, rkURDFShapeListCell *sc)
{
  sc->data.material = zXMLFindNodeAttr( node, "name" );
  if( node->children ){
    if( !sc->data.material ){
      zXMLAddNodeAttr( node, "name", sc->data.name );
      sc->data.material = sc->data.name;
    } else
    if( sc->data.material[0] == '\0' ){
      zXMLReplaceNodeAttr( node, "name", sc->data.name );
      sc->data.material = sc->data.name;
    }
    return _rkURDFEvalMaterial( node, robot_info );
  }
  return true;
}

/* evaluate visual property of a link */

static bool _rkURDFEvalLinkVisual(xmlNode *node, rkURDFRobotInfo *robot_info, rkURDFLinkListCell *lc)
{
  xmlNode *np;
  rkURDFShapeListCell *sc;
  rkURDFShapePListCell *spc;
  char buf[BUFSIZ];

  if( !( sc = zAlloc( rkURDFShapeListCell, 1 ) ) ){
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
  if( !( spc = zAlloc( rkURDFShapePListCell, 1 ) ) ){
    ZALLOCERROR();
    return false;
  }
  spc->data = &sc->data;
  zListInsertHead( &lc->data.shape_p_list, spc );
  zXMLForEachNode( node->children, np ){
    zXMLCheckElementAndExec( np, "origin",   _rkURDFEvalOrigin( np, &sc->data.fl ) ) else
    zXMLCheckElementAndExec( np, "geometry", _rkURDFEvalShapeGeometry( np->children, sc ) ) else
    zXMLCheckElementAndExec( np, "material", _rkURDFEvalShapeMaterial( np, robot_info, sc ) );
  }
  return true;
}

static bool _rkURDFEvalLink(xmlNode *node, rkURDFRobotInfo *robot_info)
{
  xmlNode *np;
  rkURDFLinkListCell *lc;

  if( !( lc = zAlloc( rkURDFLinkListCell, 1 ) ) ){
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
    zXMLCheckElementAndExec( np, "inertial",  _rkURDFEvalLinkMP( np, lc ) ) else
    zXMLCheckElementAndExec( np, "visual",    _rkURDFEvalLinkVisual( np, robot_info, lc ) );
    /* collision nodes are ignored. */
  }
  return true;
}

/* evaluate joint information */

static rkURDFJoint *_rkURDFJointAddSibling(rkURDFJoint *joint, rkURDFJoint *sibling)
{
  if( !joint->sibling ) return joint->sibling = sibling;
  return _rkURDFJointAddSibling( joint->sibling, sibling );
}

static rkURDFJoint *_rkURDFLinkAddChild(rkURDFLink *link, rkURDFJoint *child)
{
  if( !link->child ) return link->child = child;
  return _rkURDFJointAddSibling( link->child, child );
}

static void _rkURDFBindJointParent(rkURDFRobotInfo *robot_info, char *str, rkURDFJointListCell *jc)
{
  rkURDFLinkListCell *cp;
  zListFindName( &robot_info->link_list, str, &cp );
  if( cp && ( jc->data.parent = &cp->data ) )
    _rkURDFLinkAddChild( jc->data.parent, &jc->data );
}

static void _rkURDFBindJointChild(rkURDFRobotInfo *robot_info, char *str, rkURDFJointListCell *jc)
{
  rkURDFLinkListCell *cp;
  zListFindName( &robot_info->link_list, str, &cp );
  if( cp && ( jc->data.child = &cp->data ) )
    jc->data.child->joint = &jc->data;
}

static bool _rkURDFEvalJointParent(xmlNode *node, rkURDFRobotInfo *robot_info, rkURDFJointListCell *jc)
{
  _rkURDFBindJointParent( robot_info, zXMLFindNodeAttr( node, "link" ), jc );
  return true;
}

static bool _rkURDFEvalJointChild(xmlNode *node, rkURDFRobotInfo *robot_info, rkURDFJointListCell *jc)
{
  _rkURDFBindJointChild( robot_info, zXMLFindNodeAttr( node, "link" ), jc );
  return true;
}

static bool _rkURDFEvalJointAxis(xmlNode *node, rkURDFJointListCell *jc)
{
  _rkURDFReadVec( zXMLFindNodeAttr( node, "xyz" ), &jc->data.axis );
  return true;
}

static bool _rkURDFEvalJointDynamic(xmlNode *node, rkURDFJointListCell *jc)
{
  xmlAttr *attr;

  zXMLForEachAttr( node, attr ){
    zXMLCheckAttrAndExec( attr, "damping",  jc->data.damping = zXMLGetAttrVal( attr ) ) else
    zXMLCheckAttrAndExec( attr, "friction", jc->data.friction = zXMLGetAttrVal( attr ) );
  }
  return true;
}

static bool _rkURDFEvalJointLimit(xmlNode *node, rkURDFJointListCell *jc)
{
  xmlAttr *attr;

  zXMLForEachAttr( node, attr ){
    zXMLCheckAttrAndExec( attr, "lower", jc->data.lower = zXMLGetAttrVal( attr ) ) else
    zXMLCheckAttrAndExec( attr, "upper", jc->data.upper = zXMLGetAttrVal( attr ) );
    /* effort and velocity are ignored. */
  }
  return true;
}

static bool _rkURDFEvalJoint(xmlNode *node, rkURDFRobotInfo *robot_info)
{
  xmlNode *np;
  xmlAttr *attr;
  rkURDFJointListCell *jc;

  if( !( jc = zAlloc( rkURDFJointListCell, 1 ) ) ){
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
    zXMLCheckElementAndExec( np, "origin",  _rkURDFEvalOrigin( np, &jc->data.fl ) ) else
    zXMLCheckElementAndExec( np, "parent",  _rkURDFEvalJointParent( np, robot_info, jc ) ) else
    zXMLCheckElementAndExec( np, "child",   _rkURDFEvalJointChild( np, robot_info, jc ) ) else
    zXMLCheckElementAndExec( np, "axis",    _rkURDFEvalJointAxis( np, jc ) ) else
    zXMLCheckElementAndExec( np, "dynamic", _rkURDFEvalJointDynamic( np, jc ) ) else
    zXMLCheckElementAndExec( np, "limit",   _rkURDFEvalJointLimit( np, jc ) );
  }
  return true;
}

/* evaluate robot model information */

static bool _rkURDFEvalRobot(xmlNode *node, rkURDFRobotInfo *robot_info)
{
  xmlNode *np;

  zXMLForEachNode( node, np ){
    zXMLCheckElementAndExec( np, "material", _rkURDFEvalMaterial( np, robot_info ) );
  }
  zXMLForEachNode( node, np ){
    zXMLCheckElementAndExec( np, "link", _rkURDFEvalLink( np, robot_info ) );
  }
  zXMLForEachNode( node, np ){
    zXMLCheckElementAndExec( np, "joint", _rkURDFEvalJoint( np, robot_info ) );
  }
  return true;
}

static bool _rkURDFEval(xmlNode *node, rkURDFRobotInfo *robot_info)
{
  xmlNode *np;

  if( !( np = zXMLFindNodeElement( node, "robot" ) ) )
    return false;
  robot_info->name = zXMLFindNodeAttr( np, "name" );
  return _rkURDFEvalRobot( np->children, robot_info );
}

/* coordinate transformation */

static bool _rkURDFXform(rkURDFJoint *joint)
{
  rkURDFShapePListCell *spc;

  if( !joint->parent || !joint->child ){
    ZRUNERROR( RK_ERR_URDF_UNCONNECTED_JOINT, joint->name );
    return false;
  }
  if( !joint->parent->joint ) /* root link */
    zFrame3DCopy( &joint->fl, &joint->fw );
  else
    zFrame3DCascade( &joint->parent->joint->fw, &joint->fl, &joint->fw );
  zRotMat3D( zFrame3DAtt(&joint->fw), &joint->child->mp.inertia, &joint->child->_iw );
  zMulMat3DVec3D( zFrame3DAtt(&joint->fw), &joint->child->mp.com, &joint->child->_cw );
  zListForEach( &joint->child->shape_p_list, spc ){
    zFrame3DCascade( &joint->fw, &spc->data->fl, &spc->data->fw );
  }
  if( joint->child->child )
    if( !_rkURDFXform( joint->child->child ) ) return false;
  return joint->sibling ? _rkURDFXform( joint->sibling ) : true;
}

static bool _rkURDFXformInv(rkURDFJoint *joint)
{
  rkURDFShapePListCell *spc;

  if( !joint->parent || !joint->child ){
    ZRUNERROR( RK_ERR_URDF_UNCONNECTED_JOINT, joint->name );
    return false;
  }
  if( joint->parent->joint )
    zFrame3DXform( &joint->parent->joint->fw, &joint->fw, &joint->fl );
  else
    zFrame3DCopy( &joint->fw, &joint->fl );
  zRotMat3DInv( zFrame3DAtt(&joint->fw), &joint->child->_iw, &joint->child->mp.inertia );
  zMulMat3DTVec3D( zFrame3DAtt(&joint->fw), &joint->child->_cw, &joint->child->mp.com );
  zListForEach( &joint->child->shape_p_list, spc ){
    zFrame3DXform( &joint->fw, &spc->data->fw, &spc->data->fl );
  }
  if( joint->child->child )
    if( !_rkURDFXformInv( joint->child->child ) ) return false;
  return joint->sibling ? _rkURDFXformInv( joint->sibling ) : true;
}

static rkURDFLink *_rkURDFFindRoot(rkURDFRobotInfo *robot_info)
{
  rkURDFLinkListCell *lc;

  zListForEach( &robot_info->link_list, lc )
    if( !lc->data.joint ) return &lc->data;
  return NULL;
}

static bool _rkURDFJointIsFixed(rkURDFJoint *joint)
{
  return strcmp( joint->type, "fixed" ) == 0;
}

static bool _rkURDFJointIsRevol(rkURDFJoint *joint)
{
  return strcmp( joint->type, "revolute" ) == 0 || strcmp( joint->type, "continuous" ) == 0;
}

static bool _rkURDFJointIsPrism(rkURDFJoint *joint)
{
  return strcmp( joint->type, "prismatic" ) == 0;
}

static bool _rkURDFJointIsFloat(rkURDFJoint *joint)
{
  return strcmp( joint->type, "floating" ) == 0;
}

static bool _rkURDFCorrectFrame(rkURDFRobotInfo *robot_info)
{
  rkURDFLink *root;
  rkURDFJointListCell *jc;
  zVec3D aa;

  if( !( root = _rkURDFFindRoot( robot_info ) ) ){
    ZRUNERROR( RK_ERR_URDF_ROOT_LINK_NOTFOUND );
    return false;
  }
  if( root->child ) _rkURDFXform( root->child );
  zListForEach( &robot_info->joint_list, jc ){
    if( _rkURDFJointIsRevol( &jc->data ) || _rkURDFJointIsPrism( &jc->data ) ){
      zVec3DAAError( ZVEC3DZ, &jc->data.axis, &aa );
      zMat3DRotDRC( zFrame3DAtt(&jc->data.fw), &aa );
    }
  }
  if( root->child ) _rkURDFXformInv( root->child );
  return true;
}

/* output to ZTK */

static bool _rkURDF2ZTKMaterial(ZTK *ztk, zOpticalInfoList *material_list)
{
  zOpticalInfoListCell *mc;

  zListForEach( material_list, mc ){
    if( !ZTKAddTag( ztk, ZTK_TAG_ZEO_OPTIC ) ) return false;
    if( !zOpticalInfoToZTK( &mc->data, ztk ) ) return false;
  }
  return true;
}

static bool _rkURDF2ZTKShapeBox(ZTK *ztk, rkURDFShapeListCell *sc)
{
  double width, height, depth;

  if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_TYPE ) ) return false;
  if( !ZTKAddVal( ztk, zeo_shape3d_box_com.typestr ) ) return false;
  sscanf( sc->data.property1, "%lf %lf %lf", &depth, &width, &height );
  if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_DEPTH ) ) return false;
  if( !ZTKAddDouble( ztk, depth ) ) return false;
  if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_WIDTH ) ) return false;
  if( !ZTKAddDouble( ztk, width ) ) return false;
  if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_HEIGHT ) ) return false;
  if( !ZTKAddDouble( ztk, height ) ) return false;
  return true;
}

static bool _rkURDF2ZTKShapeSphere(ZTK *ztk, rkURDFShapeListCell *sc)
{
  double radius;

  if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_TYPE ) ) return false;
  if( !ZTKAddVal( ztk, zeo_shape3d_sphere_com.typestr ) ) return false;
  sscanf( sc->data.property1, "%lf", &radius );
  if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_RADIUS ) ) return false;
  if( !ZTKAddDouble( ztk, radius ) ) return false;
  return true;
}

static bool _rkURDF2ZTKShapeCylinderCapsuleBody(ZTK *ztk, rkURDFShapeListCell *sc)
{
  double radius, length;
  zVec3D center;

  sscanf( sc->data.property1, "%lf", &radius );
  sscanf( sc->data.property2, "%lf", &length );
  if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_CENTER ) ) return false;
  zVec3DCreate( &center, 0, 0, length/2 );
  if( !zVec3DToZTK( &center, ztk ) ) return false;
  if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_CENTER ) ) return false;
  zVec3DCreate( &center, 0, 0,-length/2 );
  if( !zVec3DToZTK( &center, ztk ) ) return false;
  if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_RADIUS ) ) return false;
  if( !ZTKAddDouble( ztk, radius ) ) return false;
  return true;
}

static bool _rkURDF2ZTKShapeCylinder(ZTK *ztk, rkURDFShapeListCell *sc)
{
  if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_TYPE ) ) return false;
  if( !ZTKAddVal( ztk, zeo_shape3d_cyl_com.typestr ) ) return false;
  return _rkURDF2ZTKShapeCylinderCapsuleBody( ztk, sc );
}

static bool _rkURDF2ZTKShapeCapsule(ZTK *ztk, rkURDFShapeListCell *sc)
{
  if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_TYPE ) ) return false;
  if( !ZTKAddVal( ztk, zeo_shape3d_capsule_com.typestr ) ) return false;
  return _rkURDF2ZTKShapeCylinderCapsuleBody( ztk, sc );
}

static bool _rkURDF2ZTKShapeMesh(ZTK *ztk, rkURDFShapeListCell *sc)
{
  if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_IMPORT ) ) return false;
  if( !ZTKAddVal( ztk, sc->data.property1 ) ) return false;
  if( sc->data.property2 ) /* scale */
    if( !ZTKAddVal( ztk, sc->data.property2 ) ) return false;
  return true;
}

static bool _rkURDF2ZTKShape(ZTK *ztk, rkURDFShapeList *shape_list)
{
  rkURDFShapeListCell *sc;

  zListForEach( shape_list, sc ){
    if( !sc->data.type ){
      ZRUNWARN( RK_WARN_URDF_UNNAMED_SHAPE );
      continue;
    }
    if( !ZTKAddTag( ztk, ZTK_TAG_ZEO_SHAPE ) ) return false;
    if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_NAME ) ) return false;
    if( !ZTKAddVal( ztk, zName(&sc->data) ) ) return false;
    if( !zVec3DIsTiny( zFrame3DPos(&sc->data.fl) ) ){
      if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_POS ) ) return false;
      if( !zVec3DToZTK( zFrame3DPos(&sc->data.fl), ztk ) ) return false;
    }
    if( !zMat3DIsIdent( zFrame3DAtt(&sc->data.fl) ) ){
      if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_ATT ) ) return false;
      if( !zMat3DToZTK( zFrame3DAtt(&sc->data.fl), ztk ) ) return false;
    }
    if( sc->data.material ){
      if( !ZTKAddKey( ztk, ZTK_KEY_ZEO_SHAPE3D_OPTIC ) ) return false;
      if( !ZTKAddVal( ztk, sc->data.material ) ) return false;
    }
    if( strcmp( sc->data.type, "box" ) == 0 )      _rkURDF2ZTKShapeBox( ztk, sc );      else
    if( strcmp( sc->data.type, "sphere" ) == 0 )   _rkURDF2ZTKShapeSphere( ztk, sc );   else
    if( strcmp( sc->data.type, "cylinder" ) == 0 ) _rkURDF2ZTKShapeCylinder( ztk, sc ); else
    if( strcmp( sc->data.type, "capsule" ) == 0 )  _rkURDF2ZTKShapeCapsule( ztk, sc );  else
    if( strcmp( sc->data.type, "mesh" ) == 0 )     _rkURDF2ZTKShapeMesh( ztk, sc );     else{
      ZRUNWARN( RK_WARN_URDF_UNKNOWN_SHAPETYPE, sc->data.type );
    }
  }
  return true;
}

static bool _rkURDF2ZTKJoint(ZTK *ztk, rkURDFJoint *joint)
{
  double lower, upper;

  lower = upper = 0;
  if( joint->type ){
    if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_LINK_JOINTTYPE ) ) return false;
    if( _rkURDFJointIsRevol( joint ) ){
      if( !ZTKAddVal( ztk, rk_joint_revol.typestr ) ) return false;
      if( joint->lower ) lower = zRad2Deg( atof( joint->lower ) );
      if( joint->upper ) upper = zRad2Deg( atof( joint->upper ) );
    } else
    if( _rkURDFJointIsPrism( joint ) ){
      if( !ZTKAddVal( ztk, rk_joint_prism.typestr ) ) return false;
      if( joint->lower ) lower = atof( joint->lower );
      if( joint->upper ) upper = atof( joint->upper );
    } else
    if( _rkURDFJointIsFloat( joint ) ){
      if( !ZTKAddVal( ztk, rk_joint_float.typestr ) ) return false;
    } else
    if( _rkURDFJointIsFixed( joint ) ){
      if( !ZTKAddVal( ztk, rk_joint_fixed.typestr ) ) return false;
    } else{
      ZRUNERROR( RK_ERR_URDF_UNKNOWN_JOINTTYPE, joint->type );
      return false;
    }
  }
  if( joint->lower ){
    if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_JOINT_MIN ) ) return false;
    if( !ZTKAddDouble( ztk, lower ) ) return false;
  }
  if( joint->upper ){
    if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_JOINT_MAX ) ) return false;
    if( !ZTKAddDouble( ztk, upper ) ) return false;
  }
  if( joint->damping ){
    if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_JOINT_VISCOSITY ) ) return false;
    if( !ZTKAddVal( ztk, joint->damping ) ) return false;
  }
  if( joint->friction ){
    if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_JOINT_COULOMB ) ) return false;
    if( !ZTKAddVal( ztk, joint->friction ) ) return false;
  }
  if( !zVec3DIsTiny( zFrame3DPos(&joint->fl) ) ){
    if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_LINK_POS ) ) return false;
    if( !zVec3DToZTK( zFrame3DPos(&joint->fl), ztk ) ) return false;
  }
  if( !zMat3DIsIdent( zFrame3DAtt(&joint->fl) ) ){
    if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_LINK_ATT ) ) return false;
    if( !zMat3DToZTK( zFrame3DAtt(&joint->fl), ztk ) ) return false;
  }
  return true;
}

static bool _rkURDF2ZTKLink(ZTK *ztk, rkURDFLinkList *link_list)
{
  rkURDFLinkListCell *lc;
  rkURDFShapePListCell *spc;

  zListForEach( link_list, lc ){
    if( !ZTKAddTag( ztk, ZTK_TAG_ROKI_LINK ) ) return false;
    if( lc->data.name ){
      if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_LINK_NAME ) ) return false;
      if( !ZTKAddVal( ztk, zName(&lc->data) ) ) return false;
    }
    if( lc->data.joint )
      _rkURDF2ZTKJoint( ztk, lc->data.joint );
    if( lc->data.mp.mass != 0 ){
      if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_LINK_MASS ) ) return false;
      if( !ZTKAddDouble( ztk, lc->data.mp.mass ) ) return false;
      if( !zVec3DIsTiny( &lc->data.mp.com ) ){
        if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_LINK_COM ) ) return false;
        if( !zVec3DToZTK( &lc->data.mp.com, ztk ) ) return false;
      }
      if( !zMat3DIsTiny( &lc->data.mp.inertia ) ){
        if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_LINK_INERTIA ) ) return false;
        if( !zMat3DToZTK( &lc->data.mp.inertia, ztk ) ) return false;
      }
    }
    zListForEach( &lc->data.shape_p_list, spc ){
      if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_LINK_SHAPE ) ) return false;
      if( !ZTKAddVal( ztk, spc->data->name ) ) return false;
    }
    if( lc->data.joint && lc->data.joint->parent ){
      if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_LINK_PARENT ) ) return false;
      if( !ZTKAddVal( ztk, lc->data.joint->parent->name ) ) return false;
    }
  }
  return true;
}

static bool _rkURDF2ZTK(ZTK *ztk, rkURDFRobotInfo *robot_info)
{
  if( !ZTKAddTag( ztk, ZTK_TAG_ROKI_CHAIN ) ) return false;
  if( !ZTKAddKey( ztk, ZTK_KEY_ROKI_CHAIN_NAME ) ) return false;
  if( !ZTKAddVal( ztk, robot_info->name ) ) return false;

  if( !zListIsEmpty( &robot_info->material_list ) )
    if( !_rkURDF2ZTKMaterial( ztk, &robot_info->material_list ) ) return false;
  if( !zListIsEmpty( &robot_info->shape_list ) )
    if( !_rkURDF2ZTKShape( ztk, &robot_info->shape_list ) ) return false;
  if( !zListIsEmpty( &robot_info->link_list ) )
    if( !_rkURDF2ZTKLink( ztk, &robot_info->link_list ) ) return false;
  return true;
}

/* read a URDF file and create an instance of rkChain. */
static bool _rkURDFReadZTK(ZTK *ztk, const char *filename)
{
  xmlDoc *doc;
  rkURDFRobotInfo robot_info;
  bool retval = false;

  ZTKInit( ztk );
  zXMLInit();
  if( !( doc = xmlReadFile( filename, NULL, XML_PARSE_RECOVER | XML_PARSE_COMPACT ) ) ){
    ZOPENERROR( filename );
    return false;
  }
  _rkURDFRobotInfoInit( &robot_info );
  if( !_rkURDFEval( xmlDocGetRootElement( doc ), &robot_info ) ){
    ZRUNERROR( RK_ERR_URDF_INVALID );
  } else{
    _rkURDFCorrectFrame( &robot_info );
    if( _rkURDF2ZTK( ztk, &robot_info ) ) retval = true;
  }
  _rkURDFRobotInfoDestroy( &robot_info );
  xmlFreeDoc( doc );
  xmlCleanupParser();
  return retval;
}

/* read a URDF file and create an instance of rkChain. */
rkChain *rkChainReadURDF(rkChain *chain, const char *filename)
{
  ZTK ztk;

  if( _rkURDFReadZTK( &ztk, filename ) )
    chain = rkChainFromZTK( chain, &ztk );
  ZTKDestroy( &ztk );
  return chain;
}

/* directly convert a URDF file to a ZTK file. */
bool rkURDFWriteZTK(const char *inputfilename, const char *outputfilename)
{
  FILE *fp;
  ZTK ztk;

  if( !( fp = fopen( outputfilename, "w" ) ) ){
    ZOPENERROR( outputfilename );
    return false;
  }
  ZTKInit( &ztk );
  if( _rkURDFReadZTK( &ztk, inputfilename ) )
    ZTKFPrint( fp, &ztk );
  ZTKDestroy( &ztk );
  fclose( fp );
  return true;
}
