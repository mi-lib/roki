#include <roki/rk_chain.h>

/* header .h -------------------------------------------------------------- */

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKRegSelectClass ){
  void* (*init                )(void**);
  void (*copy                 )(void*,void*);
  void (*free                 )(void**);
  bool (*select_com           )(void*);
  bool (*com                  )(void*);
  bool (*select_link          )(void*);
  bool (*link                 )(void*);
  bool (*select_pos           )(void*);
  bool (*pos                  )(void*);
  bool (*select_att           )(void*);
  bool (*att                  )(void*);
  bool (*select_am            )(void*);
  bool (*am                   )(void*);
  bool (*select_wld_frame     )(void*);
  bool (*wld_frame            )(void*);
  bool (*select_sub_link_frame)(void*);
  bool (*sub_link_frame       )(void*);
  bool (*select_force         )(void*);
  bool (*unselect_force       )(void*);
  bool (*force                )(void*);
  void (*set_name             )(void*,const char*);
  const char* (*get_name      )(void*);
  void (*set_priority         )(void*,int);
  int  (*get_priority         )(void*);
  void (*set_link_id          )(void*,int);
  int  (*get_link_id          )(void*);
  void (*set_ap               )(void*,double,double,double);
  void (*get_ap               )(void*,double*,double*,double*);
  void (*set_weight           )(void*,double,double,double);
  void (*get_weight           )(void*,double*,double*,double*);
  void (*set_sub_link_frame_id)(void*,int);
  int  (*get_sub_link_frame_id)(void*);
  void (*reset                )(void*);
  void* (*reg                 )(void*,void*);
  void* (*from_cell_name      )(void*,const char*);
  bool (*unreg                )(void*,void*);
  bool (*unreg_by_name        )(void*,const char*);
  void* (*from_ztk            )(void*,void*);
  bool (*fprint_ztk           )(FILE*,void*,void*);
};

/* declaration */
void* rkIKRegSelect_init                (void **instance);
void rkIKRegSelect_copy                 (void *src, void *dest);
void rkIKRegSelect_free                 (void **instance);
bool rkIKRegSelect_select_com           (void *instance);
bool rkIKRegSelect_com                  (void *instance);
bool rkIKRegSelect_select_link          (void *instance);
bool rkIKRegSelect_link                 (void *instance);
bool rkIKRegSelect_select_pos           (void *instance);
bool rkIKRegSelect_pos                  (void *instance);
bool rkIKRegSelect_select_att           (void *instance);
bool rkIKRegSelect_att                  (void *instance);
bool rkIKRegSelect_select_am            (void *instance);
bool rkIKRegSelect_am                   (void *instance);
bool rkIKRegSelect_select_wld_frame     (void *instance);
bool rkIKRegSelect_wld_frame            (void *instance);
bool rkIKRegSelect_select_sub_link_frame(void *instance);
bool rkIKRegSelect_sub_link_frame       (void *instance);
bool rkIKRegSelect_select_force         (void *instance);
bool rkIKRegSelect_unselect_force       (void *instance);
bool rkIKRegSelect_force                (void *instance);
void rkIKRegSelect_set_name             (void* instance, const char* name);
const char* rkIKRegSelect_get_name      (void* instance);
void rkIKRegSelect_set_priority         (void* instance, int priority);
int  rkIKRegSelect_get_priority         (void* instance);
void rkIKRegSelect_set_link_id          (void *instance, int link_id);
int  rkIKRegSelect_get_link_id          (void *instance);
void rkIKRegSelect_set_ap               (void *instance, double v1, double v2, double v3);
void rkIKRegSelect_get_ap               (void *instance, double *v1, double *v2, double *v3);
void rkIKRegSelect_set_weight           (void *instance, double w1, double w2, double w3);
void rkIKRegSelect_get_weight           (void *instance, double *w1, double *w2, double *w3);
void rkIKRegSelect_set_sub_link_frame_id(void *instance, int sub_link_id);
int  rkIKRegSelect_get_sub_link_frame_id(void *instance);
void rkIKRegSelect_reset                (void *instance);
void* rkIKRegSelect_call_reg_api        (void *instance, void *chain);
void* rkIKRegSelect_from_cell_name      (void* chain, const char* name);
bool rkIKRegSelect_unreg_by_cell        (void *chain, void* cell);
bool rkIKRegSelect_unreg_by_name        (void *chain, const char* name);
void* rkIKRegSelect_fromZTK_constraint_key(void* chain, void* ztk);
bool rkIKRegSelect_fprintZTK_as_constraint_key(FILE *fp, void* chain, void* instance);

static rkIKRegSelectClass rkIKRegSelectClassImpl = {
  rkIKRegSelect_init,
  rkIKRegSelect_copy,
  rkIKRegSelect_free,
  rkIKRegSelect_select_com,
  rkIKRegSelect_com,
  rkIKRegSelect_select_link,
  rkIKRegSelect_link,
  rkIKRegSelect_select_pos,
  rkIKRegSelect_pos,
  rkIKRegSelect_select_att,
  rkIKRegSelect_att,
  rkIKRegSelect_select_am,
  rkIKRegSelect_am,
  rkIKRegSelect_select_wld_frame,
  rkIKRegSelect_wld_frame,
  rkIKRegSelect_select_sub_link_frame,
  rkIKRegSelect_sub_link_frame,
  rkIKRegSelect_select_force,
  rkIKRegSelect_unselect_force,
  rkIKRegSelect_force,
  rkIKRegSelect_set_name,
  rkIKRegSelect_get_name,
  rkIKRegSelect_set_priority,
  rkIKRegSelect_get_priority,
  rkIKRegSelect_set_link_id,
  rkIKRegSelect_get_link_id,
  rkIKRegSelect_set_ap,
  rkIKRegSelect_get_ap,
  rkIKRegSelect_set_weight,
  rkIKRegSelect_get_weight,
  rkIKRegSelect_set_sub_link_frame_id,
  rkIKRegSelect_get_sub_link_frame_id,
  rkIKRegSelect_reset,
  rkIKRegSelect_call_reg_api,
  rkIKRegSelect_from_cell_name,
  rkIKRegSelect_unreg_by_cell,
  rkIKRegSelect_unreg_by_name,
  rkIKRegSelect_fromZTK_constraint_key,
  rkIKRegSelect_fprintZTK_as_constraint_key,
};


/* implement .c (capsuled) ------------------------------------------------ */

/**/
/* int 32bit */
/* User Defined Type (24bit) : Reference Frame Type, Target Type, Quantity Type */
static const int32_t RK_IK_ATTR_TYPE__WORLD_LINK_POS     = 0x010101;
static const int32_t RK_IK_ATTR_TYPE__WORLD_LINK_ATT     = 0x010102;
static const int32_t RK_IK_ATTR_TYPE__WORLD_LINK_AM      = 0x010103;
static const int32_t RK_IK_ATTR_TYPE__WORLD_COM_POS      = 0x010201;
static const int32_t RK_IK_ATTR_TYPE__WORLD_COM_AM       = 0x010203;
static const int32_t RK_IK_ATTR_TYPE__SUB_LINK_LINK_POS  = 0x020101;
static const int32_t RK_IK_ATTR_TYPE__SUB_LINK_LINK_ATT  = 0x020102;
/* static const int32_t RK_IK_ATTR_TYPE__SUB_LINK_LINK_AM   = 0x020103; */
/* static const int32_t RK_IK_ATTR_TYPE__SUB_LINK_COM_POS   = 0x020201; */
/* static const int32_t RK_IK_ATTR_TYPE__SUB_LINK_COM_AM    = 0x020203; */
/* Reference Frame Type for Selecting */
static const int32_t RK_IK_ATTR_TYPE_REF_FRAME           = 0xff0000;
static const int32_t RK_IK_ATTR_TYPE_REF_FRAME__WORLD    = 0x010000;
static const int32_t RK_IK_ATTR_TYPE_REF_FRAME__SUB_LINK = 0x020000;
/* Target Type for Selecting */
static const int32_t RK_IK_ATTR_TYPE_TARGET              = 0x00ff00;
static const int32_t RK_IK_ATTR_TYPE_TARGET__LINK        = 0x000100;
static const int32_t RK_IK_ATTR_TYPE_TARGET__COM         = 0x000200;
/* Quantity Type for Selecting */
static const int32_t RK_IK_ATTR_TYPE_QUANTITY           = 0x00000ff;
static const int32_t RK_IK_ATTR_TYPE_QUANTITY__POS      = 0x0000001;
static const int32_t RK_IK_ATTR_TYPE_QUANTITY__ATT      = 0x0000002;
static const int32_t RK_IK_ATTR_TYPE_QUANTITY__AM       = 0x0000003;

/**/
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKRegister ){
  /* arguments for call api */
  char *name;
  int _priority;
  rkIKAttr _attr;
};

void* rkIKRegSelect_init(void** instance)
{
  rkIKRegister* reg = (rkIKRegister*)(*instance);
  reg = zAlloc( rkIKRegister, 1 );
  zNameSet( reg, "" );
  reg->_priority = 0;
  rkIKAttrInit( &reg->_attr );
  reg->_attr.user_defined_type = 0;
  *instance = (void*)(reg);

  return instance;
}

void rkIKRegSelect_copy(void* src, void* dest)
{
  zNameFree( (rkIKRegister*)(dest) );
  zCopy( rkIKRegister, (rkIKRegister*)(src), (rkIKRegister*)(dest) );
  zNameSet( (rkIKRegister*)(dest), ((rkIKRegister*)(src))->name );
}

void rkIKRegSelect_free(void **instance)
{
  rkIKRegister* reg = (rkIKRegister*)(*instance);
  if( zNamePtr( reg ) != NULL )
    zNameFree( reg );
  zFree( reg );
  *instance = NULL;
}

bool rkIKRegSelect_select_link(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  reg->_attr.user_defined_type &= (~RK_IK_ATTR_TYPE_TARGET);
  reg->_attr.user_defined_type |= RK_IK_ATTR_TYPE_TARGET__LINK;
  return true;
}

bool rkIKRegSelect_link(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  int target = reg->_attr.user_defined_type & RK_IK_ATTR_TYPE_TARGET;
  return (target == RK_IK_ATTR_TYPE_TARGET__LINK);
}

bool rkIKRegSelect_select_com(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  reg->_attr.user_defined_type &= (~RK_IK_ATTR_TYPE_TARGET);
  reg->_attr.user_defined_type |= RK_IK_ATTR_TYPE_TARGET__COM;
  return true;
}

bool rkIKRegSelect_com(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  int target = reg->_attr.user_defined_type & RK_IK_ATTR_TYPE_TARGET;
  return (target == RK_IK_ATTR_TYPE_TARGET__COM);
}

bool rkIKRegSelect_select_pos(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  reg->_attr.user_defined_type &= (~RK_IK_ATTR_TYPE_QUANTITY);
  reg->_attr.user_defined_type |= RK_IK_ATTR_TYPE_QUANTITY__POS;
  return true;
}

bool rkIKRegSelect_pos(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  int quantity = reg->_attr.user_defined_type & RK_IK_ATTR_TYPE_QUANTITY;
  return (quantity == RK_IK_ATTR_TYPE_QUANTITY__POS);
}

bool rkIKRegSelect_select_att(void* instance){
  if( rkIKRegSelect_com( instance ) ) return false; /* validation */
  rkIKRegister* reg = (rkIKRegister*)(instance);
  reg->_attr.user_defined_type &= (~RK_IK_ATTR_TYPE_QUANTITY);
  reg->_attr.user_defined_type |= RK_IK_ATTR_TYPE_QUANTITY__ATT;
  return true;
}

bool rkIKRegSelect_att(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  int quantity = reg->_attr.user_defined_type & RK_IK_ATTR_TYPE_QUANTITY;
  return (quantity == RK_IK_ATTR_TYPE_QUANTITY__ATT);
}

bool rkIKRegSelect_select_am(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  reg->_attr.user_defined_type &= (~RK_IK_ATTR_TYPE_QUANTITY);
  reg->_attr.user_defined_type |= RK_IK_ATTR_TYPE_QUANTITY__AM;
  return true;
}

bool rkIKRegSelect_am(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  int quantity = reg->_attr.user_defined_type & RK_IK_ATTR_TYPE_QUANTITY;
  return (quantity == RK_IK_ATTR_TYPE_QUANTITY__AM);
}

bool rkIKRegSelect_select_wld_frame(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  reg->_attr.user_defined_type &= (~RK_IK_ATTR_TYPE_REF_FRAME);
  reg->_attr.user_defined_type |= RK_IK_ATTR_TYPE_REF_FRAME__WORLD;
  return true;
}

bool rkIKRegSelect_wld_frame(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  int ref_frame = reg->_attr.user_defined_type & RK_IK_ATTR_TYPE_REF_FRAME;
  return (ref_frame == RK_IK_ATTR_TYPE_REF_FRAME__WORLD);
}

bool rkIKRegSelect_select_sub_link_frame(void* instance){
  if( rkIKRegSelect_com( instance ) ) return false; /* validation. but maybe change */
  if( rkIKRegSelect_am( instance ) ) return false; /* validation */
  rkIKRegister* reg = (rkIKRegister*)(instance);
  reg->_attr.user_defined_type &= (~RK_IK_ATTR_TYPE_REF_FRAME);
  reg->_attr.user_defined_type |= RK_IK_ATTR_TYPE_REF_FRAME__SUB_LINK;
  return true;
}

bool rkIKRegSelect_sub_link_frame(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  int ref_frame = reg->_attr.user_defined_type & RK_IK_ATTR_TYPE_REF_FRAME;
  return (ref_frame == RK_IK_ATTR_TYPE_REF_FRAME__SUB_LINK);
}

bool rkIKRegSelect_select_force(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  reg->_priority = RK_IK_MAX_PRIORITY;
  return true;
}

bool rkIKRegSelect_unselect_force(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  reg->_priority = 0;
  return true;
}

bool rkIKRegSelect_force(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  return reg->_priority == RK_IK_MAX_PRIORITY;
}

/**/

void rkIKRegSelect_set_name(void* instance, const char* name){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  zNameFree( reg );
  zNameSet( reg, name );
}

const char* rkIKRegSelect_get_name(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  return reg->name;
}

void rkIKRegSelect_set_priority(void* instance, int priority){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  if( priority < 0 )
    priority = 0; /* validation */
  if( priority >= RK_IK_MAX_PRIORITY )
    reg->_priority = RK_IK_MAX_PRIORITY - 1;
  else
    reg->_priority = priority;
}

int rkIKRegSelect_get_priority(void *instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  return reg->_priority;
}

void rkIKRegSelect_set_link_id(void* instance, int link_id){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  reg->_attr.id = link_id;
}

int rkIKRegSelect_get_link_id(void *instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  return reg->_attr.id;
}

void rkIKRegSelect_set_ap(void* instance, double v1, double v2, double v3){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKAttrSetAttentionPoint( &reg->_attr, v1, v2, v3 );
}

void rkIKRegSelect_get_ap(void *instance, double *v1, double *v2, double *v3){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  *v1 = reg->_attr.attention_point.c.x;
  *v2 = reg->_attr.attention_point.c.y;
  *v3 = reg->_attr.attention_point.c.z;
}

void rkIKRegSelect_set_weight(void* instance, double w1, double w2, double w3){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKAttrSetWeight( &reg->_attr, w1, w2, w3 );
}

void rkIKRegSelect_get_weight(void *instance, double *w1, double *w2, double *w3){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  *w1 = reg->_attr.weight.c.x;
  *w2 = reg->_attr.weight.c.y;
  *w3 = reg->_attr.weight.c.z;
}

void rkIKRegSelect_set_sub_link_frame_id(void* instance, int sub_link_id){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  reg->_attr.id_sub = sub_link_id;
}

int rkIKRegSelect_get_sub_link_frame_id(void *instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  return reg->_attr.id_sub;
}

/**/

void rkIKRegSelect_reset(void *instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKAttr blank_attr;
  rkIKAttrInit( &blank_attr );
  blank_attr.user_defined_type = 0;
  zCopy( rkIKAttr, &blank_attr, &reg->_attr );
}

const rkIKConstraint* reg_api_factory(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  if( reg->_attr.user_defined_type == RK_IK_ATTR_TYPE__WORLD_LINK_POS ){
    return rkIKConstraintFind( "world_pos" );
  } else
  if( reg->_attr.user_defined_type == RK_IK_ATTR_TYPE__WORLD_LINK_ATT ){
    return rkIKConstraintFind( "world_att" );
  } else
  if( reg->_attr.user_defined_type == RK_IK_ATTR_TYPE__SUB_LINK_LINK_POS ){
    return rkIKConstraintFind( "l2l_pos" );
  } else
  if( reg->_attr.user_defined_type == RK_IK_ATTR_TYPE__SUB_LINK_LINK_ATT ){
    return rkIKConstraintFind( "l2l_att" );
  } else
  if( reg->_attr.user_defined_type == RK_IK_ATTR_TYPE__WORLD_COM_POS ) {
    return rkIKConstraintFind( "com" );
  } else
  if( reg->_attr.user_defined_type == RK_IK_ATTR_TYPE__WORLD_LINK_AM ) {
    return rkIKConstraintFind( "angular_momentum" );
  } else
  if( reg->_attr.user_defined_type == RK_IK_ATTR_TYPE__WORLD_COM_AM ) {
    return rkIKConstraintFind( "angular_momentum_about_com" );
  } else {
    return NULL;
  }
}

const int32_t get_user_defined_type(const char* type)
{
  if( strcmp( type, "world_pos" ) == 0 ){
    return RK_IK_ATTR_TYPE__WORLD_LINK_POS;
  } else
  if( strcmp( type, "world_att" ) == 0 ){
    return RK_IK_ATTR_TYPE__WORLD_LINK_ATT;
  } else
  if( strcmp( type, "l2l_pos" ) == 0 ){
    return RK_IK_ATTR_TYPE__SUB_LINK_LINK_POS;
  } else
  if( strcmp( type, "l2l_att" ) == 0 ){
    return RK_IK_ATTR_TYPE__SUB_LINK_LINK_ATT;
  } else
  if( strcmp( type, "com" ) == 0 ){
    return RK_IK_ATTR_TYPE__WORLD_COM_POS;
  } else
  if( strcmp( type, "angular_momentum" ) == 0 ){
    return RK_IK_ATTR_TYPE__WORLD_LINK_AM;
  } else
  if( strcmp( type, "angular_momentum_about_com" ) == 0 ){
    return RK_IK_ATTR_TYPE__WORLD_COM_AM;
  } else{
    return 0;
  }
}


ubyte mask_factory(void* instance){
  ubyte mask = RK_IK_ATTR_MASK_NONE;
  if( rkIKRegSelect_link( instance ) )
    mask |= RK_IK_ATTR_MASK_ID | RK_IK_ATTR_MASK_ATTENTION_POINT;
  if( rkIKRegSelect_sub_link_frame( instance ) )
    mask |= RK_IK_ATTR_MASK_ID_SUB;
  mask |= RK_IK_ATTR_MASK_WEIGHT;

  return mask;
}

void* rkIKRegSelect_call_reg_api(void* instance, void* chain){
  const rkIKConstraint* lookup = reg_api_factory( instance );
  if( lookup == NULL )
    return NULL;
  ubyte mask = mask_factory( instance );
  rkIKRegister* reg = (rkIKRegister*)(instance);
  int priority = rkIKRegSelect_force( instance ) ? RK_IK_MAX_PRIORITY : reg->_priority;
  rkIKCell* cell = rkChainRegIKCell( (rkChain*)(chain), reg->name, priority, &reg->_attr, mask, lookup, NULL );
  if( cell == NULL ){
    eprintf( "Invalid rkIKAttr Setting Pattern \n" );
    ZRUNERROR( RK_ERR_IK_CELL_NOTFOUND, lookup->typestr );
  }

  return (void*)(cell);
}

void* rkIKRegSelect_from_cell_name(void* chain, const char* name)
{
  rkIKCell* cell = rkChainFindIKCellByName( (rkChain*)(chain), name );
  if( cell == NULL )
    return NULL;
  rkIKRegister* reg;
  rkIKRegSelect_init( (void**)(&reg) );
  zCopy( rkIKAttr, &cell->data.attr, &reg->_attr );
  reg->_attr.user_defined_type = get_user_defined_type( cell->data.constraint->typestr );
  rkIKRegSelect_set_name( (void*)reg, rkIKCellName(cell) );
  int priority = rkIKCellPriority( cell );
  rkIKRegSelect_set_priority( (void*)reg, priority );
  return (void*)(reg);
}

/* just wrapper for encapsulating types */
bool rkIKRegSelect_unreg_by_cell(void *chain, void *cell){
  if( cell == NULL )
    return false;
  return rkChainUnregIKCell( (rkChain*)(chain), (rkIKCell*)(cell) );
}

bool rkIKRegSelect_unreg_by_name(void *chain, const char* name)
{
  rkIKCell* cell = rkChainFindIKCellByName( (rkChain*)(chain), name );
  if( cell == NULL )
    return false;
  return rkChainUnregIKCell( (rkChain*)(chain), (rkIKCell*)(cell) );
}

void* rkIKRegSelect_fromZTK_constraint_key(void* chain, void* ztk)
{
  const rkIKConstraint *constraint;
  rkIKAttr attr;
  ubyte mask = RK_IK_ATTR_MASK_NONE;
  int priority;
  const char *nameptr;
  const char *typestr;
  priority = ZTKInt((ZTK*)ztk);
  nameptr = ZTKVal((ZTK*)ztk);
  rkIKAttrInit( &attr );
  ZTKValNext( (ZTK*)ztk );
  typestr = ZTKVal((ZTK*)ztk);
  if( !( constraint = rkIKConstraintFind( typestr ) ) ) return NULL;
  ZTKValNext( (ZTK*)ztk );
  if( !constraint->fromZTK( (rkChain*)chain, &attr, &mask, (ZTK*)ztk ) ){
    ZRUNERROR( "in persing constraint %s", nameptr );
    return NULL;
  }
  /* set */
  rkIKRegister* reg;
  rkIKRegSelect_init( (void**)(&reg) );
  rkIKRegSelect_set_name( (void*)reg, nameptr );
  zCopy( rkIKAttr, &attr, &reg->_attr );
  reg->_attr.user_defined_type = get_user_defined_type( typestr );
  rkIKRegSelect_set_priority( (void*)reg, priority );

  return (void*)(reg);
}

bool rkIKRegSelect_fprintZTK_as_constraint_key(FILE *fp, void* chain, void* instance)
{
  rkIKCell* cp = rkIKRegSelect_call_reg_api( instance, chain );
  if( cp == NULL )
    return false;
  fprintf( fp, "constraint: %d %s %s", rkIKCellPriority(cp), rkIKCellName(cp), cp->data.constraint->typestr );
  cp->data.constraint->fprintZTK( fp, (rkChain*)chain, cp );
  rkIKRegSelect_unreg_by_cell( chain, cp );
  return true;
}

/* test code that includes the header --------------------------------------*/
#define H5_ZTK "../model/H5.ztk"
int main(int argc, char *argv[])
{
  bool com, link;
  bool pos, att, am;
  bool wld_frame, sub_link_frame;
  bool force;

  void* instance = NULL;
  rkIKRegSelectClass* test = &rkIKRegSelectClassImpl;

  test->init( &instance );
  /**/
  test->select_com( instance );
  com  = test->com( instance );
  link = test->link( instance );
  printf("select_com\n");
  printf("  com     = %d\n", com);
  printf("  link    = %d\n", link);
  /**/
  test->select_link( instance );
  com  = test->com( instance );
  link = test->link( instance );
  printf("select_link\n");
  printf("  com     = %d\n", com);
  printf("  link    = %d\n", link);
  /**/
  test->select_pos( instance );
  pos = test->pos( instance );
  att = test->att( instance );
  am  = test->am( instance );
  printf("select_pos\n");
  printf("  pos = %d\n", pos);
  printf("  att = %d\n", att);
  printf("  am  = %d\n", am);
  /**/
  test->select_att( instance );
  pos = test->pos( instance );
  att = test->att( instance );
  am  = test->am( instance );
  printf("select_att\n");
  printf("  pos = %d\n", pos);
  printf("  att = %d\n", att);
  printf("  am  = %d\n", am);
  /**/
  test->select_am( instance );
  pos = test->pos( instance );
  att = test->att( instance );
  am  = test->am( instance );
  printf("select_am\n");
  printf("  pos = %d\n", pos);
  printf("  att = %d\n", att);
  printf("  am  = %d\n", am);
  /**/
  test->select_pos( instance );
  test->select_wld_frame( instance );
  wld_frame      = test->wld_frame( instance );
  sub_link_frame = test->sub_link_frame( instance );
  printf("select_wld_frame\n");
  printf("  wld_frame      = %d\n", wld_frame);
  printf("  sub_link_frame = %d\n", sub_link_frame);
  /**/
  test->select_sub_link_frame( instance );
  wld_frame      = test->wld_frame( instance );
  sub_link_frame = test->sub_link_frame( instance );
  printf("select_sub_link_frame\n");
  printf("  wld_frame      = %d\n", wld_frame);
  printf("  sub_link_frame = %d\n", sub_link_frame);
  /**/
  test->select_force( instance );
  force  = test->force( instance );
  printf("select_force\n");
  printf("  force  = %d\n", force);
  /**/
  test->unselect_force( instance );
  force  = test->force( instance );
  printf("unselect_force\n");
  printf("  force  = %d\n", force);
  /**/
  int in_priority=999;
  int out_priority;
  int in_link_id=1;
  int out_link_id;
  double in_ap_x=0.1, in_ap_y=0.2, in_ap_z=0.3;
  double out_ap_x, out_ap_y, out_ap_z;
  double in_wx=0.01, in_wy=0.02, in_wz=0.03;
  double out_wx, out_wy, out_wz;
  int in_sub_link_frame_id=6;
  int out_sub_link_frame_id;
  /**/
  const char in_name[] = "set_name";
  printf("set/get_name\n");
  test->set_name( instance, in_name );
  const char* out_name = test->get_name( instance );
  printf( "  get_name = %s : ", out_name );
  printf( "%s\n", ((strcmp( in_name, out_name )==0) ? "OK" : "NG!!" ));
  /**/
  printf("set/get_priority : ");
  test->set_priority( instance, in_priority );
  out_priority = test->get_priority( instance );
  printf( "%s\n", ((in_priority==out_priority) ? "OK" : "NG" ));
  /**/
  printf("set/get_link_id : ");
  test->set_link_id( instance, in_link_id );
  out_link_id = test->get_link_id( instance );
  printf("%s\n", ( (in_link_id==out_link_id) ? "OK" : "NG!!" ));
  printf("  link_id = %d\n", out_link_id);
  /**/
  printf("set/get_ap : ");
  test->set_ap( instance, in_ap_x, in_ap_y, in_ap_z );
  test->get_ap( instance, &out_ap_x, &out_ap_y, &out_ap_z);
  printf("%s\n", ( ((in_ap_x==out_ap_x) && (in_ap_y==out_ap_y) && (in_ap_z==out_ap_z)) ? "OK" : "NG!!" ));
  printf("  v1 = %.2f, v2 = %.2f, v3 = %.2f\n", out_ap_x, out_ap_y, out_ap_z);
  /**/
  printf("set/get_weight : ");
  test->set_weight( instance, in_wx, in_wy, in_wz );
  test->get_weight( instance, &out_wx, &out_wy, &out_wz);
  printf("%s\n", ( ((in_wx==out_wx) && (in_wy==out_wy) && (in_wz==out_wz)) ? "OK" : "NG!!" ));
  printf("  w1 = %.2f, w2 = %.2f, w3 = %.2f\n", out_wx, out_wy, out_wz);
  /**/
  printf("set/get_sub_link_frame_id : ");
  test->set_sub_link_frame_id( instance, in_sub_link_frame_id );
  out_sub_link_frame_id = test->get_sub_link_frame_id( instance );
  printf("%s\n", ( (in_sub_link_frame_id==out_sub_link_frame_id) ? "OK" : "NG!!" ));
  printf("  sub_link_frame_id = %d\n", out_sub_link_frame_id);
  /**/
  void* chain = NULL;
  /* use wrapper as possible */
  rkChain instance_chain;
  chain = &instance_chain;
  rkChainReadZTK( (rkChain*)(chain), H5_ZTK );
  rkChainCreateIK( (rkChain*)(chain) );
  rkChainRegIKJointAll( (rkChain*)(chain), 0.001 ); /* joint weight = 0.01 */
  /**/
  test->reset( instance );
  test->select_link( instance );
  test->select_pos( instance );
  test->select_wld_frame( instance );
  test->set_link_id( instance, in_link_id );
  /* test ztk I/O */
  const char name_wld_pos[] = "test_wld_pos";
  test->set_name( instance, name_wld_pos );
  printf("test ZTK I/O: %s\n  ", name_wld_pos );
  const char ztk_filepath[] = "ik_attr_test.ztk";
  /* fprint_ztk file */
  FILE* fp = fopen( ztk_filepath, "w" );
  test->fprint_ztk( fp, chain, instance );
  fclose( fp );
  /* from_ztk file */
  ZTK ztk;
  ZTKParse( &ztk, ztk_filepath );
  ZTKKeyRewind( &ztk );
  printf( "find constraint key : %s\n  ", ZTKKeyCmp( &ztk, "constraint" ) ? "OK" : "NG" );
  void* instance_02 = test->from_ztk( chain, &ztk );
  /* fprint_ztk as stdout */
  test->fprint_ztk( stdout, chain, instance_02 );
  test->free( &instance_02 );
  /**/
  printf("call reg_api_world_pos : ");
  void* cell_wld_pos = test->reg( instance, chain );
  printf("%s\n", (cell_wld_pos!=NULL ? "OK." : "NG!!"));
  const char* out_name_wld_pos = rkIKCellName( (rkIKCell*)cell_wld_pos );
  printf( "  rkIKCellName = %s : ", out_name_wld_pos );
  printf( "%s\n", ((strcmp(name_wld_pos, out_name_wld_pos)==0) ? "OK" : "NG") );
  bool is_unreg_ok;
  is_unreg_ok = test->unreg( chain, cell_wld_pos );
  printf("  unreg %s\n", (is_unreg_ok ? "OK." : "NG!!"));
  /**/
  test->reset( instance );
  test->select_link( instance );
  test->select_att( instance );
  test->select_wld_frame( instance );
  printf("call reg_api_world_att : ");
  void* cell_wld_att = test->reg( instance, chain );
  printf("%s\n", (cell_wld_att!=NULL ? "OK." : "NG!!"));
  is_unreg_ok = test->unreg( chain, cell_wld_att );
  printf("  unreg %s\n", (is_unreg_ok ? "OK." : "NG!!"));
  /**/
  test->reset( instance );
  test->select_link( instance );
  test->select_pos( instance );
  test->select_sub_link_frame( instance );
  printf("call reg_api_l2l_pos : ");
  void* cell_l2l_pos = test->reg( instance, chain );
  printf("%s\n", (cell_l2l_pos!=NULL ? "OK." : "NG!!"));
  is_unreg_ok = test->unreg( chain, cell_l2l_pos );
  printf("  unreg %s\n", (is_unreg_ok ? "OK." : "NG!!"));
  /**/
  test->reset( instance );
  test->select_link( instance );
  test->select_att( instance );
  test->select_sub_link_frame( instance );
  printf("call reg_api_l2l_att : ");
  void* cell_l2l_att = test->reg( instance, chain );
  printf("%s\n", (cell_l2l_att!=NULL ? "OK." : "NG!!"));
  is_unreg_ok = test->unreg( chain, cell_l2l_att );
  printf("  unreg %s\n", (is_unreg_ok ? "OK." : "NG!!"));
  /**/
  test->reset( instance );
  test->select_com( instance );
  test->select_pos( instance );
  test->select_wld_frame( instance );
  printf("call reg_api_com : ");
  void* cell_com = test->reg( instance, chain );
  printf("%s\n", (cell_com!=NULL ? "OK." : "NG!!"));
  is_unreg_ok = test->unreg( chain, cell_com );
  printf("  unreg %s\n", (is_unreg_ok ? "OK." : "NG!!"));
  /**/
  test->reset( instance );
  test->select_link( instance );
  test->select_am( instance );
  test->select_wld_frame( instance );
  printf("call reg_api_am : ");
  void* cell_am = test->reg( instance, chain );
  printf("%s\n", (cell_am!=NULL ? "OK" : "NG!!"));
  is_unreg_ok = test->unreg( chain, cell_am );
  printf("  unreg %s\n", (is_unreg_ok ? "OK." : "NG!!"));
  /**/
  test->reset( instance );
  test->select_com( instance );
  test->select_am( instance );
  test->select_wld_frame( instance );
  printf("call reg_api_amcom : ");
  void* cell_comam = test->reg( instance, chain );
  printf("%s\n", (cell_comam!=NULL ? "OK" : "NG!!"));
  is_unreg_ok = test->unreg( chain, cell_comam );
  printf("  unreg %s\n", (is_unreg_ok ? "OK." : "NG!!"));

  void* copy_instance = NULL;
  test->init( &copy_instance );
  test->copy( instance, copy_instance );
  int src_link_id = test->get_link_id( instance );
  int copy_link_id = test->get_link_id( copy_instance );
  printf("copy %s\n", (src_link_id==copy_link_id ? "OK." : "NG!"));
  test->free( &instance );
  test->free( &copy_instance );
  printf("free %s\n", (instance==NULL ? "OK." : "NG!"));
  rkChainDestroy( (rkChain*)(chain) );
  return 0;
}
