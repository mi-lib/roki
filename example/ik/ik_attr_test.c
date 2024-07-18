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
  bool (*force                )(void*);
  bool (*select_weight        )(void*);
  bool (*weight               )(void*);
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
  void* (*get_ptr_by_name     )(void*,const char*);
  bool (*unreg                )(void*,void*);
  bool (*unreg_by_name        )(void*,const char*);
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
bool rkIKRegSelect_force                (void *instance);
bool rkIKRegSelect_select_weight        (void *instance);
bool rkIKRegSelect_weight               (void *instance);
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
void* rkIKRegSelect_get_pointer_by_name (void* chain, const char* name);
bool rkIKRegSelect_unreg_by_cell        (void *chain, void* cell);
bool rkIKRegSelect_unreg_by_name        (void *chain, const char* name);

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
  rkIKRegSelect_force,
  rkIKRegSelect_select_weight,
  rkIKRegSelect_weight,
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
  rkIKRegSelect_get_pointer_by_name,
  rkIKRegSelect_unreg_by_cell,
  rkIKRegSelect_unreg_by_name
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
  int _priority;
  ubyte _mode;
  rkIKAttr _attr;
};

void* rkIKRegSelect_init(void** instance)
{
  rkIKRegister* reg = (rkIKRegister*)(*instance);
  reg = zAlloc( rkIKRegister, 1 );
  rkIKAttrInit( &reg->_attr );
  reg->_attr.user_defined_type = 0;
  *instance = (void*)(reg);

  return instance;
}

void rkIKRegSelect_copy(void* src, void* dest)
{
  zCopy( rkIKRegister, (rkIKRegister*)(src), (rkIKRegister*)(dest) );
}

void rkIKRegSelect_free(void **instance)
{
  rkIKRegister* reg = (rkIKRegister*)(*instance);
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
  reg->_mode = RK_IK_CELL_MODE_FORCE;
  return true;
}

bool rkIKRegSelect_force(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  return ( (reg->_mode & RK_IK_CELL_MODE_FORCE) == RK_IK_CELL_MODE_FORCE );
}

bool rkIKRegSelect_select_weight(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  reg->_mode = 0x00;
  return true;
}

bool rkIKRegSelect_weight(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  return ( (reg->_mode & RK_IK_CELL_MODE_FORCE) != RK_IK_CELL_MODE_FORCE );
}

/**/

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

/**/

/* ZDEF_STRUCT( __ROKI_CLASS_EXPORT, _rkIKLookup ){ */
/*   rkIKCell *(*reg_ik_cell)(rkChain*,const char*,rkIKAttr*,int); */
/* }; */

/* static _rkIKLookup reg_api_world_pos = { rkChainRegIKCellWldPos }; */
/* static _rkIKLookup reg_api_world_att = { rkChainRegIKCellWldAtt }; */
/* static _rkIKLookup reg_api_l2l_pos   = { rkChainRegIKCellL2LPos }; */
/* static _rkIKLookup reg_api_l2l_att   = { rkChainRegIKCellL2LAtt }; */
/* static _rkIKLookup reg_api_com       = { rkChainRegIKCellCOM    }; */
/* static _rkIKLookup reg_api_am        = { rkChainRegIKCellAM     }; */
/* static _rkIKLookup reg_api_amcom     = { rkChainRegIKCellAMCOM  }; */

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


int mask_factory(void* instance){
  int mask = RK_IK_ATTR_MASK_NONE;
  if( rkIKRegSelect_link( instance ) )
    mask |= RK_IK_ATTR_MASK_ID | RK_IK_ATTR_MASK_ATTENTION_POINT;
  if( rkIKRegSelect_sub_link_frame( instance ) )
    mask |= RK_IK_ATTR_MASK_ID_SUB;
  if( rkIKRegSelect_weight( instance ) )
    mask |= RK_IK_ATTR_MASK_WEIGHT;

  return mask;
}

void* rkIKRegSelect_call_reg_api(void* instance, void* chain){
  const rkIKConstraint* lookup = reg_api_factory( instance );
  if( lookup == NULL )
    return NULL;
  int mask = mask_factory( instance );
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKCell* cell = rkChainRegIKCell( (rkChain*)(chain), lookup->typestr, &reg->_attr, mask, lookup, NULL );
  if( cell == NULL )
    ZRUNERROR( RK_ERR_IK_UNKNOWN, "Invalid rkIKAttr Setting Pattern" );
  if( rkIKRegSelect_force( instance ) )
    rkIKCellForce( cell );

  return (void*)(cell);
}

void* rkIKRegSelect_get_pointer_by_name(void* chain, const char* name)
{
  rkIKCell* cell = rkChainFindIKCellByName( (rkChain*)(chain), name );
  if( cell == NULL )
    return NULL;
  rkIKRegister* reg;
  rkIKRegSelect_init( (void**)(&reg) );
  zCopy( rkIKAttr, &cell->data.attr, &reg->_attr );
  if( rkIKCellIsForced( cell ) )
    rkIKRegSelect_select_force( (void*)reg );
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


/* test code that includes the header --------------------------------------*/
#define H5_ZTK "../model/H5.ztk"
int main(int argc, char *argv[])
{
  bool com, link;
  bool pos, att, am;
  bool wld_frame, sub_link_frame;
  bool force, weight;

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
  weight = test->weight( instance );
  printf("select_force\n");
  printf("  force  = %d\n", force);
  printf("  weight = %d\n", weight);
  /**/
  test->select_weight( instance );
  force  = test->force( instance );
  weight = test->weight( instance );
  printf("select_weight\n");
  printf("  force  = %d\n", force);
  printf("  weight = %d\n", weight);
  /**/
  int in_link_id=1;
  int out_link_id;
  double in_ap_x=0.1, in_ap_y=0.2, in_ap_z=0.3;
  double out_ap_x, out_ap_y, out_ap_z;
  double in_wx=0.01, in_wy=0.02, in_wz=0.03;
  double out_wx, out_wy, out_wz;
  int in_sub_link_frame_id=6;
  int out_sub_link_frame_id;
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
  printf("call reg_api_world_pos : ");
  void* cell_wld_pos = test->reg( instance, chain );
  printf("%s\n", (cell_wld_pos!=NULL ? "OK." : "NG!!"));
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
