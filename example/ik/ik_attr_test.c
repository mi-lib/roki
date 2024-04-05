#include <roki/rk_chain.h>

/* header .h -------------------------------------------------------------- */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKRegSelectClass ){
  void* (*init                )(void**);
  void (*copy                 )(void*,void*);
  void (*free                 )(void**);
  void (*select_com           )(void*);
  bool (*com                  )(void*);
  void (*select_link          )(void*);
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
  bool (*unreg                )(void*,void*);
};

/* declaration */
void* rkIKRegSelect_init                (void **instance);
void rkIKRegSelect_copy                 (void *src, void *dest);
void rkIKRegSelect_free                 (void **instance);
void rkIKRegSelect_select_com           (void *instance);
bool rkIKRegSelect_com                  (void *instance);
void rkIKRegSelect_select_link          (void *instance);
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
bool rkIKRegSelect_call_unreg_api       (void *chain, void* cell);

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
  rkIKRegSelect_call_unreg_api
};

/* implement .c (capsuled) ------------------------------------------------ */

typedef enum{
  RK_IK_TARGET_NONE,
  RK_IK_TARGET_COM,    /* Center Of Mass */
  RK_IK_TARGET_LINK   /* the link with attented point */
} rkIKTargetType;

typedef enum{
  RK_IK_TARGET_QUANTITY_NONE,
  RK_IK_TARGET_QUANTITY_POS,  /* position */
  RK_IK_TARGET_QUANTITY_ATT,  /* attitude */
  RK_IK_TARGET_QUANTITY_AM,   /* angular momentum */
} rkIKTargetQuantityType;

typedef enum{
  RK_IK_REF_FRAME_NONE,
  RK_IK_REF_FRAME_WLD,      /* on world frame */
  RK_IK_REF_FRAME_SUB_LINK, /* on sub link frame */
} rkIKReferenceFrameType;

typedef enum{
  RK_IK_PRIORITY_NONE,
  RK_IK_PRIORITY_FORCE, /* weight is forced to be max */
  RK_IK_PRIORITY_WEIGHT
} rkIKPriorityType;

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKRegSelectable ){
  /* target */
  rkIKTargetType _target;
  rkIKTargetQuantityType _quantity;
  rkIKReferenceFrameType _ref_frame;
  rkIKPriorityType _priority;
};

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKRegister ){
  /* arguments for call api */
  rkIKRegSelectable _sel;
  rkIKAttr _attr;
};

/**/

void* rkIKRegSelect_init(void** instance)
{
  rkIKRegister* reg = (rkIKRegister*)(*instance);
  reg = zAlloc( rkIKRegister, 1 );
  rkIKAttrSetAP( &reg->_attr, 0.0, 0.0, 0.0 );
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

void rkIKRegSelect_select_com(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  sel->_target = RK_IK_TARGET_COM;
}

bool rkIKRegSelect_com(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  return (sel->_target == RK_IK_TARGET_COM);
}

void rkIKRegSelect_select_link(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  sel->_target = RK_IK_TARGET_LINK;
}

bool rkIKRegSelect_link(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  return (sel->_target == RK_IK_TARGET_LINK);
}

bool rkIKRegSelect_select_pos(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  sel->_quantity = RK_IK_TARGET_QUANTITY_POS;
  return true;
}

bool rkIKRegSelect_pos(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  return (sel->_quantity == RK_IK_TARGET_QUANTITY_POS);
}

bool rkIKRegSelect_select_att(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  if( sel->_target == RK_IK_TARGET_COM ) return false; /* validation */
  sel->_quantity = RK_IK_TARGET_QUANTITY_ATT;
  return true;
}

bool rkIKRegSelect_att(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  return (sel->_quantity == RK_IK_TARGET_QUANTITY_ATT);
}

bool rkIKRegSelect_select_am(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  sel->_quantity = RK_IK_TARGET_QUANTITY_AM;
  return true;
}

bool rkIKRegSelect_am(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  return (sel->_quantity == RK_IK_TARGET_QUANTITY_AM);
}

bool rkIKRegSelect_select_wld_frame(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  sel->_ref_frame = RK_IK_REF_FRAME_WLD;
  return true;
}

bool rkIKRegSelect_wld_frame(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  return (sel->_ref_frame == RK_IK_REF_FRAME_WLD);
}

bool rkIKRegSelect_select_sub_link_frame(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  if( sel->_target == RK_IK_TARGET_COM ) return false; /* validation */
  if( sel->_quantity == RK_IK_TARGET_QUANTITY_AM ) return false; /* validation */
  sel->_ref_frame = RK_IK_REF_FRAME_SUB_LINK;
  return true;
}

bool rkIKRegSelect_sub_link_frame(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  return (sel->_ref_frame == RK_IK_REF_FRAME_SUB_LINK);
}

bool rkIKRegSelect_select_force(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  sel->_priority = RK_IK_PRIORITY_FORCE;
  return true;
}

bool rkIKRegSelect_force(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  return (sel->_priority == RK_IK_PRIORITY_FORCE);
}

bool rkIKRegSelect_select_weight(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  sel->_priority = RK_IK_PRIORITY_WEIGHT;
  return true;
}

bool rkIKRegSelect_weight(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  return (sel->_priority == RK_IK_PRIORITY_WEIGHT);
}

/**/

void rkIKRegSelect_set_link_id(void* instance, int link_id){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKAttr* attr = &reg->_attr;
  attr->id = link_id;
}

int rkIKRegSelect_get_link_id(void *instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  return reg->_attr.id;
}

void rkIKRegSelect_set_ap(void* instance, double v1, double v2, double v3){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKAttr* attr = &reg->_attr;
  rkIKAttrSetAP( attr, v1, v2, v3 );
}

void rkIKRegSelect_get_ap(void *instance, double *v1, double *v2, double *v3){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKAttr* attr = &reg->_attr;
  *v1 = attr->ap.c.x;
  *v2 = attr->ap.c.y;
  *v3 = attr->ap.c.z;
}

void rkIKRegSelect_set_weight(void* instance, double w1, double w2, double w3){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKAttr* attr = &reg->_attr;
  rkIKAttrSetWeight( attr, w1, w2, w3 );
}

void rkIKRegSelect_get_weight(void *instance, double *w1, double *w2, double *w3){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKAttr* attr = &reg->_attr;
  *w1 = attr->w.c.x;
  *w2 = attr->w.c.y;
  *w3 = attr->w.c.z;
}

void rkIKRegSelect_set_sub_link_frame_id(void* instance, int sub_link_id){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKAttr* attr = &reg->_attr;
  attr->id_sub = sub_link_id;
}

int rkIKRegSelect_get_sub_link_frame_id(void *instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  return reg->_attr.id_sub;
}

/**/

void rkIKRegSelect_reset(void *instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable defalut_sel = {RK_IK_TARGET_NONE, RK_IK_TARGET_QUANTITY_NONE, RK_IK_REF_FRAME_NONE, RK_IK_PRIORITY_NONE};
  zCopy( rkIKRegSelectable, &defalut_sel, &reg->_sel );
  rkIKAttr blank_attr;
  zCopy( rkIKAttr, &blank_attr, &reg->_attr );
}

/**/

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, _rkIKLookup ){
  rkIKCell *(*reg_ik_cell)(rkChain*,rkIKAttr*,int);
};

static _rkIKLookup reg_api_world_pos = { rkChainRegIKCellWldPos };
static _rkIKLookup reg_api_world_att = { rkChainRegIKCellWldAtt };
static _rkIKLookup reg_api_l2l_pos   = { rkChainRegIKCellL2LPos };
static _rkIKLookup reg_api_l2l_att   = { rkChainRegIKCellL2LAtt };
static _rkIKLookup reg_api_com       = { rkChainRegIKCellCOM    };
static _rkIKLookup reg_api_am        = { rkChainRegIKCellAM     };
static _rkIKLookup reg_api_amcom     = { rkChainRegIKCellAMCOM  };

_rkIKLookup* reg_api_factory(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  _rkIKLookup *lookup = NULL;
  if( sel->_target == RK_IK_TARGET_LINK &&
      sel->_quantity == RK_IK_TARGET_QUANTITY_POS &&
      sel->_ref_frame == RK_IK_REF_FRAME_WLD ){
    lookup = &reg_api_world_pos;
  } else
  if( sel->_target == RK_IK_TARGET_LINK &&
      sel->_quantity == RK_IK_TARGET_QUANTITY_ATT &&
      sel->_ref_frame == RK_IK_REF_FRAME_WLD ) {
    lookup = &reg_api_world_att;
  } else
  if( sel->_target == RK_IK_TARGET_LINK &&
      sel->_quantity == RK_IK_TARGET_QUANTITY_POS &&
      sel->_ref_frame == RK_IK_REF_FRAME_SUB_LINK ) {
    lookup = &reg_api_l2l_pos;
  } else
  if( sel->_target == RK_IK_TARGET_LINK &&
      sel->_quantity == RK_IK_TARGET_QUANTITY_ATT &&
      sel->_ref_frame == RK_IK_REF_FRAME_SUB_LINK ) {
    lookup = &reg_api_l2l_att;
  } else
  if( sel->_target == RK_IK_TARGET_COM &&
      sel->_quantity == RK_IK_TARGET_QUANTITY_POS &&
      sel->_ref_frame == RK_IK_REF_FRAME_WLD ) {
    lookup = &reg_api_com;
  } else
  if( sel->_target == RK_IK_TARGET_LINK &&
      sel->_quantity == RK_IK_TARGET_QUANTITY_AM &&
      sel->_ref_frame == RK_IK_REF_FRAME_WLD ) {
    lookup = &reg_api_am;
  } else
  if( sel->_target == RK_IK_TARGET_COM &&
      sel->_quantity == RK_IK_TARGET_QUANTITY_AM &&
      sel->_ref_frame == RK_IK_REF_FRAME_WLD ) {
    lookup = &reg_api_amcom;
  } else {
    return NULL;
  }

  return lookup;
}

int mask_factory(void* instance){
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKRegSelectable* sel = &reg->_sel;
  int mask = RK_IK_ATTR_NONE;
  if( sel->_target == RK_IK_TARGET_LINK )
    mask |= RK_IK_ATTR_ID | RK_IK_ATTR_AP;
  if( sel->_ref_frame == RK_IK_REF_FRAME_SUB_LINK )
    mask |= RK_IK_ATTR_ID_SUB;
  if( sel->_priority == RK_IK_PRIORITY_WEIGHT )
    mask |= RK_IK_ATTR_WEIGHT;
  if( sel->_priority == RK_IK_PRIORITY_FORCE )
    mask |= RK_IK_ATTR_FORCE;

  return mask;
}

void* rkIKRegSelect_call_reg_api(void* instance, void* chain){
  _rkIKLookup *lookup = reg_api_factory( instance );
  if( lookup == NULL )
    return NULL;
  int mask = mask_factory( instance );
  rkIKRegister* reg = (rkIKRegister*)(instance);
  rkIKCell* cell = lookup->reg_ik_cell( (rkChain*)(chain), &reg->_attr, mask );
  if( cell == NULL )
    ZRUNERROR( RK_ERR_IK_UNKNOWN, "Invalid rkIKAttr Setting Pattern" );

  return (void*)(cell);
}

/* just wrapper for encapsulating types */
bool rkIKRegSelect_call_unreg_api(void *chain, void *cell){
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
