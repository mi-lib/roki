#include <roki/rk_chain.h>

/* header .h -------------------------------------------------------------- */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKAttrSelectClass ){
  void* (*init                )(void**,void*);
  void (*select_com           )(void*);
  bool (*com                  )(void*);
  void (*select_link          )(void*);
  bool (*link                 )(void*);
  bool (*select_link_org      )(void*);
  bool (*link_org             )(void*);
  bool (*select_link_ap       )(void*);
  bool (*link_ap              )(void*);
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
  void (*set_ap               )(void*,double,double,double);
  void (*set_weight           )(void*,double,double,double);
  void (*set_sub_link_frame_id)(void*,int);
  void* (*call_api            )(void*);
};


/* declaration */
void* init                (void **instance, void *chain);
void select_com           (void *instance);
bool com                  (void *instance);
void select_link          (void *instance);
bool link                 (void *instance);
bool select_link_org      (void *instance);
bool link_org             (void *instance);
bool select_link_ap       (void *instance);
bool link_ap              (void *instance);
bool select_pos           (void *instance);
bool pos                  (void *instance);
bool select_att           (void *instance);
bool att                  (void *instance);
bool select_am            (void *instance);
bool am                   (void *instance);
bool select_wld_frame     (void *instance);
bool wld_frame            (void *instance);
bool select_sub_link_frame(void *instance);
bool sub_link_frame       (void *instance);
bool select_force         (void *instance);
bool force                (void *instance);
bool select_weight        (void *instance);
bool weight               (void *instance);
void set_link_id          (void *instance, int link_id);
void set_ap               (void *instance, double v1, double v2, double v3);
void set_weight           (void *instance, double w1, double w2, double w3);
void set_sub_link_frame_id(void *instance, int sub_link_id);
void* call_api            (void *instance);

static rkIKAttrSelectClass rkIKAttrSelectClassImpl = {
  init,
  select_com,
  com,
  select_link,
  link,
  select_link_org,
  link_org,
  select_link_ap,
  link_ap,
  select_pos,
  pos,
  select_att,
  att,
  select_am,
  am,
  select_wld_frame,
  wld_frame,
  select_sub_link_frame,
  sub_link_frame,
  select_force,
  force,
  select_weight,
  weight,
  set_link_id,
  set_ap,
  set_weight,
  set_sub_link_frame_id,
  call_api
};


/* implement .c (capsuled) ------------------------------------------------ */

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKAttrSelectable ){
  /* target */
  bool _com;  /* Center Of Mass */
  bool _link; /* link */
  /* target of the link */
  bool _link_org; /* the origin of link frame */
  bool _link_ap;  /* the attented point of link */
  /* value type */
  bool _pos; /* position */
  bool _att; /* attitude */
  bool _am;  /* angular momentum */
  /* reference frame */
  bool _wld_frame;      /* world frame */
  bool _sub_link_frame; /* sub link frame */
  /* priority */
  bool _force;  /* weight is forced to be max */
  bool _weight;
  /* arguments for call api */
  rkChain* _chain;
  rkIKAttr _attr;
};

/**/

void* init(void** instance, void* chain)
{
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(*instance);
  sel = zAlloc( rkIKAttrSelectable, 1 );
  *instance = (void*)(sel);
  sel->_chain = (rkChain*)(chain);
  return sel;
}

void select_com(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_com = true;
  sel->_link     = !sel->_com; /* false */
  sel->_link_org = !sel->_com;
  sel->_link_ap  = !sel->_com;
}

bool com(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_com;
}

void select_link(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_link = true;
  sel->_com = !sel->_link; /* false */
}

bool link(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_link;
}

bool select_link_org(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  if(sel->_com) return false; /* validation */
  sel->_link_org = true;
  sel->_link_ap = !sel->_link_org;
  return true;
}

bool link_org(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_link_org;
}

bool select_link_ap(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  if(sel->_com) return false; /* validation */
  sel->_link_ap = true;
  sel->_link_org = !sel->_link_ap;
  return true;
}

bool link_ap(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_link_ap;
}

bool select_pos(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_pos = true;
  sel->_att = !sel->_pos;
  sel->_am  = !sel->_pos;
  return true;
}

bool pos(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_pos;
}

bool select_att(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  if(sel->_com) return false; /* validation */
  sel->_att = true;
  sel->_pos = !sel->_att;
  sel->_am  = !sel->_att;
  return true;
}

bool att(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_att;
}

bool select_am(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_am = true;
  sel->_pos = !sel->_am;
  sel->_att = !sel->_am;
  return true;
}

bool am(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_am;
}

bool select_wld_frame(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_wld_frame = true;
  sel->_sub_link_frame = !sel->_wld_frame;
  return true;
}

bool wld_frame(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_wld_frame;
}

bool select_sub_link_frame(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  if(sel->_com) return false; /* validation */
  if(sel->_am) return false; /* validation */
  sel->_sub_link_frame = true;
  sel->_wld_frame = !sel->_sub_link_frame;
  return true;
}

bool sub_link_frame(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_sub_link_frame;
}

bool select_force(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_force = true;
  sel->_weight = !sel->_force;
  return true;
}

bool force(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_force;
}

bool select_weight(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_weight = true;
  sel->_force = !sel->_weight;
  return true;
}

bool weight(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_weight;
}

/**/

void set_link_id(void* instance, int link_id){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_attr.id = link_id;
}

void set_ap(void* instance, double v1, double v2, double v3){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  rkIKAttrSetAP( &sel->_attr, v1, v2, v3 );
}

void set_weight(void* instance, double w1, double w2, double w3){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  rkIKAttrSetWeight( &sel->_attr, w1, w2, w3 );
}

void set_sub_link_frame_id(void* instance, int sub_link_id){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_attr.id_sub = sub_link_id;
}

/**/

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, _rkIKLookup ){
  rkIKCell *(*reg_ik_cell)(rkChain*,rkIKAttr*,int);
};

static _rkIKLookup api_world_pos = { rkChainRegIKCellWldPos };
static _rkIKLookup api_world_att = { rkChainRegIKCellWldAtt };
static _rkIKLookup api_l2l_pos   = { rkChainRegIKCellL2LPos };
static _rkIKLookup api_l2l_att   = { rkChainRegIKCellL2LAtt };
static _rkIKLookup api_com       = { rkChainRegIKCellCOM    };
static _rkIKLookup api_am        = { rkChainRegIKCellAM     };
static _rkIKLookup api_amcom     = { rkChainRegIKCellAMCOM  };

_rkIKLookup* api_factory(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  _rkIKLookup *lookup = NULL;
  if( sel->_link &&
      sel->_pos &&
      sel->_wld_frame ){
    lookup = &api_world_pos;
  } else
  if( sel->_link &&
      sel->_att &&
      sel->_wld_frame ) {
    lookup = &api_world_att;
  } else
  if( sel->_link &&
      sel->_pos &&
      sel->_sub_link_frame ) {
    lookup = &api_l2l_pos;
  } else
  if( sel->_link &&
      sel->_att &&
      sel->_sub_link_frame ) {
    lookup = &api_l2l_att;
  } else
  if( sel->_com &&
      sel->_pos &&
      sel->_wld_frame ) {
    lookup = &api_com;
  } else
  if( sel->_link &&
      sel->_am &&
      sel->_wld_frame ) {
    lookup = &api_am;
  } else
  if( sel->_com &&
      sel->_am &&
      sel->_wld_frame ) {
    lookup = &api_amcom;
  } else {
    ZRUNERROR( RK_ERR_IK_UNKNOWN, "Invalid rkIKAttr Setting Pattern" );
    return NULL;
  }
  return lookup;
}

int mask_factory(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  int mask = RK_IK_ATTR_NONE;
  if( sel->_link_ap )
    mask |= RK_IK_ATTR_AP;
  if( sel->_weight )
    mask |= RK_IK_ATTR_WEIGHT;
  if( sel->_force )
    mask |= RK_IK_ATTR_FORCE;
  if( sel->_link )
    mask |= RK_IK_ATTR_ID;
  if( sel->_sub_link_frame )
    mask |= RK_IK_ATTR_ID_SUB;

  return mask;
}

void* call_api(void* instance){
  _rkIKLookup *lookup = api_factory( instance );
  if( lookup == NULL )
    return NULL;
  int mask = mask_factory( instance );
  rkIKAttrSelectable *sel = (rkIKAttrSelectable*)(instance);
  rkIKCell *cell = lookup->reg_ik_cell( sel->_chain, &sel->_attr, mask );
  return (void*)(cell);
}


/* test code that includes the header --------------------------------------*/
int main(int argc, char *argv[])
{
  rkChain chain;

  bool com, link;
  bool link_org, link_ap;
  bool pos, att, am;
  bool wld_frame, sub_link_frame;
  bool force, weight;

  void* instance = NULL;
  rkIKAttrSelectClass* test = &rkIKAttrSelectClassImpl;

  test->init( &instance, &chain );
  /**/
  test->select_com( instance );
  com  = test->com( instance );
  link = test->link( instance );
  printf("select_com\n");
  printf("  com  = %d\n", com);
  printf("  link = %d\n", link);
  /**/
  test->select_link( instance );
  com  = test->com( instance );
  link = test->link( instance );
  printf("select_link\n");
  printf("  com  = %d\n", com);
  printf("  link = %d\n", link);
  /**/
  test->select_link_org( instance );
  link_org = test->link_org( instance );
  link_ap  = test->link_ap( instance );
  printf("select_link_org\n");
  printf("  link_org = %d\n", link_org);
  printf("  link_ap  = %d\n", link_ap);
  /**/
  test->select_link_ap( instance );
  link_org = test->link_org( instance );
  link_ap  = test->link_ap( instance );
  printf("select_link_ap\n");
  printf("  link_org = %d\n", link_org);
  printf("  link_ap  = %d\n", link_ap);
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

  zFree( instance );
  return 0;
}
