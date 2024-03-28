#include <roki/rk_chain.h>

/* header */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKAttrSelectable );

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKAttrSelect ){
  void* (*init                )(void**);
  void (*select_com           )(void*);
  bool (*com                  )(void*);
  void (*select_link          )(void*);
  bool (*link                 )(void*);
  void (*select_sub_link_frame)(void*);
  bool (*sub_link_frame       )(void*);
  void (*select_link_org      )(void*);
  bool (*link_org             )(void*);
  void (*select_link_ap       )(void*);
  bool (*link_ap              )(void*);
  void (*select_force         )(void*);
  bool (*force                )(void*);
  void (*select_weight        )(void*);
  bool (*weight               )(void*);
};

/* declaration */
void* init                (void** instance);
void select_com           (void* instance);
bool com                  (void* instance);
void select_link          (void* instance);
bool link                 (void* instance);
void select_sub_link_frame(void* instance);
bool sub_link_frame       (void* instance);
void select_link_org      (void* instance);
bool link_org             (void* instance);
void select_link_ap       (void* instance);
bool link_ap              (void* instance);
void select_force         (void* instance);
bool force                (void* instance);
void select_weight        (void* instance);
bool weight               (void* instance);

static rkIKAttrSelect rkIKAttrSelectClass = {
  init,
  select_com,
  com,
  select_link,
  link,
  select_sub_link_frame,
  sub_link_frame,
  select_link_org,
  link_org,
  select_link_ap,
  link_ap,
  select_force,
  force,
  select_weight,
  weight
};

/* implement */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKAttrSelectable ){
  /* target */
  bool _com; /* Center Of Mass */
  bool _link; /* link */
  /* target of the link */
  bool _link_org; /* the origin of link frame */
  bool _link_ap; /* the attented point of link */
  /* value type */
  bool _pos;
  bool _att;
  /* reference frame */
  bool _wld_frame; /* world frame */
  bool _sub_link_frame; /* sub link frame */
  /* priority */
  bool _force; /* priority is forced to be max */
  bool _weight;
};

void* init(void** instance)
{
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(*instance);
  sel = zAlloc( rkIKAttrSelectable, 1 );
  *instance = (void*)(sel);
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

void select_sub_link_frame(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_sub_link_frame = true;
  sel->_wld_frame = !sel->_sub_link_frame; /* false */
}

bool sub_link_frame(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_sub_link_frame;
}

void select_link_org(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_link_org = true;
  sel->_link_ap = !sel->_link_org;
}

bool link_org(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_link_org;
}

void select_link_ap(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_link_ap = true;
  sel->_link_org = !sel->_link_ap;
}

bool link_ap(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_link_ap;
}

void select_force(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_force = true;
  sel->_weight = !sel->_force;
}

bool force(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_force;
}

void select_weight(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  sel->_weight = true;
  sel->_force = !sel->_weight;
}

bool weight(void* instance){
  rkIKAttrSelectable* sel = (rkIKAttrSelectable*)(instance);
  return sel->_weight;
}

int main(int argc, char *argv[])
{
  bool com, link, link_org, link_ap;
  void* instance = NULL;
  rkIKAttrSelectClass.init( &instance );
  /**/
  rkIKAttrSelectClass.select_com( instance );
  com  = rkIKAttrSelectClass.com( instance );
  link = rkIKAttrSelectClass.link( instance );
  printf("select_com\n");
  printf("  com  = %d\n", com);
  printf("  link = %d\n", link);
  /**/
  rkIKAttrSelectClass.select_link( instance );
  com  = rkIKAttrSelectClass.com( instance );
  link = rkIKAttrSelectClass.link( instance );
  printf("select_link\n");
  printf("  com  = %d\n", com);
  printf("  link = %d\n", link);
  /**/
  rkIKAttrSelectClass.select_sub_link_frame( instance );

  zFree( instance );
  return 0;
}
