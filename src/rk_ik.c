/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_ik - inverse kinematics
 */

#include <roki/rk_ik.h>

/* ********************************************************** */
/* CLASS: rkIK
 * inverse kinematics class
 * ********************************************************** */

/* initialize an inverse kinematics solver. */
static void _rkIKInit(rkIK *ik)
{
  ik->joint_sw = NULL;
  ik->joint_weight = NULL;
  ik->joint_vel = NULL;
  ik->eval = 0;

  zListInit( &ik->clist );
  ik->_c_mat_cell = NULL;
  zVec3DZero( &ik->_c_srv_cell );

  ik->_j_idx = NULL;
  ik->_j_ofs = NULL;
  ik->_j_vel = NULL;
  ik->_j_wn = NULL;
  ik->_c_mat = NULL;
  ik->_c_srv = NULL;
  ik->_c_we = NULL;
  /* default joint velocity computation method */
  ik->_jv = rkIKJointVelAD;
  ik->__c = NULL;
  zLEInit( &ik->__le );
}

/* create an inverse kinematics solver for a kinematic chain. */
rkChain *rkChainCreateIK(rkChain *chain)
{
  if( !( chain->_ik = zAlloc( rkIK, 1 ) ) ) return NULL;
  _rkIKInit( chain->_ik );
  chain->_ik->joint_sw = zAlloc( bool, rkChainLinkNum(chain) );
  chain->_ik->joint_weight = zAlloc( double, rkChainLinkNum(chain) );
  chain->_ik->joint_vel = zVecAlloc( rkChainJointSize(chain) );
  chain->_ik->_j_ofs = zIndexCreate( rkChainLinkNum(chain) );
  chain->_ik->_c_mat_cell = zMatAlloc( 3, rkChainJointSize(chain) );
  if( !chain->_ik->joint_sw || !chain->_ik->joint_weight ||
      !chain->_ik->joint_vel || !chain->_ik->_c_mat_cell ){
    ZALLOCERROR();
    rkChainDestroyIK( chain );
    return NULL;
  }
  return chain;
}

/* destroy an inverse kinematics solver. */
static void _rkIKDestroy(rkIK *ik)
{
  zFree( ik->joint_sw );
  zFree( ik->joint_weight );
  zVecFree( ik->joint_vel );
  ik->eval = 0;
  zListDestroy( rkIKCell, &ik->clist );
  zMatFree( ik->_c_mat_cell );

  zIndexFree( ik->_j_idx );
  zIndexFree( ik->_j_ofs );
  zVecFree( ik->_j_vel );
  zVecFree( ik->_j_wn );
  zMatFree( ik->_c_mat );
  zVecFree( ik->_c_srv );
  zVecFree( ik->_c_we );
  ik->_jv = NULL;
  zVecFree( ik->__c );
  zLEFree( &ik->__le );
}

/* destroy an inverse kinematics solver of a kinematic chain. */
void rkChainDestroyIK(rkChain *chain)
{
  if( chain->_ik ){
    _rkIKDestroy( chain->_ik );
    zFree( chain->_ik );
  }
}

/* allocate working memory for constraint coefficient matrix of inverse kinematics solver. */
static bool _rkIKAllocCMat(rkIK *ik)
{
  if( zListSize(&ik->clist) == 0 || zArraySize(ik->_j_idx) == 0 )
    return true;
  zMatFree( ik->_c_mat );
  if( !( ik->_c_mat = zMatAlloc( zListSize(&ik->clist)*3, zVecSizeNC(ik->_j_vel) ) ) ){
    ZALLOCERROR();
    return false;
  }
  return true;
}

/* register/unregister a cooperating joint to inverse kinematics solver. */
static bool _rkIKAllocJointIndex(rkIK *ik, rkChain *chain)
{
  uint count, ofs, i;
  int j;
  double *wp;

  for( count=0, i=0; i<rkChainLinkNum(chain); i++ ){
    if( rkChainLinkJointSize(chain,i) == 0 )
      ik->joint_sw[i] = false;
    if( ik->joint_sw[i] ) count++;
  }
  if( count == 0 ) return true;
  zIndexFree( ik->_j_idx );
  if( !( ik->_j_idx = zIndexCreate(count) ) ){
    ZALLOCERROR();
    return false;
  }
  for( count=0, ofs=0, i=0; i<rkChainLinkNum(chain); i++ )
    if( ik->joint_sw[i] ){
      zIndexSetElemNC( ik->_j_idx, count++, i );
      zIndexSetElemNC( ik->_j_ofs, i, ofs );
      ofs += rkChainLinkJointSize(chain,i);
    } else
      zIndexSetElemNC( ik->_j_ofs, i, -1 );
  /* allocate joint vector */
  zVecFree( ik->_j_vel );
  zVecFree( ik->_j_wn );
  zLEFree( &ik->__le );
  count = rkChainJointIndexSize( chain, ik->_j_idx );
  if( !( ik->_j_vel = zVecAlloc(count) ) ||
      !( ik->_j_wn = zVecAlloc(count) ) ||
      !zLEAlloc( &ik->__le, NULL, count ) ){
    ZALLOCERROR();
    return false;
  }
  for( wp=zVecBuf(ik->_j_wn), i=0; i<zArraySize(ik->_j_idx); i++ )
    for( j=0; j<rkChainLinkJointSize(chain,zIndexElemNC(ik->_j_idx,i)); j++ )
      *wp++ = ik->joint_weight[zIndexElemNC(ik->_j_idx,i)];
  return _rkIKAllocCMat( ik );
}

static bool _rkIKJointReg(rkIK *ik, rkChain *chain, uint id, bool sw, double weight)
{
  if( id < 0 || id >= rkChainLinkNum(chain) ){
    ZRUNERROR( RK_ERR_LINK_INVID, id );
    return false;
  }
  ik->joint_sw[id] = sw;
  ik->joint_weight[id] = weight;
  return _rkIKAllocJointIndex( ik, chain );
}
bool rkChainRegIKJoint(rkChain *chain, uint id, double weight){
  return _rkIKJointReg( chain->_ik, chain, id, true, weight );
}
bool rkChainUnregIKJoint(rkChain *chain, uint id){
  return _rkIKJointReg( chain->_ik, chain, id, false, 0.0 );
}

/* register all joints to inverse kinematics solver. */
bool rkChainRegIKJointAll(rkChain *chain, double weight)
{
  uint i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointSize(chain,i) > 0 )
      if( !rkChainRegIKJoint( chain, i, weight ) ) return false;
  return true;
}

/* register constraint cell to inverse kinematics solver. */
static bool _rkIKAllocSRV(rkIK *ik)
{
  if( zListSize(&ik->clist) == 0 ) return true;
  zVecFree( ik->_c_srv );
  zVecFree( ik->_c_we );
  ik->_c_srv = zVecAlloc( zListSize(&ik->clist)*3 );
  ik->_c_we = zVecAlloc( zListSize(&ik->clist)*3 );
  ik->__c = zVecAlloc( zListSize(&ik->clist)*3 );
  if( !ik->_c_srv || !ik->_c_we || !ik->__c ){
    ZALLOCERROR();
    return false;
  }
  return _rkIKAllocCMat( ik );
}
static rkIKCell *_rkIKCellReg(rkIK *ik, rkIKCellAttr *attr, int mask, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKSRV_fp vf, rkIKBind_fp bf, rkIKAcm_fp af, void *util)
{
  rkIKCell *cell, *cp;

  if( !( cell = zAlloc( rkIKCell, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  rkIKCellInit( cell, attr, mask, rf, mf, vf, bf, af, util );
  if( zListIsEmpty( &ik->clist ) ||
      ( cp = zListTail(&ik->clist) )->data.id != 0 ){
    cp = zListRoot(&ik->clist);
    cell->data.id = 0;
  } else{
    zListForEach( &ik->clist, cp )
      if( zListCellNext(cp) == zListRoot(&ik->clist) ||
          zListCellNext(cp)->data.id != cp->data.id + 1 ) break;
    cell->data.id = cp->data.id + 1;
  }
  zListInsertNext( &ik->clist, cp, cell );
  /* return registered entry no., if it succeeds. */
  return _rkIKAllocSRV( ik ) ? cell : NULL;
}
static bool _rkIKCellUnreg(rkIK *ik, rkIKCell *cell)
{
  zListPurge( &ik->clist, cell );
  zFree( cell );
  return _rkIKAllocSRV( ik );
}

rkIKCell *rkChainRegIKCell(rkChain *chain, rkIKCellAttr *attr, int mask, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKSRV_fp vf, rkIKBind_fp bf, rkIKAcm_fp af, void *util)
{
  return _rkIKCellReg( chain->_ik, attr, mask, rf, mf, vf, bf, af, util );
}

bool rkChainUnregIKCell(rkChain *chain, rkIKCell *cell)
{
  return _rkIKCellUnreg( chain->_ik, cell );
}

/* find a constraint cell of inverse kinematics solver from identifier. */
rkIKCell *rkIKFindCell(rkIK *ik, int id)
{
  rkIKCell *cp;

  zListForEach( &ik->clist, cp )
    if( cp->data.id == id ) return cp;
  return NULL;
}
rkIKCell *rkChainFindIKCell(rkChain *chain, int id)
{
  return rkIKFindCell( chain->_ik, id );
}

/* deactivate all constraint cells of inverse kinematics solver. */
static void _rkIKDeactivate(rkIK *ik)
{
  rkIKCell *cp;

  zListForEach( &ik->clist, cp )
    rkIKCellDisable( cp );
}
void rkChainDeactivateIK(rkChain *chain)
{
  _rkIKDeactivate( chain->_ik );
}

/* bind current state of properties to be constrained in inverse kinematics to references. */
void rkChainBindIK(rkChain *chain)
{
  rkIKCell *cp;

  zListForEach( &chain->_ik->clist, cp )
    rkIKCellBind( cp, chain );
}

/* zero accumulator of each cell. */
static void _rkIKAcmZero(rkIK *ik)
{
  rkIKCell *cp;

  zListForEach( &ik->clist, cp )
    rkIKCellAcmZero( cp );
}
void rkChainZeroIKAcm(rkChain *chain)
{
  _rkIKAcmZero( chain->_ik );
}

/* form the motion rate contraint equation. */
static int _rkIKCellEq(rkIK *ik, rkChain *chain, rkIKCell *cell, int s, int row)
{
  uint i;
  int j;

  if( !( ( RK_IK_CELL_XON << s ) & cell->data.attr.mode ) ) return 0;
  zVecSetElemNC( ik->_c_srv, row, ik->_c_srv_cell.e[s] );
  zVecSetElemNC( ik->_c_we, row, cell->data.attr.w.e[s] );
  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( ik->joint_sw[i] ){
      for( j=0; j<rkChainLinkJointSize(chain,i); j++ )

        zMatSetElemNC( ik->_c_mat, row, zIndexElemNC(ik->_j_ofs,i)+j,
          zMatElemNC(ik->_c_mat_cell,s,rkChainLinkOffset(chain,i)+j) );
    } else{
      for( j=0; j<rkChainLinkJointSize(chain,i); j++ )
        zVecElemNC(ik->_c_srv,row) -=
          zMatElemNC(ik->_c_mat_cell,s,rkChainLinkOffset(chain,i)+j)
            * zVecElemNC(ik->joint_vel,rkChainLinkOffset(chain,i)+j);
    }
  return 1;
}

static void _rkIKEq(rkIK *ik, rkChain *chain)
{
  int i;
  rkIKCell *cell;
  int row = 0;

  ik->eval = 0;
  zListForEach( &ik->clist, cell ){
    if( rkIKCellIsDisabled( cell ) ) continue;
    rkIKCellCMat( cell, chain, ik->_c_mat_cell );
    rkIKCellSRV( cell, chain, &ik->_c_srv_cell );
    cell->data._eval = zVec3DWSqrNorm( &ik->_c_srv_cell, &cell->data.attr.w );
    ik->eval += cell->data._eval;
    cell->data._eval = sqrt( cell->data._eval );
    if( rkIKCellIsForced( cell ) )
      rkIKCellAcm( cell, chain, &ik->_c_srv_cell );
    for( i=0; i<3; i++ )
      row += _rkIKCellEq( ik, chain, cell, i, row );
  }
  ik->eval = sqrt( ik->eval );
  zMatSetRowSize( ik->_c_mat, row );
  zVecSetSize( ik->_c_srv, row );
  zVecSetSize( ik->__c, row );
  zVecSetSize( ik->_c_we, row );
}

void rkChainCreateIKEq(rkChain *chain)
{
  _rkIKEq( chain->_ik, chain );
}

/* resolve the motion rate with MP-inverse matrix. */
zVec rkIKJointVelMP(rkIK *ik)
{
  zLESolveMP( ik->_c_mat, ik->_c_srv, ik->_j_wn, ik->_c_we, ik->_j_vel );
  return ik->_j_vel;
}

/* resolve the motion rate with SR-inverse matrix. */
zVec rkIKJointVelSR(rkIK *ik)
{
  zVecCopy( ik->_c_srv, ik->__c );
  zLESolveSRDST( ik->_c_mat, ik->__c, ik->_j_wn, ik->_c_we, ik->_j_vel, &ik->__le );
  return ik->_j_vel;
}

/* resolve the motion rate with SR-inverse matrix and auto-damping. */
zVec rkIKJointVelAD(rkIK *ik)
{
  uint i;
  double e;

  zVecAmpNC( ik->_c_srv, ik->_c_we, ik->__c );
  zMulMatTVecNC( ik->_c_mat, ik->__c, ik->__le.v1 );
  zMatTQuadNC( ik->_c_mat, ik->_c_we, ik->__le.m );
  e = zVecInnerProd( ik->_c_srv, ik->__c );
  for( i=0; i<zMatRowSizeNC(ik->__le.m); i++ )
    zMatElemNC(ik->__le.m,i,i) += zVecElemNC(ik->_j_wn,i) + e;
  zLESolveGaussDST( ik->__le.m, ik->__le.v1, ik->_j_vel, ik->__le.idx1, ik->__le.s );
  return ik->_j_vel;
}

/* resolve the motion rate into joint angle rate. */
zVec rkChainIKRate(rkChain *chain)
{
  uint i;
  int j, k;
  double *vp;

  rkChainCreateIKEq( chain );
  chain->_ik->_jv( chain->_ik ); /* DIK solution */
  for( vp=zVecBuf(chain->_ik->_j_vel), i=0; i<zArraySize(chain->_ik->_j_idx); i++ ){
    k = zIndexElemNC( chain->_ik->_j_idx, i );
    for( j=0; j<rkChainLinkJointSize(chain,k); j++ )
      zVecSetElemNC( chain->_ik->joint_vel, rkChainLinkOffset(chain,k)+j, *vp++ );
  }
  return chain->_ik->joint_vel;
}

/* solve one-step inverse kinematics based on Newton=Raphson's method. */
zVec rkChainIKOne(rkChain *chain, zVec dis, double dt)
{
  rkChainIKRate( chain );
  rkChainCatJointDisAll( chain, dis, dt, chain->_ik->joint_vel );
  rkChainSetJointDisAll( chain, dis );
  rkChainGetJointDisAll( chain, dis );
  rkChainUpdateFK( chain );
  return dis;
}

/* solve inverse kinematics based on Newton=Raphson's method. */
int rkChainIK(rkChain *chain, zVec dis, double tol, int iter)
{
  int i;
  double rest = HUGE_VAL;

  rkChainGetJointDisAll( chain, dis );
  ZITERINIT( iter );
  rkChainZeroIKAcm( chain );
  for( i=0; i<iter; i++ ){
    rkChainIKOne( chain, dis, 1.0 );
    if( zIsTol( chain->_ik->eval - rest, tol ) )
      return i; /* probably no more decrease */
    rest = chain->_ik->eval;
  }
  return -1;
}

rkIKCell *rkChainRegIKCellWldPos(rkChain *chain, rkIKCellAttr *attr, int mask){
  return rkChainRegIKCell( chain, attr, mask, rkIKRefSetPos, rkIKJacobiLinkWldLin, rkIKLinkWldPosErr, rkIKBindLinkWldPos, rkIKAcmPos, NULL );
}

rkIKCell *rkChainRegIKCellWldAtt(rkChain *chain, rkIKCellAttr *attr, int mask){
  return rkChainRegIKCell( chain, attr, mask, rkIKRefSetZYX, rkIKJacobiLinkWldAng, rkIKLinkWldAttErr, rkIKBindLinkWldAtt, rkIKAcmAtt, NULL );
}

rkIKCell *rkChainRegIKCellL2LPos(rkChain *chain, rkIKCellAttr *attr, int mask){
  return rkChainRegIKCell( chain, attr, mask, rkIKRefSetPos, rkIKJacobiLinkL2LLin, rkIKLinkL2LPosErr, rkIKBindLinkL2LPos, rkIKAcmPos, NULL );
}

rkIKCell *rkChainRegIKCellL2LAtt(rkChain *chain, rkIKCellAttr *attr, int mask){
  return rkChainRegIKCell( chain, attr, mask, rkIKRefSetZYX, rkIKJacobiLinkL2LAng, rkIKLinkL2LAttErr, rkIKBindLinkL2LAtt, rkIKAcmAtt, NULL );
}

rkIKCell *rkChainRegIKCellCOM(rkChain *chain, rkIKCellAttr *attr, int mask){
  return rkChainRegIKCell( chain, attr, mask, rkIKRefSetPos, rkIKJacobiCOM, rkIKCOMErr, rkIKBindCOM, rkIKAcmPos, NULL );
}

rkIKCell *rkChainRegIKCellAM(rkChain *chain, rkIKCellAttr *attr, int mask){
  return rkChainRegIKCell( chain, attr, mask, rkIKRefSetPos, rkIKJacobiAM, rkIKAMErr, rkIKBindAM, rkIKAcmAtt, NULL );
}

rkIKCell *rkChainRegIKCellAMCOM(rkChain *chain, rkIKCellAttr *attr, int mask){
  return rkChainRegIKCell( chain, attr, mask, rkIKRefSetPos, rkIKJacobiAMCOM, rkIKAMCOMErr, rkIKBindAMCOM, rkIKAcmAtt, NULL );
}

/* ********************************************************** */
/* IK configuration file I/O
 * ********************************************************** */

/* IK item lookup table */
static struct _rkIKLookup{
  const char *str;
  rkIKCell *(*reg_ik_cell)(rkChain*,rkIKCellAttr*,int);
} __rk_ik_lookup[] = {
  { "world_pos", rkChainRegIKCellWldPos },
  { "world_att", rkChainRegIKCellWldAtt },
  { "l2l_pos",   rkChainRegIKCellL2LPos },
  { "l2l_att",   rkChainRegIKCellL2LAtt },
  { "com",       rkChainRegIKCellCOM    },
  { "am",        rkChainRegIKCellAM     },
  { "amcom",     rkChainRegIKCellAMCOM  },
  { NULL, NULL },
};

static struct _rkIKLookup *_rkIKLookupCell(char *str)
{
  struct _rkIKLookup *lookup;

  for( lookup=__rk_ik_lookup; lookup->str; lookup++ )
    if( strcmp( str, lookup->str ) == 0 ) return lookup;
  ZRUNERROR( RK_ERR_IK_UNKNOWN, str );
  return NULL;
}

/* ZTK */

static void *_rkIKJointFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  rkLink *link;
  double w = RK_IK_JOINT_WEIGHT_DEFAULT;
  char *linkname;

  linkname = ZTKVal(ztk);
  if( ZTKValNext(ztk) ) w = ZTKDouble(ztk);
  if( strcmp( linkname, "all" ) == 0 )
    return rkChainRegIKJointAll( (rkChain*)obj, w ) ? obj : NULL;
  zArrayFindName( &((rkChain*)obj)->link, linkname, link );
  if( !link ){
    ZRUNERROR( RK_ERR_LINK_UNKNOWN, linkname );
    return NULL;
  }
  if( w == 0 ){
    rkLinkJoint(link)->com->_dis_fromZTK( rkLinkJoint(link)->prp, 0, NULL, ztk );
    return obj;
  }
  if( rkLinkJointSize(link) == 0 ) return NULL;
  return rkChainRegIKJoint( (rkChain*)obj, link - rkChainRoot((rkChain*)obj), w ) ? obj : NULL;
}
static void *_rkIKConstraintFromZTK(void *obj, int i, void *arg, ZTK *ztk){
  struct _rkIKLookup *lookup;
  rkIKCellAttr attr;
  int mask = RK_IK_CELL_ATTR_NONE;
  rkLink *link;
  int linknum = 0;

  if( !( lookup = _rkIKLookupCell( ZTKVal(ztk) ) ) ) return NULL;
  ZTKValNext( ztk );
  while( ztk->val_cp ){
    if( ZTKValCmp( ztk, "at" ) ){
      ZTKValNext( ztk );
      zVec3DFromZTK( &attr.ap, ztk );
      mask |= RK_IK_CELL_ATTR_AP;
    } else
    if( ZTKValCmp( ztk, "w" ) ){
      ZTKValNext( ztk );
      zVec3DFromZTK( &attr.w, ztk );
      mask |= RK_IK_CELL_ATTR_WEIGHT;
    } else
    if( ZTKValCmp( ztk, "f" ) ){
      ZTKValNext( ztk );
      mask |= RK_IK_CELL_ATTR_FORCE;
    } else{
      zArrayFindName( &((rkChain*)obj)->link, ZTKVal(ztk), link );
      if( !link ){
        ZRUNERROR( RK_ERR_LINK_UNKNOWN, ZTKVal(ztk) );
        return NULL;
      }
      if( linknum++ == 0 ){
        attr.id = link - rkChainRoot((rkChain*)obj);
        mask |= RK_IK_CELL_ATTR_ID;
      } else{
        attr.id_sub = link - rkChainRoot((rkChain*)obj);
        mask |= RK_IK_CELL_ATTR_ID_SUB;
      }
      ZTKValNext( ztk );
    }
  }
  return lookup->reg_ik_cell( (rkChain *)obj, &attr, mask ) ? obj : NULL;
}

static ZTKPrp __ztk_prp_rkik[] = {
  { "joint", -1, _rkIKJointFromZTK, NULL },
  { "constraint", -1, _rkIKConstraintFromZTK, NULL },
};

static void *_rkIKFromZTK(void *obj, int i, void *arg, ZTK *ztk)
{
  if( !ZTKEvalKey( obj, NULL, ztk, __ztk_prp_rkik ) ) return NULL;
  return obj;
}

static ZTKPrp __ztk_prp_tag_rkik[] = {
  { ZTK_TAG_RKIK, 1, _rkIKFromZTK, NULL },
};

rkChain *rkChainIKConfFromZTK(rkChain *chain, ZTK *ztk)
{
  ZTKEvalTag( chain, NULL, ztk, __ztk_prp_tag_rkik );
  return chain;
}

rkChain *rkChainIKConfReadZTK(rkChain *chain, char filename[])
{
  ZTK ztk;

  if( !rkChainCreateIK( chain ) ) return NULL;
  ZTKInit( &ztk );
  if( ZTKParse( &ztk, filename ) )
    chain = rkChainIKConfFromZTK( chain, &ztk );
  ZTKDestroy( &ztk );
  return chain;
}
