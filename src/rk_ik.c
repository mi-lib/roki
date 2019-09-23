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

/* initialize inverse kinematics solver. */
static void _rkIKInit(rkIK *ik)
{
  ik->chain = NULL;
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
  ik->__m = NULL;
  ik->__v = ik->__s = ik->__c = NULL;
  ik->__idx = NULL;
}

/* create inverse kinematics solver. */
rkIK *rkIKCreate(rkIK *ik, rkChain *chain)
{
  _rkIKInit( ik );
  ik->chain = chain;
  ik->joint_sw = zAlloc( bool, rkChainLinkNum(chain) );
  ik->joint_weight = zAlloc( double, rkChainLinkNum(chain) );
  ik->joint_vel = zVecAlloc( rkChainJointSize(chain) );
  ik->_j_ofs = zIndexCreate( rkChainLinkNum(chain) );
  ik->_c_mat_cell = zMatAlloc( 3, rkChainJointSize(chain) );
  if( !ik->joint_sw || !ik->joint_weight ||
      !ik->joint_vel || !ik->_c_mat_cell ){
    ZALLOCERROR();
    return NULL;
  }
  return ik;
}

/* destroy inverse kinematics solver. */
void rkIKDestroy(rkIK *ik)
{
  ik->chain = NULL;
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
  zLEFreeWork( ik->__m, ik->__v, ik->__s, ik->__idx );
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
static bool _rkIKAllocJointIndex(rkIK *ik)
{
  register int i, j;
  int count, ofs;
  double *wp;

  for( count=0, i=0; i<rkChainLinkNum(ik->chain); i++ ){
    if( rkChainLinkJointSize(ik->chain,i) == 0 )
      ik->joint_sw[i] = false;
    if( ik->joint_sw[i] ) count++;
  }
  if( count == 0 ) return true;
  zIndexFree( ik->_j_idx );
  if( !( ik->_j_idx = zIndexCreate(count) ) ){
    ZALLOCERROR();
    return false;
  }
  for( count=0, ofs=0, i=0; i<rkChainLinkNum(ik->chain); i++ )
    if( ik->joint_sw[i] ){
      zIndexSetElemNC( ik->_j_idx, count++, i );
      zIndexSetElemNC( ik->_j_ofs, i, ofs );
      ofs += rkChainLinkJointSize(ik->chain,i);
    } else
      zIndexSetElemNC( ik->_j_ofs, i, -1 );
  /* allocate joint vector */
  zVecFree( ik->_j_vel );
  zVecFree( ik->_j_wn );
  zLEFreeWork( ik->__m, ik->__v, ik->__s, ik->__idx );
  count = rkChainJointIndexSize( ik->chain, ik->_j_idx );
  if( !( ik->_j_vel = zVecAlloc(count) ) ||
      !( ik->_j_wn = zVecAlloc(count) ) ||
      !zLEAllocWork( &ik->__m, &ik->__v, &ik->__s, &ik->__idx, count ) ){
    ZALLOCERROR();
    return false;
  }
  for( wp=zVecBuf(ik->_j_wn), i=0; i<zArraySize(ik->_j_idx); i++ )
    for( j=0; j<rkChainLinkJointSize(ik->chain,zIndexElemNC(ik->_j_idx,i)); j++ )
      *wp++ = ik->joint_weight[zIndexElemNC(ik->_j_idx,i)];
  return _rkIKAllocCMat( ik );
}

static bool _rkIKJointReg(rkIK *ik, int id, bool sw, double weight)
{
  if( id < 0 || id >= rkChainLinkNum(ik->chain) ){
    ZRUNERROR( RK_ERR_LINK_INVID, id );
    return false;
  }
  ik->joint_sw[id] = sw;
  ik->joint_weight[id] = weight;
  return _rkIKAllocJointIndex( ik );
}
bool rkIKJointReg(rkIK *ik, int id, double weight){
  return _rkIKJointReg( ik, id, true, weight );
}
bool rkIKJointUnreg(rkIK *ik, int id){
  return _rkIKJointReg( ik, id, false, 0.0 );
}

/* register all joints to inverse kinematics solver. */
bool rkIKJointRegAll(rkIK *ik, double weight)
{
  register int i;

  for( i=0; i<rkChainLinkNum(ik->chain); i++ )
    if( rkChainLinkJointSize(ik->chain,i) > 0 )
      if( !rkIKJointReg( ik, i, weight ) ) return false;
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
rkIKCell *rkIKCellReg(rkIK *ik, rkIKCellAttr *attr, int mask, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKSRV_fp vf, rkIKBind_fp bf, rkIKAcm_fp af, void *util)
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
bool rkIKCellUnreg(rkIK *ik, rkIKCell *cell)
{
  zListPurge( &ik->clist, cell );
  zFree( cell );
  return _rkIKAllocSRV( ik );
}

/* find a constraint cell of inverse kinematics solver from identifier. */
rkIKCell *rkIKFindCell(rkIK *ik, int id)
{
  rkIKCell *cp;

  zListForEach( &ik->clist, cp )
    if( cp->data.id == id ) return cp;
  return NULL;
}

/* deactivate all constraint cells of inverse kinematics solver. */
void rkIKDeactivate(rkIK *ik)
{
  rkIKCell *cp;

  zListForEach( &ik->clist, cp )
    rkIKCellDisable( cp );
}

/* bind current state of properties to be constrained in inverse kinematics to references. */
void rkIKBind(rkIK *ik)
{
  rkIKCell *cp;

  zListForEach( &ik->clist, cp )
    rkIKCellBind( cp, ik->chain );
}

/* zero accumulator of each cell. */
void rkIKAcmZero(rkIK *ik)
{
  rkIKCell *cp;

  zListForEach( &ik->clist, cp )
    rkIKCellAcmZero( cp );
}

/* form the motion rate contraint equation. */
static int _rkIKCellEq(rkIK *ik, rkIKCell *cell, int s, int row)
{
  register int i, j;

  if( !( ( RK_IK_CELL_XON << s ) & cell->data.attr.mode ) ) return 0;
  zVecSetElemNC( ik->_c_srv, row, ik->_c_srv_cell.e[s] );
  zVecSetElemNC( ik->_c_we, row, cell->data.attr.w.e[s] );
  for( i=0; i<rkChainLinkNum(ik->chain); i++ )
    if( ik->joint_sw[i] ){
      for( j=0; j<rkChainLinkJointSize(ik->chain,i); j++ )
        zMatSetElemNC( ik->_c_mat, row, zIndexElemNC(ik->_j_ofs,i)+j,
          zMatElemNC(ik->_c_mat_cell,s,rkChainLinkOffset(ik->chain,i)+j) );
    } else{
      for( j=0; j<rkChainLinkJointSize(ik->chain,i); j++ )
        zVecElemNC(ik->_c_srv,row) -=
          zMatElemNC(ik->_c_mat_cell,s,rkChainLinkOffset(ik->chain,i)+j)
            * zVecElemNC(ik->joint_vel,rkChainLinkOffset(ik->chain,i)+j);
    }
  return 1;
}
void rkIKEq(rkIK *ik)
{
  register int i;
  rkIKCell *cell;
  int row = 0;

  ik->eval = 0;
  zListForEach( &ik->clist, cell ){
    if( rkIKCellIsDisabled( cell ) ) continue;
    rkIKCellCMat( cell, ik->chain, ik->_c_mat_cell );
    rkIKCellSRV( cell, ik->chain, &ik->_c_srv_cell );
    cell->data._eval = zVec3DWSqrNorm( &ik->_c_srv_cell, &cell->data.attr.w );
    ik->eval += cell->data._eval;
    cell->data._eval = sqrt( cell->data._eval );
    if( rkIKCellIsForced( cell ) )
      rkIKCellAcm( cell, ik->chain, &ik->_c_srv_cell );
    for( i=0; i<3; i++ )
      row += _rkIKCellEq( ik, cell, i, row );
  }
  ik->eval = sqrt( ik->eval );
  zMatSetRowSize( ik->_c_mat, row );
  zVecSetSize( ik->_c_srv, row );
  zVecSetSize( ik->__c, row );
  zVecSetSize( ik->_c_we, row );
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
  zLESolveSRDST( ik->_c_mat, ik->__c, ik->_j_wn, ik->_c_we, ik->_j_vel, ik->__m, ik->__v, ik->__idx, ik->__s );
  return ik->_j_vel;
}

/* resolve the motion rate with SR-inverse matrix and auto-damping. */
zVec rkIKJointVelAD(rkIK *ik)
{
  register int i;
  double e;

  zVecAmpNC( ik->_c_srv, ik->_c_we, ik->__c );
  zMulMatTVecNC( ik->_c_mat, ik->__c, ik->__v );
  zMatTQuadNC( ik->_c_mat, ik->_c_we, ik->__m );
  e = zVecInnerProd( ik->_c_srv, ik->__c );
  for( i=0; i<zMatRowSizeNC(ik->__m); i++ )
    zMatElemNC(ik->__m,i,i) += zVecElemNC(ik->_j_wn,i) + e;
  zLESolveGaussDST( ik->__m, ik->__v, ik->_j_vel, ik->__idx, ik->__s );
  return ik->_j_vel;
}

/* resolve the motion rate into joint angle rate. */
zVec rkIKSolveRate(rkIK *ik)
{
  register int i, j, k;
  double *vp;

  rkIKEq( ik );
  ik->_jv( ik );
  for( vp=zVecBuf(ik->_j_vel), i=0; i<zArraySize(ik->_j_idx); i++ ){
    k = zIndexElemNC( ik->_j_idx, i );
    for( j=0; j<rkChainLinkJointSize(ik->chain,k); j++ )
      zVecSetElemNC( ik->joint_vel, rkChainLinkOffset(ik->chain,k)+j, *vp++ );
  }
  return ik->joint_vel;
}

/* solve one-step inverse kinematics based on Newton=Raphson's method. */
zVec rkIKSolveOne(rkIK *ik, zVec dis, double dt)
{
  rkIKSolveRate( ik );
  rkChainCatJointDisAll( ik->chain, dis, dt, ik->joint_vel );
  rkChainSetJointDisAll( ik->chain, dis );
  rkChainGetJointDisAll( ik->chain, dis );
  rkChainUpdateFK( ik->chain );
  return dis;
}

/* solve inverse kinematics based on Newton=Raphson's method. */
int rkIKSolve(rkIK *ik, zVec dis, double tol, int iter)
{
  register int i;
  double rest = HUGE_VAL;

  rkChainGetJointDisAll( ik->chain, dis );
  ZITERINIT( iter );
  rkIKAcmZero( ik );
  for( i=0; i<iter; i++ ){
    rkIKSolveOne( ik, dis, 1.0 );
    if( zIsTol( ik->eval - rest, tol ) )
      return i; /* probably no more decrease */
    rest = ik->eval;
  }
  return -1;
}

/* ********************************************************** */
/* IK configuration file I/O
 * ********************************************************** */

rkIKCell *rkIKCellRegWldPos(rkIK *ik, rkIKCellAttr *attr, int mask)
{
  return rkIKCellReg( ik, attr, mask, rkIKRefSetPos, rkIKJacobiLinkWldLin, rkIKLinkWldPosErr, rkIKBindLinkWldPos, rkIKAcmPos, NULL );
}

rkIKCell *rkIKCellRegWldAtt(rkIK *ik, rkIKCellAttr *attr, int mask)
{
  return rkIKCellReg( ik, attr, mask, rkIKRefSetZYX, rkIKJacobiLinkWldAng, rkIKLinkWldAttErr, rkIKBindLinkWldAtt, rkIKAcmAtt, NULL );
}

rkIKCell *rkIKCellRegL2LPos(rkIK *ik, rkIKCellAttr *attr, int mask)
{
  return rkIKCellReg( ik, attr, mask, rkIKRefSetPos, rkIKJacobiLinkL2LLin, rkIKLinkL2LPosErr, rkIKBindLinkL2LPos, rkIKAcmPos, NULL );
}

rkIKCell *rkIKCellRegL2LAtt(rkIK *ik, rkIKCellAttr *attr, int mask)
{
  return rkIKCellReg( ik, attr, mask, rkIKRefSetZYX, rkIKJacobiLinkL2LAng, rkIKLinkL2LAttErr, rkIKBindLinkL2LAtt, rkIKAcmAtt, NULL );
}

rkIKCell *rkIKCellRegCOM(rkIK *ik, rkIKCellAttr *attr, int mask)
{
  return rkIKCellReg( ik, attr, mask, rkIKRefSetPos, rkIKJacobiCOM, rkIKCOMErr, rkIKBindCOM, rkIKAcmPos, NULL );
}

rkIKCell *rkIKCellRegAM(rkIK *ik, rkIKCellAttr *attr, int mask)
{
  return rkIKCellReg( ik, attr, mask, rkIKRefSetPos, rkIKJacobiAM, rkIKAMErr, rkIKBindAM, rkIKAcmAtt, NULL );
}

rkIKCell *rkIKCellRegAMCOM(rkIK *ik, rkIKCellAttr *attr, int mask)
{
  return rkIKCellReg( ik, attr, mask, rkIKRefSetPos, rkIKJacobiAMCOM, rkIKAMCOMErr, rkIKBindAMCOM, rkIKAcmAtt, NULL );
}

/* IK item lookup table */
static struct _rkIKLookup{
  char *str;
  rkIKCell *(*ik_cell_reg)(rkIK*,rkIKCellAttr*,int);
} __rk_ik_lookup[] = {
  { "world_pos", rkIKCellRegWldPos },
  { "world_att", rkIKCellRegWldAtt },
  { "l2l_pos",   rkIKCellRegL2LPos },
  { "l2l_att",   rkIKCellRegL2LAtt },
  { "com",       rkIKCellRegCOM    },
  { "am",        rkIKCellRegAM     },
  { "amcom",     rkIKCellRegAMCOM  },
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
    return rkIKJointRegAll( (rkIK*)obj, w ) ? obj : NULL;
  zArrayFindName( &((rkIK*)obj)->chain->link, linkname, link );
  if( !link ){
    ZRUNERROR( RK_ERR_LINK_UNKNOWN, linkname );
    return NULL;
  }
  if( w == 0 ){
    rkLinkJoint(link)->com->_dis_fromZTK( rkLinkJoint(link)->prp, 0, NULL, ztk );
    return obj;
  }
  if( rkLinkJointSize(link) == 0 ) return NULL;
  return rkIKJointReg( (rkIK*)obj, link - rkChainRoot(((rkIK*)obj)->chain), w ) ? obj : NULL;
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
      zArrayFindName( &((rkIK*)obj)->chain->link, ZTKVal(ztk), link );
      if( !link ){
        ZRUNERROR( RK_ERR_LINK_UNKNOWN, ZTKVal(ztk) );
        return NULL;
      }
      if( linknum++ == 0 ){
        attr.id = link - rkChainRoot(((rkIK*)obj)->chain);
        mask |= RK_IK_CELL_ATTR_ID;
      } else{
        attr.id_sub = link - rkChainRoot(((rkIK*)obj)->chain);
        mask |= RK_IK_CELL_ATTR_ID_SUB;
      }
      ZTKValNext( ztk );
    }
  }
  return lookup->ik_cell_reg( obj, &attr, mask ) ? obj : NULL;
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

rkIK *rkIKConfFromZTK(rkIK *ik, ZTK *ztk)
{
  ZTKEvalTag( ik, NULL, ztk, __ztk_prp_tag_rkik );
  return ik;
}

rkIK *rkIKConfReadZTK(rkIK *ik, rkChain *chain, char filename[])
{
  ZTK ztk;

  if( !rkIKCreate( ik, chain ) ) return NULL;
  ZTKInit( &ztk );
  if( !ZTKDefRegPrp( &ztk, ZTK_TAG_RKIK, __ztk_prp_rkik ) ) return NULL;
  if( ZTKParse( &ztk, filename ) )
    ik = rkIKConfFromZTK( ik, &ztk );
  ZTKDestroy( &ztk );
  return ik;
}
