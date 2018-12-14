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

static void _rkIKInit(rkIK *ik);
static bool _rkIKAllocCMat(rkIK *ik);
static bool _rkIKAllocJointIndex(rkIK *ik);
static bool _rkIKJointReg(rkIK *ik, int id, bool sw, double weight);
static bool _rkIKAllocSRV(rkIK *ik);

static int _rkIKCellEq(rkIK *ik, rkIKCell *cell, int s, int row);

/* (static)
 * _rkIKInit
 * - initialize inverse kinematics solver.
 */
void _rkIKInit(rkIK *ik)
{
  ik->chain = NULL;
  ik->joint_sw = NULL;
  ik->joint_weight = NULL;
  ik->joint_vel = NULL;
  ik->eval = 0;

  zListInit( &ik->clist );
  ik->_c_mat_cell = NULL;
  zVec3DClear( &ik->_c_srv_cell );

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

/* rkIKCreate
 * create inverse kinematics solver.
 */
rkIK *rkIKCreate(rkIK *ik, rkChain *chain)
{
  _rkIKInit( ik );
  ik->chain = chain;
  ik->joint_sw = zAlloc( bool, rkChainNum(chain) );
  ik->joint_weight = zAlloc( double, rkChainNum(chain) );
  ik->joint_vel = zVecAlloc( rkChainJointSize(chain) );
  ik->_j_ofs = zIndexCreate( rkChainNum(chain) );
  ik->_c_mat_cell = zMatAlloc( 3, rkChainJointSize(chain) );
  if( !ik->joint_sw || !ik->joint_weight ||
      !ik->joint_vel || !ik->_c_mat_cell ){
    ZALLOCERROR();
    return NULL;
  }
  return ik;
}

/* rkIKDestroy
 * destroy inverse kinematics solver.
 */
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

/* (static)
 * _rkIKAllocCMat
 * - allocate working memory for constraint coefficient matrix.
 */
bool _rkIKAllocCMat(rkIK *ik)
{
  if( zListNum(&ik->clist) == 0 || zArrayNum(ik->_j_idx) == 0 )
    return true;
  zMatFree( ik->_c_mat );
  if( !( ik->_c_mat = zMatAlloc( zListNum(&ik->clist)*3, zVecSizeNC(ik->_j_vel) ) ) ){
    ZALLOCERROR();
    return false;
  }
  return true;
}

/* rkIKJointReg, rkIKJointUnreg
 * - register/unregister cooperating joint.
 */
bool _rkIKAllocJointIndex(rkIK *ik)
{
  register int i, j;
  int count, ofs;
  double *wp;

  for( count=0, i=0; i<rkChainNum(ik->chain); i++ ){
    if( rkChainLinkJointType(ik->chain,i) == RK_JOINT_FIXED )
      ik->joint_sw[i] = false;
    if( ik->joint_sw[i] ) count++;
  }
  if( count == 0 ) return true;
  zIndexFree( ik->_j_idx );
  if( !( ik->_j_idx = zIndexCreate(count) ) ){
    ZALLOCERROR();
    return false;
  }
  for( count=0, ofs=0, i=0; i<rkChainNum(ik->chain); i++ )
    if( ik->joint_sw[i] ){
      zIndexSetElem( ik->_j_idx, count++, i );
      zIndexSetElem( ik->_j_ofs, i, ofs );
      ofs += rkChainLinkJointSize(ik->chain,i);
    } else
      zIndexSetElem( ik->_j_ofs, i, -1 );
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
  for( wp=zVecBuf(ik->_j_wn), i=0; i<zArrayNum(ik->_j_idx); i++ )
    for( j=0; j<rkChainLinkJointSize(ik->chain,zIndexElem(ik->_j_idx,i)); j++ )
      *wp++ = ik->joint_weight[zIndexElem(ik->_j_idx,i)];
  return _rkIKAllocCMat( ik );
}
bool _rkIKJointReg(rkIK *ik, int id, bool sw, double weight)
{
  if( id < 0 || id >= rkChainNum(ik->chain) ){
    ZRUNERROR( "invalid link #%d specified", id );
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

/* rkIKJointRegAll
 * - register all joints.
 */
bool rkIKJointRegAll(rkIK *ik, double weight)
{
  register int i;

  for( i=0; i<rkChainNum(ik->chain); i++ )
    if( rkChainLinkJointType(ik->chain,i) != RK_JOINT_FIXED )
      if( !rkIKJointReg( ik, i, weight ) ) return false;
  return true;
}

/* rkIKCellReg
 * - register constraint cell.
 */
bool _rkIKAllocSRV(rkIK *ik)
{
  if( zListNum(&ik->clist) == 0 ) return true;
  zVecFree( ik->_c_srv );
  zVecFree( ik->_c_we );
  ik->_c_srv = zVecAlloc( zListNum(&ik->clist)*3 );
  ik->_c_we = zVecAlloc( zListNum(&ik->clist)*3 );
  ik->__c = zVecAlloc( zListNum(&ik->clist)*3 );
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

/* rkIKFindCell
 * - find cell from identifier.
 */
rkIKCell *rkIKFindCell(rkIK *ik, int id)
{
  rkIKCell *cp;

  zListForEach( &ik->clist, cp )
    if( cp->data.id == id ) return cp;
  return NULL;
}

/* rkIKDeactivate
 * - deactivate all the constraint cells.
 */
void rkIKDeactivate(rkIK *ik)
{
  rkIKCell *cp;

  zListForEach( &ik->clist, cp )
    rkIKCellDisable( cp );
}

/* rkIKBind
 * - bind current status of constrained properties
 *   to the references.
 */
void rkIKBind(rkIK *ik)
{
  rkIKCell *cp;

  zListForEach( &ik->clist, cp )
    rkIKCellBind( cp, ik->chain );
}

/* rkIKAcmClear
 * - clear accumulator of each cell.
 */
void rkIKAcmClear(rkIK *ik)
{
  rkIKCell *cp;

  zListForEach( &ik->clist, cp )
    rkIKCellAcmClear( cp );
}

/* rkIKEq
 * - form motion rate contraint equation.
 */
int _rkIKCellEq(rkIK *ik, rkIKCell *cell, int s, int row)
{
  register int i, j;

  if( !( ( RK_IK_CELL_XON << s ) & cell->data.attr.mode ) ) return 0;
  zVecSetElem( ik->_c_srv, row, ik->_c_srv_cell.e[s] );
  zVecSetElem( ik->_c_we, row, cell->data.attr.w.e[s] );
  for( i=0; i<rkChainNum(ik->chain); i++ )
    if( ik->joint_sw[i] ){
      for( j=0; j<rkChainLinkJointSize(ik->chain,i); j++ )
        zMatSetElem( ik->_c_mat, row, zIndexElem(ik->_j_ofs,i)+j,
          zMatElem(ik->_c_mat_cell,s,rkChainLinkOffset(ik->chain,i)+j) );
    } else{
      for( j=0; j<rkChainLinkJointSize(ik->chain,i); j++ )
        zVecElem( ik->_c_srv, row ) -=
          zMatElem(ik->_c_mat_cell,s,rkChainLinkOffset(ik->chain,i)+j)
            * zVecElem(ik->joint_vel,rkChainLinkOffset(ik->chain,i)+j);
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

/* rkIKJointVelMP
 * - resolve the motion rate with MP-inverse matrix.
 */
zVec rkIKJointVelMP(rkIK *ik)
{
  zLESolveMP( ik->_c_mat, ik->_c_srv, ik->_j_wn, ik->_c_we, ik->_j_vel );
  return ik->_j_vel;
}

/* rkIKJointVelSR
 * - resolve the motion rate with SR-inverse matrix.
 */
zVec rkIKJointVelSR(rkIK *ik)
{
  zVecCopy( ik->_c_srv, ik->__c );
  zLESolveSRDST( ik->_c_mat, ik->__c, ik->_j_wn, ik->_c_we, ik->_j_vel, ik->__m, ik->__v, ik->__idx, ik->__s );
  return ik->_j_vel;
}

/* rkIKJointVelAD
 * - resolve the motion rate with SR-inverse matrix and auto-damping.
 */
zVec rkIKJointVelAD(rkIK *ik)
{
  register int i;
  double e;

  zVecAmpNC( ik->_c_srv, ik->_c_we, ik->__c );
  zMulMatTVecNC( ik->_c_mat, ik->__c, ik->__v );
  zMatTQuadNC( ik->_c_mat, ik->_c_we, ik->__m );
  e = zVecInnerProd( ik->_c_srv, ik->__c );
  for( i=0; i<zMatRowSizeNC(ik->__m); i++ )
    zMatElem(ik->__m,i,i) += zVecElem(ik->_j_wn,i) + e;
  zLESolveGaussDST( ik->__m, ik->__v, ik->_j_vel, ik->__idx, ik->__s );
  return ik->_j_vel;
}

/* rkIKSolveRate
 * - resolve the motion rate into joint angle rate.
 */
zVec rkIKSolveRate(rkIK *ik)
{
  register int i, j, k;
  double *vp;

  rkIKEq( ik );
  ik->_jv( ik );
  for( vp=zVecBuf(ik->_j_vel), i=0; i<zArrayNum(ik->_j_idx); i++ ){
    k = zIndexElem( ik->_j_idx, i );
    for( j=0; j<rkChainLinkJointSize(ik->chain,k); j++ )
      zVecSetElem( ik->joint_vel, rkChainLinkOffset(ik->chain,k)+j, *vp++ );
  }
  return ik->joint_vel;
}

/* rkIKSolveOne
 * - solve one-step inverse kinematics based on Newton=Raphson's method.
 */
zVec rkIKSolveOne(rkIK *ik, zVec dis, double dt)
{
  rkIKSolveRate( ik );
  rkChainCatJointDisAll( ik->chain, dis, dt, ik->joint_vel );
  rkChainSetJointDisAll( ik->chain, dis );
  rkChainGetJointDisAll( ik->chain, dis );
  rkChainUpdateFK( ik->chain );
  return dis;
}

/* rkIKSolve
 * - solve inverse kinematics based on Newton=Raphson's method.
 */
int rkIKSolve(rkIK *ik, zVec dis, double tol, int iter)
{
  register int i;
  double rest = HUGE_VAL;

  rkChainGetJointDisAll( ik->chain, dis );
  ZITERINIT( iter );
  rkIKAcmClear( ik );
  for( i=0; i<iter; i++ ){
    rkIKSolveOne( ik, dis, 1.0 );
/*
printf( "%.12g %.12g %.12g %12g\n", ik->eval, zListHead(&ik->clist)->data._eval, zListTail(&ik->clist)->data._eval, zVec3DNorm(&zListTail(&ik->clist)->data.acm.pos) );
*/
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

static struct _rkIKLookup *_rkIKLookupCell(char *str);
static bool _rkIKConfFieldIsTerminated(FILE *fp, char *buf);
static bool _rkIKConfParseJoint(FILE *fp, char *buf, rkIK *ik);
static bool _rkIKConfParseConstraint(FILE *fp, char *buf, rkChain *chain, rkIKCellAttr *attr, int *mask);
static bool __rkIKConfFRead(FILE *fp, void *instance, char *buf, bool *success);
static bool _rkIKConfFRead(FILE *fp, void *instance, char *buf, bool *success);

struct _rkIKLookup *_rkIKLookupCell(char *str)
{
  struct _rkIKLookup *lookup;

  for( lookup=__rk_ik_lookup; lookup->str; lookup++ )
    if( strcmp( str, lookup->str ) == 0 ) return lookup;
  ZRUNERROR( "unknown constraint type %s", str );
  return NULL;
}

bool _rkIKConfFieldIsTerminated(FILE *fp, char *buf)
{
  long cur;
  bool ret = false;

  if( !zFSkipDefaultComment( fp ) ) return true;
  cur = ftell( fp );
  if( !zFToken( fp, buf, BUFSIZ ) ) return true;
  if( zTokenIsTag( buf ) ||
      strcmp( buf, "joint" ) == 0 ||
      strcmp( buf, "constraint" ) == 0 ){
    ret = true;
  }
  fseek( fp, cur, SEEK_SET );
  return ret;
}

bool _rkIKConfParseJoint(FILE *fp, char *buf, rkIK *ik)
{
  rkLink *l;
  double w = RK_IK_JOINT_WEIGHT_DEFAULT;

  zFToken( fp, buf, BUFSIZ );
  if( strcmp( buf, "all" ) == 0 ){
    if( !_rkIKConfFieldIsTerminated( fp, buf ) )
      w = zFDouble( fp );
    return rkIKJointRegAll( ik, w );
  }
  zNameFind( rkChainRoot(ik->chain), rkChainNum(ik->chain), buf, l );
  if( !l ){
    ZRUNERROR( "unknown link %s", buf );
    return false;
  }
  if( !_rkIKConfFieldIsTerminated( fp, buf ) )
    w = zFDouble( fp );
  if( w == 0 ){
    rkJointQueryFRead( fp, "dis", rkLinkJoint(l), NULL, 0 );
    return true;
  } else
    return rkLinkJointType(l) != RK_JOINT_FIXED ?
      rkIKJointReg( ik, l - rkChainRoot(ik->chain), w ) : false;
}

bool _rkIKConfParseConstraint(FILE *fp, char *buf, rkChain *chain, rkIKCellAttr *attr, int *mask)
{
  rkLink *l;
  int linknum = 0;

  *mask = RK_IK_CELL_ATTR_NONE;
  while( !_rkIKConfFieldIsTerminated( fp, buf ) ){
    zFToken( fp, buf, BUFSIZ );
    if( strcmp( buf, "at" ) == 0 ){
      zVec3DFRead( fp, &attr->ap );
      *mask |= RK_IK_CELL_ATTR_AP;
    } else
    if( strcmp( buf, "w" ) == 0 ){
      zVec3DFRead( fp, &attr->w );
      *mask |= RK_IK_CELL_ATTR_WEIGHT;
    } else
    if( strcmp( buf, "f" ) == 0 ){
      *mask |= RK_IK_CELL_ATTR_FORCE;
    } else{
      zNameFind( rkChainRoot(chain), rkChainNum(chain), buf, l );
      if( !l ){
        ZRUNERROR( "unknown link name %s", buf );
        return false;
      }
      if( linknum++ == 0 ){
        attr->id = l - rkChainRoot(chain);
        *mask |= RK_IK_CELL_ATTR_ID;
      } else{
        attr->id_sub = l - rkChainRoot(chain);
        *mask |= RK_IK_CELL_ATTR_ID_SUB;
      }
    }
  }
  return true;
}

bool __rkIKConfFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  rkIKCellAttr attr;
  int mask;
  struct _rkIKLookup *lookup;

  if( strcmp( buf, "joint" ) == 0 )
    return _rkIKConfParseJoint( fp, buf, instance );
  if( strcmp( buf, "constraint" ) == 0 ){
    zFToken( fp, buf, BUFSIZ );
    if( !( lookup = _rkIKLookupCell( buf ) ) ) return false;
    _rkIKConfParseConstraint( fp, buf, ((rkIK *)instance)->chain, &attr, &mask );
    return lookup->ik_cell_reg( instance, &attr, mask ) ? true : false;
  }
  return false;
}

bool _rkIKConfFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  if( strcmp( buf, "ik" ) == 0 )
    return zFieldFRead( fp, __rkIKConfFRead, instance );
  return true;
}

/* rkIKConfFRead
 * - read the IK configuration file.
 */
bool rkIKConfFRead(FILE *fp, rkIK *ik, rkChain *chain)
{
  if( !rkIKCreate( ik, chain ) ) return false;
  rewind( fp );
  return zTagFRead( fp, _rkIKConfFRead, ik );
}

/* rkIKConfReadFile
 * - read the IK configuration file.
 */
bool rkIKConfReadFile(rkIK *ik, rkChain *chain, char *filename)
{
  FILE *fp;
  bool result;

  if( !( fp = fopen( filename, "r" ) ) ){
    ZOPENERROR( filename );
    return false;
  }
  result = rkIKConfFRead( fp, ik, chain );
  fclose( fp );
  return result;
}
