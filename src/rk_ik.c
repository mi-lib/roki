/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_ik - inverse kinematics
 */

#include <roki/rk_chain.h>

/* ********************************************************** */
/* CLASS: rkIK
 * inverse kinematics class
 * ********************************************************** */

/* initialize an inverse kinematics solver. */
static void _rkIKInit(rkIK *ik)
{
  ik->joint_is_enabled = NULL;
  ik->joint_weight = NULL;
  ik->joint_vec = NULL;
  ik->eval = 0;

  zListInit( &ik->_c_list );
  ik->_c_mat_cell = NULL;
  zVec3DZero( &ik->_c_vec_cell );

  ik->_j_idx = NULL;
  ik->_j_ofs = NULL;
  ik->_j_vec = NULL;
  ik->_j_wn = NULL;
  ik->_c_mat = NULL;
  ik->_c_vec = NULL;
  ik->_c_we = NULL;
  ik->_solve_eq = rkIKSolveEqED; /* default motion constraint solver */
  ik->__c = NULL;
  zLEWorkspaceInit( &ik->__le );
}

/* create an inverse kinematics solver for a kinematic chain. */
rkChain *rkChainCreateIK(rkChain *chain)
{
  int joint_size;

  if( ( joint_size = rkChainJointSize(chain) ) == 0 )
    return chain; /* no need to create an inverse kinematics solver */
  if( !( chain->_ik = zAlloc( rkIK, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  _rkIKInit( chain->_ik );
  chain->_ik->joint_is_enabled = zAlloc( bool, rkChainLinkNum(chain) );
  chain->_ik->joint_weight = zAlloc( double, rkChainLinkNum(chain) );
  chain->_ik->joint_vec = zVecAlloc( rkChainJointSize(chain) );
  chain->_ik->_j_ofs = zIndexCreate( rkChainLinkNum(chain) );
  chain->_ik->_c_mat_cell = zMatAlloc( 3, rkChainJointSize(chain) );
  if( !chain->_ik->joint_is_enabled || !chain->_ik->joint_weight ||
      !chain->_ik->joint_vec || !chain->_ik->_j_ofs || !chain->_ik->_c_mat_cell ){
    ZALLOCERROR();
    rkChainDestroyIK( chain );
    return NULL;
  }
  return chain;
}

/* destroy an inverse kinematics solver. */
static void _rkIKDestroy(rkIK *ik)
{
  zFree( ik->joint_is_enabled );
  zFree( ik->joint_weight );
  zVecFree( ik->joint_vec );
  ik->eval = 0;
  rkIKCellListDestroy( &ik->_c_list );
  zMatFree( ik->_c_mat_cell );

  zIndexFree( ik->_j_idx );
  zIndexFree( ik->_j_ofs );
  zVecFree( ik->_j_vec );
  zVecFree( ik->_j_wn );
  zMatFree( ik->_c_mat );
  zVecFree( ik->_c_vec );
  zVecFree( ik->_c_we );
  ik->_solve_eq = NULL;
  zVecFree( ik->__c );
  zLEWorkspaceFree( &ik->__le );
}

/* destroy an inverse kinematics solver of a kinematic chain. */
void rkChainDestroyIK(rkChain *chain)
{
  if( chain->_ik ){
    _rkIKDestroy( chain->_ik );
    zFree( chain->_ik );
  }
}

/* clone an inverse kinematics solver. */
static rkIK *_rkIKClone(rkIK *src)
{
  rkIK *cln;

  if( !( cln = zAlloc( rkIK, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  _rkIKInit( cln );
  cln->joint_is_enabled = zClone( src->joint_is_enabled, bool, zIndexSizeNC(src->_j_ofs) );
  cln->joint_weight = zClone( src->joint_weight, double, zIndexSizeNC(src->_j_ofs) );
  if( !cln->joint_is_enabled || !cln->joint_weight ){
    ZALLOCERROR();
    goto FAILURE;
  }
  cln->joint_vec = zVecClone( src->joint_vec );
  cln->eval = src->eval;
  cln->_c_mat_cell = zMatClone( src->_c_mat_cell );
  zVec3DCopy( &src->_c_vec_cell, &cln->_c_vec_cell );
  cln->_j_idx = zIndexClone( src->_j_idx );
  cln->_j_ofs = zIndexClone( src->_j_ofs );
  cln->_j_vec = zVecClone( src->_j_vec );
  cln->_j_wn = zVecClone( src->_j_wn );
  cln->_c_mat = zMatClone( src->_c_mat );
  cln->_c_vec = zVecClone( src->_c_vec );
  cln->_c_we = zVecClone( src->_c_we );
  cln->_solve_eq = src->_solve_eq;
  cln->__c = zVecClone( src->__c );
  if( !cln->joint_vec || !cln->_c_mat_cell || !cln->_j_idx || !cln->_j_ofs ||
      !cln->_j_vec || !cln->_j_wn || !cln->_c_mat || !cln->_c_vec || !cln->_c_we || !cln->__c ||
      !zLEWorkspaceClone( &src->__le, &cln->__le ) )
    goto FAILURE;
  if( !rkIKCellListClone( &src->_c_list, &cln->_c_list ) ) goto FAILURE;
  if( zListSize( &cln->_c_list ) != zListSize( &src->_c_list ) ) goto FAILURE;
  return cln;

 FAILURE:
  _rkIKDestroy( cln );
  free( cln );
  return NULL;
}

/* clone an inverse kinematics solver of a kinematic chain. */
bool rkChainCloneIK(rkChain *src, rkChain *dest)
{
  if( src->_ik )
    if( !( dest->_ik = _rkIKClone( src->_ik ) ) ) return false;
  return true;
}

/* allocate working memory for constraint coefficient matrix of inverse kinematics solver. */
static bool _rkIKAllocCMat(rkIK *ik)
{
  if( zListSize(&ik->_c_list) == 0 || zArraySize(ik->_j_idx) == 0 )
    return true;
  zMatFree( ik->_c_mat );
  return ( ik->_c_mat = zMatAlloc( zListSize(&ik->_c_list)*3, zVecSizeNC(ik->_j_vec) ) ) ?
    true : false;
}

/* register/unregister a cooperating joint to the inverse kinematics solver. */
static bool _rkIKAllocJointIndex(rkIK *ik, rkChain *chain)
{
  int count, ofs, i;
  int j;
  double *wp;

  for( count=0, i=0; i<rkChainLinkNum(chain); i++ ){
    if( rkChainLinkJointDOF(chain,i) == 0 )
      ik->joint_is_enabled[i] = false;
    if( ik->joint_is_enabled[i] ) count++;
  }
  if( count == 0 ) return true;
  zIndexFree( ik->_j_idx );
  if( !( ik->_j_idx = zIndexCreate(count) ) ) return false;
  for( count=0, ofs=0, i=0; i<rkChainLinkNum(chain); i++ )
    if( ik->joint_is_enabled[i] ){
      zIndexSetElemNC( ik->_j_idx, count++, i );
      zIndexSetElemNC( ik->_j_ofs, i, ofs );
      ofs += rkChainLinkJointDOF(chain,i);
    } else
      zIndexSetElemNC( ik->_j_ofs, i, -1 );
  /* allocate joint vector */
  zVecFree( ik->_j_vec );
  zVecFree( ik->_j_wn );
  zLEWorkspaceFree( &ik->__le );
  count = rkChainJointIndexSize( chain, ik->_j_idx );
  if( !( ik->_j_vec = zVecAlloc(count) ) ||
      !( ik->_j_wn = zVecAlloc(count) ) ||
      !zLEWorkspaceAlloc( &ik->__le, NULL, count ) ) return false;
  for( wp=zVecBuf(ik->_j_wn), i=0; i<zArraySize(ik->_j_idx); i++ )
    for( j=0; j<rkChainLinkJointDOF(chain,zIndexElemNC(ik->_j_idx,i)); j++ )
      *wp++ = ik->joint_weight[zIndexElemNC(ik->_j_idx,i)];
  return _rkIKAllocCMat( ik );
}

static bool _rkIKRegJointID(rkIK *ik, rkChain *chain, int id, bool sw, double weight)
{
  if( id < 0 || id >= rkChainLinkNum(chain) ){
    ZRUNERROR( RK_ERR_LINK_INVID, id );
    return false;
  }
  ik->joint_is_enabled[id] = sw;
  ik->joint_weight[id] = weight;
  return _rkIKAllocJointIndex( ik, chain );
}
bool rkChainRegIKJointID(rkChain *chain, int id, double weight){
  return _rkIKRegJointID( chain->_ik, chain, id, true, weight );
}
bool rkChainUnregIKJointID(rkChain *chain, int id){
  return _rkIKRegJointID( chain->_ik, chain, id, false, 0.0 );
}

/* register all joints to the inverse kinematics solver. */
bool rkChainRegIKJointAll(rkChain *chain, double weight)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointDOF(chain,i) > 0 )
      if( !rkChainRegIKJointID( chain, i, weight ) ) return false;
  return true;
}

/* register constraint cell to the inverse kinematics solver. */
static bool _rkIKAllocCVec(rkIK *ik)
{
  if( zListSize(&ik->_c_list) == 0 ) return true;
  zVecFree( ik->_c_vec );
  zVecFree( ik->_c_we );
  ik->_c_vec = zVecAlloc( zListSize(&ik->_c_list)*3 );
  ik->_c_we = zVecAlloc( zListSize(&ik->_c_list)*3 );
  ik->__c = zVecAlloc( zListSize(&ik->_c_list)*3 );
  return ( !ik->_c_vec || !ik->_c_we || !ik->__c ) ? false : _rkIKAllocCMat( ik );
}
static rkIKCell *_rkIKRegCell(rkIK *ik, const char *name, int priority, rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util)
{
  rkIKCell *cell, *cp;

  if( !( cell = rkIKCellCreate( name, priority, attr, mask, constraint, util ) ) ) return NULL;
  rkIKCellEnable( cell );
  zListForEach( &ik->_c_list, cp )
    if( rkIKCellPriority(cp) > rkIKCellPriority(cell) ) break;
  zListInsertPrev( &ik->_c_list, cp, cell );
  return _rkIKAllocCVec( ik ) ? cell : NULL;
}
static bool _rkIKUnregCell(rkIK *ik, rkIKCell *cell)
{
  zListPurge( &ik->_c_list, cell );
  rkIKCellDestroy( cell );
  zFree( cell );
  return _rkIKAllocCVec( ik );
}

rkIKCell *rkChainRegIKCell(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util)
{
  return _rkIKRegCell( chain->_ik, name, priority, attr, mask, constraint, util );
}

bool rkChainUnregIKCell(rkChain *chain, rkIKCell *cell)
{
  return _rkIKUnregCell( chain->_ik, cell );
}

/* register constraint cell for world position to the inverse kinematics solver. */
rkIKCell *rkChainRegIKCellWldPos(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_link_world_pos, NULL );
}

/* register constraint cell for world attitude to the inverse kinematics solver. */
rkIKCell *rkChainRegIKCellWldAtt(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_link_world_att, NULL );
}

/* register constraint cell for link-to-link position to the inverse kinematics solver. */
rkIKCell *rkChainRegIKCellL2LPos(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_link2link_pos, NULL );
}

/* register constraint cell for link-to-link attitude to the inverse kinematics solver. */
rkIKCell *rkChainRegIKCellL2LAtt(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_link2link_att, NULL );
}

/* register constraint cell for center of mass to the inverse kinematics solver. */
rkIKCell *rkChainRegIKCellCOM(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_world_com, NULL );
}

/* register constraint cell for angular momentum to the inverse kinematics solver. */
rkIKCell *rkChainRegIKCellAM(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_world_angular_momentum, NULL );
}

/* register constraint cell for angular momentum about center of mass to the inverse kinematics solver. */
rkIKCell *rkChainRegIKCellAMCOM(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_world_angular_momentum_about_com, NULL );
}

/* find a constraint cell of inverse kinematics solver by name. */
rkIKCell *rkChainFindIKCellByName(rkChain *chain, const char *name)
{
  rkIKCell *cp;

  zListForEach( &chain->_ik->_c_list, cp )
    if( strcmp( zNamePtr(&cp->data), name ) == 0 ) return cp;
  return NULL;
}

/* disable all constraint cells of inverse kinematics solver. */
void rkChainDisableIK(rkChain *chain)
{
  rkIKCell *cp;

  zListForEach( &chain->_ik->_c_list, cp )
    rkIKCellDisable( cp );
}

/* bind current state of properties to be constrained in inverse kinematics to references. */
void rkChainBindIK(rkChain *chain)
{
  rkIKCell *cp;

  zListForEach( &chain->_ik->_c_list, cp )
    rkIKCellBind( cp, chain );
}

/* zero accumulator of each cell. */
static void _rkIKAcmZero(rkIK *ik)
{
  rkIKCell *cp;

  zListForEach( &ik->_c_list, cp )
    rkIKCellAcmZero( cp );
}
void rkChainZeroIKAcm(rkChain *chain)
{
  _rkIKAcmZero( chain->_ik );
}

/* form the motion contraint equation. */
static int _rkIKCellEq(rkIK *ik, rkChain *chain, rkIKCell *cell, int s, int row)
{
  int i, j;

  if( !( ( RK_IK_CELL_MODE_X << s ) & cell->data.mode ) ) return 0;
  zVecSetElemNC( ik->_c_vec, row, ik->_c_vec_cell.e[s] );
  zVecSetElemNC( ik->_c_we, row, rkIKCellWeight(cell)->e[s] );
  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( ik->joint_is_enabled[i] ){
      for( j=0; j<rkChainLinkJointDOF(chain,i); j++ )
        zMatSetElemNC( ik->_c_mat, row, zIndexElemNC(ik->_j_ofs,i)+j,
          zMatElemNC(ik->_c_mat_cell,s,rkChainLinkJointIDOffset(chain,i)+j) );
    } else{
      for( j=0; j<rkChainLinkJointDOF(chain,i); j++ )
        zVecElemNC(ik->_c_vec,row) -=
          zMatElemNC(ik->_c_mat_cell,s,rkChainLinkJointIDOffset(chain,i)+j)
            * zVecElemNC(ik->joint_vec,rkChainLinkJointIDOffset(chain,i)+j);
    }
  return 1;
}

static void _rkIKCreateEq(rkIK *ik, rkChain *chain, int max_priority, rkIKCell *terminator)
{
  int i;
  rkIKCell *cell;
  int row = 0;

  ik->eval = 0;
  zListForEach( &ik->_c_list, cell ){
    if( cell == terminator ) break;
    if( !rkIKCellIsEnabled( cell ) ) continue;
    rkIKCellGetCMat( cell, chain, ik->_c_mat_cell );
    rkIKCellGetCVec( cell, chain, &ik->_c_vec_cell );
    cell->data._eval = zVec3DWSqrNorm( &ik->_c_vec_cell, rkIKCellWeight(cell) );
    ik->eval += cell->data._eval;
    cell->data._eval = sqrt( cell->data._eval );
    if( rkIKCellPriority(cell) < max_priority )
      rkIKCellGetAcm( cell, chain, &ik->_c_vec_cell );
    for( i=0; i<3; i++ )
      row += _rkIKCellEq( ik, chain, cell, i, row );
  }
  ik->eval = sqrt( ik->eval );
  zMatSetRowSize( ik->_c_mat, row );
  zVecSetSize( ik->_c_vec, row );
  zVecSetSize( ik->__c, row );
  zVecSetSize( ik->_c_we, row );
}

/* solve the motion constraint equation with MP-inverse matrix. */
zVec rkIKSolveEqMP(rkIK *ik)
{
  zLESolveMP( ik->_c_mat, ik->_c_vec, ik->_j_wn, ik->_c_we, ik->_j_vec );
  return ik->_j_vec;
}

/* solve the motion constraint equation with SR-inverse matrix. */
zVec rkIKSolveEqSR(rkIK *ik)
{
  zVecCopy( ik->_c_vec, ik->__c );
  zLESolveSRDST( ik->_c_mat, ik->__c, ik->_j_wn, ik->_c_we, ik->_j_vec, &ik->__le );
  return ik->_j_vec;
}

/* solve the motion constraint equation with error-damped inverse matrix. */
zVec rkIKSolveEqED(rkIK *ik)
{
  zVecCopy( ik->_c_vec, ik->__c );
  zLESolveSRBiasDST( ik->_c_mat, ik->__c, ik->_j_wn, ik->_c_we, zVecInnerProd( ik->_c_vec, ik->__c ), ik->_j_vec, &ik->__le );
  return ik->_j_vec;
}

/* solve the motion constraint equation with error-and-manipulability-damped inverse matrix. */
zVec rkIKSolveEqSRED(rkIK *ik)
{
  zVecCopy( ik->_c_vec, ik->__c );
  zLESolveSRBiasDST( ik->_c_mat, ik->__c, ik->_j_wn, ik->_c_we,
    zVecInnerProd( ik->_c_vec, ik->__c ) + 1.0e1 * _zVecMean(ik->_j_wn) * zMax( 1 - rkJacobiManip( ik->_c_mat ), 0 ),
    ik->_j_vec, &ik->__le );
  return ik->_j_vec;
}

/* solve the motion constraint equation. */
static zVec _rkChainIKSolveEq(rkChain *chain)
{
  int i, j, k;
  double *vp;

  chain->_ik->_solve_eq( chain->_ik );
  for( vp=zVecBuf(chain->_ik->_j_vec), i=0; i<zArraySize(chain->_ik->_j_idx); i++ ){
    k = zIndexElemNC( chain->_ik->_j_idx, i );
    for( j=0; j<rkChainLinkJointDOF(chain,k); j++ )
      zVecSetElemNC( chain->_ik->joint_vec, rkChainLinkJointIDOffset(chain,k)+j, *vp++ );
  }
  return chain->_ik->joint_vec;
}

/* solve one-step inverse kinematics based on Newton=Raphson's method. */
static zVec _rkChainIKOne(rkChain *chain, zVec dis, double dt)
{
  _rkChainIKSolveEq( chain );
  rkChainCatJointDisAll( chain, dis, dt, chain->_ik->joint_vec );
  rkChainSetJointDisAll( chain, dis );
  rkChainGetJointDisAll( chain, dis );
  rkChainUpdateFK( chain );
  return dis;
}

/* solve one-step inverse kinematics based on Newton=Raphson's method. */
zVec rkChainIKOne(rkChain *chain, zVec dis, double dt)
{
  _rkIKCreateEq( chain->_ik, chain, 0, NULL );
  return _rkChainIKOne( chain, dis, dt );
}

static zVec _rkChainGetJointDisRJO(rkChain *chain, zVec dis)
{
  return rkChainGetJointDis( chain, chain->_ik->_j_idx, dis );
}

/* solve one-step inverse kinematics only for registered joints based on Newton=Raphson's method. */
static zVec _rkChainIKOneRJO(rkChain *chain, zVec dis, double dt)
{
  int i, k;
  double *dp, *vp;

  chain->_ik->_solve_eq( chain->_ik );
  for( dp=zVecBuf(dis), vp=zVecBuf(chain->_ik->_j_vec), i=0; i<zArraySize(chain->_ik->_j_idx); i++ ){
    k = zIndexElemNC(chain->_ik->_j_idx,i);
    rkJointCatDis( rkChainLinkJoint(chain,k), dp, dt, vp );
    rkJointSetDis( rkChainLinkJoint(chain,k), dp );
    rkJointGetDis( rkChainLinkJoint(chain,k), dp );
    dp += rkChainLinkJointDOF(chain,k);
    vp += rkChainLinkJointDOF(chain,k);
  }
  rkChainUpdateFK( chain );
  return dis;
}

/* solve (prioritized) inverse kinematics based on Newton=Raphson's method. */
static int _rkChainIK(rkChain *chain, zVec dis, zVec (* _get_joint_dis)(rkChain*,zVec), zVec (*_solve_ik_one)(rkChain*,zVec,double), double tol, int iter)
{
  int i, iter_count = 0;
  double rest = HUGE_VAL;
  rkIKCell *terminator;
  int current_max_priority;

  _get_joint_dis( chain, dis );
  ZITERINIT( iter );
  rkChainZeroIKAcm( chain );
  current_max_priority = rkIKCellPriority( zListTail(&chain->_ik->_c_list) );
  for( terminator = zListTail(&chain->_ik->_c_list);
       terminator != zListRoot(&chain->_ik->_c_list);
       current_max_priority = rkIKCellPriority(terminator) ){
    while( terminator != zListRoot(&chain->_ik->_c_list) &&
           rkIKCellPriority(terminator) <= current_max_priority )
      terminator = zListCellNext(terminator);
    for( i=0; i<iter; i++ ){
      _rkIKCreateEq( chain->_ik, chain, current_max_priority, terminator );
      _solve_ik_one( chain, dis, 1.0 );
      if( zIsTol( chain->_ik->eval - rest, tol ) ){
        iter_count += i;
        break; /* further decrease not expected */
      }
      rest = chain->_ik->eval;
    }
  }
  return iter_count;
}
int rkChainIK(rkChain *chain, zVec dis, double tol, int iter){
  return _rkChainIK( chain, dis, rkChainGetJointDisAll, _rkChainIKOne, tol, iter );
}
int rkChainIK_RJO(rkChain *chain, zVec dis, double tol, int iter){
  return _rkChainIK( chain, dis, _rkChainGetJointDisRJO, _rkChainIKOneRJO, tol, iter );
}

/* ********************************************************** */
/* IK configuration file I/O
 * ********************************************************** */

/* ZTK */

static void *_rkIKJointFromZTK(void *obj, int i, void *arg, ZTK *ztk)
{
  rkLink *link;
  double w = RK_IK_JOINT_WEIGHT_DEFAULT;
  const char *linkname;

  linkname = ZTKVal(ztk);
  if( ZTKValNext(ztk) ) w = ZTKDouble(ztk);
  if( strcmp( linkname, "all" ) == 0 )
    return rkChainRegIKJointAll( (rkChain*)obj, w ) ? obj : NULL;
  if( !( link = rkChainFindLink( (rkChain*)obj, linkname ) ) ){
    ZRUNERROR( RK_ERR_LINK_UNKNOWN, linkname );
    return NULL;
  }
  if( rkLinkJointDOF(link) == 0 ) return NULL;
  if( ZTKValPtr(ztk) )
    rkLinkJoint(link)->com->_dis_fromZTK( rkLinkJoint(link), 0, NULL, ztk );
  if( w == 0 ) return obj;
  return rkChainRegIKJointID( (rkChain*)obj, link - rkChainRoot((rkChain*)obj), w ) ? obj : NULL;
}

static void *_rkIKConstraintFromZTK(void *obj, int i, void *arg, ZTK *ztk)
{
  const rkIKConstraint *constraint;
  rkIKAttr attr;
  ubyte mask = RK_IK_ATTR_MASK_NONE;
  rkLink *link;
  int linknum = 0;
  int priority;
  const char *nameptr;

  priority = ZTKInt(ztk);
  nameptr = ZTKVal(ztk);
  rkIKAttrInit( &attr );
  ZTKValNext( ztk );
  if( !( constraint = rkIKConstraintFind( ZTKVal(ztk) ) ) ) return NULL;
  ZTKValNext( ztk );
  while( ztk->val_cp ){
    if( ZTKValCmp( ztk, "at" ) ){
      ZTKValNext( ztk );
      zVec3DFromZTK( &attr.attention_point, ztk );
      mask |= RK_IK_ATTR_MASK_ATTENTION_POINT;
    } else
    if( ZTKValCmp( ztk, "w" ) ){
      ZTKValNext( ztk );
      zVec3DFromZTK( &attr.weight, ztk );
      mask |= RK_IK_ATTR_MASK_WEIGHT;
    } else{
      if( !( link = rkChainFindLink( (rkChain*)obj, ZTKVal(ztk) ) ) ){
        ZRUNERROR( RK_ERR_LINK_UNKNOWN, ZTKVal(ztk) );
        return NULL;
      }
      if( linknum++ == 0 ){
        attr.id = link - rkChainRoot((rkChain*)obj);
        mask |= RK_IK_ATTR_MASK_ID;
      } else{
        attr.id_sub = link - rkChainRoot((rkChain*)obj);
        mask |= RK_IK_ATTR_MASK_ID_SUB;
      }
      ZTKValNext( ztk );
    }
  }
  return rkChainRegIKCell( (rkChain *)obj, nameptr, priority, &attr, mask, constraint, NULL ) ? obj : NULL;
}

static void _rkChainIKConfJointFPrintZTK(FILE *fp, rkChain *chain)
{
  zVec6D dis;
  int i;
  bool dis_is_zero;

  for( i=0; i<rkChainLinkNum(chain); i++ ){
    if( rkChainLinkJointDOF(chain,i) == 0 ) continue;
    zVec6DZero( &dis );
    rkChainLinkJointGetDis( chain, i, dis.e );
    dis_is_zero = _zVec6DIsTiny( &dis );
    if( chain->_ik->joint_is_enabled[i] || ( !chain->_ik->joint_is_enabled[i] && !dis_is_zero ) ){
      fprintf( fp, "joint: %s\t%g", zName(rkChainLink(chain,i)), chain->_ik->joint_weight[i] );
      if( !dis_is_zero ){
        fputc( ' ', fp );
        rkChainLinkJoint(chain,i)->com->_dis_fprintZTK( fp, i, rkChainLinkJoint(chain,i) );
      } else
        fprintf( fp, "\n" );
    }
  }
}

static void _rkIKConstraintFPrintZTK(FILE *fp, rkChain *chain)
{
  rkIKCell *cp;

  zListForEach( &chain->_ik->_c_list, cp ){
    fprintf( fp, "constraint: %d %s %s", rkIKCellPriority(cp), rkIKCellName(cp), cp->data.constraint->typestr );
    if( cp->data.attr.mask & RK_IK_ATTR_MASK_ID )
      fprintf( fp, " %s", rkChainLinkName(chain,rkIKCellLinkID(cp)) );
    if( cp->data.attr.mask & RK_IK_ATTR_MASK_ID_SUB )
      fprintf( fp, " %s", rkChainLinkName(chain,rkIKCellLinkID2(cp)) );
    if( cp->data.attr.mask & RK_IK_ATTR_MASK_ATTENTION_POINT ){
      fprintf( fp, " at" );
      zVec3DDataFPrint( fp, rkIKCellAttentionPoint(cp) );
    }
    if( cp->data.attr.mask & RK_IK_ATTR_MASK_WEIGHT ){
      fprintf( fp, " w" );
      zVec3DDataFPrint( fp, rkIKCellWeight(cp) );
    }
    fprintf( fp, "\n" );
  }
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

/* read the inverse kinematics configuration of a kinematic chain from ZTK. */
rkChain *rkChainIKConfFromZTK(rkChain *chain, ZTK *ztk)
{
  ZTKEvalTag( chain, NULL, ztk, __ztk_prp_tag_rkik );
  return chain;
}

/* read the inverse kinematics configuration of a kinematic chain from a file. */
rkChain *rkChainIKConfReadZTK(rkChain *chain, const char *filename)
{
  ZTK ztk;

  if( !rkChainCreateIK( chain ) ) return NULL;
  ZTKInit( &ztk );
  if( ZTKParse( &ztk, (char *)filename ) )
    chain = rkChainIKConfFromZTK( chain, &ztk );
  ZTKDestroy( &ztk );
  return chain;
}

/* print the inverse kinematics configuration of a kinematic chain out to the current position of a file. */
void rkChainIKConfFPrintZTK(FILE *fp, rkChain *chain)
{
  fprintf( fp, "[%s]\n", ZTK_TAG_RKIK );
  _rkChainIKConfJointFPrintZTK( fp, chain );
  _rkIKConstraintFPrintZTK( fp, chain );
  fprintf( fp, "\n" );
}

/* write the inverse kinematics configuration of a kinematic chain to a file in ZTK format. */
bool rkChainIKConfWriteZTK(rkChain *chain, const char *filename)
{
  FILE *fp;

  if( !( fp = zOpenZTKFile( (char *)filename, "w" ) ) ) return false;
  rkChainIKConfFPrintZTK( fp, chain );
  fclose( fp );
  return true;
}
