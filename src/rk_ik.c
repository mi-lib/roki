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
  zListInit( &ik->cell_list );

  ik->_eval = 0;
  ik->_c_mat_cell = NULL;
  zVec3DZero( &ik->_c_vec_cell );
  ik->_j_idx = NULL;
  ik->_j_ofs = NULL;
  ik->_j_vec = NULL;
  ik->_j_wn = NULL;
  ik->_c_mat = NULL;
  ik->_c_vec = NULL;
  ik->_c_we = NULL;
  ik->_solve_eq = rkIKSolveEquationED; /* default motion constraint solver */
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
  ik->_eval = 0;
  rkIKCellListDestroy( &ik->cell_list );

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
  cln->_eval = src->_eval;
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
  if( !rkIKCellListClone( &src->cell_list, &cln->cell_list ) ) goto FAILURE;
  if( zListSize( &cln->cell_list ) != zListSize( &src->cell_list ) ) goto FAILURE;
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

/* allocate working memory for constraint coefficient matrix of an inverse kinematics solver. */
static bool _rkIKAllocCMat(rkIK *ik)
{
  zMatFree( ik->_c_mat );
  if( zListSize(&ik->cell_list) == 0 || !ik->_j_idx || zArraySize(ik->_j_idx) == 0 ){
    ik->_c_mat = NULL;
    return true;
  }
  return ( ik->_c_mat = zMatAlloc( zListSize(&ik->cell_list)*3, zVecSizeNC(ik->_j_vec) ) ) ?
    true : false;
}

/* register/unregister a cooperating joint to an inverse kinematics solver. */
static bool _rkIKAllocJointIndex(rkIK *ik, rkChain *chain)
{
  int count, ofs, i;
  int j;
  double *wp;

  zIndexFree( ik->_j_idx );
  zVecFree( ik->_j_vec );
  zVecFree( ik->_j_wn );
  zLEWorkspaceFree( &ik->__le );
  for( count=0, i=0; i<rkChainLinkNum(chain); i++ ){
    if( rkChainLinkJointDOF(chain,i) == 0 )
      ik->joint_is_enabled[i] = false;
    if( ik->joint_is_enabled[i] ) count++;
  }
  if( count == 0 ) return true;
  if( !( ik->_j_idx = zIndexCreate(count) ) ) return false;
  for( count=0, ofs=0, i=0; i<rkChainLinkNum(chain); i++ )
    if( ik->joint_is_enabled[i] ){
      zIndexSetElemNC( ik->_j_idx, count++, i );
      zIndexSetElemNC( ik->_j_ofs, i, ofs );
      ofs += rkChainLinkJointDOF(chain,i);
    } else
      zIndexSetElemNC( ik->_j_ofs, i, -1 );
  /* allocate joint vector */
  count = rkChainJointIndexSize( chain, ik->_j_idx );
  if( !( ik->_j_vec = zVecAlloc(count) ) ||
      !( ik->_j_wn = zVecAlloc(count) ) ||
      !zLEWorkspaceAlloc( &ik->__le, NULL, count ) ) return false;
  for( wp=zVecBuf(ik->_j_wn), i=0; i<zArraySize(ik->_j_idx); i++ )
    for( j=0; j<rkChainLinkJointDOF(chain,zIndexElemNC(ik->_j_idx,i)); j++ )
      *wp++ = ik->joint_weight[zIndexElemNC(ik->_j_idx,i)];
  return _rkIKAllocCMat( ik );
}

static bool _rkIKRegisterJointID(rkIK *ik, rkChain *chain, int id, bool sw, double weight)
{
  if( id < 0 || id >= rkChainLinkNum(chain) ){
    ZRUNERROR( RK_ERR_LINK_INVALID_ID, id );
    return false;
  }
  ik->joint_is_enabled[id] = sw;
  ik->joint_weight[id] = weight;
  return _rkIKAllocJointIndex( ik, chain );
}
bool rkChainRegisterIKJointID(rkChain *chain, int id, double weight){
  if( !chain->_ik )
    if( !rkChainCreateIK( chain ) ) return false;
  return _rkIKRegisterJointID( chain->_ik, chain, id, true, weight );
}
bool rkChainUnregisterIKJointID(rkChain *chain, int id){
  if( !chain->_ik )
    if( !rkChainCreateIK( chain ) ) return false;
  return _rkIKRegisterJointID( chain->_ik, chain, id, false, 0.0 );
}

/* register all joints to an inverse kinematics solver. */
bool rkChainRegisterIKJointAll(rkChain *chain, double weight)
{
  int i;

  for( i=0; i<rkChainLinkNum(chain); i++ )
    if( rkChainLinkJointDOF(chain,i) > 0 )
      if( !rkChainRegisterIKJointID( chain, i, weight ) ) return false;
  return true;
}

/* register constraint cell to an inverse kinematics solver. */
static bool _rkIKAllocCVec(rkIK *ik)
{
  zVecFree( ik->_c_vec );
  zVecFree( ik->_c_we );
  zVecFree( ik->__c );
  if( zListSize(&ik->cell_list) == 0 ){
    ik->_c_vec = ik->_c_we = ik->__c = NULL;
    return true;
  }
  ik->_c_vec = zVecAlloc( zListSize(&ik->cell_list)*3 );
  ik->_c_we = zVecAlloc( zListSize(&ik->cell_list)*3 );
  ik->__c = zVecAlloc( zListSize(&ik->cell_list)*3 );
  return ( !ik->_c_vec || !ik->_c_we || !ik->__c ) ? false : _rkIKAllocCMat( ik );
}

static rkIKCell *_rkIKAddCell(rkIK *ik, rkIKCell *cell){
  rkIKCell *cp;

  if( !cell ) return NULL;
  rkIKCellEnable( cell );
  zListForEach( &ik->cell_list, cp )
    if( rkIKCellPriority(cp) < rkIKCellPriority(cell) ) break;
  zListInsertPrev( &ik->cell_list, cp, cell );
  return _rkIKAllocCVec( ik ) ? cell : NULL;
}

static rkIKCell *_rkIKRegisterCell(rkIK *ik, const char *name, int priority, rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util)
{
  rkIKCell *cell;

  if( !( cell = rkIKCellCreate( name, priority, attr, mask, constraint, util ) ) ) return NULL;
  return _rkIKAddCell( ik, cell );
}

static bool _rkIKUnregisterCell(rkIK *ik, rkIKCell *cell)
{
  if( !cell ) return false;
  zListPurge( &ik->cell_list, cell );
  return _rkIKAllocCVec( ik );
}

static bool _rkIKUnregisterAndDestroyCell(rkIK *ik, rkIKCell *cell)
{
  bool result;

  result = _rkIKUnregisterCell( ik, cell );
  if( cell ){
    rkIKCellDestroy( cell );
    zFree( cell );
  }
  return result;
}

rkIKCell *rkChainAddIKCell(rkChain *chain, rkIKCell *cell)
{
  return _rkIKAddCell( chain->_ik, cell );
}

rkIKCell *rkChainRegisterIKCell(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util)
{
  if( !chain->_ik )
    if( !rkChainCreateIK( chain ) ) return NULL;
  return _rkIKRegisterCell( chain->_ik, name, priority, attr, mask, constraint, util );
}

bool rkChainUnregisterIKCell(rkChain *chain, rkIKCell *cell)
{
  if( !chain->_ik )
    if( !rkChainCreateIK( chain ) ) return false;
  return _rkIKUnregisterCell( chain->_ik, cell );
}

bool rkChainUnregisterAndDestroyIKCell(rkChain *chain, rkIKCell *cell)
{
  if( !chain->_ik )
    if( !rkChainCreateIK( chain ) ) return false;
  return _rkIKUnregisterAndDestroyCell( chain->_ik, cell );
}

/* register constraint cell for world position to an inverse kinematics solver. */
rkIKCell *rkChainRegisterIKCellWldPos(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegisterIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_link_world_pos, NULL );
}

/* register constraint cell for world attitude to an inverse kinematics solver. */
rkIKCell *rkChainRegisterIKCellWldAtt(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegisterIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_link_world_att, NULL );
}

/* register constraint cell for link-to-link position to an inverse kinematics solver. */
rkIKCell *rkChainRegisterIKCellL2LPos(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegisterIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_link2link_pos, NULL );
}

/* register constraint cell for link-to-link attitude to an inverse kinematics solver. */
rkIKCell *rkChainRegisterIKCellL2LAtt(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegisterIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_link2link_att, NULL );
}

/* register constraint cell for center of mass to an inverse kinematics solver. */
rkIKCell *rkChainRegisterIKCellCOM(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegisterIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_world_com, NULL );
}

/* register constraint cell for angular momentum to an inverse kinematics solver. */
rkIKCell *rkChainRegisterIKCellAM(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegisterIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_world_angular_momentum, NULL );
}

/* register constraint cell for angular momentum about center of mass to an inverse kinematics solver. */
rkIKCell *rkChainRegisterIKCellAMCOM(rkChain *chain, const char *name, int priority, rkIKAttr *attr, ubyte mask){
  return rkChainRegisterIKCell( chain, name, priority, attr, mask, &rk_ik_constraint_world_angular_momentum_about_com, NULL );
}

/* find a constraint cell of an inverse kinematics solver by name. */
rkIKCell *rkChainFindIKCellByName(rkChain *chain, const char *name)
{
  rkIKCell *cp;

  zListForEach( &chain->_ik->cell_list, cp )
    if( strcmp( zNamePtr(&cp->data), name ) == 0 ) return cp;
  return NULL;
}

/* set priority of a constraint cell of an inverse kinematics solver. */
static bool _rkIKCellSetPriority(rkIK *ik, rkIKCell *cell, int priority)
{
  rkIKCell *cp;

  if( !cell ){
    ZRUNERROR( RK_ERR_IK_CELL_IS_NULL );
    return false;
  }
  if( priority > rkIKCellPriority(cell) ){
    for( cp = zListCellPrev(cell);
         cp != zListRoot(&ik->cell_list) && rkIKCellPriority(cp) < priority;
         cp = zListCellPrev(cp) );
    zListCellPurge( cell );
    zListCellInsertNext( cp, cell );
  } else
  if( priority < rkIKCellPriority(cell) ){
    for( cp = zListCellNext(cell);
         cp != zListRoot(&ik->cell_list) && rkIKCellPriority(cp) > priority;
         cp = zListCellNext(cp) );
    zListCellPurge( cell );
    zListCellInsertPrev( cp, cell );
  }
  rkIKCellPriority(cell) = priority;
  return true;
}
bool rkChainSetIKCellPriority(rkChain *chain, rkIKCell *cell, int priority)
{
  return _rkIKCellSetPriority( chain->_ik, cell, priority );
}

/* disable all constraint cells of an inverse kinematics solver. */
void rkChainDisableIK(rkChain *chain)
{
  rkIKCell *cp;

  zListForEach( &chain->_ik->cell_list, cp )
    rkIKCellDisable( cp );
}

/* bind current state of properties to be constrained in inverse kinematics to references. */
void rkChainBindIK(rkChain *chain)
{
  rkIKCell *cp;

  zListForEach( &chain->_ik->cell_list, cp )
    rkIKCellBind( cp, chain );
}

/* zero accumulator of each cell. */
static void _rkIKAcmZero(rkIK *ik)
{
  rkIKCell *cp;

  zListForEach( &ik->cell_list, cp )
    rkIKCellAcmZero( cp );
}
void rkChainZeroIKAcm(rkChain *chain)
{
  _rkIKAcmZero( chain->_ik );
}

/* get a cell-wise motion contraint equation. */
static int _rkIKCellGetEquation(rkIK *ik, rkChain *chain, rkIKCell *cell, int s, int row)
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

/* create a motion constraint equation of an inverse kinematics solver. */
static bool _rkIKCreateEquation(rkIK *ik, rkChain *chain, int min_priority, rkIKCell *terminator)
{
  int i;
  rkIKCell *cell;
  int row = 0;

  if( !ik->_j_idx ){ /* joints are not registered. */
    ZRUNERROR( RK_ERR_IK_JOINT_UNREGISTERED );
    return false;
  }
  ik->_eval = 0;
  zListForEach( &ik->cell_list, cell ){
    if( cell == terminator ) break;
    if( !rkIKCellIsEnabled( cell ) ) continue;
    rkIKCellGetCMat( cell, chain, ik->_c_mat_cell );
    rkIKCellGetCVec( cell, chain, &ik->_c_vec_cell );
    cell->data._eval = zVec3DWSqrNorm( &ik->_c_vec_cell, rkIKCellWeight(cell) );
    ik->_eval += cell->data._eval;
    cell->data._eval = sqrt( cell->data._eval );
    if( rkIKCellPriority(cell) > min_priority )
      rkIKCellGetAcm( cell, chain, &ik->_c_vec_cell );
    for( i=0; i<3; i++ )
      row += _rkIKCellGetEquation( ik, chain, cell, i, row );
  }
  ik->_eval = sqrt( ik->_eval );
  if( row == 0 ) return false; /* the list of constraint cells is empty. */
  zMatSetRowSize( ik->_c_mat, row );
  zVecSetSize( ik->_c_vec, row );
  zVecSetSize( ik->__c, row );
  zVecSetSize( ik->_c_we, row );
  return true;
}

/* solve the motion constraint equation with MP-inverse matrix. */
zVec rkIKSolveEquationMP(rkIK *ik)
{
  zLESolveMP( ik->_c_mat, ik->_c_vec, ik->_j_wn, ik->_c_we, ik->_j_vec );
  return ik->_j_vec;
}

/* solve the motion constraint equation with SR-inverse matrix. */
zVec rkIKSolveEquationSR(rkIK *ik)
{
  zVecCopy( ik->_c_vec, ik->__c );
  zLESolveSRDST( ik->_c_mat, ik->__c, ik->_j_wn, ik->_c_we, ik->_j_vec, &ik->__le );
  return ik->_j_vec;
}

/* solve the motion constraint equation with error-damped inverse matrix. */
zVec rkIKSolveEquationED(rkIK *ik)
{
  zVecCopy( ik->_c_vec, ik->__c );
  zLESolveSRBiasDST( ik->_c_mat, ik->__c, ik->_j_wn, ik->_c_we, zVecInnerProd( ik->_c_vec, ik->__c ), ik->_j_vec, &ik->__le );
  return ik->_j_vec;
}

/* solve the motion constraint equation with error-and-manipulability-damped inverse matrix. */
zVec rkIKSolveEquationSRED(rkIK *ik)
{
  zVecCopy( ik->_c_vec, ik->__c );
  zLESolveSRBiasDST( ik->_c_mat, ik->__c, ik->_j_wn, ik->_c_we,
    zVecInnerProd( ik->_c_vec, ik->__c ) + 1.0e1 * _zVecElemMean(ik->_j_wn) * zMax( 1 - rkJacobiManip( ik->_c_mat ), 0 ),
    ik->_j_vec, &ik->__le );
  return ik->_j_vec;
}

/* solve the motion constraint equation. */
static zVec _rkChainIKSolveEquation(rkChain *chain)
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
  _rkChainIKSolveEquation( chain );
  rkChainCatJointDisAll( chain, dis, dt, chain->_ik->joint_vec );
  rkChainSetJointDisAll( chain, dis );
  rkChainGetJointDisAll( chain, dis );
  rkChainUpdateFK( chain );
  return dis;
}

/* solve one-step inverse kinematics based on Newton=Raphson's method. */
zVec rkChainIKOne(rkChain *chain, zVec dis, double dt)
{
  if( !_rkIKCreateEquation( chain->_ik, chain, 0, NULL ) ) return NULL;
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
  double rest;
  rkIKCell *terminator, *terminator_prev;
  int current_min_priority;

  if( !chain->_ik ) return -1;
  _get_joint_dis( chain, dis );
  ZITERINIT( iter );
  rkChainZeroIKAcm( chain );
  terminator_prev = zListTail(&chain->_ik->cell_list);
  current_min_priority = rkIKCellPriority( terminator_prev );
  for( terminator=terminator_prev; ; current_min_priority=rkIKCellPriority(terminator) ){
    while( terminator != zListRoot(&chain->_ik->cell_list) &&
           rkIKCellPriority(terminator) >= current_min_priority )
      terminator = zListCellNext(terminator);
    for( rest=HUGE_VAL, i=0; i<iter; i++ ){
      if( !_rkIKCreateEquation( chain->_ik, chain, current_min_priority, terminator ) ) return -1;
      _solve_ik_one( chain, dis, 1.0 );
      if( zIsTol( chain->_ik->_eval - rest, tol ) ){
        iter_count += i;
        break; /* further decrease not expected */
      }
      rest = chain->_ik->_eval;
    }
    if( terminator == zListRoot(&chain->_ik->cell_list) ) break;
    for( ; terminator_prev!=terminator; terminator_prev=zListCellNext(terminator_prev) )
      if( rkIKCellIsEnabled( terminator_prev ) )
        rkIKCellBind( terminator_prev, chain );
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
    return rkChainRegisterIKJointAll( (rkChain*)obj, w ) ? obj : NULL;
  if( !( link = rkChainFindLink( (rkChain*)obj, linkname ) ) ){
    ZRUNERROR( RK_ERR_LINK_UNKNOWN, linkname );
    return NULL;
  }
  if( rkLinkJointDOF(link) == 0 ) return NULL;
  if( ZTKValPtr(ztk) )
    rkLinkJoint(link)->com->_dis_fromZTK( rkLinkJoint(link), 0, NULL, ztk );
  if( w == 0 ) return obj;
  return rkChainRegisterIKJointID( (rkChain*)obj, rkChainLinkIDOffset((rkChain*)obj,link), w ) ? obj : NULL;
}

static void *_rkIKConstraintFromZTK(void *obj, int i, void *arg, ZTK *ztk)
{
  const rkIKConstraint *constraint;
  rkIKAttr attr;
  ubyte mask = RK_IK_ATTR_MASK_NONE;
  int priority;
  const char *nameptr;

  priority = ZTKInt(ztk);
  nameptr = ZTKVal(ztk);
  rkIKAttrInit( &attr );
  ZTKValNext( ztk );
  if( !( constraint = rkIKConstraintFind( ZTKVal(ztk) ) ) ) return NULL;
  ZTKValNext( ztk );
  if( !constraint->fromZTK( (rkChain*)obj, &attr, &mask, ztk ) ){
    ZRUNERROR( "in persing constraint %s", nameptr );
    return NULL;
  }
  return rkChainRegisterIKCell( (rkChain *)obj, nameptr, priority, &attr, mask, constraint, NULL ) ? obj : NULL;
}

static const ZTKPrp __ztk_prp_rkik[] = {
  { ZTK_KEY_ROKI_CHAIN_IK_JOINT,      -1, _rkIKJointFromZTK, NULL },
  { ZTK_KEY_ROKI_CHAIN_IK_CONSTRAINT, -1, _rkIKConstraintFromZTK, NULL },
};

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
      fprintf( fp, "%s: %s\t%g", ZTK_KEY_ROKI_CHAIN_IK_JOINT, zName(rkChainLink(chain,i)), chain->_ik->joint_weight[i] );
      if( !dis_is_zero ){
        fputc( ' ', fp );
        rkJointDisFPrintZTK( fp, rkChainLinkJoint(chain,i) );
      } else
        fprintf( fp, "\n" );
    }
  }
}

static void _rkIKConstraintFPrintZTK(FILE *fp, rkChain *chain)
{
  rkIKCell *cp;

  zListForEach( &chain->_ik->cell_list, cp ){
    fprintf( fp, "%s: %d %s %s", ZTK_KEY_ROKI_CHAIN_IK_CONSTRAINT, rkIKCellPriority(cp), rkIKCellName(cp), cp->data.constraint->typestr );
    cp->data.constraint->fprintZTK( fp, chain, cp );
  }
}

static void *_rkIKFromZTK(void *obj, int i, void *arg, ZTK *ztk)
{
  if( !_ZTKEvalKey( obj, NULL, ztk, __ztk_prp_rkik ) ) return NULL;
  return obj;
}

static const ZTKPrp __ztk_prp_tag_roki_chain_ik[] = {
  { ZTK_TAG_ROKI_CHAIN_IK, 1, _rkIKFromZTK, NULL },
};

/* read the inverse kinematics configuration of a kinematic chain from ZTK. */
rkChain *rkChainIKConfFromZTK(rkChain *chain, ZTK *ztk)
{
  _ZTKEvalTag( chain, NULL, ztk, __ztk_prp_tag_roki_chain_ik );
  return chain;
}

/* read the inverse kinematics configuration of a kinematic chain from a file. */
rkChain *rkChainIKConfReadZTK(rkChain *chain, const char *filename)
{
  ZTK ztk;

  if( !rkChainCreateIK( chain ) ) return NULL;
  ZTKInit( &ztk );
  if( ZTKParse( &ztk, filename ) )
    chain = rkChainIKConfFromZTK( chain, &ztk );
  ZTKDestroy( &ztk );
  return chain;
}

/* print the inverse kinematics configuration of a kinematic chain out to the current position of a file. */
void rkChainIKConfFPrintZTK(FILE *fp, rkChain *chain)
{
  fprintf( fp, "[%s]\n", ZTK_TAG_ROKI_CHAIN_IK );
  _rkChainIKConfJointFPrintZTK( fp, chain );
  _rkIKConstraintFPrintZTK( fp, chain );
  fprintf( fp, "\n" );
}

/* write the inverse kinematics configuration of a kinematic chain to a file in ZTK format. */
bool rkChainIKConfWriteZTK(rkChain *chain, const char *filename)
{
  FILE *fp;

  if( !( fp = zOpenZTKFile( filename, "wb" ) ) ) return false;
  rkChainIKConfFPrintZTK( fp, chain );
  fclose( fp );
  return true;
}
