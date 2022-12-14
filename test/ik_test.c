#include <roki/rk_ik.h>

#define NJ 100
#define NC 10
#define NL 5

#define N 100

void chain_create_link(rkChain *chain, int i, rkJointCom *com)
{
  char name[BUFSIZ];

  sprintf( name, "link#%02d", i );
  rkLinkInit( rkChainLink(chain,i) );
  zNameSet( rkChainLink(chain,i), name );
  rkJointAssign( rkChainLinkJoint(chain,i), com );
}

void chain_add_link(rkChain *chain, int i, int p, rkJointCom *com)
{
  chain_create_link( chain, i, com );
  zVec3DCreate( rkChainLinkOrgPos(chain,i), 1, 0, 0 );
  zFrame3DCopy( rkChainLinkOrgFrame(chain,i), rkChainLinkAdjFrame(chain,i) );
  rkLinkAddChild( rkChainLink(chain,p), rkChainLink(chain,i) );
}

void chain_ik_init(rkChain *chain, rkIK *ik)
{
  int typenum;
  register int i;

  for( typenum=0; rk_joint_com[typenum]; typenum++ );
  rkChainInit( chain );
  zArrayAlloc( &chain->link, rkLink, NJ );
  for( i=0; i<NJ; i++ ){
    chain_create_link( chain, i, rk_joint_com[zRandI(0,typenum-1)] );
    if( i > 0 )
      rkLinkAddChild( rkChainLink(chain,i-1), rkChainLink(chain,i) );
  }
  rkChainSetOffset( chain );
  rkIKCreate( ik, chain );
}

bool assert_joint_reg_one(void)
{
  rkChain chain;
  rkIK ik;
  register int i, j;
  double w;
  zIndex idx, ofs;
  int ofs_head;
  zVec wn;
  bool result = true;

  chain_ik_init( &chain, &ik );
  idx = zIndexCreate( rkChainLinkNum(&chain) );
  ofs = zIndexCreate( rkChainLinkNum(&chain) );
  wn  = zVecAlloc( rkChainJointSize(&chain) );
  zArraySize(idx) = 0;
  zVecSetSize( wn, 0 );

  ofs_head = 0;
  for( i=0; i<NJ; i++ ){
    if( zRandI(0,1) == 0 && rkChainLinkJointSize(&chain,i) > 0 ){
      rkIKJointReg( &ik, i, ( w = zRandF(0,100) ) );
      zIndexIncSize( idx );
      zIndexSetElem( idx, zArraySize(idx)-1, i );
      zIndexSetElem( ofs, i, ofs_head );
      ofs_head += rkChainLinkJointSize(&chain,i);
      for( j=0; j<rkChainLinkJointSize(&chain,i); j++ )
        zVecSetElemNC( wn, zVecSize(wn)+j, w );
      zVecSizeNC(wn) += rkChainLinkJointSize(&chain,i);
    } else
      zIndexSetElem( ofs, i, -1 );
  }

  if( !zIndexIsEqual( idx, ik._j_idx ) ){
    eprintf( "counted index:    " );
    zIndexFPrint( stderr, idx );
    eprintf( "registered index: " );
    zIndexFPrint( stderr, ik._j_idx );
    result = false;
  }
  if( !zIndexIsEqual( ofs, ik._j_ofs ) ){
    eprintf( "counted offset:    " );
    zIndexFPrint( stderr, ofs );
    eprintf( "registered offset: " );
    zIndexFPrint( stderr, ik._j_ofs );
    result = false;
  }
  if( !zVecIsEqual( wn, ik._j_wn, zTOL ) ){
    eprintf( "counted weighting vector:    " );
    zVecFPrint( stderr, wn );
    eprintf( "registered weighting vector: " );
    zVecFPrint( stderr, ik._j_wn );
    result = false;
  }

  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  zIndexFree( idx );
  zIndexFree( ofs );
  zVecFree( wn );
  return result;
}

void assert_joint_reg(void)
{
  register int i;
  bool result = true;

  for( i=0; i<N; i++ )
    if( !assert_joint_reg_one() ) result = false;
  zAssert( rkIKJointReg, result );
}

bool assert_cell_reg_one(void)
{
  rkChain chain;
  rkIK ik;
  rkIKCell *cell[NC];
  int cellcount;
  register int i;
  bool result = true;
  rkIKCell *(*ik_cell_reg[])(rkIK*,rkIKCellAttr*,int) = {
    rkIKCellRegWldPos,
    rkIKCellRegWldAtt,
    rkIKCellRegL2LPos,
    rkIKCellRegL2LAtt,
    rkIKCellRegCOM,
    rkIKCellRegAM,
    rkIKCellRegAMCOM,
    NULL,
  };

  chain_ik_init( &chain, &ik );
  for( cellcount=0; ik_cell_reg[cellcount]; cellcount++ );
  rkIKJointRegAll( &ik, 1 );
  for( i=0; i<NC; i++ )
    cell[i] = ik_cell_reg[zRandI(0,cellcount-1)]( &ik, NULL, RK_IK_CELL_ATTR_NONE );

  cellcount = NC;
  for( i=0; i<NC; i++ ){
    if( zRandI(0,1) == 0 ){
      rkIKCellUnreg( &ik, cell[i] );
      cellcount--;
    }
    if( cellcount == 1 ) break; /* a rare case */
  }
  cellcount *= 3;
  if( zMatRowSizeNC(ik._c_mat) != cellcount ){
    eprintf( "row size of constraint matrix = %d / %d\n", zMatRowSizeNC(ik._c_mat), cellcount );
    result = false;
  }
  if( zVecSizeNC(ik._c_srv) != cellcount ){
    eprintf( "size of SRV = %d / %d\n", zVecSizeNC(ik._c_srv), cellcount );
    result = false;
  }
  if( zVecSizeNC(ik._c_we) != cellcount ){
    eprintf( "size of weighting vector on residual = %d / %d\n", zVecSizeNC(ik._c_we), cellcount );
    result = false;
  }

  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  return result;
}

void assert_cell_reg(void)
{
  register int i;
  bool result = true;

  for( i=0; i<N; i++ ){
    if( !assert_cell_reg_one() ) result = false;
  }
  zAssert( rkIKCellReg + rkIKCellUnreg, result );
}

void assert_ik_revol(void)
{
  register int i;
  rkChain chain;
  rkIKCell *cell;
  rkIKCellAttr attr;
  rkIK ik;
  zVec dis;
  zVec3D err;
  bool result = true;

  rkChainInit( &chain );
  zArrayAlloc( &chain.link, rkLink, NL );
  for( i=0; i<NL; i++ ){
    chain_create_link( &chain, i, i < NL-1 ? &rk_joint_revol : &rk_joint_fixed );
    if( i > 0 ){
      rkLinkAddChild( rkChainLink(&chain,i-1), rkChainLink(&chain,i) );
      zVec3DCreate( rkChainLinkOrgPos(&chain,i), 1, 0, 0 );
    }
    zFrame3DCopy( rkChainLinkOrgFrame(&chain,i), rkChainLinkAdjFrame(&chain,i) );
  }
  rkChainSetMass( &chain, 1.0 ); /* dummy weight */
  rkChainSetOffset( &chain );
  rkChainUpdateFK( &chain );
  rkChainUpdateIDGravity( &chain );

  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkIKCreate( &ik, &chain );
  rkIKJointRegAll( &ik, 0.01 );

  attr.id = rkChainLinkNum(&chain)-1;
  zVec3DZero( &attr.ap );
  cell = rkIKCellRegWldPos( &ik, &attr, RK_IK_CELL_ATTR_ID );

  for( i=0; i<N;i ++ ){
    rkIKDeactivate( &ik );
    rkIKBind( &ik );
    zVec2DCreatePolar( (zVec2D*)&cell->data.ref.pos, zRandF(1,rkChainLinkNum(&chain)-2), zRandF(-zPI,zPI) );
    cell->data.ref.pos.c.z = 0;
    zVecRandUniform( dis, -zPI_2, zPI_2 );
    rkIKSolve( &ik, dis, zTOL, 0 );
    zVec3DSub( &cell->data.ref.pos, zFrame3DPos(rkChainLinkWldFrame(&chain,rkChainLinkNum(&chain)-1)), &err );
    if( !zVec3DIsTol( &err, zTOL*10 ) ){
      zVecFPrint( stderr, dis );
      zVec3DFPrint( stderr, &cell->data.ref.pos );
      zVec3DFPrint( stderr, zFrame3DPos(rkChainLinkWldFrame(&chain,rkChainLinkNum(&chain)-1)) );
      eprintf( "error: " );
      zVec3DFPrint( stderr, &err );
      result = false;
    }
  }
  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  zVecFree( dis );
  zAssert( rkIKSolve (revolute joint), result );
}

void assert_ik_spher(void)
{
  rkChain chain;
  rkIK ik;
  rkIKCellAttr attr;
  zVec dis;
  zVec3D err1, err2;
  rkIKCell *ca0, *ca1;
  register int i;
  bool result = true;

  rkChainInit( &chain );
  zArrayAlloc( &chain.link, rkLink, 3 );
  /* link #0 */
  chain_create_link( &chain, 0, &rk_joint_fixed );
  zMat3DCreate( rkChainLinkOrgAtt(&chain,0), 1, 0, 0, 0, 0, 1, 0,-1, 0 );
  zFrame3DCopy( rkChainLinkOrgFrame(&chain,0), rkChainLinkAdjFrame(&chain,0) );
  /* link #1 */
  chain_create_link( &chain, 1, &rk_joint_spher );
  zFrame3DCopy( rkChainLinkOrgFrame(&chain,1), rkChainLinkAdjFrame(&chain,1) );
  rkLinkAddChild( rkChainLink(&chain,0), rkChainLink(&chain,1) );
  /* link #2 */
  chain_create_link( &chain, 2, &rk_joint_spher );
  zFrame3DCopy( rkChainLinkOrgFrame(&chain,2), rkChainLinkAdjFrame(&chain,2) );
  rkLinkAddChild( rkChainLink(&chain,1), rkChainLink(&chain,2) );

  rkChainSetMass( &chain, 1.0 ); /* dummy weight */
  rkChainSetOffset( &chain );
  rkChainUpdateFK( &chain );
  rkChainUpdateIDGravity( &chain );

  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainGetJointDisAll( &chain, dis );

  rkIKCreate( &ik, &chain );
  rkIKJointReg( &ik, 1, 0.01 );
  rkIKJointReg( &ik, 2, 0.00001 );

  attr.id = 1;
  ca0 = rkIKCellRegWldAtt( &ik, &attr, RK_IK_CELL_ATTR_ID );
  attr.id = 2;
  ca1 = rkIKCellRegWldAtt( &ik, &attr, RK_IK_CELL_ATTR_ID );

  for( i=0; i<N; i++ ){
    rkIKDeactivate( &ik );
    rkIKBind( &ik ); /* bind current status to the reference. */
    rkIKCellSetRef( ca0, zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
    rkIKCellSetRef( ca1, zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );

    rkIKSolve( &ik, dis, zTOL, 0 );
    rkChainFK( ik.chain, dis );
    zMat3DError( &ca0->data.ref.att, rkChainLinkWldAtt(ik.chain,1), &err1 );
    zMat3DError( &ca1->data.ref.att, rkChainLinkWldAtt(ik.chain,2), &err2 );
    if( !zVec3DIsTol( &err1, zTOL*10 ) && zVec3DIsTol( &err2, zTOL*10 ) ){
      eprintf( "error\n" );
      zVec3DFPrint( stderr, &err1 );
      zVec3DFPrint( stderr, &err2 );
      result = false;
    }
  }
  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  zVecFree( dis );
  zAssert( rkIKSolve (spherical joint), result );
}

void assert_ik_float(void)
{
  rkChain chain;
  rkIK ik;
  rkIKCellAttr attr;
  zVec dis;
  zVec6D err1, err2;
  rkIKCell *cl0, *ca0, *cl1, *ca1;
  register int i;
  bool result = true;

  rkChainInit( &chain );
  zArrayAlloc( &chain.link, rkLink, 3 );
  /* link #0 */
  chain_create_link( &chain, 0, &rk_joint_fixed );
  zMat3DCreate( rkChainLinkOrgAtt(&chain,0), 1, 0, 0, 0, 0, 1, 0,-1, 0 );
  zFrame3DCopy( rkChainLinkOrgFrame(&chain,0), rkChainLinkAdjFrame(&chain,0) );
  /* link #1 */
  chain_create_link( &chain, 1, &rk_joint_float );
  zFrame3DCopy( rkChainLinkOrgFrame(&chain,1), rkChainLinkAdjFrame(&chain,1) );
  rkLinkAddChild( rkChainLink(&chain,0), rkChainLink(&chain,1) );
  /* link #2 */
  chain_create_link( &chain, 2, &rk_joint_float );
  zFrame3DCopy( rkChainLinkOrgFrame(&chain,2), rkChainLinkAdjFrame(&chain,2) );
  rkLinkAddChild( rkChainLink(&chain,1), rkChainLink(&chain,2) );

  rkChainSetMass( &chain, 1.0 ); /* dummy weight */
  rkChainSetOffset( &chain );
  rkChainUpdateFK( &chain );
  rkChainUpdateIDGravity( &chain );

  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainGetJointDisAll( &chain, dis );

  rkIKCreate( &ik, &chain );
  rkIKJointReg( &ik, 1, 0.001 );
  rkIKJointReg( &ik, 2, 0.001 );

  attr.id = 1;
  cl0 = rkIKCellRegWldPos( &ik, &attr, RK_IK_CELL_ATTR_ID );
  ca0 = rkIKCellRegWldAtt( &ik, &attr, RK_IK_CELL_ATTR_ID );
  attr.id = 2;
  cl1 = rkIKCellRegWldPos( &ik, &attr, RK_IK_CELL_ATTR_ID );
  ca1 = rkIKCellRegWldAtt( &ik, &attr, RK_IK_CELL_ATTR_ID );

  for( i=0; i<N;i ++ ){
    rkIKDeactivate( &ik );
    rkIKBind( &ik ); /* bind current status to the reference. */
    rkIKCellSetRef( cl0, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
    rkIKCellSetRef( ca0, zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
    rkIKCellSetRef( cl1, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
    rkIKCellSetRef( ca1, zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );

    rkIKSolve( &ik, dis, zTOL, 0 );
    rkChainFK( ik.chain, dis );
    zVec3DSub( &cl0->data.ref.pos, rkChainLinkWldPos(ik.chain,1), zVec6DLin(&err1) );
    zMat3DError( &ca0->data.ref.att, rkChainLinkWldAtt(ik.chain,1), zVec6DAng(&err1) );
    zVec3DSub( &cl1->data.ref.pos, rkChainLinkWldPos(ik.chain,2), zVec6DLin(&err2) );
    zMat3DError( &ca1->data.ref.att, rkChainLinkWldAtt(ik.chain,2), zVec6DAng(&err2) );
    if( !zVec6DIsTol( &err1, zTOL*10 ) && zVec6DIsTol( &err2, zTOL*10 ) ){
      eprintf( "error\n" );
      zVec6DFPrint( stderr, &err1 );
      zVec6DFPrint( stderr, &err2 );
      result = false;
    }
  }
  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  zVecFree( dis );
  zAssert( rkIKSolve (float joint), result );
}

void assert_ik_l2l(void)
{
  rkChain chain;
  rkIK ik;
  rkIKCellAttr attr;
  zVec dis;
  zVec3D err;
  zMat3D rl;
  rkIKCell *cell[4];
  register int i;
  bool result = true;

  rkChainInit( &chain );
  zArrayAlloc( &chain.link, rkLink, NL*2+1 );

  chain_create_link( &chain, 0, &rk_joint_fixed );
  chain_add_link( &chain, 1, 0, &rk_joint_spher );
  chain_add_link( &chain, NL+1, 0, &rk_joint_spher );
  for( i=1; i<NL-1; i++ ){
    chain_add_link( &chain, i+1, i, &rk_joint_spher );
    chain_add_link( &chain, NL+i+1, NL+i, &rk_joint_spher );
  }
  chain_add_link( &chain, NL, NL-1, &rk_joint_spher );
  chain_add_link( &chain, NL*2, NL*2-1, &rk_joint_spher );

  rkChainSetMass( &chain, 1.0 ); /* dummy weight */
  rkChainSetOffset( &chain );

  rkChainUpdateFK( &chain );
  rkChainUpdateIDGravity( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainGetJointDisAll( &chain, dis );

  rkIKCreate( &ik, &chain );
  rkIKJointRegAll( &ik, 0.001 );

  attr.id = NL;
  cell[0] = rkIKCellRegWldPos( &ik, &attr, RK_IK_CELL_ATTR_ID );
  cell[1] = rkIKCellRegWldAtt( &ik, &attr, RK_IK_CELL_ATTR_ID );
  attr.id = NL*2;
  attr.id_sub = NL;
  cell[2] = rkIKCellRegL2LPos( &ik, &attr, RK_IK_CELL_ATTR_ID|RK_IK_CELL_ATTR_ID_SUB );
  cell[3] = rkIKCellRegL2LAtt( &ik, &attr, RK_IK_CELL_ATTR_ID|RK_IK_CELL_ATTR_ID_SUB );

  for( i=0; i<N; i++ ){
    rkIKDeactivate( &ik );
    rkIKBind( &ik );
    zVec3DCreatePolar( &cell[0]->data.ref.pos, zRandF(0,NL-2), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
    rkIKCellSetRef( cell[1], zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
    rkIKCellSetRef( cell[2], 0, 0, 0 );
    rkIKCellSetRef( cell[3], 0, 0, 0 );
    rkIKSolve( &ik, dis, zTOL, 0 );

    zVec3DSub( &cell[0]->data.ref.pos, rkChainLinkWldPos(ik.chain,NL), &err );
    if( !zVec3DIsTol( &err, zTOL*10 ) ){
    zVecFPrint( stderr, dis );
      zVec3DFPrint( stderr, &err );
    }
    zMat3DError( &cell[1]->data.ref.att, rkChainLinkWldAtt(ik.chain,NL), &err );
    if( !zVec3DIsTol( &err, zTOL*10 ) ){
      zVec3DFPrint( stderr, &err );
    }
    zVec3DSub( rkChainLinkWldPos(ik.chain,NL), rkChainLinkWldPos(ik.chain,NL*2), &err );
    zVec3DSubDRC( &err, &cell[2]->data.ref.pos );
    if( !zVec3DIsTol( &err, zTOL*10 ) ){
      zVec3DFPrint( stderr, &err );
    }
    zMulMat3DTMat3D( rkChainLinkWldAtt(ik.chain,NL*2), rkChainLinkWldAtt(ik.chain,NL), &rl );
    zMat3DError( &cell[3]->data.ref.att, &rl, &err );
    if( !zVec3DIsTol( &err, zTOL*10 ) ){
      zVec3DFPrint( stderr, &err );
    }
  }
  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  zVecFree( dis );
  zAssert( rkIKSolve (link-to-link), result );
}

int main(void)
{
  zRandInit();
  assert_joint_reg();
  assert_cell_reg();
  assert_ik_revol();
  assert_ik_spher();
  assert_ik_float();
  assert_ik_l2l();
  return 0;
}
