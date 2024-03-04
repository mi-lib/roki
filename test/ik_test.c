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

void chain_ik_init(rkChain *chain)
{
  int typenum;
  register int i;

  for( typenum=0; rk_joint_com[typenum]; typenum++ );
  rkChainInit( chain );
  rkLinkArrayAlloc( rkChainLinkArray(chain), NJ );
  for( i=0; i<NJ; i++ ){
    chain_create_link( chain, i, rk_joint_com[zRandI(0,typenum-1)] );
    if( i > 0 )
      rkLinkAddChild( rkChainLink(chain,i-1), rkChainLink(chain,i) );
  }
  rkChainSetJointIDOffset( chain );
  rkChainCreateIK( chain );
}

bool assert_joint_reg_one(void)
{
  rkChain chain;
  register int i, j;
  double w;
  zIndex idx, ofs;
  int ofs_head;
  zVec wn;
  bool result = true;

  chain_ik_init( &chain );
  idx = zIndexCreate( rkChainLinkNum(&chain) );
  ofs = zIndexCreate( rkChainLinkNum(&chain) );
  wn  = zVecAlloc( rkChainJointSize(&chain) );
  zArraySize(idx) = 0;
  zVecSetSize( wn, 0 );

  ofs_head = 0;
  for( i=0; i<NJ; i++ ){
    if( zRandI(0,1) == 0 && rkChainLinkJointDOF(&chain,i) > 0 ){
      rkChainRegIKJointID( &chain, i, ( w = zRandF(0,100) ) );
      zIndexIncSize( idx );
      zIndexSetElem( idx, zArraySize(idx)-1, i );
      zIndexSetElem( ofs, i, ofs_head );
      ofs_head += rkChainLinkJointDOF(&chain,i);
      for( j=0; j<rkChainLinkJointDOF(&chain,i); j++ )
        zVecSetElemNC( wn, zVecSize(wn)+j, w );
      zVecSizeNC(wn) += rkChainLinkJointDOF(&chain,i);
    } else
      zIndexSetElem( ofs, i, -1 );
  }

  if( !zIndexIsEqual( idx, chain._ik->_j_idx ) ){
    eprintf( "counted index:    " );
    zIndexFPrint( stderr, idx );
    eprintf( "registered index: " );
    zIndexFPrint( stderr, chain._ik->_j_idx );
    result = false;
  }
  if( !zIndexIsEqual( ofs, chain._ik->_j_ofs ) ){
    eprintf( "counted offset:    " );
    zIndexFPrint( stderr, ofs );
    eprintf( "registered offset: " );
    zIndexFPrint( stderr, chain._ik->_j_ofs );
    result = false;
  }
  if( !zVecIsEqual( wn, chain._ik->_j_wn, zTOL ) ){
    eprintf( "counted weighting vector:    " );
    zVecFPrint( stderr, wn );
    eprintf( "registered weighting vector: " );
    zVecFPrint( stderr, chain._ik->_j_wn );
    result = false;
  }
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
  zAssert( rkChainRegIKJoint, result );
}

bool assert_cell_reg_one(void)
{
  rkChain chain;
  rkIKCell *cell[NC];
  int cellcount;
  register int i;
  bool result = true;
  rkIKCell *(*reg_ik_cell[])(rkChain*,rkIKAttr*,int) = {
    rkChainRegIKCellWldPos,
    rkChainRegIKCellWldAtt,
    rkChainRegIKCellL2LPos,
    rkChainRegIKCellL2LAtt,
    rkChainRegIKCellCOM,
    rkChainRegIKCellAM,
    rkChainRegIKCellAMCOM,
    NULL,
  };

  chain_ik_init( &chain );
  for( cellcount=0; reg_ik_cell[cellcount]; cellcount++ );
  rkChainRegIKJointAll( &chain, 1 );
  for( i=0; i<NC; i++ )
    cell[i] = reg_ik_cell[zRandI(0,cellcount-1)]( &chain, NULL, RK_IK_ATTR_NONE );

  cellcount = NC;
  for( i=0; i<NC; i++ ){
    if( zRandI(0,1) == 0 ){
      rkChainUnregIKCell( &chain, cell[i] );
      cellcount--;
    }
    if( cellcount == 1 ) break; /* a rare case */
  }
  cellcount *= 3;
  if( zMatRowSizeNC(chain._ik->_c_mat) != cellcount ){
    eprintf( "row size of constraint matrix = %d / %d\n", zMatRowSizeNC(chain._ik->_c_mat), cellcount );
    result = false;
  }
  if( zVecSizeNC(rkChainIKConstraintVec(&chain)) != cellcount ){
    eprintf( "size of SRV = %d / %d\n", zVecSizeNC(rkChainIKConstraintVec(&chain)), cellcount );
    result = false;
  }
  if( zVecSizeNC(chain._ik->_c_we) != cellcount ){
    eprintf( "size of weighting vector on residual = %d / %d\n", zVecSizeNC(chain._ik->_c_we), cellcount );
    result = false;
  }
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
  zAssert( rkChainRegIKCell + rkChainUnregIKCell, result );
}

void assert_ik_revol(void)
{
  register int i;
  rkChain chain;
  rkIKCell *cell;
  rkIKAttr attr;
  zVec dis;
  zVec3D err;
  bool result = true;

  rkChainInit( &chain );
  rkLinkArrayAlloc( rkChainLinkArray(&chain), NL );
  for( i=0; i<NL; i++ ){
    chain_create_link( &chain, i, i < NL-1 ? &rk_joint_revol : &rk_joint_fixed );
    if( i > 0 ){
      rkLinkAddChild( rkChainLink(&chain,i-1), rkChainLink(&chain,i) );
      zVec3DCreate( rkChainLinkOrgPos(&chain,i), 1, 0, 0 );
    }
    zFrame3DCopy( rkChainLinkOrgFrame(&chain,i), rkChainLinkAdjFrame(&chain,i) );
  }
  rkChainSetMass( &chain, 1.0 ); /* dummy weight */
  rkChainSetJointIDOffset( &chain );
  rkChainUpdateFK( &chain );
  rkChainUpdateID( &chain );

  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainCreateIK( &chain );
  rkChainRegIKJointAll( &chain, 0.01 );

  attr.id = rkChainLinkNum(&chain)-1;
  zVec3DZero( &attr.ap );
  cell = rkChainRegIKCellWldPos( &chain, &attr, RK_IK_ATTR_ID );

  for( i=0; i<N;i ++ ){
    rkChainDeactivateIK( &chain );
    rkChainBindIK( &chain );
    zVec2DCreatePolar( (zVec2D*)&cell->data.ref.pos, zRandF(1,rkChainLinkNum(&chain)-2), zRandF(-zPI,zPI) );
    cell->data.ref.pos.c.z = 0;
    zVecRandUniform( dis, -zPI_2, zPI_2 );
    rkChainIK( &chain, dis, zTOL, 0 );
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
  rkChainDestroy( &chain );
  zVecFree( dis );
  zAssert( rkChainIK (revolute joint), result );
}

void assert_ik_spher(void)
{
  rkChain chain;
  rkIKAttr attr;
  zVec dis;
  zVec3D err1, err2;
  rkIKCell *ca0, *ca1;
  register int i;
  bool result = true;

  rkChainInit( &chain );
  rkLinkArrayAlloc( rkChainLinkArray(&chain), 3 );
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
  rkChainSetJointIDOffset( &chain );
  rkChainUpdateFK( &chain );
  rkChainUpdateID( &chain );

  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainGetJointDisAll( &chain, dis );

  rkChainCreateIK( &chain );
  rkChainRegIKJointID( &chain, 1, 0.01 );
  rkChainRegIKJointID( &chain, 2, 0.00001 );

  attr.id = 1;
  ca0 = rkChainRegIKCellWldAtt( &chain, &attr, RK_IK_ATTR_ID );
  attr.id = 2;
  ca1 = rkChainRegIKCellWldAtt( &chain, &attr, RK_IK_ATTR_ID );

  for( i=0; i<N; i++ ){
    rkChainDeactivateIK( &chain );
    rkChainBindIK( &chain ); /* bind current status to the reference. */
    rkIKCellSetRef( ca0, zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
    rkIKCellSetRef( ca1, zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );

    rkChainIK( &chain, dis, zTOL, 0 );
    rkChainFK( &chain, dis );
    zMat3DError( &ca0->data.ref.att, rkChainLinkWldAtt(&chain,1), &err1 );
    zMat3DError( &ca1->data.ref.att, rkChainLinkWldAtt(&chain,2), &err2 );
    if( !zVec3DIsTol( &err1, zTOL*10 ) && zVec3DIsTol( &err2, zTOL*10 ) ){
      eprintf( "error\n" );
      zVec3DFPrint( stderr, &err1 );
      zVec3DFPrint( stderr, &err2 );
      result = false;
    }
  }
  rkChainDestroy( &chain );
  zVecFree( dis );
  zAssert( rkChainIK (spherical joint), result );
}

void assert_ik_float(void)
{
  rkChain chain;
  rkIKAttr attr;
  zVec dis;
  zVec6D err1, err2;
  rkIKCell *cl0, *ca0, *cl1, *ca1;
  register int i;
  bool result = true;

  rkChainInit( &chain );
  rkLinkArrayAlloc( rkChainLinkArray(&chain), 3 );
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
  rkChainSetJointIDOffset( &chain );
  rkChainUpdateFK( &chain );
  rkChainUpdateID( &chain );

  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainGetJointDisAll( &chain, dis );

  rkChainCreateIK( &chain );
  rkChainRegIKJointID( &chain, 1, 0.001 );
  rkChainRegIKJointID( &chain, 2, 0.001 );

  attr.id = 1;
  cl0 = rkChainRegIKCellWldPos( &chain, &attr, RK_IK_ATTR_ID );
  ca0 = rkChainRegIKCellWldAtt( &chain, &attr, RK_IK_ATTR_ID );
  attr.id = 2;
  cl1 = rkChainRegIKCellWldPos( &chain, &attr, RK_IK_ATTR_ID );
  ca1 = rkChainRegIKCellWldAtt( &chain, &attr, RK_IK_ATTR_ID );

  for( i=0; i<N;i ++ ){
    rkChainDeactivateIK( &chain );
    rkChainBindIK( &chain ); /* bind current status to the reference. */
    rkIKCellSetRef( cl0, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
    rkIKCellSetRef( ca0, zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
    rkIKCellSetRef( cl1, zRandF(-10,10), zRandF(-10,10), zRandF(-10,10) );
    rkIKCellSetRef( ca1, zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );

    rkChainIK( &chain, dis, zTOL, 0 );
    rkChainFK( &chain, dis );
    zVec3DSub( &cl0->data.ref.pos, rkChainLinkWldPos(&chain,1), zVec6DLin(&err1) );
    zMat3DError( &ca0->data.ref.att, rkChainLinkWldAtt(&chain,1), zVec6DAng(&err1) );
    zVec3DSub( &cl1->data.ref.pos, rkChainLinkWldPos(&chain,2), zVec6DLin(&err2) );
    zMat3DError( &ca1->data.ref.att, rkChainLinkWldAtt(&chain,2), zVec6DAng(&err2) );
    if( !zVec6DIsTol( &err1, zTOL*10 ) && zVec6DIsTol( &err2, zTOL*10 ) ){
      eprintf( "error\n" );
      zVec6DFPrint( stderr, &err1 );
      zVec6DFPrint( stderr, &err2 );
      result = false;
    }
  }
  rkChainDestroy( &chain );
  zVecFree( dis );
  zAssert( rkChainIK (float joint), result );
}

void assert_ik_l2l(void)
{
  rkChain chain;
  rkIKAttr attr;
  zVec dis;
  zVec3D err;
  zMat3D rl;
  rkIKCell *cell[4];
  register int i;
  bool result = true;

  rkChainInit( &chain );
  rkLinkArrayAlloc( rkChainLinkArray(&chain), NL*2+1 );
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
  rkChainSetJointIDOffset( &chain );

  rkChainUpdateFK( &chain );
  rkChainUpdateID( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainGetJointDisAll( &chain, dis );

  rkChainCreateIK( &chain );
  rkChainRegIKJointAll( &chain, 0.001 );

  attr.id = NL;
  cell[0] = rkChainRegIKCellWldPos( &chain, &attr, RK_IK_ATTR_ID );
  cell[1] = rkChainRegIKCellWldAtt( &chain, &attr, RK_IK_ATTR_ID );
  attr.id = NL*2;
  attr.id_sub = NL;
  cell[2] = rkChainRegIKCellL2LPos( &chain, &attr, RK_IK_ATTR_ID | RK_IK_ATTR_ID_SUB );
  cell[3] = rkChainRegIKCellL2LAtt( &chain, &attr, RK_IK_ATTR_ID | RK_IK_ATTR_ID_SUB );

  for( i=0; i<N; i++ ){
    rkChainDeactivateIK( &chain );
    rkChainBindIK( &chain );
    zVec3DCreatePolar( &cell[0]->data.ref.pos, zRandF(0,NL-2), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
    rkIKCellSetRef( cell[1], zRandF(-zPI,zPI), zRandF(-zPI,zPI), zRandF(-zPI,zPI) );
    rkIKCellSetRef( cell[2], 0, 0, 0 );
    rkIKCellSetRef( cell[3], 0, 0, 0 );
    rkChainIK( &chain, dis, zTOL, 0 );

    zVec3DSub( &cell[0]->data.ref.pos, rkChainLinkWldPos(&chain,NL), &err );
    if( !zVec3DIsTol( &err, zTOL*10 ) ){
    zVecFPrint( stderr, dis );
      zVec3DFPrint( stderr, &err );
    }
    zMat3DError( &cell[1]->data.ref.att, rkChainLinkWldAtt(&chain,NL), &err );
    if( !zVec3DIsTol( &err, zTOL*10 ) ){
      zVec3DFPrint( stderr, &err );
    }
    zVec3DSub( rkChainLinkWldPos(&chain,NL), rkChainLinkWldPos(&chain,NL*2), &err );
    zVec3DSubDRC( &err, &cell[2]->data.ref.pos );
    if( !zVec3DIsTol( &err, zTOL*10 ) ){
      zVec3DFPrint( stderr, &err );
    }
    zMulMat3DTMat3D( rkChainLinkWldAtt(&chain,NL*2), rkChainLinkWldAtt(&chain,NL), &rl );
    zMat3DError( &cell[3]->data.ref.att, &rl, &err );
    if( !zVec3DIsTol( &err, zTOL*10 ) ){
      zVec3DFPrint( stderr, &err );
    }
  }
  rkChainDestroy( &chain );
  zVecFree( dis );
  zAssert( rkChainIK (link-to-link), result );
}

void assert_ik_arm(void)
{
  rkChain chain;
  zVec dis;
  zVec3D err;
  rkIKCell *cell;
  rkIKAttr attr;
  int i;
  bool result = true;

  rkChainReadZTK( &chain, "../example/model/arm.ztk" );
  rkChainCreateIK( &chain );
  rkChainRegIKJointAll( &chain, 0.001 );
  attr.id = 5;
  cell = rkChainRegIKCellWldPos( &chain, &attr, RK_IK_ATTR_ID );

  dis = zVecAlloc( rkChainJointSize( &chain ) );
  for( i=0; i<N; i++ ){
    zVecRandUniform( dis, -zPI, zPI );
    rkChainFK( &chain, dis );
    rkIKCellSetRefVec( cell, rkChainLinkWldPos(&chain,attr.id) );
    rkChainFK( &chain, zVecZero(dis) );
    rkChainIK( &chain, dis, zTOL, 0 );
    zVec3DSub( rkIKCellRefPos(cell), rkChainLinkWldPos(&chain,attr.id), &err );
    if( !zVec3DIsTiny( &err ) ) result = false;
  }
  rkChainDestroy( &chain );
  zVecFree( dis );
  zAssert( rkChainIK (5-link arm), result );
}

void assert_puma_arm(void)
{
  rkChain chain;
  zVec dis;
  zVec6D err;
  rkIKCell *cell[2];
  rkIKAttr attr;
  int i;
  bool result = true;
  const double tol = 1.0e-3;

  rkChainReadZTK( &chain, "../example/model/puma.ztk" );
  rkChainCreateIK( &chain );
  rkChainRegIKJointAll( &chain, 0.001 );
  attr.id = 6;
  cell[0] = rkChainRegIKCellWldAtt( &chain, &attr, RK_IK_ATTR_ID );
  cell[1] = rkChainRegIKCellWldPos( &chain, &attr, RK_IK_ATTR_ID );

  dis = zVecAlloc( rkChainJointSize( &chain ) );
  for( i=0; i<N; i++ ){
    zVecRandUniform( dis, -zPI, zPI );
    rkChainFK( &chain, dis );
    rkIKCellSetRefAtt( cell[0], rkChainLinkWldAtt(&chain,attr.id) );
    rkIKCellSetRefVec( cell[1], rkChainLinkWldPos(&chain,attr.id) );
    rkChainFK( &chain, zVecZero(dis) );
    rkChainIK( &chain, dis, zTOL, 0 );
    zMat3DError( rkIKCellRefAtt(cell[0]), rkChainLinkWldAtt(&chain,attr.id), zVec6DAng(&err) );
    zVec3DSub( rkIKCellRefPos(cell[1]), rkChainLinkWldPos(&chain,attr.id), zVec6DLin(&err) );
    if( !zVec6DIsTol( &err, tol ) ){
      zVec6DPrint( &err );
      result = false;
    }
  }
  rkChainDestroy( &chain );
  zVecFree( dis );
  zAssert( rkChainIK (PUMA), result );
}

bool assert_ik_rjo_test(rkChain *chain, rkIKCell *entry[], zVec q, double px, double py, double pz, double ax, double ay, double az)
{
  zVec6D ref, err;
  zFrame3D fd;

  rkChainDeactivateIK( chain );
  rkChainBindIK( chain );
  zVec6DCreate( &ref, px, py, pz, ax, ay, az );
  zVec6DToFrame3DZYX( &ref, &fd );
  rkIKCellSetRefVec( entry[0], zVec6DLin(&ref) );
  rkIKCellSetRefVec( entry[1], zVec6DAng(&ref) );
  rkChainIK_RJO( chain, q, zTOL, 0 );
  zFrame3DError( &fd, rkChainLinkWldFrame(chain,rkIKCellAttr(entry[0])->id), &err );
  return zVec6DIsTiny( &err );
}

void assert_ik_rjo(void)
{
  rkChain chain;
  rkIKAttr attr;
  rkIKCell *entry[2];
  zVec q;
  const double wn = 0.001;
  bool result = true;

  rkChainReadZTK( &chain, "../example/model/H5.ztk" );
  rkChainCreateIK( &chain );
  rkChainRegIKJoint( &chain, "left_hip_rotation",    wn );
  rkChainRegIKJoint( &chain, "left_hip_abduction",   wn );
  rkChainRegIKJoint( &chain, "left_hip_flexion",     wn );
  rkChainRegIKJoint( &chain, "left_knee_flexion",    wn );
  rkChainRegIKJoint( &chain, "left_ankle_flexion",   wn );
  rkChainRegIKJoint( &chain, "left_ankle_abduction", wn );
  zVec3DZero( &attr.ap );
  rkIKAttrSetLinkID( &attr, &chain, "left_foot" );
  entry[0] = rkChainRegIKCellWldPos( &chain, &attr, RK_IK_ATTR_ID | RK_IK_ATTR_AP );
  entry[1] = rkChainRegIKCellWldAtt( &chain, &attr, RK_IK_ATTR_ID );

  q = zVecAlloc( zArraySize( rkChainIKJointIndex(&chain) ) );
  if( !assert_ik_rjo_test( &chain, entry, q, 0.01,-0.02, 0.05, 0, 0, 0 ) ) result = false;
  if( !assert_ik_rjo_test( &chain, entry, q, 0.1,  0.2,  0.2,  0, 0, 0 ) ) result = false;
  if( !assert_ik_rjo_test( &chain, entry, q, 0.01,-0.02, 0.05,-0.25*zPI, 0, 0 ) ) result = false;
  if( !assert_ik_rjo_test( &chain, entry, q, 0.01,-0.02, 0.05, 0.25*zPI, 0, 0 ) ) result = false;
  if( !assert_ik_rjo_test( &chain, entry, q, 0.01,-0.02, 0.05, 0,-0.25*zPI, 0 ) ) result = false;
  if( !assert_ik_rjo_test( &chain, entry, q, 0.01,-0.02, 0.05, 0, 0.25*zPI, 0 ) ) result = false;
  if( !assert_ik_rjo_test( &chain, entry, q, 0.01,-0.02, 0.05, 0, 0,-0.25*zPI ) ) result = false;
  if( !assert_ik_rjo_test( &chain, entry, q, 0.01,-0.02, 0.05, 0, 0, 0.25*zPI ) ) result = false;

  rkChainDestroy( &chain );
  zVecFree( q );
  zAssert( rkChainIK_RJO (H5), result );
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
  assert_ik_arm();
  assert_puma_arm();
  assert_ik_rjo();
  return 0;
}
