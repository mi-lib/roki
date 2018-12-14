#include <roki/rk_ik.h>

/*
base(float)
 --link1(revol)--link2(revol)--link3(fix)
 --link4(revol)--link5(revol)--link6(fix)
 --link7(revol)--link8(revol)--link9(fix)
 */

void chain_init(rkChain *chain)
{
  register int i;
  char name[BUFSIZ];

  rkChainInit( chain );
  zArrayAlloc( &chain->link, rkLink, 10 );
  for( i=0; i<10; i++ ){
    sprintf( name, "link#%02d", i );
    rkLinkInit( rkChainLink(chain,i) );
    zNameSet( rkChainLink(chain,i), name );
  }
  rkJointCreate( rkChainLinkJoint(chain,0), RK_JOINT_FLOAT );
  for( i=0; i<3; i++ ){
    rkJointCreate( rkChainLinkJoint(chain,i*3+1), RK_JOINT_REVOL );
    rkJointCreate( rkChainLinkJoint(chain,i*3+2), RK_JOINT_REVOL );
    rkJointCreate( rkChainLinkJoint(chain,i*3+3), RK_JOINT_REVOL );
    zVec3DCreate( rkChainLinkOrgPos(chain,i*3+2), 0, 1, 0 );
    zVec3DCreate( rkChainLinkOrgPos(chain,i*3+3), 0, 1, 0 );
    rkLinkAddChild( rkChainLink(chain,0), rkChainLink(chain,i*3+1) );
    rkLinkAddChild( rkChainLink(chain,i*3+1), rkChainLink(chain,i*3+2) );
    rkLinkAddChild( rkChainLink(chain,i*3+2), rkChainLink(chain,i*3+3) );
  }
  for( i=0; i<10; i++ )
    zFrame3DCopy( rkChainLinkOrgFrame(chain,i), rkChainLinkAdjFrame(chain,i) );
  rkChainSetMass( chain, 1.0 ); /* dummy weight */
  rkChainSetOffset( chain );
  rkChainUpdateFK( chain );
  rkChainUpdateID( chain );
  rkChainWriteFile( chain, "trident.zkc" );
}

#define DIV 200

int main(int argc, char *argv[])
{
  rkChain chain;
  rkIK ik;
  rkIKCellAttr attr;
  zVec dis;
  zVec6D err;
  rkIKCell *cell[6];
  int i, j;

  chain_init( &chain );
  dis = zVecAlloc( rkChainJointSize(&chain) );
  rkChainFK( &chain, dis );

  rkIKCreate( &ik, &chain );
  rkIKJointRegAll( &ik, 0.01 );

  for( i=0; i<3; i++ ){
    attr.id = i*3+3;
    attr.mode = RK_IK_CELL_FORCE;
    cell[i*2]   = rkIKCellRegWldPos( &ik, &attr, RK_IK_CELL_ATTR_ID | RK_IK_CELL_ATTR_FORCE );
    cell[i*2+1] = rkIKCellRegWldAtt( &ik, &attr, RK_IK_CELL_ATTR_ID | RK_IK_CELL_ATTR_FORCE );
  }
  rkIKDeactivate( &ik );
  rkIKBind( &ik ); /* bind current status to the reference. */

  zVec3DCreate( &cell[0]->data.ref.pos, 1,  1, 0 );
  zVec3DCreate( &cell[2]->data.ref.pos, 1, -1, 0 );
#if 0 /* strong weight */
  rkIKCellSetWeight( cell[4], 0.1, 0.1, 0.1 );
#else
  rkIKCellForce( cell[0] );
  rkIKCellForce( cell[1] );
  rkIKCellForce( cell[2] );
  rkIKCellForce( cell[3] );
#endif

  for( i=0; i<=DIV; i++ ){
    zVec3DCreate( &cell[4]->data.ref.pos,-4.0*(double)i/DIV,  0, 0 );
    rkIKSolve( &ik, dis, zTOL, 0 );
    rkChainFK( ik.chain, dis );
    for( j=0; j<3; j++ ){
      zVec3DSub( &cell[j*2]->data.ref.pos, zFrame3DPos(rkChainLinkWldFrame(ik.chain,j*3+3)), zVec6DLin(&err) );
      zMat3DError( &cell[j*2+1]->data.ref.att, rkChainLinkWldAtt(ik.chain,j*3+3), zVec6DAng(&err) );
      printf( "%g %g ", zVec3DNorm(zVec6DLin(&err)), zVec3DNorm(zVec6DAng(&err)) );
    }
    zEndl();
  }

  rkIKDestroy( &ik );
  rkChainDestroy( &chain );
  zVecFree( dis );
  return 0;
}
