#include <roki/rk_ik.h>

#define DT 0.001

static zVec3D des_acc;

zVec3D *pos_srv(rkChain *chain, rkIKCellAttr *attr, void *priv, rkIKRef *ref, zVec3D *srv)
{
  zVec3D v;

  zMulMatVec3D( rkChainLinkWldAtt(chain,6), rkChainLinkLinVel(chain,6), &v );
  return zVec3DCat( &v, DT, &des_acc, srv );
}

zVec3D *att_srv(rkChain *chain, rkIKCellAttr *attr, void *priv, rkIKRef *ref, zVec3D *srv)
{
  return ZVEC3DZERO;
}


void init(rkChain *puma, rkChain *puma_v, rkIK *ik, rkIKSRV_fp srv_fp, zVec *dis, rkIKCell *cell[])
{
  rkIKCellAttr attr;

  if( !rkChainReadFile( puma, "../model/puma.zkc" ) ) exit( 1 );
  rkChainClone( puma, puma_v );
  rkIKCreate( ik, puma_v );
  rkIKJointRegAll( ik, 0.001 );

  attr.id = 6;
  cell[0] = rkIKCellReg( ik, &attr, RK_IK_CELL_ATTR_ID, rkIKRefSetAA,  rkIKJacobiLinkWldAng, att_srv, rkIKBindLinkWldAtt, NULL, NULL );
  cell[1] = rkIKCellReg( ik, &attr, RK_IK_CELL_ATTR_ID, rkIKRefSetPos, rkIKJacobiLinkWldLin, srv_fp, rkIKBindLinkWldPos, NULL, NULL );

  rkIKSetJointVelMethod( ik, rkIKJointVelSR );
  rkIKDeactivate( ik );
  rkIKBind( ik ); /* bind current status to the reference. */

  *dis = zVecAlloc( rkChainJointSize( puma ) );
}

void destroy(rkChain *puma, rkChain *puma_v, rkIK *ik, zVec dis)
{
  rkIKDestroy( ik );
  rkChainDestroy( puma );
  rkChainDestroy( puma_v );
  zVecFree( dis );
}


#define T  5.0

void cmp(rkChain *ra, rkChain *rb)
{
  zVec3D a, e;

  zVec3DDataWrite( &des_acc );
  zMulMatVec3D( rkChainLinkWldAtt(ra,6), rkChainLinkLinAcc(ra,6), &a );
  zVec3DDataWrite( &a );
  zVec3DSub( &des_acc, &a, &e );
  zVec3DDataWrite( &e );
  zMulMatVec3D( rkChainLinkWldAtt(rb,6), rkChainLinkLinAcc(rb,6), &a );
  zVec3DDataWrite( &a );
  zVec3DSub( &des_acc, &a, &e );
  zVec3DDataNLWrite( &e );
}

int main(int argc, char *argv[])
{
  rkChain pumaA, pumaA_v, pumaB, pumaB_v;
  rkIK ikA, ikB;
  zVec disA, disB;
  rkIKCell *cellA[2], *cellB[2];
  double phase, x, y, z;
  zVec3D v;
  int i, step;
  FILE *fpA, *fpB;

  /* type A: task space tracking */
  init( &pumaA, &pumaA_v, &ikA, rkIKLinkWldPosErr, &disA, cellA );
  /* type B: configuration space tracking */
  init( &pumaB, &pumaB_v, &ikB, pos_srv, &disB, cellB );
  x = rkChainLinkWldPos(&pumaA,6)->e[zX];
  y = rkChainLinkWldPos(&pumaA,6)->e[zY];
  z = rkChainLinkWldPos(&pumaA,6)->e[zZ];

  fpA = fopen( "a.zvs", "w" );
  fpB = fopen( "b.zvs", "w" );
  step = T / DT;
  for( i=0; i<=step; i++ ){
    phase = 4 * zPI * i / step;
    zVec3DCreate( &des_acc, 0.1*cos(phase), 0.1*sin(2*phase), 0.05*sin(phase) );
    zMulMatVec3D( rkChainLinkWldAtt(&pumaA,6), rkChainLinkLinVel(&pumaA,6), &v );
    zVec3DCatDRC( &v, DT, &des_acc );

    x += v.e[zX] * DT;
    y += v.e[zY] * DT;
    z += v.e[zZ] * DT;
    rkIKCellSetRef( cellA[1], x, y, z );
    rkIKSync( &ikA, &pumaA );
    rkIKSolve( &ikA, disA, zTOL, 1000 );
    rkChainFKCNT( &pumaA, disA, DT );
    fprintf( fpA, "%f ", DT ); zVecFWrite( fpA, disA );

    rkIKSync( &ikB, &pumaB );
    rkIKSolveOne( &ikB, disB, DT );
    rkChainFKCNT( &pumaB, disB, DT );
    fprintf( fpB, "%f ", DT ); zVecFWrite( fpB, disB );

    cmp( &pumaA, &pumaB );
  }
  fclose( fpA );
  fclose( fpB );

  destroy( &pumaA, &pumaA_v, &ikA, disA );
  destroy( &pumaB, &pumaB_v, &ikB, disB );
  return 0;
}
