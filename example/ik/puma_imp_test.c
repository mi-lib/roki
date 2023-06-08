#include <roki/rk_chain.h>

rkIKImp imp_pos = { { { 10, 10, 10 } }, { { 0.1, 0.1, 0.1 } } };
rkIKImp imp_att = { { { 10, 10, 10 } }, { { 0.1, 0.1, 0.1 } } };

void init(rkChain *puma, rkChain *puma_v, rkIKCell *cell[])
{
  rkIKAttr attr;

  if( !rkChainReadZTK( puma, "../model/puma.ztk" ) ) exit( 1 );
  rkChainClone( puma, puma_v );
  rkChainCreateIK( puma_v );
  rkChainRegIKJointAll( puma_v, 0.001 );

  attr.id = 6;
  cell[0] = rkChainRegIKCell( puma_v, &attr, RK_IK_ATTR_ID, rkIKRefSetAA,  rkIKJacobiLinkWldAng, rkIKImpWldAtt, rkIKBindLinkWldAtt, NULL, &imp_att );
  cell[1] = rkChainRegIKCell( puma_v, &attr, RK_IK_ATTR_ID, rkIKRefSetPos, rkIKJacobiLinkWldLin, rkIKImpWldPos, rkIKBindLinkWldPos, NULL, &imp_pos );

  rkIKSetEqSolver( puma_v->_ik, rkIKSolveEqSR );
  rkChainDeactivateIK( puma_v );
  rkChainBindIK( puma_v );
}

#define DT 0.001
#define STEP 5000

int main(int argc, char *argv[])
{
  rkChain puma, puma_v;
  zVec dis;
  rkIKCell *cell[2];
  double x0, y0, z0, x, y, z, t;
  register int i;
  zVec6D err;
  zFrame3D goal;
  FILE *fp;

  init( &puma, &puma_v, cell );
  dis = zVecAlloc( rkChainJointSize( &puma ) );

  /* initial position and attitude */
  x0 = rkChainLinkWldPos(&puma_v,6)->e[zX] + 0.05;
  y0 = 0;
  z0 = rkChainLinkWldPos(&puma_v,6)->e[zZ] + 0.1;

  fp = fopen( "rmr", "w" );
  for( i=0; i<=STEP; i++ ){
    t = 4 * zPI * i / STEP;
    x = x0 + 0.05 * sin(2*t);
    y = y0 + 0.10 * sin(t);
    z = z0;
    rkIKCellSetRef( cell[1], x, y, z );
    rkChainCopyState( &puma, &puma_v );
    rkChainIKOne( &puma_v, dis, DT );
    rkChainFKCNT( &puma, dis, DT );
    printf( "%f ", DT ); zVecPrint( dis );
    fprintf( fp, "%f %f %f %f ", DT*i/STEP, x, y, z );
    zVec3DDataFPrint( fp, rkChainLinkWldPos(&puma_v,6) );

    zFrame3DCreate( &goal, &cell[1]->data.ref.pos, &cell[0]->data.ref.att );
    zFrame3DError( &goal, rkChainLinkWldFrame(&puma_v,6), &err );
    eprintf( "%.16f %.16f\n", zVec3DNorm(zVec6DLin(&err)), zVec3DNorm(zVec6DAng(&err)) );
  }
  fclose( fp );

  zVecFree( dis );
  rkChainDestroy( &puma );
  rkChainDestroy( &puma_v );
  return 0;
}
