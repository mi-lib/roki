#include <roki/rk_chain.h>

static zVec3D des_vel;

zVec3D *pos_srv(rkChain *chain, rkIKAttr *attr, void *priv, rkIKRef *ref, zVec3D *srv)
{
  zVec3DCopy( &des_vel, srv );
  return srv;
}

zVec3D *att_srv(rkChain *chain, rkIKAttr *attr, void *priv, rkIKRef *ref, zVec3D *srv)
{
  return ZVEC3DZERO;
}

const rkIKConstraint rk_ik_constraint_link_world_lin_vel = {
  typestr: "world_lin_vel",
  ref_fp: rkIKRefSetPos,
  cmat_fp: rkIKJacobiLinkWldLin,
  cvec_fp: pos_srv,
  bind_fp: rkIKBindLinkWldPos,
  acm_fp: NULL,
};

const rkIKConstraint rk_ik_constraint_link_world_ang_vel = {
  typestr: "world_ang_vel",
  ref_fp: rkIKRefSetAA,
  cmat_fp: rkIKJacobiLinkWldAng,
  cvec_fp: att_srv,
  bind_fp: rkIKBindLinkWldAtt,
  acm_fp: NULL,
};

void init(rkChain *puma, rkChain *puma_v, const rkIKConstraint *constraint_lin, const rkIKConstraint *constraint_ang, zVec *dis, rkIKCell *cell[])
{
  rkIKAttr attr;

  if( !rkChainReadZTK( puma, "../model/puma.ztk" ) ) exit( 1 );
  rkChainClone( puma, puma_v );
  rkChainRegisterIKJointAll( puma_v, 0.001 );

  attr.id = 6;
  cell[0] = rkChainRegisterIKCell( puma_v, NULL, 0, &attr, RK_IK_ATTR_MASK_ID, constraint_ang, NULL );
  cell[1] = rkChainRegisterIKCell( puma_v, NULL, 0, &attr, RK_IK_ATTR_MASK_ID, constraint_lin, NULL );

  rkIKSetEquationSolver( puma_v->_ik, rkIKSolveEquationSR );
  rkChainBindIK( puma_v );

  *dis = zVecAlloc( rkChainJointSize( puma ) );
}

void destroy(rkChain *puma, rkChain *puma_v, zVec dis)
{
  rkChainDestroy( puma );
  rkChainDestroy( puma_v );
  zVecFree( dis );
}

#define DT 0.1
#define T  5.0

void cmp(rkChain *ra, rkChain *rb)
{
  zVec3D v, e;

  zVec3DValuePrint( &des_vel );
  zMulMat3DVec3D( rkChainLinkWldAtt(ra,6), rkChainLinkLinVel(ra,6), &v );
  zVec3DValuePrint( &v );
  zVec3DSub( &des_vel, &v, &e );
  zVec3DValuePrint( &e );
  zMulMat3DVec3D( rkChainLinkWldAtt(rb,6), rkChainLinkLinVel(rb,6), &v );
  zVec3DValuePrint( &v );
  zVec3DSub( &des_vel, &v, &e );
  zVec3DValueNLPrint( &e );
}

int main(int argc, char *argv[])
{
  rkChain pumaA, pumaA_v, pumaB, pumaB_v;
  zVec disA, disB;
  rkIKCell *cellA[2], *cellB[2];
  double phase, x, y, z;
  int i, step;
  FILE *fpA, *fpB;

  /* type A: task space tracking */
  init( &pumaA, &pumaA_v, &rk_ik_constraint_link_world_pos, &rk_ik_constraint_link_world_att, &disA, cellA );
  /* type B: configuration space tracking */
  init( &pumaB, &pumaB_v, &rk_ik_constraint_link_world_lin_vel, &rk_ik_constraint_link_world_ang_vel, &disB, cellB );
  x = rkChainLinkWldPos(&pumaA,6)->e[zX];
  y = rkChainLinkWldPos(&pumaA,6)->e[zY];
  z = rkChainLinkWldPos(&pumaA,6)->e[zZ];

  fpA = fopen( "a.zvs", "w" );
  fpB = fopen( "b.zvs", "w" );
  step = T / DT;
  for( i=0; i<=step; i++ ){
    phase = 6 * zPI * i / step;
    zVec3DCreate( &des_vel, 0.1*cos(phase), 0.1*sin(2*phase), 0.05*sin(phase) );

    x += des_vel.e[zX] * DT;
    y += des_vel.e[zY] * DT;
    z += des_vel.e[zZ] * DT;
    rkIKCellSetRef( cellA[1], x, y, z );
    rkChainIK( &pumaA_v, disA, zTOL, 1000 );
    rkChainFKCNT( &pumaA, disA, DT );
    fprintf( fpA, "%f ", DT ); zVecFPrint( fpA, disA );

    rkChainIKOne( &pumaB_v, disB, DT );
    rkChainFKCNT( &pumaB, disB, DT );
    fprintf( fpB, "%f ", DT ); zVecFPrint( fpB, disB );

    cmp( &pumaA, &pumaB );
  }
  fclose( fpA );
  fclose( fpB );

  destroy( &pumaA, &pumaA_v, disA );
  destroy( &pumaB, &pumaB_v, disB );
  return 0;
}
