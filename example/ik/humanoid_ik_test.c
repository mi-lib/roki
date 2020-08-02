#include <roki/rk_ik.h>

#define T      1.0
#define DIV  100
#define S      0.2
#define W      0.05
#define WG     0.03
#define HG     0.66
#define H      0.05
#define D      0.03

#define DT     ( T / DIV )

void ik_cell_output(FILE *fp, double dt, zVec3D *pg, zVec3D *pl, zVec3D *pr)
{
  fprintf( fp, "%.10g 6 ", dt );
  fprintf( fp, " 0 1 1 1" );
  zVec3DDataFPrint( fp, pg );
  fprintf( fp, " 1 1 1 1 0 0 0" );
  fprintf( fp, " 2 1 1 1" );
  zVec3DDataFPrint( fp, pl );
  fprintf( fp, " 3 1 1 1 0 0 0" );
  fprintf( fp, " 4 1 1 1" );
  zVec3DDataFPrint( fp, pr );
  fprintf( fp, " 5 1 1 1 0 0 0\n" );
}

void ik_solve(FILE *fout_vs, FILE *fout_cs, rkIK *ik, rkIKCell *entry[], zVec q, zVec3D *pg, zVec3D *pl, zVec3D *pr)
{
  rkIKDeactivate( ik );
  rkIKBind( ik );
  rkIKCellSetRefVec( entry[0], pg );
  rkIKCellSetRefVec( entry[1], ZVEC3DZERO );
  rkIKCellSetRefVec( entry[2], pl );
  rkIKCellSetRefVec( entry[3], ZVEC3DZERO );
  rkIKCellSetRefVec( entry[4], pr );
  rkIKCellSetRefVec( entry[5], ZVEC3DZERO );
  rkIKSolve( ik, q, zTOL, 0 );
  fprintf( fout_vs, "%g ", DT );
  zVecFPrint( fout_vs, q );
  ik_cell_output( fout_cs, DT, pg, pl, pr );
}

void step1(FILE *fout_vs, FILE *fout_cs, double t, rkIK *ik, rkIKCell *entry[], zVec q)
{
  zVec3D pg, pl, pr;

  zVec3DCreate( &pg, 0.5*S*(cosh(t/T)-1)/(cosh(1)-1), -WG*sin(zPI*t/T), HG-D*zCycloidX(t/T) );
  zVec3DCreate( &pl, S*zCycloidX(t/T), W, H*zCycloidY(t/T) );
  zVec3DCreate( &pr, 0.0, -W, 0.0 );
  ik_solve( fout_vs, fout_cs, ik, entry, q, &pg, &pl, &pr );
}

void step2(FILE *fout_vs, FILE *fout_cs, double t, rkIK *ik, rkIKCell *entry[], zVec q)
{
  zVec3D pg, pl, pr;

  zVec3DCreate( &pg, 1.5*S-S*(cosh(1-t/T)-1)/(cosh(1)-1), WG*sin(zPI*t/T), D*zCycloidY(t/T)-D+HG );
  zVec3DCreate( &pl, S, W, 0.0 );
  zVec3DCreate( &pr, 2*S*zCycloidX(t/T), -W, H*zCycloidY(t/T) );
  ik_solve( fout_vs, fout_cs, ik, entry, q, &pg, &pl, &pr );
}

void step3(FILE *fout_vs, FILE *fout_cs, double t, rkIK *ik, rkIKCell *entry[], zVec q)
{
  zVec3D pg, pl, pr;

  zVec3DCreate( &pg, 1.5*S+S*(cosh(t/T)-1)/(cosh(1)-1), -WG*sin(zPI*t/T), D*zCycloidY(t/T)-D+HG );
  zVec3DCreate( &pl, 2*S*zCycloidX(t/T)+S, W, H*zCycloidY(t/T) );
  zVec3DCreate( &pr, 2*S, -W, 0.0 );
  ik_solve( fout_vs, fout_cs, ik, entry, q, &pg, &pl, &pr );
}

void step4(FILE *fout_vs, FILE *fout_cs, double t, rkIK *ik, rkIKCell *entry[], zVec q)
{
  zVec3D pg, pl, pr;

  zVec3DCreate( &pg, 3*S-0.5*S*(cosh(1-t/T)-1)/(cosh(1)-1), WG*sin(zPI*t/T), D*zCycloidX(t/T)-D+HG );
  zVec3DCreate( &pl, 3*S, W, 0.0 );
  zVec3DCreate( &pr, S*zCycloidX(t/T)+S*2, -W, H*zCycloidY(t/T) );
  ik_solve( fout_vs, fout_cs, ik, entry, q, &pg, &pl, &pr );
}

int main(int argc, char *argv[])
{
  rkChain robot;
  rkIK ik;
  register int i;
  rkIKCell *entry[6];
  zVec q;
  FILE *fout_vs, *fout_cs;

  rkChainReadZTK( &robot, "../model/humanoid.ztk" );
  rkIKConfReadZTK( &ik, &robot, "../model/humanoid.ztk" );
  q = zVecAlloc( rkChainJointSize(&robot) );
  for( i=0; i<6; i++ )
    entry[i] = rkIKFindCell( &ik, i );
  fout_vs = fopen( "walk.zvs", "w" );
  fout_cs = fopen( "walk.zcs", "w" );
  for( i=0; i<DIV; i++ ) step1( fout_vs, fout_cs, T*i/DIV, &ik, entry, q );
  for( i=0; i<DIV; i++ ) step2( fout_vs, fout_cs, T*i/DIV, &ik, entry, q );
  for( i=0; i<DIV; i++ ) step3( fout_vs, fout_cs, T*i/DIV, &ik, entry, q );
  for( i=0; i<DIV; i++ ) step4( fout_vs, fout_cs, T*i/DIV, &ik, entry, q );
  fclose( fout_vs );
  fclose( fout_cs );
  zVecFree( q );
  rkIKDestroy( &ik );
  rkChainDestroy( &robot );
  return 0;
}
