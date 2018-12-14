/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_ik_seq - inverse kinematics: sequence
 */

#include <roki/rk_ik.h>

/* rkIKSeqListCellSet
 * - set IK sequence cell to IK solver.
 */
void rkIKSeqCellSet(rkIK *ik, rkIKSeqCell *c)
{
  register int i;
  rkIKEntry *e;
  rkIKCell *cell;

  for( i=0; i<c->nc; i++ ){
    e = &c->entry[i];
    if( !( cell = rkIKFindCell( ik, e->id ) ) ){
      ZRUNWARN( "IK cell not found" );
      continue;
    }
    rkIKCellSetWeight( cell, e->w[0], e->w[1], e->w[2] );
    rkIKCellSetRef( cell, e->val[0], e->val[1], e->val[2] );
  }
}

/* rkIKSeqInit
 * - initialize IK sequence.
 */
rkIKSeq *rkIKSeqInit(rkIKSeq *seq)
{
  zListInit( seq );
  zListRoot(seq)->data.dt = 0;
  zListRoot(seq)->data.nc = 0;
  zListRoot(seq)->data.entry = NULL;
  return seq;
}

/* rkIKSeqFree
 * - free IK sequence.
 */
void rkIKSeqFree(rkIKSeq *seq)
{
  rkIKSeqListCell *cp;

  while( !zListIsEmpty( seq ) ){
    zQueueDequeue( seq, &cp );
    rkIKSeqListCellFree( cp );
  }
}

/* rkIKSeqReadFile
 * - read an IK sequence from a file.
 */
bool rkIKSeqReadFile(rkIKSeq *seq, char filename[])
{
  FILE *fp;

  if( !( fp = zOpenFile( filename, RK_IKSEQ_SUFFIX, "r" ) ) )
    return false;
  rkIKSeqFRead( fp, seq );
  fclose( fp );
  return true;
}

/* rkIKSeqFRead
 * - read an IK sequence from the current position of a file.
 */
rkIKSeq *rkIKSeqFRead(FILE *fp, rkIKSeq *seq)
{
  rkIKSeqListCell *cp;
  register int i;

  rkIKSeqInit( seq );
  while( !feof(fp) ){
    if( !zFSkipDelimiter( fp ) ) break;
    if( !( cp = zAlloc( rkIKSeqListCell, 1 ) ) ){
      ZALLOCERROR();
      break;
    }
    cp->data.dt = zFDouble(fp);
    cp->data.nc = zFInt(fp);
    if( !( cp->data.entry = zAlloc( rkIKEntry, cp->data.nc ) ) ){
      ZALLOCERROR();
      free( cp );
      break;
    }
    for( i=0; i<cp->data.nc; i++ ){
      cp->data.entry[i].id = zFInt(fp);
      cp->data.entry[i].w[0] = zFDouble(fp);
      cp->data.entry[i].w[1] = zFDouble(fp);
      cp->data.entry[i].w[2] = zFDouble(fp);
      cp->data.entry[i].val[0] = zFDouble(fp);
      cp->data.entry[i].val[1] = zFDouble(fp);
      cp->data.entry[i].val[2] = zFDouble(fp);
    }
    zQueueEnqueue( seq, cp );
  }
  return seq;
}

/* rkIKSeqWriteFile
 * - output IK sequence to a file.
 */
bool rkIKSeqWriteFile(rkIKSeq *seq, char filename[])
{
  char fname[BUFSIZ];
  FILE *fp;

  zAddSuffix( filename, RK_IKSEQ_SUFFIX, fname, BUFSIZ );
  if( !( fp = fopen( fname, "w" ) ) ){
    ZOPENERROR( fname );
    return false;
  }
  rkIKSeqFWrite( fp, seq );
  fclose( fp );
  return true;
}

/* rkIKSeqFWrite
 * - output IK sequence to the current position of a file.
 */
void rkIKSeqFWrite(FILE *fp, rkIKSeq *seq)
{
  rkIKSeqListCell *cp;
  register int i;

  zListForEachRew( seq, cp ){
    fprintf( fp, "%.10g ", cp->data.dt );
    fprintf( fp, "%d", cp->data.nc );
    for( i=0; i<cp->data.nc; i++ )
      fprintf( fp, " %d %.10g %.10g %.10g %.10g %.10g %.10g",
        cp->data.entry[i].id,
        cp->data.entry[i].w[0],
        cp->data.entry[i].w[1],
        cp->data.entry[i].w[2],
        cp->data.entry[i].val[0],
        cp->data.entry[i].val[1],
        cp->data.entry[i].val[2] );
    fprintf( fp, "\n" );
  }
}
