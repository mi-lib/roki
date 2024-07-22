/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_ik_seq - inverse kinematics: sequence
 */

#include <roki/rk_chain.h>

/* set IK sequence cell to IK solver. */
bool rkChainSetIKSeqCell(rkChain *chain, rkIKSeqCell *c)
{
  int i;
  rkIKEntry *e;
  rkIKCell *cell;
  bool ret = true;

  for( i=0; i<c->nc; i++ ){
    e = &c->entry[i];
    if( !( cell = rkChainFindIKCellByName( chain, zNamePtr(e) ) ) ){
      ZRUNWARN( RK_ERR_IK_CELL_NOTFOUND, zNamePtr(e) );
      ret = false;
      continue;
    }
    rkIKCellSetWeight( cell, e->w[0], e->w[1], e->w[2] );
    rkIKCellSetRef( cell, e->val[0], e->val[1], e->val[2] );
  }
  return ret;
}

/* free IK sequence cell. */
void rkIKSeqListCellFree(rkIKSeqListCell *cell)
{
  int i;

  for( i=0; i<cell->data.nc; i++ )
    if( zNamePtr( &cell->data.entry[i] ) ) zNameFree( &cell->data.entry[i] );
  free( cell->data.entry );
  free( cell );
}

/* initialize IK sequence. */
rkIKSeq *rkIKSeqInit(rkIKSeq *seq)
{
  zListInit( seq );
  zListRoot(seq)->data.dt = 0;
  zListRoot(seq)->data.nc = 0;
  zListRoot(seq)->data.entry = NULL;
  return seq;
}

/* free IK sequence. */
void rkIKSeqFree(rkIKSeq *seq)
{
  rkIKSeqListCell *cp;

  while( !zListIsEmpty( seq ) ){
    zQueueDequeue( seq, &cp );
    rkIKSeqListCellFree( cp );
  }
}

/* scan an IK sequence from a file. */
bool rkIKSeqScanFile(rkIKSeq *seq, char filename[])
{
  FILE *fp;

  if( !( fp = zOpenFile( filename, RK_IKSEQ_SUFFIX, "r" ) ) )
    return false;
  rkIKSeqFScan( fp, seq );
  fclose( fp );
  return true;
}

/* scan an IK sequence from the current position of a file. */
rkIKSeq *rkIKSeqFScan(FILE *fp, rkIKSeq *seq)
{
  rkIKSeqListCell *cp;
  char name[BUFSIZ];
  int i;

  rkIKSeqInit( seq );
  while( !feof(fp) ){
    if( !zFSkipDelimiter( fp ) ) break;
    if( !( cp = zAlloc( rkIKSeqListCell, 1 ) ) ){
      ZALLOCERROR();
      break;
    }
    zFDouble( fp, &cp->data.dt );
    zFInt( fp, &cp->data.nc );
    if( !( cp->data.entry = zAlloc( rkIKEntry, cp->data.nc ) ) ){
      ZALLOCERROR();
      free( cp );
      break;
    }
    for( i=0; i<cp->data.nc; i++ ){
      zFToken( fp, name, BUFSIZ );
      zNameSet( &cp->data.entry[i], name );
      zFDouble( fp, &cp->data.entry[i].w[0] );
      zFDouble( fp, &cp->data.entry[i].w[1] );
      zFDouble( fp, &cp->data.entry[i].w[2] );
      zFDouble( fp, &cp->data.entry[i].val[0] );
      zFDouble( fp, &cp->data.entry[i].val[1] );
      zFDouble( fp, &cp->data.entry[i].val[2] );
    }
    zQueueEnqueue( seq, cp );
  }
  return seq;
}

/* print IK sequence out to a file. */
bool rkIKSeqPrintFile(rkIKSeq *seq, char filename[])
{
  char fname[BUFSIZ];
  FILE *fp;

  zAddSuffix( filename, RK_IKSEQ_SUFFIX, fname, BUFSIZ );
  if( !( fp = fopen( fname, "w" ) ) ){
    ZOPENERROR( fname );
    return false;
  }
  rkIKSeqFPrint( fp, seq );
  fclose( fp );
  return true;
}

/* print IK sequence out to the current position of a file. */
void rkIKSeqFPrint(FILE *fp, rkIKSeq *seq)
{
  rkIKSeqListCell *cp;
  int i;

  zListForEachRew( seq, cp ){
    fprintf( fp, "%.10g ", cp->data.dt );
    fprintf( fp, "%d", cp->data.nc );
    for( i=0; i<cp->data.nc; i++ )
      fprintf( fp, " %s %.10g %.10g %.10g %.10g %.10g %.10g",
        zName(&cp->data.entry[i]),
        cp->data.entry[i].w[0],
        cp->data.entry[i].w[1],
        cp->data.entry[i].w[2],
        cp->data.entry[i].val[0],
        cp->data.entry[i].val[1],
        cp->data.entry[i].val[2] );
    fprintf( fp, "\n" );
  }
}
