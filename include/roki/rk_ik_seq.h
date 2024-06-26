/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_ik_seq - inverse kinematics: sequence
 */

#ifndef __RK_IK_SEQ_H__
#define __RK_IK_SEQ_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/*! \brief IK entry scanned from a file. */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKEntry ){
  Z_NAMED_CLASS;
  double w[3];   /*!< weight on constraint */
  double val[3]; /*!< referential values */
};

/*! \brief IK sequence cell scanned from a file. */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKSeqCell ){
  double dt;
  int nc;
  rkIKEntry *entry;
};

/*! \brief initialize IK sequence cell. */
#define rkIKSeqCellInit(c) do{\
  (c)->dt = 0;\
  (c)->nc = 0;\
  (c)->entry = NULL;\
} while(0)

/*! \brief set IK cell to IK solver. */
__ROKI_EXPORT bool rkChainSetIKSeqCell(rkChain *chain, rkIKSeqCell *c);

/*! \brief IK sequence. */
zListClass( rkIKSeq, rkIKSeqListCell, rkIKSeqCell);

/*! \brief free IK sequence cell. */
__ROKI_EXPORT void rkIKSeqListCellFree(rkIKSeqListCell *cell);

/*! \brief initialize IK sequence. */
__ROKI_EXPORT rkIKSeq *rkIKSeqInit(rkIKSeq *seq);
/*! \brief destroy IK sequence. */
__ROKI_EXPORT void rkIKSeqFree(rkIKSeq *seq);

/*! \brief suffix to an IK sequence file. */
#define RK_IKSEQ_SUFFIX "zcs"

/*! \brief scan an IK sequence from a file. */
__ROKI_EXPORT bool rkIKSeqScanFile(rkIKSeq *seq, char filename[]);
/*! \brief scan an IK sequence from the current position of a file. */
__ROKI_EXPORT rkIKSeq *rkIKSeqFScan(FILE *fp, rkIKSeq *seq);
#define rkIKSeqScan(seq) rkIKSeqFScan( stdin, seq )

/*! \brief print an IK sequence out to a file. */
__ROKI_EXPORT bool rkIKSeqPrintFile(rkIKSeq *seq, char filename[]);
/*! \brief print an IK sequence out to the current position of a file. */
__ROKI_EXPORT void rkIKSeqFPrint(FILE *fp, rkIKSeq *seq);
#define rkIKSeqPrint(seq) rkIKSeqFPrint( stdout, seq )

__END_DECLS

#endif /* __RK_IK_SEQ_H__ */
