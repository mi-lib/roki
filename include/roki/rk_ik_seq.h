/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_ik_seq - inverse kinematics: sequence
 */

#ifndef __RK_IK_SEQ_H__
#define __RK_IK_SEQ_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/*! \brief IK entry read from a file. */
typedef struct{
  int id;        /*!< cell identifier */
  double w[3];   /*!< weight on constraint */
  double val[3]; /*!< referential values */
} rkIKEntry;

/*! \brief IK sequence cell read from a file. */
typedef struct{
  double dt;
  int nc;
  rkIKEntry *entry;
} rkIKSeqCell;

/*! \brief initialize IK sequence cell. */
#define rkIKSeqCellInit(c) do{\
  (c)->dt = 0;\
  (c)->nc = 0;\
  (c)->entry = NULL;\
} while(0)

/*! \brief set IK cell to IK solver. */
__EXPORT void rkIKSeqCellSet(rkIK *ik, rkIKSeqCell *c);

/*! \brief IK sequence. */
zListClass( rkIKSeq, rkIKSeqListCell, rkIKSeqCell);

/*! \brief destroy IK sequence cell. */
#define rkIKSeqListCellFree(c) do{\
  if( (c) ){\
    zFree( (c)->data.entry );\
    free( (c) );\
  }\
} while(0)

/*! \brief initialize IK sequence. */
__EXPORT rkIKSeq *rkIKSeqInit(rkIKSeq *seq);
/*! \brief destroy IK sequence. */
__EXPORT void rkIKSeqFree(rkIKSeq *seq);

/*! \brief suffix to an IK sequence file. */
#define RK_IKSEQ_SUFFIX "zen"

/*! \brief read an IK sequence from a file. */
__EXPORT bool rkIKSeqReadFile(rkIKSeq *seq, char filename[]);
/*! \brief read an IK sequence from the current position of a file. */
__EXPORT rkIKSeq *rkIKSeqFRead(FILE *fp, rkIKSeq *seq);
#define rkIKSeqRead(seq) rkIKSeqFRead( stdin, seq )

/*! \brief write an IK sequence to a file. */
__EXPORT bool rkIKSeqWriteFile(rkIKSeq *seq, char filename[]);
/*! \brief write an IK sequence to the current position of a file. */
__EXPORT void rkIKSeqFWrite(FILE *fp, rkIKSeq *seq);
#define rkIKSeqWrite(seq) rkIKSeqFWrite( stdout, seq )

__END_DECLS

#endif /* __RK_IK_SEQ_H__ */
