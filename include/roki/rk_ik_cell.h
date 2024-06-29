/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_ik_cell - inverse kinematics: cell
 */

#ifndef __RK_IK_CELL_H__
#define __RK_IK_CELL_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkIKCell
 * inverse kinematics cell class
 * ********************************************************** */

ZDECL_STRUCT( rkChain );

ZDEF_UNION( __ROKI_CLASS_EXPORT, rkIKRef ){
  zVec3D pos; /*!< position reference */
  zMat3D att; /*!< attitude reference */
#ifdef __cplusplus
  rkIKRef() : att{*ZMAT3DZERO} {};
#endif /* __cplusplus */
};

#define rkIKRefClear(ref) zMat3DZero(&(ref)->att)

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKAcm ){
  union{
    zVec3D p; /*!< accumulated position error */
    zEP e;    /*!< accumulated attitude error by Euler parameter */
  } ae, e_old; /*!< accumulated error */
  zVec3D h_old;
#ifdef __cplusplus
  rkIKAcm() : ae{*ZVEC3DZERO}, e_old{*ZVEC3DZERO}, h_old{*ZVEC3DZERO} {};
#endif /* __cplusplus */
};

/*! \brief masks to specify attributes to be set */
#define RK_IK_ATTR_NONE            0x00
#define RK_IK_ATTR_ID              0x01
#define RK_IK_ATTR_ID_SUB          0x02
#define RK_IK_ATTR_ATTENTION_POINT 0x04
#define RK_IK_ATTR_FORCE           0x08
#define RK_IK_ATTR_WEIGHT          0x10

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKAttr ){
  int user_defined_type; /*!< user-defined type identifier */
  int id;                /*!< attention link IDs */
  int id_sub;            /*!< attention subordinate link IDs */
  zVec3D attention_point; /*!< attention point */
  byte mode;             /*!< constraint mode */
  zVec3D weight;         /*!< weight on constraint */
};

#define RK_IK_CELL_XON    0x1
#define RK_IK_CELL_YON    0x2
#define RK_IK_CELL_ZON    0x4
#define RK_IK_CELL_FORCE  0x8
#define RK_IK_CELL_ON     ( RK_IK_CELL_XON | RK_IK_CELL_YON | RK_IK_CELL_ZON )

/* set constraint mode */
#define rkIKAttrUserDefinedType(attr)         (attr)->user_defined_type
#define rkIKAttrSetUserDefinedType(attr,type) ( (attr)->user_defined_type = (type) )
#define rkIKAttrSetLinkID(attr,chain,i)       ( (attr)->id = rkChainFindLinkID( chain, i ) )
#define rkIKAttrSetLinkID2(attr,chain,i)      ( (attr)->id_sub = rkChainFindLinkID( chain, i ) )
#define rkIKAttrSetAttentionPoint(attr,x,y,z) zVec3DCreate( &(attr)->attention_point, (x), (y), (z) )
#define rkIKAttrCopyAttentionPoint(attr,pos)  zVec3DCopy( pos, &(attr)->attention_point )
#define rkIKAttrSetMode(attr,m)               ( (attr)->mode |= (m) )
#define rkIKAttrUnsetMode(attr,m)             ( (attr)->mode &= ~(m) )
#define rkIKAttrEnable(attr)                  rkIKAttrSetMode( attr, RK_IK_CELL_ON )
#define rkIKAttrDisable(attr)                 rkIKAttrUnsetMode( attr, RK_IK_CELL_ON )
#define rkIKAttrForce(attr)                   rkIKAttrSetMode( attr, RK_IK_CELL_ON | RK_IK_CELL_FORCE )

/* set weight on constraint of IK cell */
#define rkIKAttrSetWeight(attr,w1,w2,w3) zVec3DCreate( &(attr)->weight, w1, w2, w3 )

typedef void (* rkIKRef_fp)(rkIKRef *ref, double v1, double v2, double v3);
typedef zMat (* rkIKCMat_fp)(rkChain*,rkIKAttr*,zMat);
typedef zVec3D* (* rkIKCVec_fp)(rkChain*,rkIKAttr*,void*,rkIKRef*,zVec3D*);
typedef void (* rkIKBind_fp)(rkChain*,rkIKAttr*,void*,rkIKRef*);
typedef zVec3D* (* rkIKAcm_fp)(rkChain*,rkIKAcm*,void*,zVec3D*);

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKCellDat ){
  Z_NAMED_CLASS; /*!< name of constraint */
  rkIKAttr attr; /*!< attributes of attention quantity */
  rkIKRef ref;   /*!< referential position or attitude */
  rkIKAcm acm;   /*!< error accumulation correction */

  rkIKRef_fp _ref_fp;
  rkIKCMat_fp _cmat_fp;
  rkIKCVec_fp _cvec_fp;
  rkIKBind_fp _bind_fp;
  rkIKAcm_fp _acm_fp;
  int index_offset;
  double _eval;  /* weighted-squared norm of residual */
  /* void-type pointer for utility */
  void *_util;
};

zListClass( rkIKCellList, rkIKCell, rkIKCellDat );

#define rkIKCellAttr(cell)           ( &(cell)->data.attr )
#define rkIKCellLinkID(cell)         rkIKCellAttr(cell)->id
#define rkIKCellLinkID2(cell)        rkIKCellAttr(cell)->id_sub
#define rkIKCellAttentionPoint(cell) ( &rkIKCellAttr(cell)->attention_point )
#define rkIKCellMode(cell)           rkIKCellAttr(cell)->mode
#define rkIKCellWeight(cell)         ( &rkIKCellAttr(cell)->weight )

#define rkIKCellRef(cell)            ( &(cell)->data.ref )
#define rkIKCellRefPos(cell)         ( &rkIKCellRef(cell)->pos )
#define rkIKCellRefAtt(cell)         ( &rkIKCellRef(cell)->att )

#define rkIKCellIndexOffset(cell)    (cell)->data.index_offset

/*! \brief intialize an IK cell */
__ROKI_EXPORT void rkIKCellInit(rkIKCell *cell, rkIKAttr *attr, int mask, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKCVec_fp vf, rkIKBind_fp bf, rkIKAcm_fp af, void *util);

/*! \brief create an IK cell. */
__ROKI_EXPORT rkIKCell *rkIKCellCreate(const char *name, rkIKAttr *attr, int mask, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKCVec_fp vf, rkIKBind_fp bf, rkIKAcm_fp af, void *util);

/*! \brief clone an IK cell. */
__ROKI_EXPORT rkIKCell *rkIKCellClone(rkIKCell *src);

/*! \brief destroy an IK cell. */
__ROKI_EXPORT void rkIKCellDestroy(rkIKCell *cell);

/* set constraint mode */
#define rkIKCellSetMode(cell,m)   rkIKAttrSetMode( rkIKCellAttr(cell), m )
#define rkIKCellUnsetMode(cell,m) rkIKAttrUnsetMode( rkIKCellAttr(cell), m )
#define rkIKCellEnable(cell)      rkIKAttrEnable( rkIKCellAttr(cell) )
#define rkIKCellDisable(cell)     rkIKAttrDisable( rkIKCellAttr(cell) )
#define rkIKCellForce(cell)       rkIKAttrForce( rkIKCellAttr(cell) )
#define rkIKCellIsEnabled(cell)   ( ( rkIKCellMode(cell) & RK_IK_CELL_ON ) != 0 )
#define rkIKCellIsDisabled(cell)  ( ( rkIKCellMode(cell) & RK_IK_CELL_ON ) == 0 )
#define rkIKCellIsForced(cell)    ( ( rkIKCellMode(cell) & RK_IK_CELL_FORCE ) != 0 )

/* set weight on constraint of an IK cell */
#define rkIKCellSetWeight(cell,w1,w2,w3) rkIKAttrSetWeight( rkIKCellAttr(cell), w1, w2, w3 )

#define rkIKCellSetRef(cell,v1,v2,v3) do{\
  rkIKCellEnable( cell );\
  (cell)->data._ref_fp( rkIKCellRef(cell), v1, v2, v3 );\
} while(0)
#define rkIKCellSetRefVec(cell,v) \
  rkIKCellSetRef(cell,(v)->e[0],(v)->e[1],(v)->e[2])
#define rkIKCellSetRefAtt(cell,r) do{\
  rkIKCellEnable( cell );\
  zMat3DCopy( r, rkIKCellRefAtt(cell) );\
} while(0)

#define rkIKCellSetRefForce(cell,v1,v2,v3) do{\
  rkIKCellForce( cell );\
  (cell)->data._ref_fp( rkIKCellRef(cell), v1, v2, v3 );\
} while(0)
#define rkIKCellSetRefVecForce(cell,v) \
  rkIKCellSetRefForce(cell,(v)->e[0],(v)->e[1],(v)->e[2])

/*! \brief zero the accumulated error of a highly-prioritized IK constraint. */
__ROKI_EXPORT void rkIKCellAcmZero(rkIKCell *cell);

#define rkIKCellCMat(cell,r,mat) \
  (cell)->data._cmat_fp( r, rkIKCellAttr(cell), mat )
#define rkIKCellCVec(cell,r,vec) \
  (cell)->data._cvec_fp( r, rkIKCellAttr(cell), (cell)->data._util, rkIKCellRef(cell), vec )
#define rkIKCellBind(cell,r) do{\
  rkIKCellEnable( cell );\
  (cell)->data._bind_fp( r, rkIKCellAttr(cell), (cell)->data._util, rkIKCellRef(cell) );\
} while(0)
#define rkIKCellAcm(cell,r,v) \
  (cell)->data._acm_fp( r, &(cell)->data.acm, (cell)->data._util, v )

/* reference */
__ROKI_EXPORT void rkIKRefSetPos(rkIKRef *ref, double x, double y, double z);
__ROKI_EXPORT void rkIKRefSetZYX(rkIKRef *ref, double azim, double elev, double tilt);
__ROKI_EXPORT void rkIKRefSetZYZ(rkIKRef *ref, double heading, double pitch, double bank);
__ROKI_EXPORT void rkIKRefSetAA(rkIKRef *ref, double x, double y, double z);

/* Jacobian matrix */
__ROKI_EXPORT zMat rkIKJacobiLinkWldLin(rkChain *chain, rkIKAttr *attr, zMat j);
__ROKI_EXPORT zMat rkIKJacobiLinkWldAng(rkChain *chain, rkIKAttr *attr, zMat j);
__ROKI_EXPORT zMat rkIKJacobiLinkL2LLin(rkChain *chain, rkIKAttr *attr, zMat j);
__ROKI_EXPORT zMat rkIKJacobiLinkL2LAng(rkChain *chain, rkIKAttr *attr, zMat j);
__ROKI_EXPORT zMat rkIKJacobiCOM(rkChain *chain, rkIKAttr *attr, zMat j);
__ROKI_EXPORT zMat rkIKJacobiAM(rkChain *chain, rkIKAttr *attr, zMat j);
__ROKI_EXPORT zMat rkIKJacobiAMCOM(rkChain *chain, rkIKAttr *attr, zMat j);

/* displacement error */
__ROKI_EXPORT zVec3D *rkIKLinkWldPosErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err);
__ROKI_EXPORT zVec3D *rkIKLinkWldAttErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err);
__ROKI_EXPORT zVec3D *rkIKLinkL2LPosErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err);
__ROKI_EXPORT zVec3D *rkIKLinkL2LAttErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err);
__ROKI_EXPORT zVec3D *rkIKCOMErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err);
__ROKI_EXPORT zVec3D *rkIKAMErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err);
__ROKI_EXPORT zVec3D *rkIKAMCOMErr(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref, zVec3D *err);

/* bind current position/attitude */
__ROKI_EXPORT void rkIKBindLinkWldPos(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref);
__ROKI_EXPORT void rkIKBindLinkWldAtt(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref);
__ROKI_EXPORT void rkIKBindLinkL2LPos(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref);
__ROKI_EXPORT void rkIKBindLinkL2LAtt(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref);
__ROKI_EXPORT void rkIKBindCOM(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref);
__ROKI_EXPORT void rkIKBindAM(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref);
__ROKI_EXPORT void rkIKBindAMCOM(rkChain *chain, rkIKAttr *attr, void *util, rkIKRef *ref);

/* error accumulation correction */

__ROKI_EXPORT zVec3D *rkIKAcmPos(rkChain *chain, rkIKAcm *acm, void *util, zVec3D *vec);
__ROKI_EXPORT zVec3D *rkIKAcmAtt(rkChain *chain, rkIKAcm *acm, void *util, zVec3D *vec);

/*! \brief destroy a list of IK cells. */
__ROKI_EXPORT void rkIKCellListDestroy(rkIKCellList *list);

/*! \brief clone an IK cell list \a src to \a dest. */
__ROKI_EXPORT rkIKCellList *rkIKCellListClone(rkIKCellList *src, rkIKCellList *dest);

__END_DECLS

#endif /* __RK_IK_CELL_H__ */
