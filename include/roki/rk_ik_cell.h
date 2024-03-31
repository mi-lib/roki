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

/*! \brief masks to specify attributes to be reflected */
#define RK_IK_ATTR_NONE   0x00
#define RK_IK_ATTR_ID     0x01
#define RK_IK_ATTR_ID_SUB 0x02
#define RK_IK_ATTR_AP     0x04
#define RK_IK_ATTR_FORCE  0x08
#define RK_IK_ATTR_WEIGHT 0x10

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKAttr ){
  int id;     /*!< attention link IDs */
  int id_sub; /*!< attention subordinate link IDs */
  zVec3D ap;  /*!< attention point */
  byte mode;  /*!< constraint mode */
  zVec3D w;   /*!< weight on constraint */
};

#define RK_IK_CELL_XON    0x1
#define RK_IK_CELL_YON    0x2
#define RK_IK_CELL_ZON    0x4
#define RK_IK_CELL_FORCE  0x8
#define RK_IK_CELL_ON     ( RK_IK_CELL_XON | RK_IK_CELL_YON | RK_IK_CELL_ZON )

/* set constraint mode */
#define rkIKAttrSetLinkID(a,c,n)  ( (a)->id = rkChainFindLinkID( c, n ) )
#define rkIKAttrSetLinkID2(a,c,n) ( (a)->id_sub = rkChainFindLinkID( c, n ) )
#define rkIKAttrSetAP(a,x,y,z)    zVec3DCreate( &(a)->ap, (x), (y), (z) )
#define rkIKAttrCopyAP(a,p)       zVec3DCopy( p, &(a)->ap )
#define rkIKAttrSetMode(a,m)      ( (a)->mode |= (m) )
#define rkIKAttrUnsetMode(a,m)    ( (a)->mode &= ~(m) )
#define rkIKAttrEnable(a)         rkIKAttrSetMode( a, RK_IK_CELL_ON )
#define rkIKAttrDisable(a)        rkIKAttrUnsetMode( a, RK_IK_CELL_ON )
#define rkIKAttrForce(a)          rkIKAttrSetMode( a, RK_IK_CELL_ON | RK_IK_CELL_FORCE )

/* set weight on constraint of IK cell */
#define rkIKAttrSetWeight(a,w1,w2,w3) zVec3DCreate( &(a)->w, w1, w2, w3 )

typedef void (* rkIKRef_fp)(rkIKRef *ref, double v1, double v2, double v3);
typedef zMat (* rkIKCMat_fp)(rkChain*,rkIKAttr*,zMat);
typedef zVec3D* (* rkIKCVec_fp)(rkChain*,rkIKAttr*,void*,rkIKRef*,zVec3D*);
typedef void (* rkIKBind_fp)(rkChain*,rkIKAttr*,void*,rkIKRef*);
typedef zVec3D* (* rkIKAcm_fp)(rkChain*,rkIKAcm*,void*,zVec3D*);

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKCellDat ){
  int id;        /*!< identifier */
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

#define rkIKCellID(c)        ( (c)->data.id )

#define rkIKCellAttr(c)      ( &(c)->data.attr )
#define rkIKCellLinkID(c)    rkIKCellAttr(c)->id
#define rkIKCellLinkID2(c)   rkIKCellAttr(c)->id_sub
#define rkIKCellAP(c)        ( &rkIKCellAttr(c)->ap )
#define rkIKCellMode(c)      rkIKCellAttr(c)->mode
#define rkIKCellWeight(c)    ( &rkIKCellAttr(c)->w )

#define rkIKCellRef(c)       ( &(c)->data.ref )
#define rkIKCellRefPos(c)    ( &rkIKCellRef(c)->pos )
#define rkIKCellRefAtt(c)    ( &rkIKCellRef(c)->att )

#define rkIKCellIndexOffset(c) (c)->data.index_offset

/* intialize a cell */
__ROKI_EXPORT void rkIKCellInit(rkIKCell *cell, rkIKAttr *attr, int mask, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKCVec_fp vf, rkIKBind_fp bf, rkIKAcm_fp af, void *util);

/*! \brief copy a cell to another. */
#define rkIKCellCopy(src,dest) zCopy( rkIKCell, src, dest )

/* set constraint mode */
#define rkIKCellSetMode(c,m)   rkIKAttrSetMode( rkIKCellAttr(c), m )
#define rkIKCellUnsetMode(c,m) rkIKAttrUnsetMode( rkIKCellAttr(c), m )
#define rkIKCellEnable(c)      rkIKAttrEnable( rkIKCellAttr(c) )
#define rkIKCellDisable(c)     rkIKAttrDisable( rkIKCellAttr(c) )
#define rkIKCellForce(c)       rkIKAttrForce( rkIKCellAttr(c) )
#define rkIKCellIsEnabled(c)   ( ( rkIKCellMode(c) & RK_IK_CELL_ON ) != 0 )
#define rkIKCellIsDisabled(c)  ( ( rkIKCellMode(c) & RK_IK_CELL_ON ) == 0 )
#define rkIKCellIsForced(c)    ( ( rkIKCellMode(c) & RK_IK_CELL_FORCE ) != 0 )

/* set weight on constraint of IK cell */
#define rkIKCellSetWeight(c,w1,w2,w3) rkIKAttrSetWeight( rkIKCellAttr(c), w1, w2, w3 )

#define rkIKCellSetRef(c,v1,v2,v3) do{\
  rkIKCellEnable( c );\
  (c)->data._ref_fp( rkIKCellRef(c), v1, v2, v3 );\
} while(0)
#define rkIKCellSetRefVec(c,v) \
  rkIKCellSetRef(c,(v)->e[0],(v)->e[1],(v)->e[2])
#define rkIKCellSetRefAtt(c,r) do{\
  rkIKCellEnable( c );\
  zMat3DCopy( r, rkIKCellRefAtt(c) );\
} while(0)

#define rkIKCellSetRefForce(c,v1,v2,v3) do{\
  rkIKCellForce( c );\
  (c)->data._ref_fp( rkIKCellRef(c), v1, v2, v3 );\
} while(0)
#define rkIKCellSetRefVecForce(c,v) \
  rkIKCellSetRefForce(c,(v)->e[0],(v)->e[1],(v)->e[2])

/*! \brief zero accumulated errors of high-priority constraints. */
__ROKI_EXPORT void rkIKCellAcmZero(rkIKCell *cell);

#define rkIKCellCMat(c,r,mat) \
  (c)->data._cmat_fp( r, rkIKCellAttr(c), mat )
#define rkIKCellCVec(c,r,vec) \
  (c)->data._cvec_fp( r, rkIKCellAttr(c), (c)->data._util, rkIKCellRef(c), vec )
#define rkIKCellBind(c,r) do{\
  rkIKCellEnable( c );\
  (c)->data._bind_fp( r, rkIKCellAttr(c), (c)->data._util, rkIKCellRef(c) );\
} while(0)
#define rkIKCellAcm(c,r,v) \
  (c)->data._acm_fp( r, &(c)->data.acm, (c)->data._util, v )

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

__END_DECLS

#endif /* __RK_IK_CELL_H__ */
