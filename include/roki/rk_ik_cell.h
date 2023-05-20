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
#define RK_IK_CELL_ATTR_NONE   0x00
#define RK_IK_CELL_ATTR_ID     0x01
#define RK_IK_CELL_ATTR_ID_SUB 0x02
#define RK_IK_CELL_ATTR_AP     0x04
#define RK_IK_CELL_ATTR_FORCE  0x08
#define RK_IK_CELL_ATTR_WEIGHT 0x10

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKCellAttr ){
  int id;     /*!< attented link IDs */
  int id_sub; /*!< attented subordinate link IDs */
  zVec3D ap;  /*!< attented point */
  byte mode;  /*!< constraint mode */
  zVec3D w;   /*!< weight on constraint */
};

#define RK_IK_CELL_XON    0x1
#define RK_IK_CELL_YON    0x2
#define RK_IK_CELL_ZON    0x4
#define RK_IK_CELL_FORCE  0x8
#define RK_IK_CELL_ON     ( RK_IK_CELL_XON | RK_IK_CELL_YON | RK_IK_CELL_ZON )

/* set constraint mode */
#define rkIKCellAttrSetLink(a,c,n)    ( (a)->id = rkChainFindLinkID( c, n ) )
#define rkIKCellAttrSetSubLink(a,c,n) ( (a)->id_sub = rkChainFindLinkID( c, n ) )
#define rkIKCellAttrSetMode(a,m)      ( (a)->mode |= (m) )
#define rkIKCellAttrUnsetMode(a,m)    ( (a)->mode &= ~(m) )
#define rkIKCellAttrEnable(a)         rkIKCellAttrSetMode( a, RK_IK_CELL_ON )
#define rkIKCellAttrDisable(a)        rkIKCellAttrUnsetMode( a, RK_IK_CELL_ON )
#define rkIKCellAttrForce(a)          rkIKCellAttrSetMode( a, RK_IK_CELL_ON | RK_IK_CELL_FORCE )

/* set weight on constraint of IK cell */
#define rkIKCellAttrSetWeight(a,w1,w2,w3) zVec3DCreate( &(a)->w, w1, w2, w3 )

typedef void (* rkIKRef_fp)(rkIKRef *ref, double v1, double v2, double v3);
typedef zMat (* rkIKCMat_fp)(rkChain*,rkIKCellAttr*,zMat);
typedef zVec3D* (* rkIKCVec_fp)(rkChain*,rkIKCellAttr*,void*,rkIKRef*,zVec3D*);
typedef void (* rkIKBind_fp)(rkChain*,rkIKCellAttr*,void*,rkIKRef*);
typedef zVec3D* (* rkIKAcm_fp)(rkChain*,rkIKAcm*,void*,zVec3D*);

ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKCellDat ){
  int id;            /*!< identifier */
  rkIKCellAttr attr; /*!< attributes of attented quantity */
  rkIKRef ref;       /*!< referential position or attitude */
  rkIKAcm acm;       /*!< error accumulation correction */

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

/* intialize a cell */
__ROKI_EXPORT void rkIKCellInit(rkIKCell *cell, rkIKCellAttr *attr, int mask, rkIKRef_fp rf, rkIKCMat_fp mf, rkIKCVec_fp vf, rkIKBind_fp bf, rkIKAcm_fp af, void *util);

/* set constraint mode */
#define rkIKCellSetMode(c,m)   rkIKCellAttrSetMode( &(c)->data.attr, m )
#define rkIKCellUnsetMode(c,m) rkIKCellAttrUnsetMode( &(c)->data.attr, m )
#define rkIKCellEnable(c)      rkIKCellAttrEnable( &(c)->data.attr )
#define rkIKCellDisable(c)     rkIKCellAttrDisable( &(c)->data.attr )
#define rkIKCellForce(c)       rkIKCellAttrForce( &(c)->data.attr )
#define rkIKCellIsEnabled(c)   ( ( (c)->data.attr.mode & RK_IK_CELL_ON ) != 0 )
#define rkIKCellIsDisabled(c)  ( ( (c)->data.attr.mode & RK_IK_CELL_ON ) == 0 )
#define rkIKCellIsForced(c)    ( ( (c)->data.attr.mode & RK_IK_CELL_FORCE ) != 0 )

/* set weight on constraint of IK cell */
#define rkIKCellSetWeight(c,w1,w2,w3) rkIKCellAttrSetWeight( &(c)->data.attr, w1, w2, w3 )

#define rkIKCellSetRef(c,v1,v2,v3) do{\
  rkIKCellEnable( c );\
  (c)->data._ref_fp( &(c)->data.ref, v1, v2, v3 );\
} while(0)
#define rkIKCellSetRefVec(c,v) \
  rkIKCellSetRef(c,(v)->e[0],(v)->e[1],(v)->e[2])

#define rkIKCellSetRefForce(c,v1,v2,v3) do{\
  rkIKCellForce( c );\
  (c)->data._ref_fp( &(c)->data.ref, v1, v2, v3 );\
} while(0)
#define rkIKCellSetRefVecForce(c,v) \
  rkIKCellSetRefForce(c,(v)->e[0],(v)->e[1],(v)->e[2])

/*! \brief zero accumulated errors of high-priority constraints. */
__ROKI_EXPORT void rkIKCellAcmZero(rkIKCell *cell);

#define rkIKCellCMat(c,r,mat) \
  (c)->data._cmat_fp( r, &(c)->data.attr, mat )
#define rkIKCellCVec(c,r,vec) \
  (c)->data._cvec_fp( r, &(c)->data.attr, (c)->data._util, &(c)->data.ref, vec )
#define rkIKCellBind(c,r) do{\
  rkIKCellEnable( c );\
  (c)->data._bind_fp( r, &(c)->data.attr, (c)->data._util, &(c)->data.ref );\
} while(0)
#define rkIKCellAcm(c,r,v) \
  (c)->data._acm_fp( r, &(c)->data.acm, (c)->data._util, v )

/* reference */
__ROKI_EXPORT void rkIKRefSetPos(rkIKRef *ref, double x, double y, double z);
__ROKI_EXPORT void rkIKRefSetZYX(rkIKRef *ref, double azim, double elev, double tilt);
__ROKI_EXPORT void rkIKRefSetZYZ(rkIKRef *ref, double heading, double pitch, double bank);
__ROKI_EXPORT void rkIKRefSetAA(rkIKRef *ref, double x, double y, double z);

/* Jacobian matrix */
__ROKI_EXPORT zMat rkIKJacobiLinkWldLin(rkChain *chain, rkIKCellAttr *attr, zMat j);
__ROKI_EXPORT zMat rkIKJacobiLinkWldAng(rkChain *chain, rkIKCellAttr *attr, zMat j);
__ROKI_EXPORT zMat rkIKJacobiLinkL2LLin(rkChain *chain, rkIKCellAttr *attr, zMat j);
__ROKI_EXPORT zMat rkIKJacobiLinkL2LAng(rkChain *chain, rkIKCellAttr *attr, zMat j);
__ROKI_EXPORT zMat rkIKJacobiCOM(rkChain *chain, rkIKCellAttr *attr, zMat j);
__ROKI_EXPORT zMat rkIKJacobiAM(rkChain *chain, rkIKCellAttr *attr, zMat j);
__ROKI_EXPORT zMat rkIKJacobiAMCOM(rkChain *chain, rkIKCellAttr *attr, zMat j);

/* displacement error */
__ROKI_EXPORT zVec3D *rkIKLinkWldPosErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err);
__ROKI_EXPORT zVec3D *rkIKLinkWldAttErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err);
__ROKI_EXPORT zVec3D *rkIKLinkL2LPosErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err);
__ROKI_EXPORT zVec3D *rkIKLinkL2LAttErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err);
__ROKI_EXPORT zVec3D *rkIKCOMErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err);
__ROKI_EXPORT zVec3D *rkIKAMErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err);
__ROKI_EXPORT zVec3D *rkIKAMCOMErr(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref, zVec3D *err);

/* bind current position/attitude */
__ROKI_EXPORT void rkIKBindLinkWldPos(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref);
__ROKI_EXPORT void rkIKBindLinkWldAtt(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref);
__ROKI_EXPORT void rkIKBindLinkL2LPos(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref);
__ROKI_EXPORT void rkIKBindLinkL2LAtt(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref);
__ROKI_EXPORT void rkIKBindCOM(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref);
__ROKI_EXPORT void rkIKBindAM(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref);
__ROKI_EXPORT void rkIKBindAMCOM(rkChain *chain, rkIKCellAttr *attr, void *util, rkIKRef *ref);

/* error accumulation correction */

__ROKI_EXPORT zVec3D *rkIKAcmPos(rkChain *chain, rkIKAcm *acm, void *util, zVec3D *vec);
__ROKI_EXPORT zVec3D *rkIKAcmAtt(rkChain *chain, rkIKAcm *acm, void *util, zVec3D *vec);

__END_DECLS

#endif /* __RK_IK_CELL_H__ */
