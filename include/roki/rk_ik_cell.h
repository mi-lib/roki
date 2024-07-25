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

/*! \class reference data class */
ZDEF_UNION( __ROKI_CLASS_EXPORT, rkIKRef ){
  zVec3D pos; /*!< position reference */
  zMat3D att; /*!< attitude reference */
#ifdef __cplusplus
  rkIKRef() : att{*ZMAT3DZERO} {};
#endif /* __cplusplus */
};

#define rkIKRefClear(ref) zMat3DZero(&(ref)->att)

/*! \class error accumulator class */
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

/*! \class IK attribute class */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKAttr ){
  int user_defined_type;  /*!< user-defined type identifier */
  int id;                 /*!< attention link IDs */
  int id_sub;             /*!< attention subordinate link IDs */
  zVec3D attention_point; /*!< attention point */
  zVec3D weight;          /*!< weight on constraint */
  ubyte mask;             /*!< mask to specify enabled attributes */
};

/*! \brief masks to specify attributes to be set */
#define RK_IK_ATTR_MASK_NONE            0x00
#define RK_IK_ATTR_MASK_ID              0x01
#define RK_IK_ATTR_MASK_ID_SUB          0x02
#define RK_IK_ATTR_MASK_ATTENTION_POINT 0x04
#define RK_IK_ATTR_MASK_WEIGHT          0x08

/*! \brief initialize IK attribute */
__ROKI_EXPORT rkIKAttr* rkIKAttrInit(rkIKAttr *attr);

/* set constraint mode */

#define rkIKAttrSetLinkID(attr,chain,i)       ( (attr)->id = rkChainFindLinkID( chain, i ) )
#define rkIKAttrSetLinkID2(attr,chain,i)      ( (attr)->id_sub = rkChainFindLinkID( chain, i ) )
#define rkIKAttrSetAttentionPoint(attr,x,y,z) zVec3DCreate( &(attr)->attention_point, (x), (y), (z) )
#define rkIKAttrCopyAttentionPoint(attr,pos)  zVec3DCopy( pos, &(attr)->attention_point )

/* set weight on constraint of IK cell */
#define rkIKAttrSetWeight(attr,w1,w2,w3) zVec3DCreate( &(attr)->weight, w1, w2, w3 )

/*! \brief reference function pointer */
typedef void (* rkIKRef_fp)(rkIKRef *ref, double v1, double v2, double v3);
/*! \brief constraint matrix function pointer */
typedef zMat (* rkIKCMat_fp)(rkChain*,rkIKAttr*,zMat);
/*! \brief constraint vector function pointer */
typedef zVec3D* (* rkIKCVec_fp)(rkChain*,rkIKAttr*,void*,rkIKRef*,zVec3D*);
/*! \brief reference binding function pointer */
typedef void (* rkIKBind_fp)(rkChain*,rkIKAttr*,void*,rkIKRef*);
/*! \brief error accumulation function pointer */
typedef zVec3D* (* rkIKAcm_fp)(rkChain*,rkIKAcm*,void*,zVec3D*);

ZDECL_STRUCT( rkIKCell );

/*! \class IK constraint class */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKConstraint ){
  const char *typestr; /*!< a string to identify constraint type */
  rkIKRef_fp  ref_fp;  /*!< reference function pointer */
  rkIKCMat_fp cmat_fp; /*!< constraint matrix function pointer */
  rkIKCVec_fp cvec_fp; /*!< constraint vector function pointer */
  rkIKBind_fp bind_fp; /*!< reference binding function pointer */
  rkIKAcm_fp  acm_fp;  /*!< error accumulation function pointer */
  /* I/O */
  bool (* fromZTK)(rkChain *chain, rkIKAttr *attr, ubyte *mask, ZTK *ztk); /*!< a function to read attributes of an IK cell from ZTK */
  void (* fprintZTK)(FILE*,rkChain*,rkIKCell*); /*!< a function to print attributes of an IK cell from ZTK */
};
zListClass( rkIKConstraintList, rkIKConstraintListCell, const rkIKConstraint* );

/*! \class IK constraint cell class */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkIKCellDat ){
  Z_NAMED_CLASS; /*!< name of constraint */
  const rkIKConstraint *constraint; /*!< constraint */
  rkIKRef ref;   /*!< referential position or attitude */
  rkIKAttr attr; /*!< attributes of attention quantity */
  int priority;  /*!< priority of the constraint (The larger the number becomes, the higher the priority is.) */
  ubyte mode;    /*!< constraint mode */
  /*! \cond */
  rkIKAcm _acm;  /* error accumulation correction */
  double _eval;  /* weighted-squared norm of residual */
  void *_util;   /* void-type pointer for utility */
  /*! \endcond */
};

zListClass( rkIKCellList, rkIKCell, rkIKCellDat );

#define rkIKCellName(cell)           zName( &(cell)->data )

#define rkIKCellAttr(cell)           ( &(cell)->data.attr )
#define rkIKCellLinkID(cell)         rkIKCellAttr(cell)->id
#define rkIKCellLinkID2(cell)        rkIKCellAttr(cell)->id_sub
#define rkIKCellAttentionPoint(cell) ( &rkIKCellAttr(cell)->attention_point )
#define rkIKCellWeight(cell)         ( &rkIKCellAttr(cell)->weight )

#define rkIKCellPriority(cell)              (cell)->data.priority

#define RK_IK_CELL_MODE_X            0x01
#define RK_IK_CELL_MODE_Y            0x02
#define RK_IK_CELL_MODE_Z            0x04
#define RK_IK_CELL_MODE_XYZ          ( RK_IK_CELL_MODE_X | RK_IK_CELL_MODE_Y | RK_IK_CELL_MODE_Z )
#define RK_IK_CELL_MODE_ENABLE       0x08

/* set constraint mode */

#define rkIKCellEnable(cell)         ( (cell)->data.mode |= RK_IK_CELL_MODE_ENABLE )
#define rkIKCellDisable(cell)        ( (cell)->data.mode &=~RK_IK_CELL_MODE_ENABLE )
#define rkIKCellIsEnabled(cell)      ( (cell)->data.mode & RK_IK_CELL_MODE_ENABLE )

#define rkIKCellSetActiveComponent(cell,xyz) \
  ( (cell)->data.mode = ( (cell)->data.mode & RK_IK_CELL_MODE_ENABLE ) | (xyz) )

#define rkIKCellRef(cell)            ( &(cell)->data.ref )
#define rkIKCellRefPos(cell)         ( &rkIKCellRef(cell)->pos )
#define rkIKCellRefAtt(cell)         ( &rkIKCellRef(cell)->att )

/*! \brief intialize an IK cell */
__ROKI_EXPORT void rkIKCellInit(rkIKCell *cell, int priority, rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util);

/*! \brief create an IK cell. */
__ROKI_EXPORT rkIKCell *rkIKCellCreate(const char *name, int priority, rkIKAttr *attr, ubyte mask, const rkIKConstraint *constraint, void *util);

/*! \brief clone an IK cell. */
__ROKI_EXPORT rkIKCell *rkIKCellClone(rkIKCell *src);

/*! \brief destroy an IK cell. */
__ROKI_EXPORT void rkIKCellDestroy(rkIKCell *cell);

/* set weight on constraint of an IK cell */
#define rkIKCellSetWeight(cell,w1,w2,w3) rkIKAttrSetWeight( rkIKCellAttr(cell), w1, w2, w3 )

/* set referential value and enable the constraint */
#define rkIKCellSetRef(cell,v1,v2,v3) do{\
  rkIKCellEnable( cell );\
  (cell)->data.constraint->ref_fp( rkIKCellRef(cell), v1, v2, v3 );\
} while(0)
#define rkIKCellSetRefVec(cell,v) \
  rkIKCellSetRef(cell,(v)->e[0],(v)->e[1],(v)->e[2])
#define rkIKCellSetRefAtt(cell,r) do{\
  rkIKCellEnable( cell );\
  zMat3DCopy( r, rkIKCellRefAtt(cell) );\
} while(0)

/*! \brief zero the accumulated error of a highly-prioritized IK constraint. */
__ROKI_EXPORT void rkIKCellAcmZero(rkIKCell *cell);

#define rkIKCellGetCMat(cell,r,mat) \
  (cell)->data.constraint->cmat_fp( r, rkIKCellAttr(cell), mat )
#define rkIKCellGetCVec(cell,r,vec) \
  (cell)->data.constraint->cvec_fp( r, rkIKCellAttr(cell), (cell)->data._util, rkIKCellRef(cell), vec )
#define rkIKCellBind(cell,r) do{\
  rkIKCellEnable( cell );\
  (cell)->data.constraint->bind_fp( r, rkIKCellAttr(cell), (cell)->data._util, rkIKCellRef(cell) );\
} while(0)
#define rkIKCellGetAcm(cell,r,v) \
  (cell)->data.constraint->acm_fp( r, &(cell)->data._acm, (cell)->data._util, v )

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

/* ********************************************************** */
/* CLASS: rkIKConstraint
 * inverse kinematics constraint class
 * ********************************************************** */

__ROKI_EXPORT const rkIKConstraint *rkIKConstraintFind(const char *typestr);

__ROKI_EXPORT rkIKConstraintListCell *rkIKConstraintListAdd(const rkIKConstraint *constraint);
__ROKI_EXPORT void rkIKConstraintListDestroy(void);

__ROKI_EXPORT const rkIKConstraint rk_ik_constraint_link_world_pos;
__ROKI_EXPORT const rkIKConstraint rk_ik_constraint_link_world_att;
__ROKI_EXPORT const rkIKConstraint rk_ik_constraint_link2link_pos;
__ROKI_EXPORT const rkIKConstraint rk_ik_constraint_link2link_att;
__ROKI_EXPORT const rkIKConstraint rk_ik_constraint_world_com;
__ROKI_EXPORT const rkIKConstraint rk_ik_constraint_world_angular_momentum;
__ROKI_EXPORT const rkIKConstraint rk_ik_constraint_world_angular_momentum_about_com;

__END_DECLS

#endif /* __RK_IK_CELL_H__ */
