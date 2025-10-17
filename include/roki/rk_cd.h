/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_cd - collision detection
 * contributer: 2014-2015 Ken'ya Tanaka
 * contributer: 2014-2015 Naoki Wakisaka
 */

#ifndef __RK_CD_H__
#define __RK_CD_H__

#include <roki/rk_chain.h>

__BEGIN_DECLS

/*! \brief type to classify stationary or movable shape */
typedef enum{ RK_CD_CELL_STAT, RK_CD_CELL_MOVE } rkCDCellType;

/* ********************************************************** */
/*! \brief volume-based collision detection cell class.
 *//* ******************************************************* */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkCDCellDat ){
  zShape3D *shape;   /*!< shape to be checked */
  rkLink *link;      /*!< link which the shape belongs to */
  rkChain *chain;    /*!< chain which the shape belongs to */
  rkCDCellType type; /*!< stationary or movable shape */
  zBox3D bb;         /*!< bounding box in local frame */
  zAABox3D aabb;     /*!< axis aligned bounding box in the world frame */
  zBox3D obb;        /*!< oriented bounding box in the world frame */
  zPH3D ph;          /*!< polyhedron in the world frame */
  /* for a fake-crawler */
  bool slide_mode;
  double slide_vel;
  zVec3D slide_axis;
  /*! \cond */
  bool _bb_is_uptodate; /* check if bounding box is up-to-date */
  bool _ph_is_uptodate; /* check if polyhedron is up-to-date */
  /*! \endcond */
};
zListClass( rkCDCellList, rkCDCell, rkCDCellDat );

/*! \brief register collision detection cell.
 */
__ROKI_EXPORT rkCDCell *rkCDCellReg(rkCDCellList *clist, rkChain *chain, rkLink *link, zShape3D *shape, rkCDCellType type);

/*! \brief update the bounding box of a collision detection cell.
 */
__ROKI_EXPORT void rkCDCellUpdateBB(rkCDCell *cell);

/*! \brief update the polyhedron of a collision detection cell.
 */
__ROKI_EXPORT void rkCDCellUpdatePH(rkCDCell *cell);

/*! \brief update a collision detection cell.
 */
__ROKI_EXPORT void rkCDCellUpdate(rkCDCell *cell);

/* ********************************************************** */
/*! \brief vertex-based collision detection cell class.
 *//* ******************************************************* */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkCDVertDat ){
  rkCDCell *cell;  /*!< collision detection cell */
  zVec3D *vert;    /*!< the vertex to be checked */
  zVec3D norm;     /*!< normal vector at the vertex */
  zVec3D axis[3];  /*!< z-x-y */
  zVec3D pro;      /*!< projection point of the vertex */
  zVec3D ref;      /*!< referential point to measure displacement of the vertex */
  /*! \cond */
  zVec3D _norm;    /* normal vector at the vertex in the link frame */
  zVec3D _axis[3]; /* axes of contact frame */
  zVec3D _pro;     /* projection point of the vertex in the link frame */
  zVec3D _ref;     /* referential point to measure displacement of the vertex in the link frame */
  zVec3D f;        /* contact force */
  rkContactFricType type; /* type to classify stick/slip mode */
  zVec3D dir;      /* direction of kinetic friction */
  zVec3D vel;      /* velocity of the vertex */
  /*! \endcond */
};
zListClass( rkCDVertList, rkCDVert, rkCDVertDat );

/* ********************************************************** */
/*! \brief collision plane class.
 *//* ******************************************************* */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkCDPlaneDat ){
  zVec3D v;
  zVec3D norm;
  zVec2D r; /* pos on plane coordinate */
  zVec2D s; /* sliding direction */
};
zListClass( rkCDPlaneList, rkCDPlane, rkCDPlaneDat );

/* ********************************************************** */
/*! \brief pair of collision class.
 *//* ******************************************************* */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkCDPairDat ){
  rkCDCell *cell[2];    /*!< a pair of collision detection cells */
  bool is_col;          /*!< flag to check collision */
  rkCDVertList vlist;   /*!< a list of vertices */
  zVec3D norm;          /*!< normal vector of the pair */
  zVec3D axis[3];       /*!< contact bases */
  zVec3D center;        /*!< center of collision volume */
  zPH3D colvol;         /*!< collision volume */
  rkCDPlaneList cplane; /*!< contact plane */
  rkContactInfo *ci;    /*!< contact information */
  zVec6D wrench;        /*!< contact wrench */
  zFrame3D ref[2];
  rkContactFricType type; /* type to classify stick/slip mode */
};
zListClass( rkCDPairList, rkCDPair, rkCDPairDat );

/* ********************************************************** */
/*! \brief collision detector class.
 *//* ******************************************************* */
ZDEF_STRUCT( __ROKI_CLASS_EXPORT, rkCD ){
  rkCDCellList clist; /*!< a list of collision detection cells */
  rkCDPairList plist; /*!< a list of collision detection pairs */
  int colnum;         /*!< number of collision pairs */
  rkContactFricType def_type; /*!< default friction type */
};

__ROKI_EXPORT rkCD *rkCDCreate(rkCD *cd);
__ROKI_EXPORT void rkCDDestroy(rkCD *cd);
__ROKI_EXPORT void rkCDReset(rkCD *cd);
__ROKI_EXPORT void rkCDSetDefaultFricType(rkCD *cd, rkContactFricType type);

__ROKI_EXPORT rkCD *rkCDChainReg(rkCD *cd, rkChain *chain, rkCDCellType type);
__ROKI_EXPORT void rkCDChainUnreg(rkCD *cd, rkChain *chain);

__ROKI_EXPORT rkCD *rkCDPairReg(rkCD *cd, rkLink *link1, rkLink *link2);
__ROKI_EXPORT void rkCDPairUnreg(rkCD *cd, rkLink *link1, rkLink *link2);
__ROKI_EXPORT void rkCDPairChainUnreg(rkCD *cd, rkChain *chain);

__ROKI_EXPORT void rkCDPairPrint(rkCD *cd);
__ROKI_EXPORT void rkCDPairVertPrint(rkCD *cd);

__ROKI_EXPORT bool rkCDColChkAABB(rkCD *cd);     /* AABB */
__ROKI_EXPORT bool rkCDColChkOBB(rkCD *cd);      /* AABB->OBB */
__ROKI_EXPORT bool rkCDColChkGJK(rkCD *cd);      /* AABB->OBB->GJK */
__ROKI_EXPORT void rkCDColChkVert(rkCD *cd);     /* AABB->OBB->Vert(PH) */
__ROKI_EXPORT void rkCDColChkOBBVert(rkCD *cd);  /* AABB->OBB->Vert(OBB) */

__ROKI_EXPORT void rkCDColChkGJKOnly(rkCD *cd);  /* GJK */

__ROKI_EXPORT void rkCDColVol(rkCD *cd);         /* AABB->OBB->GJK->MP */
__ROKI_EXPORT void rkCDColVolDestroy(rkCD *cd);
__ROKI_EXPORT void rkCDColVolBREP(rkCD *cd);     /* AABB->OBB->BREP->CH */
__ROKI_EXPORT void rkCDColVolBREPFast(rkCD *cd); /* AABB->OBB->BREP->CH(Fast) */

__ROKI_EXPORT void rkCDColVolBREPVert(rkCD *cd); /* AABB->OBB->BREP->CH, Vert */

/* for fd */
__ROKI_EXPORT rkCDPlaneList *rkCDPlaneListQuickSort(rkCDPlaneList *list, int (*cmp)(void*,void*,void*), void *priv);

__END_DECLS

#endif /* __RK_CD_H__ */
