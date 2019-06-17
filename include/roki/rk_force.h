/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_force - wrench (six-axis force vector)
 */

#ifndef __RK_FORCE_H__
#define __RK_FORCE_H__

#include <zm/zm.h>
#include <zeo/zeo.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: rkWrench & rkWrenchList
 * wrench list class
 * ********************************************************** */

typedef struct{
  zVec6D w;
  zVec3D p; /* point of action */
} rkWrenchData;
zListClass( rkWrenchList, rkWrench, rkWrenchData );

#define rkWrenchW(c)           ( &(c)->data.w )
#define rkWrenchForce(c)       zVec6DLin( rkWrenchW(c) )
#define rkWrenchTorque(c)      zVec6DAng( rkWrenchW(c) )
#define rkWrenchPos(c)         ( &(c)->data.p )
#define rkWrenchSetW(c,f)      zVec6DCopy( f, rkWrenchW(c) )
#define rkWrenchSetForce(c,f)  zVec3DCopy( f, rkWrenchForce(c) )
#define rkWrenchSetTorque(c,m) zVec3DCopy( m, rkWrenchTorque(c) )
#define rkWrenchSetPos(c,p)    zVec3DCopy( p, rkWrenchPos(c) )

#define rkWrenchZero(c) do{\
  rkWrenchSetW( c, ZVEC6DZERO );\
  rkWrenchSetPos( c, ZVEC3DZERO );\
} while(0)

#define rkWrenchInit(c) do{\
  zListCellInit( c );\
  rkWrenchZero( c );\
} while(0)

/*! \brief push from and pop to wrench list.
 *
 * rkWrenchListPush() pushes a new force list cell \a f to the
 * list \a fl.
 *
 * rkWrenchListPop() pops the latest force list cell from \a fl.
 *
 * rkWrenchListDestroy() destroys \a fl, freeing all cells.
 * \notes
 * When \a fl includes statically-allocated cells, rkWrenchListDestroy()
 * causes segmentation fault.
 * \retval
 * rkWrenchListPush() returns a pointer to the cell pushed.
 * rkWrenchListPop() returns a pointer to the cell poped.
 * rkWrenchListDestroy() returns no value.
 */
#define rkWrenchListPush(l,w)  zListInsertTail( l, w )
__EXPORT rkWrench *rkWrenchListPop(rkWrenchList *wl);
#define rkWrenchListDestroy(l) zListDestroy( rkWrench, l )

/*! \brief shift and add wrench.
 *
 * rkWrenchShift() shifts a wrench of a list cell \a cell to the
 * equivalent wrench \a w acting at the original point.
 *
 * rkWrenchListNet() accumulates a list of wrenches \a list to
 * the resultant net 6D force \a w acting at the original point.
 * \retval
 * rkWrenchShift() and rkWrenchListNet() return a pointer \a force.
 */
__EXPORT zVec6D *rkWrenchShift(rkWrench *cell, zVec6D *w);
__EXPORT zVec6D *rkWrenchListNet(rkWrenchList *list, zVec6D *w);

/*! \brief print out a wrench.
 *
 * rkWrenchFPrint() prints out properties of the wrench list
 * cell \a cell to the current position of a file \a fp in the
 * following format.
 *
 *  force: { <x>, <y>, <z> }
 *  moment: { <x>, <y>, <z> }
 *  point of action: { <x>, <y>, <z> }
 *
 * rkWrenchPrint() prints out properties of \a cell to the
 * standard output.
 * \retval
 * Neither rkWrenchFPrint() nor rkWrenchPrint() return any values.
 */
__EXPORT void rkWrenchFPrint(FILE *fp, rkWrench *cell);
#define rkWrenchPrint(c) rkWrenchFPrint( stdout, (c) )

__END_DECLS

#endif /* __RK_FORCE_H__ */
