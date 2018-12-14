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

#define rkWrenchClear(c) do{\
  rkWrenchSetW( c, ZVEC6DZERO );\
  rkWrenchSetPos( c, ZVEC3DZERO );\
} while(0)

#define rkWrenchInit(c) do{\
  zListCellInit( c );\
  rkWrenchClear( c );\
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

/*! \brief convert wrench list to a net wrench.
 *
 * rkWrenchToForce6D() converts the properties of the force list
 * cell \a cell, a 6D force acting at a certain point, to the
 * equivalent 6D force vector \a force acting at the original point.
 *
 * rkWrenchListToForce6D() converts \a list to the resultant 6D
 * force \a force acting at the original point.
 * \retval
 * rkWrenchToForce6D() and rkWrenchListToForce6D() return a
 * pointer to \a force.
 */
__EXPORT zVec6D *rkWrenchXfer(rkWrench *cell, zVec6D *w);
__EXPORT zVec6D *rkWrenchListNet(rkWrenchList *list, zVec6D *w);

/*! \brief output wrench.
 *
 * rkWrenchFWrite() writes out the properties of the force list cell
 * \a cell to the current position in the file \a fp in the following
 * format.
 *
 *  force: { <x>, <y>, <z> }
 *  moment: { <x>, <y>, <z> }
 *  point of action: { <x>, <y>, <z> }
 *
 * rkWrenchWrite() writes out the properties of \a cell to the
 * standard output.
 * \retval
 * Neither rkWrenchFWrite() nor rkWrenchWrite() returns any values.
 */
__EXPORT void rkWrenchFWrite(FILE *fp, rkWrench *cell);
#define rkWrenchWrite(c) rkWrenchFWrite( stdout, (c) )

__END_DECLS

#endif /* __RK_FORCE_H__ */
