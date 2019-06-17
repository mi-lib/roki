/* RoKi - Robot Kinetics library
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * rk_force - wrench (six-axis force vector)
 */

#include <roki/rk_force.h>

/* ********************************************************** */
/* CLASS: rkWrench & rkWrenchList
 * wrench list class
 * ********************************************************** */

/* pop a wrench from a list. */
rkWrench *rkWrenchListPop(rkWrenchList *wl)
{
  rkWrench *cell;

  if( zListIsEmpty( wl ) ) return NULL;
  zListDeleteTail( wl, &cell );
  return cell;
}

/* shift a wrench. */
zVec6D *rkWrenchShift(rkWrench *cell, zVec6D *w)
{
  return zVec6DAngShift( rkWrenchW(cell), rkWrenchPos(cell), w );
}

/* calculate the net wrench of a list. */
zVec6D *rkWrenchListNet(rkWrenchList *list, zVec6D *w)
{
  rkWrench *cell;
  zVec6D n;

  zVec6DZero( w );
  zListForEach( list, cell ){
    rkWrenchShift( cell, &n );
    zVec6DAddDRC( w, &n );
  }
  return w;
}

/* print wrench out to a file. */
void rkWrenchFPrint(FILE *fp, rkWrench *cell)
{
  if( !cell ) return;

  fprintf( fp, " force: " );
  zVec3DFPrint( fp, rkWrenchForce( cell ) );
  fprintf( fp, " torque: " );
  zVec3DFPrint( fp, rkWrenchTorque( cell ) );
  fprintf( fp, " point of action: " );
  zVec3DFPrint( fp, rkWrenchPos( cell ) );
}
